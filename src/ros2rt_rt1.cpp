#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rosrt_rt1/msg/rt1_sensor.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <queue>
#include <mutex>
#include <cstring>
#include <cmath>

#define VERSION_NUMBER "2.0.1"
#define REAR_TREAD_RT1 0.42  // Rear tread of RT1 robot in meters
#define SPEED_COEFF_RT1 311.111  // Speed conversion coefficient
#define SENSOR_GAP_RT1 0.16  // Gap between pressure sensors in meters

class RosrtRt1 : public rclcpp::Node
{
public:
    RosrtRt1() : Node("rosrt_rt1")
    {
        RCLCPP_INFO(this->get_logger(), "rosrt_rt1 : version %s", VERSION_NUMBER);

        this->declare_parameter("port", "/dev/ttyUSB0");
        port_ = this->get_parameter("port").as_string();

        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&RosrtRt1::twistCallback, this, std::placeholders::_1));
        sensor_pub_ = this->create_publisher<rosrt_rt1::msg::Rt1Sensor>("rosrt_rt1", 10);

        fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", port_.c_str());
            rclcpp::shutdown();
            return;
        }

        setupUART();
        initializeRT1();
        serial_thread_ = std::thread(&RosrtRt1::serialThread, this);
    }

    ~RosrtRt1()
    {
        if (fd_ >= 0) {
            sendRT1Command("fdrive0");
            sendRT1Command("mtlr1");
            sendRT1Command("mtrr1");
            sendRT1Command("sar1");
            sendRT1Command("syr1");
            sendRT1Command("sfr1");
            close(fd_);
        }
        if (serial_thread_.joinable()) {
            serial_thread_.join();
        }
    }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        twist_queue_.push(*msg);
    }

    void serialThread()
    {
        while (rclcpp::ok()) {
            processIncomingData();
            sendTwistCommands();
            publishSensorData();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void processIncomingData()
    {
        char buffer[256];
        ssize_t bytes_read = read(fd_, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            for (ssize_t i = 0; i < bytes_read; ++i) {
                if (buffer[i] == '\n') {
                    receive_buffer[receive_len] = '\0';
                    parseReceivedData(receive_buffer);
                    receive_len = 0;
                } else if (buffer[i] >= 0x20) {
                    receive_buffer[receive_len++] = buffer[i];
                    if (receive_len >= 255) receive_len = 255;
                }
            }
        }
    }

    void parseReceivedData(const char* data)
    {
        if (data[0] == 'm' && data[1] == 't') {
            if (data[2] == 'l') {
                latest_speed_left_ = std::atoi(data + 3) / 3600.0;  // m/h to m/s
            } else if (data[2] == 'r') {
                latest_speed_right_ = std::atoi(data + 3) / 3600.0;  // m/h to m/s
            }
        } else if (data[0] == 's') {
            if (data[1] == 'a') {
                int pos = 2;
                std::strtol(data + pos, nullptr, 10);  // Skip first two values
                std::strtol(data + pos, nullptr, 10);
                sensor_data_.accel.linear.y = std::strtol(data + pos, nullptr, 10) / -1000.0;
                std::strtol(data + pos, nullptr, 10);
                std::strtol(data + pos, nullptr, 10);
                sensor_data_.accel.linear.x = std::strtol(data + pos, nullptr, 10) / 1000.0;
                sensor_data_.accel.linear.z = std::strtol(data + pos, nullptr, 10) / 1000.0;
            } else if (data[1] == 'y') {
                int pos = 2;
                sensor_data_.velocity.angular.y = std::strtol(data + pos, nullptr, 10) / -10000.0;
                sensor_data_.velocity.angular.x = std::strtol(data + pos, nullptr, 10) / 10000.0;
                sensor_data_.velocity.angular.z = std::strtol(data + pos, nullptr, 10) / 10000.0;
            } else if (data[1] == 'f') {
                int pos = 2;
                latest_force_left_ = std::strtol(data + pos, nullptr, 10) / 1000.0;
                latest_force_right_ = std::strtol(data + pos, nullptr, 10) / 1000.0;
            }
        }
    }

    void sendTwistCommands()
    {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        if (!twist_queue_.empty()) {
            auto twist = twist_queue_.front();
            twist_queue_.pop();
            
            int fspeed, fradiu;
            convertTwistToRT1Commands(twist, fspeed, fradiu);

            sendRT1Command("fspeed", fspeed);
            sendRT1Command("fradiu", fradiu);

            last_command_time_ = std::chrono::steady_clock::now();
        } else {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_command_time_).count() >= 1) {
                sendRT1Command("fdrive1");
                sendRT1Command("fturn1");
                sendRT1Command("fspeed2048");
                sendRT1Command("fradiu2048");
            }
        }
    }

    void convertTwistToRT1Commands(const geometry_msgs::msg::Twist& twist, int& fspeed, int& fradiu)
    {
        double speed = twist.linear.x;
        double rotate = twist.angular.z;

        if (std::abs(rotate) < 1e-6 && std::abs(speed) < 1e-6) {
            fspeed = 2048;
            fradiu = 2048;
            return;
        }

        double speed_left = speed - rotate * REAR_TREAD_RT1 / 2.0;
        double speed_right = speed + rotate * REAR_TREAD_RT1 / 2.0;

        speed_left *= SPEED_COEFF_RT1;
        speed_right *= SPEED_COEFF_RT1;

        if (speed > 0.0) {
            if (rotate > 0.0) {
                fspeed = static_cast<int>(speed_right);
                fradiu = static_cast<int>(1000.0 * speed_left / speed_right) - 1000;
            } else {
                fspeed = static_cast<int>(speed_left);
                fradiu = 1000 - static_cast<int>(1000.0 * speed_right / speed_left);
            }
        } else {
            if (rotate < 0.0) {
                fspeed = static_cast<int>(speed_right);
                fradiu = static_cast<int>(1000.0 * speed_left / speed_right) - 1000;
            } else {
                fspeed = static_cast<int>(speed_left);
                fradiu = 1000 - static_cast<int>(1000.0 * speed_right / speed_left);
            }
        }

        fspeed = std::clamp(fspeed, -2000, 2000);
        fradiu = std::clamp(fradiu, -2000, 2000);

        fspeed += 2048;
        fradiu += 2048;
    }

    void sendRT1Command(const std::string& command, int value = -1)
    {
        std::string cmd = command;
        if (value >= 0) {
            cmd += std::to_string(value);
        }
        cmd += "\r\n";
        write(fd_, cmd.c_str(), cmd.length());
        write(fd_, cmd.c_str(), cmd.length());
        write(fd_, cmd.c_str(), cmd.length());
    }

    void publishSensorData()
    {
        sensor_data_.velocity.linear.x = (latest_speed_left_ + latest_speed_right_) / 2.0;
        sensor_data_.handle.force.x = (latest_force_left_ + latest_force_right_) / 2.0;
        sensor_data_.handle.torque.z = (latest_force_right_ - latest_force_left_) / SENSOR_GAP_RT1;

        sensor_pub_->publish(sensor_data_);
    }

    void setupUART()
    {
        struct termios tty;
        tcgetattr(fd_, &tty);
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_oflag &= ~OPOST;
        tcsetattr(fd_, TCSANOW, &tty);
    }

    void initializeRT1()
    {
        sendRT1Command("setmode dbg rtw12345");
        sendRT1Command("fdrive1");
        sendRT1Command("fturn1");
        sendRT1Command("fspeed2048");
        sendRT1Command("fradiu2048");
        sendRT1Command("mtlr2");
        sendRT1Command("mtrr2");
        sendRT1Command("sar2");
        sendRT1Command("syr2");
        sendRT1Command("sfr2");
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<rosrt_rt1::msg::Rt1Sensor>::SharedPtr sensor_pub_;
    std::string port_;
    int fd_;
    std::thread serial_thread_;
    std::queue<geometry_msgs::msg::Twist> twist_queue_;
    std::mutex twist_mutex_;
    char receive_buffer[256];
    int receive_len = 0;
    double latest_speed_left_ = 0.0;
    double latest_speed_right_ = 0.0;
    double latest_force_left_ = 0.0;
    double latest_force_right_ = 0.0;
    rosrt_rt1::msg::Rt1Sensor sensor_data_;
    std::chrono::steady_clock::time_point last_command_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosrtRt1>());
    rclcpp::shutdown();
    return 0;
}