# Package Summary

rosrt_rt1 is a packege of ROS node to manipulate RT.works' robot assist walker RT.1.
This explains how to use it.

**NOTE:RT.works does not support using RT.1 with ROS. This may break the product and it is not covered by warranty.**

# Preparation

## Hardware

Open the maintenance port located on rear bottom of the body, and find connector CN3.
PIN #2 and PIN #3 are for serial communication(pulled up to 5V), and PIN #8 is GND.

Get USB to TTL Serial Converter Cable
(For example : https://akizukidenshi.com/catalog/g/gM-05841 )
and connect those 3 pins to the cable.

**NOTE:communication lines are pulled up to 5v. Choose proper converter cable.**

Pin asignments are:

PIN #2 of CN3(RXD pulled up to 5v) --- PIN #4(ORANGE) of Converter

PIN #3 of CN3(TXD pulled up to 5v) --- PIN #5(YELLOW) of Converter

PIN #8 of CN3(GND) --- PIN #1(BLACK) of Converder

**NOTE:Once communication line is disconnected during ROS' operation, the RT.1 keeps running. There is no automatic brake system. Please be careful.**

## Software

get source codes from Github repository(https://github.com/alexandrokatayama/rosrt_rt1.git )

File	|Explanation
--	|--
src/rosrt_rt1.cpp | source code of ROS node.
msg/Rt1Sensor.msg | definition of message from the node.
script/sample.py  | sample script to send command to the node.

Declare the src/rosrt_rt1.cpp as executable in CMakeLists.txt.
Declare the msg/Rt1Sensor.msg as message in CMakeLists.txt.
Then build them in the workspace.

# Operation

## Power up the RT.1

Push power button on the control box.

## On the PC

### Start roscore
	$ roscore
### Set up the serial port
```
# stty -F /dev/ttyUSB0 raw -echo speed 115200
```
If you need to use another port like /dev/ttyUSB1, edit the source code of the node.
### Start the node
```
$ rosrun ros_start rosrt_rt1
```
### Start monitoring sensor values
```
$ rostopic echo /rosrt_rt1
```
### Start script
```
$ rosrun ros_start sample.py /mobile_base/commands/velocity:=/cmd_vel
```
Then RT.1 moves forward, backward, and turn left and right.

# License

Copyright (c) 2022 RT.Works co., ltd.
This software is released under the MIT License, see LICENSE.txt

# Disclaimer

**RT.Works co., ltd. is not responsible for any damage caused by RT.1 using with ROS or any other external device.
There is a risk of damage to RT.1, or the colliding with surroundings or people, resulting in an unexpected serious accident.
Please use this software with sufficient attention to safety at your own risk.**
