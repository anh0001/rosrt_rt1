cmake_minimum_required(VERSION 3.8)
project(rosrt_rt1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Rt1Sensor.msg"
  DEPENDENCIES geometry_msgs
)

# Declare a C++ executable
add_executable(rosrt_rt1_node src/ros2rt_rt1.cpp)
target_include_directories(rosrt_rt1_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rosrt_rt1_node PUBLIC c_std_99 cxx_std_17)  # Require C++17
ament_target_dependencies(rosrt_rt1_node
  rclcpp
  geometry_msgs
)

# Make sure the executable can find the generated message headers
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(rosrt_rt1_node "${cpp_typesupport_target}")

install(TARGETS rosrt_rt1_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()