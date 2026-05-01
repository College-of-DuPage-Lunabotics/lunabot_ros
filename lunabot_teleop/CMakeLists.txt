cmake_minimum_required(VERSION 3.10)
project(lunabot_teleop)

# Configure compiler definitions
add_definitions(-w)

# Set default C standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Set default C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Enable compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lunabot_msgs REQUIRED)
find_package(sparkcan REQUIRED)
find_package(lunabot_logger REQUIRED)

# Define build targets
add_executable(controller_teleop src/controller_teleop.cpp)
add_executable(motor_controller src/motor_controller.cpp)

ament_target_dependencies(
  controller_teleop
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  std_msgs
  lunabot_msgs
  sparkcan
  lunabot_logger)

ament_target_dependencies(
  motor_controller
  rclcpp
  std_msgs
  lunabot_msgs
  sparkcan
  lunabot_logger)

# Install package files
install(TARGETS controller_teleop motor_controller DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS src/keyboard_teleop.py DESTINATION lib/${PROJECT_NAME})

ament_package()
