cmake_minimum_required(VERSION 3.10)
project(lunabot_teleop)

# Compiler settings
add_definitions(-w)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compile options for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sparkcan REQUIRED)

# Define executables
add_executable(controller_teleop src/controller_teleop.cpp)

# Set target dependencies
ament_target_dependencies(controller_teleop rclcpp sensor_msgs geometry_msgs
                          std_msgs sparkcan)

# Install targets
install(TARGETS controller_teleop DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS src/keyboard_teleop.py DESTINATION lib/${PROJECT_NAME})

ament_package()
