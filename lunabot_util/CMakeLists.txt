cmake_minimum_required(VERSION 3.10)
project(lunabot_util)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Define build targets
add_executable(topic_remapper src/topic_remapper.cpp)
add_executable(livox_reorient src/livox_reorient.cpp)

ament_target_dependencies(topic_remapper rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(livox_reorient rclcpp geometry_msgs sensor_msgs tf2 tf2_ros livox_ros_driver2)

# Install package files
install(TARGETS topic_remapper livox_reorient DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS src/bandwidth_monitor.py src/image_compressor.py
                 src/power_monitor.py src/fisheye_rotation.py
                 src/fisheye_camera.py src/encoder_reader.py
                 DESTINATION lib/${PROJECT_NAME})

ament_package()
