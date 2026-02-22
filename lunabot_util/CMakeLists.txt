cmake_minimum_required(VERSION 3.10)
project(lunabot_util)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Define executables
add_executable(topic_remapper src/topic_remapper.cpp)

# Set target dependencies
ament_target_dependencies(topic_remapper rclcpp geometry_msgs nav_msgs)

# Install targets
install(TARGETS
  topic_remapper
  DESTINATION lib/${PROJECT_NAME})

ament_package()
