cmake_minimum_required(VERSION 3.10)
project(lunabot_msgs)

# Set policies to suppress warnings from system packages
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlState.msg"
  "msg/PowerMonitor.msg"
  "action/Localization.action"
  "action/Excavation.action"
  "action/Dumping.action"
  "action/Homing.action"
  DEPENDENCIES std_msgs
)

# Export the package
ament_package()
