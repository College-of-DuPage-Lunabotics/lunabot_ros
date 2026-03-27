cmake_minimum_required(VERSION 3.10)
project(lunabot_msgs)

# Set required CMake policy
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Define build targets
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/ControlState.msg"
  "msg/PowerMonitor.msg"
  "srv/LaunchSystem.srv"
  "srv/StopSystem.srv"
  "action/Localization.action"
  "action/Excavation.action"
  "action/Depositing.action"
  "action/Homing.action"
  DEPENDENCIES
  std_msgs)

ament_package()
