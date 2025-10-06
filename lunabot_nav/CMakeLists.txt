cmake_minimum_required(VERSION 3.5)
project(lunabot_nav)

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

# Find dependencies
find_package(geometry_msgs REQUIRED)
find_package(lunabot_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sparkcan REQUIRED)

set(DEPENDENCIES
    geometry_msgs
    lunabot_msgs
    nav2_core
    nav2_costmap_2d
    nav2_msgs
    nav2_util
    nav_msgs
    pluginlib
    rclcpp
    rclcpp_action
    sensor_msgs
    tf2_ros
    sparkcan)

# Define executables
add_executable(excavation_server src/excavation_server.cpp)
add_executable(localization_server src/localization_server.cpp)
add_executable(navigation_client src/navigation_client.cpp)

add_library(straight_line_planner SHARED src/straight_line_planner.cpp)

# Set target dependencies
ament_target_dependencies(excavation_server ${DEPENDENCIES})
ament_target_dependencies(localization_server ${DEPENDENCIES})
ament_target_dependencies(navigation_client ${DEPENDENCIES})
ament_target_dependencies(straight_line_planner ${DEPENDENCIES})

# Install targets
install(TARGETS excavation_server localization_server navigation_client
                straight_line_planner DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(nav2_core straight_line_planner.xml)

ament_package()
