cmake_minimum_required(VERSION 3.10)
project(lunabot_nav)

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
find_package(geometry_msgs REQUIRED)
find_package(lunabot_logger REQUIRED)
find_package(lunabot_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(DEPENDENCIES
    geometry_msgs
    lunabot_logger
    lunabot_msgs
    nav2_msgs
    nav_msgs
    rclcpp
    rclcpp_action
    std_msgs
    std_srvs
    tf2
    tf2_ros)

set(SIM_DEPENDENCIES
    geometry_msgs
    lunabot_logger
    lunabot_msgs
    rclcpp
    rclcpp_action
    std_msgs)

# Define build targets
add_executable(auto_excavation_server src/auto_excavation_server.cpp)
add_executable(auto_depositing_server src/auto_depositing_server.cpp)
add_executable(navigation_client src/navigation_client.cpp)

add_executable(excavation_server_sim src/excavation_server_sim.cpp)
add_executable(depositing_server_sim src/depositing_server_sim.cpp)

ament_target_dependencies(auto_excavation_server ${DEPENDENCIES})
ament_target_dependencies(auto_depositing_server ${DEPENDENCIES})
ament_target_dependencies(navigation_client ${DEPENDENCIES})

ament_target_dependencies(excavation_server_sim ${SIM_DEPENDENCIES})
ament_target_dependencies(depositing_server_sim ${SIM_DEPENDENCIES})

# Install package files
install(
  TARGETS auto_excavation_server
          auto_depositing_server
          navigation_client
          excavation_server_sim
          depositing_server_sim
  DESTINATION lib/${PROJECT_NAME})

ament_package()
