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

# Find required packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(lunabot_msgs REQUIRED)

# Define executables
add_executable(excavation_server src/excavation_server.cpp)
add_executable(localization_server src/localization_server.cpp)
add_executable(navigation_client src/navigation_client.cpp)

# Set target dependencies
ament_target_dependencies(excavation_server rclcpp rclcpp_action geometry_msgs nav_msgs tf2_ros lunabot_msgs)
ament_target_dependencies(localization_server rclcpp rclcpp_action sensor_msgs geometry_msgs tf2_ros cv_bridge lunabot_msgs OpenCV)
ament_target_dependencies(navigation_client rclcpp rclcpp_action nav2_msgs lunabot_msgs)

# Link OpenCV libraries to localization_server
target_link_libraries(localization_server ${OpenCV_LIBS})

# Install targets
install(TARGETS
  excavation_server
  localization_server
  navigation_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()