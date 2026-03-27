cmake_minimum_required(VERSION 3.10)
project(lunabot_bringup)

# Set default C standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Set default C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Enable compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)

# Install package files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS scripts/launch_manager_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()
