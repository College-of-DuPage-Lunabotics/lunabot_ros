cmake_minimum_required(VERSION 3.10)
project(lunabot_logger)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

# Install C++ header
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

# Install Python package
ament_python_install_package(lunabot_logger)

ament_package()
