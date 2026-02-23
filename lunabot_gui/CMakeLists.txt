cmake_minimum_required(VERSION 3.10)
project(lunabot_gui)

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
find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS 
  src/lunabot_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lunabot_gui)

install(FILES
  src/gui_styles.py
  src/ros_interface.py
  src/ui_widgets.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
