cmake_minimum_required(VERSION 3.5)
project(servo_move_keyboard)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(i2c_interfaces REQUIRED)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclpy)
ament_export_dependencies(i2c_interfaces)
ament_export_include_directories(include)

ament_package()