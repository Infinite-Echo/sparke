cmake_minimum_required(VERSION 3.5)
project(spot_micro_keyboard_command)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_include_directories(include)

ament_package()