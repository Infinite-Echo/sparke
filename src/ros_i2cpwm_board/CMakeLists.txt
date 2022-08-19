cmake_minimum_required(VERSION 3.5)
project(ros_i2cpwm_board)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_executable(${PROJECT_NAME} src/i2cpwm_controller.cpp)
target_link_libraries(${PROJECT_NAME} i2c)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(i2c_interfaces REQUIRED)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(i2c_interfaces)
ament_export_include_directories(include)

ament_target_dependencies(${PROJECT_NAME}
"rclcpp" "std_msgs" "std_srvs" "geometry_msgs" "i2c_interfaces")

ament_package()