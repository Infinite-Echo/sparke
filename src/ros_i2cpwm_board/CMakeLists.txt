# cmake_minimum_required(VERSION 3.5)
# project(ros_i2cpwm_board)

# # Add support for C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# add_executable(pwmboard src/i2cpwm_controller.cpp)
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME} "msg/Servo.msg"
#   "msg/ServoArray.msg" "msg/ServoConfig.msg" "msg/ServoConfigArray.msg"
#   "msg/Position.msg" "msg/PositionArray.msg"
#   "srv/IntValue.srv" "srv/ServosConfig.srv" "srv/DriveMode.srv"
#   "srv/StopServos.srv"
#   DEPENDENCIES builtin_interfaces std_msgs)

# install(TARGETS
#   pwmboard
#   DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING
#   PATTERN "*.h")
# install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# ament_export_dependencies(ament_cmake)
# ament_export_dependencies(rclcpp)
# ament_export_dependencies(rosidl_default_runtime)
# ament_export_dependencies(std_msgs)
# ament_export_dependencies(geometry_msgs)
# ament_export_include_directories(${INCLUDE_DIRS})

# ament_package()


# ament_target_dependencies(ros_i2cpwm_board
# rclcpp example_interfaces)


# ament_package()


###############################################################################

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