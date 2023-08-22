cmake_minimum_required(VERSION 3.5)
project(spot_micro_motion_cmd)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(i2c_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_library(spot_micro_kinematics 
  libs/spot_micro_kinematics_cpp/src/spot_micro_kinematics.cpp
        libs/spot_micro_kinematics_cpp/src/utils.cpp
        libs/spot_micro_kinematics_cpp/src/spot_micro_leg.cpp
)

include_directories(
  include/spot_micro_motion_cmd
  src/smfsm
  src/rate_limited_first_order_filter
  libs/spot_micro_kinematics_cpp/include
)

add_library(smfsm 
            src/smfsm/spot_micro_state.cpp 
            src/smfsm/spot_micro_idle.cpp
            src/smfsm/spot_micro_stand.cpp
                        src/smfsm/spot_micro_transition_stand.cpp
                        src/smfsm/spot_micro_transition_idle.cpp
                        src/smfsm/spot_micro_walk.cpp
)

target_link_libraries(smfsm
                spot_micro_kinematics
)

ament_target_dependencies(smfsm
"rclcpp" "std_msgs" "geometry_msgs" "i2c_interfaces" "tf2" "tf2_ros" "tf2_geometry_msgs" "tf2_eigen")

add_executable(${PROJECT_NAME}_node src/spot_micro_motion_cmd.cpp src/spot_micro_motion_cmd_node.cpp src/utils.cpp)

target_link_libraries(${PROJECT_NAME}_node
   smfsm
   spot_micro_kinematics
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(i2c_interfaces)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(tf2_eigen)
ament_export_include_directories(include)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  include/spot_micro_motion_cmd
  src/smfsm
  src/rate_limited_first_order_filter
)

ament_target_dependencies(${PROJECT_NAME}_node
"rclcpp" "std_msgs" "geometry_msgs" "i2c_interfaces" "tf2" "tf2_ros" "tf2_geometry_msgs" "tf2_eigen")

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()