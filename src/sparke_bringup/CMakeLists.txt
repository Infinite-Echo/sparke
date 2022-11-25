cmake_minimum_required(VERSION 3.8)
project(sparke_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(ROS2_CONTROL_URL "https://github.com/rohitmenon86/ros2_controllers")
# set(ROS2_CONTROL_DOWNLOAD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../..)
# set(ROS2_CONTROL_DEPENDENCY_PATH ${ROS2_CONTROL_DOWNLOAD_PATH}/ros2_controllers)

# if (NOT EXISTS "${ROS2_CONTROL_DEPENDENCY_PATH}")
#   message(STATUS "Downloading ros2_controllers")
#   message(STATUS "${ROS2_CONTROL_DOWNLOAD_PATH}")

#   execute_process(
#     COMMAND git clone ${ROS2_CONTROL_URL} ${ROS2_CONTROL_DEPENDENCY_PATH} -b $ENV{ROS_DISTRO}
#   )

#   execute_process(
#     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../..
#     COMMAND rosdep install --from-paths src -i -r -y
#   )

#   execute_process(
#     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../..
#     COMMAND colcon build --packages-select joint_trajectory_controller
#   )
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
