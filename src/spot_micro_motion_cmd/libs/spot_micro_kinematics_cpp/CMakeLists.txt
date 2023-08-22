cmake_minimum_required(VERSION 3.5)

# Set the project name
project (spot_micro_kinematics)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)

# Add an library for the example classes
add_library(spot_micro_kinematics 
    src/spot_micro_kinematics.cpp
	src/utils.cpp
	src/spot_micro_leg.cpp
)

target_include_directories(spot_micro_kinematics
	 PUBLIC 
		${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(spot_micro_kinematics
	Eigen3
  tf2_eigen
)