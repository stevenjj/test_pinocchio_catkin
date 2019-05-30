cmake_minimum_required(VERSION 2.8.12)
project(test_pinocchio_catkin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS eigenpy pinocchio urdf
	roscpp
	sensor_msgs
	tf
	rviz)

add_definitions(-Wno-deprecated-declarations)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)
find_package(Eigen3 3.0.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Build
SET (THIS_PACKAGE_PATH "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/cmake/config.h.cmake ${PROJECT_SOURCE_DIR}/include/Configuration.h)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_pinnochio
  CATKIN_DEPENDS eigenpy pinocchio urdf roscpp sensor_msgs tf rviz
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIR})

# Compile the bridge to use rviz for visualizing results
add_subdirectory(src/bridge)


add_executable(overview-simple src/overview-simple.cpp)
add_executable(inverse-kinematics src/inverse-kinematics.cpp)
add_executable(test-urdf src/test-urdf.cpp)
add_executable(test-floating-base src/test-floating-base.cpp)
target_link_libraries(overview-simple ${catkin_LIBRARIES})
target_link_libraries(inverse-kinematics ${catkin_LIBRARIES})
target_link_libraries(test-urdf ${catkin_LIBRARIES})
target_link_libraries(test-floating-base ${catkin_LIBRARIES})

add_executable(visualize_ik_solution src/visualize_ik_solution.cpp)
target_link_libraries(visualize_ik_solution ${catkin_LIBRARIES})