cmake_minimum_required(VERSION 2.8.3)
project(colibri_docking)

set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ") 
set (CMAKE_VERBOSE_MAKEFILE ON) 

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rosconsole
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

find_package(PkgConfig) 
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5) 
if(NEW_YAMLCPP_FOUND) 
add_definitions(-DHAVE_NEW_YAMLCPP) 
endif(NEW_YAMLCPP_FOUND) 

include_directories( ${catkin_INCLUDE_DIRS} ) 
INCLUDE_DIRECTORIES(/usr/local/include) 
link_directories(/usr/local/lib) 
INCLUDE_DIRECTORIES(include/colibri_docking/)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES colibri_docking
#  CATKIN_DEPENDS nav_msgs rosconsole roscpp rospy sensor_msgs std_msgs std_srvs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(docking_test_node src/docking_test.cc src/docking.cc) 
target_link_libraries(docking_test_node ${catkin_LIBRARIES})