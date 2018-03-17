cmake_minimum_required(VERSION 2.8.3)
project(colibri_stdnav)

set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ") 
set (CMAKE_VERBOSE_MAKEFILE ON) 

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig) 
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5) 
if(NEW_YAMLCPP_FOUND) 
add_definitions(-DHAVE_NEW_YAMLCPP) 
endif(NEW_YAMLCPP_FOUND) 

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../../devel/lib/colibri_stdnav/)

###########
## Build ##
###########

include_directories( ${catkin_INCLUDE_DIRS} ) 
INCLUDE_DIRECTORIES(/usr/local/include) 
link_directories(/usr/local/lib) 
INCLUDE_DIRECTORIES(include/colibri_stdnav/)

add_executable(std_safe_node src/safe_node.cpp src/protect.cpp ) 
target_link_libraries(std_safe_node ${catkin_LIBRARIES})

#add_executable(nav_mult_rvizgoal src/nav_to_multipoint.cpp src/task_mgr.cpp src/global_planner.cpp src/colibri_action.cpp src/colibri_local_nav.cpp    
#	src/colibri_ca.cpp src/PID_controller.cpp src/nav_node_proc.cpp)
#target_link_libraries(nav_mult_rvizgoal yaml-cpp
#  ${catkin_LIBRARIES}
#)
 
#add_executable(node_test src/rectangle_test.cpp src/nav_node_proc.cpp) 
#target_link_libraries(node_test yaml-cpp
#  ${catkin_LIBRARIES}
#)

#add_executable(crab_nav_node src/crab_nav.cpp src/task_mgr.cpp src/global_planner.cpp src/colibri_action.cpp src/colibri_local_nav.cpp src/colibri_ca.cpp src/PID_controller.cpp src/nav_node_proc.cpp) 
#target_link_libraries(crab_nav_node yaml-cpp  ${catkin_LIBRARIES})

#add_executable(std_nav src/crab_nav.cpp src/task_mgr.cpp src/global_planner.cpp src/colibri_action.cpp src/colibri_local_nav.cpp src/colibri_ca.cpp src/PID_controller.cpp src/nav_node_proc.cpp) 
#target_link_libraries(std_nav yaml-cpp ${catkin_LIBRARIES})

#add_executable(std_aux_node src/crab_aux.cpp)
#target_link_libraries(std_aux_node yaml-cpp ${catkin_LIBRARIES})

add_executable(pid_test_node src/PID_test.cc src/PID_controller.cc)
target_link_libraries(pid_test_node ${catkin_LIBRARIES})

add_executable(APF_test_node src/APF_test.cc src/APF_avoiding.cc)
target_link_libraries(APF_test_node ${catkin_LIBRARIES})

