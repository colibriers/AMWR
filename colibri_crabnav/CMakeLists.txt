cmake_minimum_required(VERSION 2.8.3)
project(colibri_crabnav)

set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ") 
set (CMAKE_VERBOSE_MAKEFILE ON)

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

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig) 
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5) 
if(NEW_YAMLCPP_FOUND) 
add_definitions(-DHAVE_NEW_YAMLCPP) 
endif(NEW_YAMLCPP_FOUND) 

catkin_package(
	INCLUDE_DIRS include
)
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../../devel/lib/colibri_crabnav/)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories( ${catkin_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES(/usr/local/include) 
link_directories(/usr/local/lib)
INCLUDE_DIRECTORIES(include/colibri_crabnav/)

add_library(colibri_action
   src/colibri_action.cpp
 )
add_library(colibri_pid
   src/PID_controller.cpp
 )

 add_executable(crab_safe_node src/safe_node.cpp src/protect.cpp )
 target_link_libraries(crab_safe_node
   ${catkin_LIBRARIES}
 )

 #add_executable(nav_mult_rvizgoal src/nav_to_multipoint.cpp src/task_mgr.cpp src/global_planner.cpp src/colibri_action.cpp src/colibri_local_nav.cpp 
 #   src/colibri_ca.cpp src/PID_controller.cpp src/nav_node_proc.cpp)
 #target_link_libraries(nav_mult_rvizgoal yaml-cpp
 #  ${catkin_LIBRARIES}
 #)

 #add_executable(node_test src/rectangle_test.cpp src/nav_node_proc.cpp)
 #target_link_libraries(node_test yaml-cpp
 #  ${catkin_LIBRARIES}
 #)

# add_executable(crab_nav_node src/crab_nav.cpp src/task_mgr.cpp src/global_planner.cpp src/colibri_action.cpp src/colibri_local_nav.cpp 
#    src/colibri_ca.cpp src/PID_controller.cpp src/nav_node_proc.cpp)
# target_link_libraries(crab_nav_node yaml-cpp
#   ${catkin_LIBRARIES}
# )

  add_executable(crab_nav src/crab_nav.cpp src/task_mgr.cpp src/global_planner.cpp src/colibri_action.cpp src/colibri_local_nav.cpp 
    src/colibri_ca.cpp src/PID_controller.cpp src/nav_node_proc.cpp)
 target_link_libraries(crab_nav yaml-cpp
   ${catkin_LIBRARIES}
 )

 add_executable(crab_aux_node src/crab_aux.cpp)
 target_link_libraries(crab_aux_node yaml-cpp
   ${catkin_LIBRARIES}
 )


