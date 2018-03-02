cmake_minimum_required(VERSION 2.8.3)
project(colibri_avoidence)

#set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
#set (CMAKE_VERBOSE_MAKEFILE ON)

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

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(
#  ./include
#  ${catkin_INCLUDE_DIRS}
#)

include_directories( ${catkin_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES(/usr/local/include) 
link_directories(/usr/local/lib)
INCLUDE_DIRECTORIES(include/colibri_avoidence/)

## Declare a C++ library
# add_library(colibri_avoidence
#   src/${PROJECT_NAME}/colibri_avoidence.cpp
# )

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../../devel/lib/colibri_avoidence/)

## Declare a C++ executable
 add_executable(colibri_ca_node src/colibri_ca_node.cpp src/colibri_ca.cpp )
 target_link_libraries(colibri_ca_node
   ${catkin_LIBRARIES}
 )


 add_executable(local_nav_node src/fixedpoint_local_nav.cpp src/colibri_local_nav.cpp src/colibri_ca.cpp )
 target_link_libraries(local_nav_node
   ${catkin_LIBRARIES}
 )


 add_executable(approach_point_node src/approaching_point.cpp 
 src/PID_controller.cpp src/colibri_action.cpp src/colibri_local_nav.cpp src/colibri_ca.cpp )
 target_link_libraries(approach_point_node
   ${catkin_LIBRARIES}
 )

 add_executable(safe_node src/safe_node.cpp src/protect.cpp)
 target_link_libraries(safe_node
   ${catkin_LIBRARIES}
 )

 add_executable(getpath_node src/get_path_node.cpp)
 target_link_libraries(getpath_node
   ${catkin_LIBRARIES}
 )

 add_executable(path_node src/path_plan_node.cpp src/global_planner.cpp)
 target_link_libraries(path_node
   ${catkin_LIBRARIES}
 )

 add_executable(follow_path_node src/follow_path_node.cpp 
    src/global_planner.cpp src/colibri_action.cpp src/colibri_local_nav.cpp 
    src/colibri_ca.cpp src/PID_controller.cpp)

 target_link_libraries(follow_path_node
   ${catkin_LIBRARIES}
 )


 add_executable(task_node src/task_node.cpp src/task_mgr.cpp)
 target_link_libraries(task_node
   ${catkin_LIBRARIES}
 )

 add_executable(nav_rvizgoal_node src/nav_to_point.cpp src/task_mgr.cpp src/global_planner.cpp src/colibri_action.cpp src/colibri_local_nav.cpp 
    src/colibri_ca.cpp src/PID_controller.cpp)
 target_link_libraries(nav_rvizgoal_node
   ${catkin_LIBRARIES}
 )

 add_executable(mult_rvizgoal src/nav_to_multipoint.cpp src/task_mgr.cpp src/global_planner.cpp src/colibri_action.cpp src/colibri_local_nav.cpp 
    src/colibri_ca.cpp src/PID_controller.cpp)
 target_link_libraries(mult_rvizgoal
   ${catkin_LIBRARIES}
 )

 add_executable(pub_auxinfo src/colibri_aux.cpp)
 target_link_libraries(pub_auxinfo
   ${catkin_LIBRARIES}
 )

