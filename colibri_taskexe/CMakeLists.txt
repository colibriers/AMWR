cmake_minimum_required(VERSION 2.8.3)
project(colibri_taskexe)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  roscpp
  tf
  visualization_msgs
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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   move_base_msgs#   visualization_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES colibri_taskexe
#  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs roscpp tf visualization_msgs
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

## Declare a C++ executable
 add_executable(single_goal_move src/single_goal.cpp)
 target_link_libraries(single_goal_move
   ${catkin_LIBRARIES}
 )

 add_executable(two_goal_move src/two_goal.cpp)
 target_link_libraries(two_goal_move
   ${catkin_LIBRARIES}
 )

 add_executable(mult_import_move src/mult_goal_import.cpp)
 target_link_libraries(mult_import_move
   ${catkin_LIBRARIES}
 )

 add_executable(import_nav src/import_nav.cpp)
 target_link_libraries(import_nav
   ${catkin_LIBRARIES}
 )


