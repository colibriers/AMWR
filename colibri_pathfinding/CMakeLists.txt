cmake_minimum_required(VERSION 2.8.3)
project(colibri_pathfinding)

set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ") 
set (CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

find_package(PkgConfig) 
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5) 
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP) 
endif(NEW_YAMLCPP_FOUND)

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

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES colibri_pathfinding
#  CATKIN_DEPENDS cv_bridge image_transport nav_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(/usr/local/include)
link_directories(/usr/local/lib)

INCLUDE_DIRECTORIES(./include/colibri_pathfinding)

add_executable(findpath_node src/findpath.cpp)
target_link_libraries(findpath_node ${catkin_LIBRARIES})

#add_executable(path_plan_node src/main.cpp)
#target_link_libraries(path_plan_node ${catkin_LIBRARIES})


#add_executable(rounting_node src/rounting.cpp src/MapSearch.cpp)
#target_link_libraries(rounting_node ${catkin_LIBRARIES})


#add_executable(modular_path_node src/simple_fp.cpp src/MapSearch.cpp src/map_proc.cpp)
#target_link_libraries(modular_path_node ${catkin_LIBRARIES})

add_executable(fp_srv_node src/fp_srv.cpp src/MapSearch.cpp src/map_proc.cpp)
target_link_libraries(fp_srv_node yaml-cpp ${catkin_LIBRARIES})


