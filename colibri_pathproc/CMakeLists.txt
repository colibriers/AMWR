cmake_minimum_required(VERSION 2.8.3)
project(colibri_pathproc)

set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g -std=c++11") 
set (CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
)

find_package(PkgConfig) 
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND) 
	add_definitions(-DHAVE_NEW_YAMLCPP) 
endif(NEW_YAMLCPP_FOUND)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES colibri_pathproc
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(/usr/local/include)
link_directories(/usr/local/lib)

INCLUDE_DIRECTORIES(include/colibri_pathproc)

#add_executable(pathproc_node src/path_proc_test.cpp src/path_proc.cpp)
#target_link_libraries(pathproc_node yaml-cpp ${catkin_LIBRARIES})


add_executable(path_handle_node src/path_handle.cpp src/path_proc.cpp)
target_link_libraries(path_handle_node yaml-cpp ${catkin_LIBRARIES})

#add_executable(path_handle src/path_handle.cpp src/path_proc.cpp)
#target_link_libraries(path_handle yaml-cpp ${catkin_LIBRARIES})

add_executable(routes_handle_node src/routes_handle.cc src/routes_proc.cc 
src/routes_smoothing.cc)
target_link_libraries(routes_handle_node yaml-cpp ${catkin_LIBRARIES})



