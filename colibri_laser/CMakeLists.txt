cmake_minimum_required(VERSION 2.8.3)
project(colibri_laser)

set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ") 
set (CMAKE_VERBOSE_MAKEFILE ON)

find_package(PkgConfig) 
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5) 
if(NEW_YAMLCPP_FOUND) 
    add_definitions(-DHAVE_NEW_YAMLCPP) 
endif(NEW_YAMLCPP_FOUND) 


INCLUDE_DIRECTORIES(/usr/local/include) 
link_directories(/usr/local/lib)
INCLUDE_DIRECTORIES(include/colibri_laser)


# Build ROS-independent library.
find_package(console_bridge REQUIRED)
include_directories(include ${console_bridge_INCLUDE_DIRS})
add_library(LMS1xx src/LMS1xx.cpp)
target_link_libraries(LMS1xx ${console_bridge_LIBRARIES})

# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(LMS1xx_node src/LMS1xx_node.cpp)
target_link_libraries(LMS1xx_node LMS1xx ${catkin_LIBRARIES})


install(TARGETS LMS1xx LMS1xx_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(LMS1xxinv_node src/LMS1xxinv_node.cpp)
target_link_libraries(LMS1xxinv_node LMS1xx ${catkin_LIBRARIES})


install(TARGETS LMS1xx LMS1xxinv_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(LMS1xx_cartogmap src/LMS1xx_cartogmap.cpp src/LMS1xx.cpp )
target_link_libraries(LMS1xx_cartogmap yaml-cpp ${catkin_LIBRARIES})

