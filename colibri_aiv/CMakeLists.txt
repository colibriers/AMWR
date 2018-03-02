cmake_minimum_required(VERSION 2.8.3)
project(colibri_aiv)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g ")
set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  nav_msgs
  tf
  std_msgs 
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Ultrasonic.msg
   Bumper.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES colibri_aiv
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
)

add_executable(aiv_driver src/driver.cpp src/cartodom_correct.cpp src/colibri_aiv_node.cpp)

target_link_libraries(aiv_driver
   ${catkin_LIBRARIES}
 )
