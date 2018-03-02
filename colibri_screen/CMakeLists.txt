cmake_minimum_required(VERSION 2.8.3)
project(colibri_screen)

set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
set (CMAKE_VERBOSE_MAKEFILE ON)
 
find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
) 

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Screen.msg
   Battery.msg
   Ipc.msg
 )

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
 generate_messages(
   DEPENDENCIES
   std_msgs#   std_msgs
 )
 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aiv_monitor
  CATKIN_DEPENDS   message_runtime
#  DEPENDS system_lib
)
 
include_directories(
  ./modbus_src
  ${catkin_INCLUDE_DIRS}
)
 
#add_dependencies(aiv_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set(LIBMODBUS_SRC ./modbus_src/modbus.c ./modbus_src/modbus-data.c ./modbus_src/modbus-rtu.c ./modbus_src/modbus-tcp.c)
add_library(modbus STATIC ${LIBMODBUS_SRC})


link_directories( ./lib)

 add_executable(unit_server_node src/unit-test-server.cpp src/unit-test.h) 
target_link_libraries(unit_server_node
  libmodbus.a
  ${catkin_LIBRARIES}
)

