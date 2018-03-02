cmake_minimum_required(VERSION 2.8.3)
project(colibri_tmpdbg)
 
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
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
 
 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES colibri_tmpdbg
  CATKIN_DEPENDS nav_msgs roscpp std_msgs tf
  DEPENDS system_lib
)
  
add_executable(extract_virutal_aivpose src/extract_virtual_aivpose.cpp)
add_executable(pub_virutal_aivpose src/pub_virtual_aivpose.cpp)

add_executable(pub_constang src/pub_constang.cpp)
add_executable(pub_constvel src/pub_constvel.cpp)
  
 target_link_libraries(extract_virutal_aivpose
   ${catkin_LIBRARIES}
 )
 target_link_libraries(pub_virutal_aivpose
   ${catkin_LIBRARIES}
 )

 target_link_libraries(pub_constang
   ${catkin_LIBRARIES}
 )
 target_link_libraries(pub_constvel
   ${catkin_LIBRARIES}
 )
 
add_executable(pub_nav_state src/pub_nav_state.cpp)
  
 target_link_libraries(pub_nav_state
   ${catkin_LIBRARIES}
 )





