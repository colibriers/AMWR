#!/bin/bash
source /opt/ros/indigo/setup.bash
source /home/aiv-2/colibri_ws/devel/setup.bash

echo "*** --------------------------------- ***" 
echo "*** -  Hello From Colibri AIV Robot - ***" 
echo "*** --------------------------------- ***" 
echo "*** Please waiting for system initial ***"
sleep 30

echo "+++--->>> starting the aiv basic.launch"
roslaunch cartographer_ros basic.launch &
sleep 3
echo "+++--->>> started the aiv basic.launch"

rosrun colibri_battery battery_pub_wood.py &
sleep 1
echo "+++--->>> running battery node for monitor"

rosrun colibri_screen unit_server_node &
sleep 1
echo "+++--->>> running screen node "

roslaunch colibri_nav colibri_amcl.launch &
sleep 1
echo "+++--->>> running crab nav amcl node "

rosrun colibri_crabnav crab_aux_node &
sleep 1
echo "+++--->>> running crab aux node "

rosrun colibri_crabnav crab_safe_node &
sleep 1
echo "+++--->>> running crab safe node "

rosrun colibri_host robot_comm.py &
sleep 1
echo "+++--->>> running crab comm node "

roslaunch colibri_pathproc crab_path_handle.launch &
sleep 1
echo "+++--->>> running crab pathproc node "

roslaunch colibri_crabnav crab_nav.launch
sleep 1
echo "+++--->>> running crab nav node "
