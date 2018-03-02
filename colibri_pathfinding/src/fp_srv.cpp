
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <ctime>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "stlastar.h" 
#include "map_search.h"
#include "map_proc.h"

#define DEBUG_LISTS 1
#define DEBUG_LIST_LENGTHS_ONLY 0

// Global data
const int MAP_WIDTH_MAX = 1000;
const int MAP_HEIGHT_MAX = 1000; // mean 50m*50m at resolution 0.05m/grid
int world_map[ MAP_WIDTH_MAX * MAP_HEIGHT_MAX ] = {};
int MAP_WIDTH = 1;
int MAP_HEIGHT = 1;

string map_pgm_name;
string map_yaml_name;

AStarSearch<MapSearchNode> astarsearch;

int main( int argc, char *argv[] )
{

	ros::init(argc, argv, "fp_node");
	string name(argv[1]);
	map_pgm_name.assign(name + ".pgm");
	map_yaml_name.assign(name + ".yaml");
	map_proc mapObj;

	mapObj.SearchMapPreProc();

	ros::Rate loop_rate(10); 
	
	ROS_INFO("Ready to provide path_planning srv.");
	cout<<"map_origin:"<<mapObj.map_origin[0]<<" "<<mapObj.map_origin[1]<<endl;
	cout<<"map_resol:"<<mapObj.map_resol<<endl;


	while (mapObj.nh_img.ok()) 
	{
	  mapObj.StdNavPath(mapObj.nav_path);
	  mapObj.pub4path.publish(mapObj.plan_path);

	  ros::spinOnce();
		
	  loop_rate.sleep();
	}
	
	return 0;
}



