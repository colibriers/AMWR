#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "routes_proc.h"

using namespace std;
string routes_path;

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "Routes_Handle_Node");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(5);
	
#ifdef MANUAL_PATH

#else
	routes_path.assign(argv[1]);
	cout<<"Load Routes YAML Name: "<<routes_path<<endl;
#endif

	Routes ObjRoutes;
	PathProc pathProcObj;
	pathProcObj.ptrRoutes_ = &ObjRoutes;

	point2d_map cur_robot = {0.0, 0.0};
	
	pathProcObj.ptrRoutes_->SetupMapping();
	pathProcObj.InitMarkers();
	
	while(ros::ok())
	{

		cur_robot.x = pathProcObj.robot_nav_state_.robot.x;
		cur_robot.y = pathProcObj.robot_nav_state_.robot.y;
		
		if(get_coordinator_flag == true)
		{
			get_coordinator_flag = false;
			pathProcObj.HandleRecvRoute();
			pathProcObj.pub_route_.publish(pathProcObj.plan_path_);
			pathProcObj.CalcRobotOnCurSeg(cur_robot, pathProcObj.cur_route_, pathProcObj.route_map_);
			pathProcObj.FillMarkerPose(pathProcObj.cur_route_);
		}
		
		pathProcObj.pub_marker_.publish(pathProcObj.goalmark_list_);	
		//cout<<"robot cur_seg: "<<pathProcObj.cur_seg_<<endl;
		
		ros::spinOnce();	  
		loop_rate.sleep();
	}



	return 0;
}

