#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>

#include "path_proc.h"


using namespace std;
string taskpath;
ofstream  file1; 

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "pathproc_test_node");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(10);
	
#ifndef MANUAL_PATH
	taskpath.assign(argv[1]);
	cout<<"Load YAML Name: "<<taskpath<<endl;
#endif

	PathProc pathProcObj;

	point2d_pix tmp_start;
	point2d_pix tmp_end;
	vector<point2d_pix> tmp_line_points;
	tmp_start.x = 30;
	tmp_start.y = 10;
	tmp_end.x = 40;
	tmp_end.y = 10;	

	int sp_nodes[] = {3, 5, 7};
	float heading[] = {0.0, 0.0, 0.0, 90.0, 0.0, 90.0, 90.0, 0.0};


	int cnt_nodes = sizeof(sp_nodes) / sizeof(int);
	pathProcObj.InitKneeNodes(sp_nodes, cnt_nodes);
	
	pathProcObj.ConfigNodesHeading(heading, pathProcObj.segs_num_);

	int sub_route_num = 0;

	pathProcObj.MakeNodeSegMap(pathProcObj.nodes_heading_);
	pathProcObj.CalcAllPointsInSegs();
	pathProcObj.InitMarkers();

/*	
	route_list route;
	route.target_id = 9;
	route.target_heading = 0.0;
	for(int j= 0; j < 9; j++)
	{
		route.seg_list.push_back(j);
	}

	pathProcObj.AddTargetNode2KneeNodes(route.target_id);
	pathProcObj.DecomposeRoute(route.seg_list, pathProcObj.knee_nodes_, sub_route_num);
		
	pathProcObj.CatSeg2Route(route);	

*/

#ifdef REC_PATH
	file1.open ("route.txt");
#endif

	int i = 0; 
	while(ros::ok())
	{
		/*
		i++;
		if(i <= 10)
		{
			pathProcObj.CatSeg2Route(pathProcObj.sub_route_vec_[0]);
		}
		else if(i <= 20 && i > 10)
		{
			pathProcObj.CatSeg2Route(pathProcObj.sub_route_vec_[1]);		
		}
		else if(i <= 30 && i > 20)
		{
			pathProcObj.CatSeg2Route(pathProcObj.sub_route_vec_[2]);
		}
		else if(i <= 40 && i > 30)
		{
			pathProcObj.CatSeg2Route(route);
			
		}
		else
		{
			i = 0;
		}
		*/

		pathProcObj.FillRobotCmd();
		//pathProcObj.StdNavPath(pathProcObj.route_map_);
		if(pathProcObj.req4path_flag)
		{
			pathProcObj.pub_route_.publish(pathProcObj.plan_path_);
			pathProcObj.pub_marker_.publish(pathProcObj.goalmark_list_);
			pathProcObj.pub_robot_cmd_.publish(pathProcObj.robot_cmd_);
			cout<<"robot_nav_state_.at_target_flag: "<<pathProcObj.robot_nav_state_.at_target_flag<<endl;
			cout<<"robot_nav_state_.achieve_flag: "<<pathProcObj.robot_nav_state_.achieve_flag<<endl;
			cout<<"sub_seg_index: "<<sub_seg_index<<endl;
		}

		ros::spinOnce();	  
		loop_rate.sleep();
	}


	return 0;
}

