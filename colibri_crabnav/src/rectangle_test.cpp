#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <algorithm>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include "colibri_msgs/AngPotnEngy.h"
#include "colibri_aiv/Ultrasonic.h"
#include "colibri_msgs/EnvSecurity.h"
#include "colibri_msgs/Ultrasonic.h"
#include "colibri_msgs/NavState.h"
#include "colibri_msgs/Coordinator.h"
#include "colibri_msgs/RobotCmd.h"
#include "colibri_msgs/NavNode.h"

#include "nav_node_proc.h"

nav_state cur_nav_state;
colibri_msgs::Coordinator coordintor;
colibri_msgs::RobotCmd robot_cmd;

#define MAX_SEG_NUM 50



void FillCoordinator(const coordinator & coord);
void NavStateCallback(const colibri_msgs::NavState::ConstPtr & nav_state);

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "existed_route_node");

	ros::NodeHandle nh_test;
	ros::Subscriber sub4NavState;
	ros::Publisher pub4Coordinator;
	ros::Publisher pub4Robot_cmd;

	colibri_msgs::NavNode cur_node;

	NavNodeProc nodeObj;

	nodeObj.InitNodeAndSegMap(nodeObj.segs_num_);
	nodeObj.LoadExistedRoute();

	//sub4NavState = nh_test.subscribe<colibri_msgs::NavState>("/nav_state", 1, NavStateCallback);
	pub4Coordinator = nh_test.advertise<colibri_msgs::Coordinator>("/coordinator", 1);
	//pub4Robot_cmd = nh_test.advertise<colibri_msgs::RobotCmd>("/robot_cmd", 1);

	ros::Rate loop_rate(10);

	vector<coordinator>::iterator it = nodeObj.exist_route_.begin();
	bool send_nav_node = false;
	while(ros::ok())
	{
		// obatain init nav state 
		// cmd to n1
		// if reach n1 ok
		   // go to n5
		   //else

		FillCoordinator(*it);
	

		if(send_nav_node == false)
		{
			cur_node.node_id = it->target_node;
			send_nav_node = true;
		}
		else
		{

		}

		pub4Coordinator.publish(coordintor);

		ros::spinOnce();
		loop_rate.sleep();
		
	}	
	
	return 0;
}

void Init(void)
{
	cur_nav_state.target_node = 0;
	cur_nav_state.target_heading = 0.0;
	cur_nav_state.cur_seg = 44;
	cur_nav_state.robot.x = 0.0;
	cur_nav_state.robot.y = 0.0;
	cur_nav_state.robot.yaw = 0.0;
	
	cur_nav_state.target.x = 10.0;
	cur_nav_state.target.y = 10.0;
	cur_nav_state.target.yaw = 0.0;
	cur_nav_state.err_code = 0;

	cur_nav_state.at_target_flag= false;
	cur_nav_state.achieve_flag = false;
	cur_nav_state.task_succ_flag = 0;

}
void NavStateCallback(const colibri_msgs::NavState::ConstPtr & nav_state)
{
	cur_nav_state.target_node = nav_state->target_node;
	cur_nav_state.target_heading = nav_state->target_heading;
	
	cur_nav_state.cur_seg = nav_state->cur_seg;
	cur_nav_state.robot.x = nav_state->cur_x;
	cur_nav_state.robot.y = nav_state->cur_y;
	cur_nav_state.robot.yaw = nav_state->cur_yaw;
	
	cur_nav_state.target.x = nav_state->target_x;
	cur_nav_state.target.y = nav_state->target_y;
	cur_nav_state.target.yaw = nav_state->target_yaw; 
	
	cur_nav_state.err_code = nav_state->err_code;

	cur_nav_state.at_target_flag= nav_state->at_target_flag;
	cur_nav_state.achieve_flag = nav_state->achieve_flag;

	cur_nav_state.task_succ_flag = nav_state->task_succ_flag;

}


void FillCoordinator(const coordinator & coord)
{
	coordintor.header.stamp = ros::Time::now();
	coordintor.header.frame_id = "robot";
	coordintor.basic_ctrl = coord.basic_ctrl;
	coordintor.target_node = coord.target_node;
	coordintor.target_heading = coord.target_heading;
	coordintor.route_segs_num = coord.route_seg_num;
	for(int i = 0; i < coord.route_seg_num; i++)
	{
		coordintor.segs_vector[i] = coord.seg_array[i];
	}
	for(int j = coord.route_seg_num; j < MAX_SEG_NUM; j++)
	{
		coordintor.segs_vector[j] = 127;	// the remain null seg set to be an invalid value 255
	}	
		
}

void FillRobotCmd(void)
{
	robot_cmd.target_node = 0;
	robot_cmd.clr_at_target = 0;
	robot_cmd.clr_achieve_target = 0;
	robot_cmd.basic_ctrl = 0;		
	robot_cmd.cur_seg = 44;
	robot_cmd.pre_situated_node = 0;
	robot_cmd.task_succ_flag = 0;
	robot_cmd.music_mode = 127;
	robot_cmd.screen_mode = 127;

}

// this is only used for a series continuous node to construct the route without jump
void SetSuccessiveRoute(const int & start_id, const int & end_id, coordinator & coord_out) 
{

}


