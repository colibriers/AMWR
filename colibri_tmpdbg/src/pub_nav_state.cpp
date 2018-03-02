#include <ros/ros.h>
#include "colibri_msgs/NavState.h"

colibri_msgs::NavState navstate;
static int data = 0;

void FillNavState(int &node_id)
{
	//nav_state.header.stamp = ros::Time::now();
	//nav_state.header.frame_id = "/robot";

	navstate.achieve_flag = 1;
	navstate.at_target_flag = 0;
	navstate.cur_x = 12;
	navstate.cur_y = 45;
	navstate.cur_yaw = 30;
	navstate.err_code = 0;
	navstate.target_heading = 90;
	navstate.target_node = node_id;
	navstate.target_x = 11;
	navstate.target_y = 33;
	navstate.target_yaw = 10;

}

void FillNavState()
{
	if(data < 40)
	{
		data += 2;
	}
	else
	{
		data = 0;
	}
		
	navstate.achieve_flag = data;
	navstate.at_target_flag = data;
	navstate.cur_x = data;
	navstate.cur_y = data;
	navstate.cur_yaw = data;
	navstate.err_code = data;
	navstate.target_heading = data;
	navstate.target_node = data;
	navstate.target_x = data;
	navstate.target_y = data;
	navstate.target_yaw = data;

}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "pub_state");
	ros::NodeHandle nh;
	ros::Publisher pub_nav_state;
	pub_nav_state = nh.advertise<colibri_msgs::NavState>("/nav_state", 1);
	
	ros::Rate loop(10);
	int i = 0;
	while(ros::ok())
	{			
	
		FillNavState(i);
	
		pub_nav_state.publish(navstate);
		loop.sleep();
		ros::spinOnce();
		i++;
		if(i > 120)
		{
			i = 0;
		}

	}

	return 0;	
}
