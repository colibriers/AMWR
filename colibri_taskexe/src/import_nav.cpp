#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

#include <sstream>
#include <iostream>

#define	DGR2RAD(dgr)	M_PI*dgr/180.0

using namespace std;

ros::Publisher marker_pub;
float amcl_pos[3] = {0.0, 0.0, 0.0};
int twist_zero_flag = 0;

void shutdown(int sig)
{
	ros::Duration(1).sleep(); // sleep for  a second
	ROS_INFO("import task exectute terminated for external trigging !");
	ros::shutdown();
}

void init_markers(visualization_msgs::Marker *marker)
{
	marker->ns       = "waypoints";
	marker->id       = 0;
	marker->type     = visualization_msgs::Marker::CUBE_LIST;
	marker->action   = visualization_msgs::Marker::ADD;
	marker->lifetime = ros::Duration();//0 is forever
	marker->scale.x  = 0.2;
	marker->scale.y  = 0.2;
	marker->color.r  = 1.0;
	marker->color.g  = 0.0;
	marker->color.b  = 0.0;
	marker->color.a  = 1.0;

	marker->header.frame_id = "map";
	marker->header.stamp = ros::Time::now();

}

void AmclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose)
{
	float tmp_yaw_rad = 0.0;
	
	amcl_pos[0] = amcl_pose->pose.pose.position.x;
	amcl_pos[1] = amcl_pose->pose.pose.position.y;
	 
}

void CmdVelCallBack(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	if(cmd_vel->linear.x < 0.02 && abs(cmd_vel->angular.z) < 0.01)
	{
		twist_zero_flag = 1;
	}
	else
	{
		twist_zero_flag = 0;
	}
		 
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "import_task_exe");
	ros::NodeHandle tasknh("~");

	ros::Publisher pub_simple_goal;
	pub_simple_goal = tasknh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
	ros::Subscriber sub_amcl_pose;
	sub_amcl_pose = tasknh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &AmclPoseCallBack);
	ros::Subscriber sub_cmd_vel;
	sub_cmd_vel = tasknh.subscribe<geometry_msgs::Twist>("/t_cmd_vel", 1, &CmdVelCallBack);
 

	//Subscribe to the move_base action server
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);

	//Define a marker publisher.
	marker_pub = tasknh.advertise<visualization_msgs::Marker>("waypoint_markers", 10);

	//for init_markers function
	visualization_msgs::Marker  goalmark_list;

	signal(SIGINT, shutdown);
	ROS_INFO("import task exec start...");

	// get the task list
	int goal_num;
	string ref_frame;
	bool delta_flag;
	tasknh.param("goal_num", goal_num, 0);

	ROS_INFO("goal_num:%d", goal_num);

	tasknh.param<std::string>("ref_frame", ref_frame, "map");
	tasknh.param("delta_flag", delta_flag, false);

	//ROS_INFO("ref_frame:%s", ref_frame);
	//ROS_INFO("delta_flag:%s", delta_flag);
      
	geometry_msgs::Point point;
	
	geometry_msgs::Quaternion quaternions[10]; //to have max=10 stops
	geometry_msgs::Pose goal_list[10];
	double goal_info[10][4];
	string goal_index[10] = {"goal_0","goal_1","goal_2","goal_3","goal_4","goal_5","goal_6","goal_7","goal_8","goal_9"};

	std::vector<double> goal;
	float delta = 0;
	float delta_x = 0;
	float delta_y = 0;
	ros::Rate loop_rate(1);
	
	if(delta_flag == true)
	{
		//ROS_INFO("point.x:%d", point.x);
	}
	else
	{
		for(int index=0; index < goal_num; index++)
		{
			tasknh.getParam(goal_index[index], goal);
			for (int j = 0; j < goal.size(); ++j) 
			{
				goal_info[index][j] = goal[j];
			}
			point.x = goal_info[index][0];
			point.y = goal_info[index][1];
			point.z = 0.0;
			quaternions[index] = tf::createQuaternionMsgFromRollPitchYaw(0, 0, DGR2RAD(goal_info[index][2]));
			goal_list[index].position = point;
			goal_list[index].orientation = quaternions[index];			
		}
	} 
	
	for (int k=0; k<goal_num; k++)
	{
		ROS_INFO("set goal_index%d",k);
		ROS_INFO("set goal_lispos.x:%f", goal_list[k].position.x );
		ROS_INFO("set goal_lispos.y:%f", goal_list[k].position.y ); 
	}


	//Initialize the visualization markers for RViz
	init_markers(&goalmark_list);

	//Set a visualization marker at each waypoint
	for(int i = 0; i < goal_num; i++)
	{
		goalmark_list.points.push_back(goal_list[i].position);
	}

	ROS_INFO("Waiting for move_base action server...");
	//Wait 60 seconds for the action server to become available
	if(!ac.waitForServer(ros::Duration(60)))
	{
		ROS_INFO("Can't connected to move base server");
		return 1;
	}
	ROS_INFO("Connected to move base server");
	ROS_INFO("Starting imported multi-goals navigation");

	int goal_cnt = 0;
	int timer_cnt = 0;
	int wait_cnt = 0;
	bool send_flag = false;

	while( (goal_cnt < goal_num) && (ros::ok()) )
	{
		 //Update the marker display
		 marker_pub.publish(goalmark_list);

		 //Intialize the waypoint goal
		 move_base_msgs::MoveBaseGoal goal;
		 geometry_msgs::PoseStamped simple_goal;

		 //Use the map frame to define goal poses
		 simple_goal.header.frame_id = ref_frame;

		 //Set the time stamp to "now"
		 simple_goal.header.stamp = ros::Time::now();

		 //Set the goal pose to the i-th waypoint
		 simple_goal.pose = goal_list[goal_cnt];

		 if(send_flag == false)
		 {
			 pub_simple_goal.publish(simple_goal);
			 send_flag = true;
		 }


		 delta_x = simple_goal.pose.position.x - amcl_pos[0];
		 delta_y = simple_goal.pose.position.y - amcl_pos[1];
 
		 delta = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
 
		if(delta < 0.5 && (twist_zero_flag == 1))
		{
			ROS_INFO("Goal succeeded!");
			wait_cnt++;
			if(wait_cnt > 5)
			{
				goal_cnt += 1;
				send_flag = false;
				wait_cnt = 0;
			}
				
		}
		else
		{
			timer_cnt++;
			ROS_INFO("AIV is running for moveing");
			if(timer_cnt > 100)  //00 sec
			{
				ROS_INFO("The moving failed for some reason");
				ROS_INFO("robot is stopped in goal [%d]", goal_cnt+1);
			}	
		}

		ros::spinOnce();
		loop_rate.sleep();
			
	}
	
	ROS_INFO("mult_goal_moving end...");
	return 0;
}
