#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
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
int main(int argc, char** argv)
{
	ros::init(argc, argv, "import_task_exe");
	ros::NodeHandle tasknh("~");
	
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
		ROS_INFO("debug goal_index%d",k);
		ROS_INFO("debug goal_lispos.x:%f", goal_list[k].position.x );
		ROS_INFO("debug goal_lispos.y:%f", goal_list[k].position.y );
		ROS_INFO("debug goal_lisori.w:%f", goal_list[k].orientation.w );
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

	int count = 0;

	while( (count < goal_num) && (ros::ok()) )
	{
		 //Update the marker display
		 marker_pub.publish(goalmark_list);

		 //Intialize the waypoint goal
		 move_base_msgs::MoveBaseGoal goal;

		 //Use the map frame to define goal poses
		 goal.target_pose.header.frame_id = ref_frame;

		 //Set the time stamp to "now"
		 goal.target_pose.header.stamp = ros::Time::now();

		 //Set the goal pose to the i-th waypoint
		 goal.target_pose.pose = goal_list[count];

		 //Start the robot moving toward the goal
		 //Send the goal pose to the MoveBaseAction server
		 ac.sendGoal(goal);

		//Allow 1 minute to get there
		bool finished_within_time = ac.waitForResult(ros::Duration(60));

		//If we dont get there in time, abort the goal
		if(!finished_within_time)
		{
			ac.cancelGoal();
			ROS_INFO("Timed out achieving goal");
		}
		else
		{
			//We made it!
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Goal succeeded!");
			}
			else
			{
				ROS_INFO("The base failed for some reason");
			}
		}
		count += 1;
	}
	
	ROS_INFO("mult_goal_moving end...");
	return 0;
}
