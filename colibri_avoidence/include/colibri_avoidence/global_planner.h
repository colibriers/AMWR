#ifndef _GLOBAL_PLANNER_H_
#define _GLOBAL_PLANNER_H_

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <unistd.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <boost/foreach.hpp>

#include "colibri_local_nav.h"

#define ForEach BOOST_FOREACH

#define GOAL_NUM	10
#define GOAL_TOLLERANCE	0.5
#define GRAVATON_RADIUS 1.2		// gravation in path array distance to robot 

#define PLAN_INTERVAL 3.0	

using namespace std;

typedef struct st_path_point
{
	float x;
	float y;
	float yaw;	// unit in degree
	
}path_point;

typedef struct st_path_delta
{
	unsigned int index;
	float delta_dis;	//path point relative yo certain aiv point
	
}path_delta;

class planner
{
	public:
		
		ros::NodeHandle nh_planner;
		
		string srv4make_plan;	
		ros::ServiceClient serviceClient;
		nav_msgs::GetPlan path_srv;
		
		float goal_series[GOAL_NUM][POS_DIM];
		float cur_robot_state[POS_DIM];
		float cur_goal_state[POS_DIM];
		
		geometry_msgs::Quaternion goal_quat;
		geometry_msgs::Quaternion start_quat;
		
		vector<path_point> path_array;
		int array_num;
		vector<path_delta> path2robot_array;

		path_point gravaton;

		planner();
		~planner();
		
		void FillPathRequest(nav_msgs::GetPlan::Request &request, float* start_pos, float* goal_pos);
		void CallPlanningService(ros::ServiceClient &srv_client, nav_msgs::GetPlan &srv);
		void ObtainPathArray(ros::ServiceClient &srv_client, nav_msgs::GetPlan &srv, float* start_pos, float* goal_pos, bool *obtain_finish);

		void CalcPath2RobotDeltaDis(vector<path_point> &path_array, float* cur_robot_state);
		int CalcGravatonFromPath(vector<path_point> &path_array, vector<path_delta> &path2robot_array, unsigned int & search_start, path_point &gravation, bool &exist_gravaton);

		bool ExecMonoPlanAndGravaton(planner &plannerObj, float* start_pos, float* goal_pos, unsigned int &start_index,unsigned int &gravaton_index);
	
	private:
		
		void Quaternion2Yaw(geometry_msgs::PoseStamped &pose, float &yaw);


};


#endif
