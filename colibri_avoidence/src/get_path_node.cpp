#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <ctime>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>

#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/foreach.hpp>

#define ForEach BOOST_FOREACH

void FillPathRequest(nav_msgs::GetPlan::Request &request)
{
	request.start.header.frame_id ="map";
	request.start.pose.position.x = 0.0;
	request.start.pose.position.y = 0.0;
	request.start.pose.orientation.w = 1.0;
	
	request.goal.header.frame_id = "map";
	request.goal.pose.position.x = 6;
	request.goal.pose.position.y = -1.5;
	request.goal.pose.orientation.w = 1.0;
	
	request.tolerance = 0.5;
}

void CallPlanningService(ros::ServiceClient &srv_client, nav_msgs::GetPlan &srv)
{
	// Perform the actual path planner call
	if (srv_client.call(srv)) 
	{
		if (!srv.response.plan.poses.empty()) 
		{
			ForEach(const geometry_msgs::PoseStamped &p, srv.response.plan.poses)
			{
				ROS_INFO("x = %f, y = %f", p.pose.position.x, p.pose.position.y);
			}
			
		}
		else
		{
			ROS_WARN("Got empty plan");
		}
	}
	else
	{
		ROS_ERROR("Failed to call service %s - is the robot moving?",srv_client.getService().c_str());
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "make_plan_node");
	ros::NodeHandle nh;
	
	// Init service query for make plan
	std::string service_name = "/move_base/make_plan";
	
	while (!ros::service::waitForService(service_name, ros::Duration(3.0)))
	{
		ROS_INFO("Waiting for service move_base/make_plan to become available");
	}

	ros::ServiceClient serviceClient = nh.serviceClient<nav_msgs::GetPlan>(service_name, true);
	
	if (!serviceClient)
	{
		ROS_FATAL("Could not initialize get plan service from %s",
		serviceClient.getService().c_str());
		return -1;
	}
	
	nav_msgs::GetPlan path_srv;
	
	FillPathRequest(path_srv.request);

	if (!serviceClient) 
	{
		ROS_FATAL("Persistent service connection to %s failed",
		serviceClient.getService().c_str());
		return -1;
	}
	
	ROS_INFO("conntect to %s",serviceClient.getService().c_str());
	CallPlanningService(serviceClient, path_srv);

	return 0;
}

