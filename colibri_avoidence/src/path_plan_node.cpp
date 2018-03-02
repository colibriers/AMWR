#include "global_planner.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "planner_node");
	planner plannerObj;

	float newstart[3] = {2, 0, 0};
	float newgoal[3] = {1.2,-1,60};
	
	while (!ros::service::waitForService(plannerObj.srv4make_plan, ros::Duration(0.2)))
	{
		ROS_INFO("Waiting for service move_base/make_plan to become available");
	}

	bool tmp_obtain_flag = false;
	bool *obtain_flag = &tmp_obtain_flag;
	bool exist_gravaton_flag = false;

	plannerObj.ObtainPathArray(plannerObj.serviceClient, plannerObj.path_srv,plannerObj.cur_robot_state, plannerObj.cur_goal_state,obtain_flag);

	plannerObj.CalcPath2RobotDeltaDis(plannerObj.path_array, plannerObj.cur_robot_state);

	unsigned int search_start = 0;

	plannerObj.CalcGravatonFromPath(plannerObj.path_array,plannerObj.path2robot_array, search_start, plannerObj.gravaton,exist_gravaton_flag);

	cout<<"gravaton.x : "<<plannerObj.gravaton.x<<endl;
	cout<<"gravaton.y : "<<plannerObj.gravaton.y<<endl;
	cout<<"gravaton.yaw : "<<plannerObj.gravaton.yaw<<endl;

	sleep(5);

	tmp_obtain_flag = false;
	plannerObj.ObtainPathArray(plannerObj.serviceClient, plannerObj.path_srv, newstart, newgoal,obtain_flag);

	return 0;
}

