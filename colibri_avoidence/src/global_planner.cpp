#include "global_planner.h"

planner::planner()
{

	cur_robot_state[0] = -OFFSET_LASER2ROBOT;
	cur_robot_state[1] = 0;
	cur_robot_state[2] = 0;

	cur_goal_state[0] = 5.2;
	cur_goal_state[1] = -2.3;
	cur_goal_state[2] = 90;

	for(int i = 0; i < GOAL_NUM; i++)
	{
		memset(goal_series[i], 0, POS_DIM);
	}

	srv4make_plan = "/move_base/make_plan";
	serviceClient = nh_planner.serviceClient<nav_msgs::GetPlan>(srv4make_plan, true);

	start_quat.x = 0;
	start_quat.y = 0;
	start_quat.z = 0;
	start_quat.w = 1;

	goal_quat.x = 0;
	goal_quat.y = 0;
	goal_quat.z = 0;
	goal_quat.w = 1;
	
	array_num = 0;
	
	gravaton.x = -OFFSET_LASER2ROBOT;
	gravaton.y = 0.0;
	gravaton.yaw = 0.0;

}

planner::~planner()
{

}

void planner::FillPathRequest(nav_msgs::GetPlan::Request &request, float* start_pos, float* goal_pos)
{
	request.start.header.frame_id ="map";
	request.start.pose.position.x = *start_pos;
	request.start.pose.position.y = *(start_pos + 1);

	start_quat = tf::createQuaternionMsgFromYaw(*(start_pos + 2) * DEG2RAD);		
	request.start.pose.orientation = start_quat;

	request.goal.header.frame_id = "map";
	request.goal.pose.position.x = *goal_pos;
	request.goal.pose.position.y = *(goal_pos + 1);

	goal_quat = tf::createQuaternionMsgFromYaw(*(goal_pos + 2) * DEG2RAD);		
	request.goal.pose.orientation = goal_quat;

	request.tolerance = GOAL_TOLLERANCE;
}

void planner::CallPlanningService(ros::ServiceClient &srv_client, nav_msgs::GetPlan &srv)
{
	path_point tmp_path_point;
	geometry_msgs::PoseStamped tmp_point;

	if (srv_client.call(srv)) 
	{
		if (!srv.response.plan.poses.empty()) 
		{
			ForEach(const geometry_msgs::PoseStamped &p, srv.response.plan.poses)
			{
				tmp_path_point.x = p.pose.position.x;
				tmp_path_point.y = p.pose.position.y;

				tmp_point.pose.orientation = p.pose.orientation;		
				Quaternion2Yaw(tmp_point, tmp_path_point.yaw);

				path_array.push_back(tmp_path_point);

			}
			
		}
		else
		{
			ROS_WARN("Got Empty Plan in CallPlanningService()...");
		}
	}
	else
	{
		ROS_ERROR("Failed to Call Path Plan Service %s - robot moving?", srv_client.getService().c_str());
	}
}

void planner::ObtainPathArray(ros::ServiceClient &srv_client, nav_msgs::GetPlan &srv, float* start_pos, float* goal_pos, bool *obtain_finish)
{
	path_array.clear();
	//path2robot_array.clear();
	array_num = 0;

	FillPathRequest(srv.request, start_pos, goal_pos);

	if (!srv_client) 
	{
		ROS_FATAL("Persistent service connection to %s failed",
		srv_client.getService().c_str());
		array_num = 0;
		*obtain_finish = false;

		return;
	}

	CallPlanningService(srv_client, srv);
	
	array_num = path_array.size();

	*obtain_finish = true;
}

void planner::CalcPath2RobotDeltaDis(vector<path_point> &path_array, float* cur_robot_state)
{
	int num = 0;
	float tmp_delta_x = 0.0;
	float tmp_delta_y = 0.0;
	path_delta tmp_path_delta;

	path2robot_array.clear();
	
	num = path_array.size();
	
	for(unsigned int index = 0; index < num; index++)
	{
		tmp_delta_x = path_array[index].x - *(cur_robot_state);
		tmp_delta_y = path_array[index].y - *(cur_robot_state + 1);

		tmp_path_delta.index = index;
		tmp_path_delta.delta_dis = sqrt(pow(tmp_delta_x, 2) + pow(tmp_delta_y, 2));
		path2robot_array.push_back(tmp_path_delta);
	}

}

int planner::CalcGravatonFromPath(vector<path_point> &path_array, vector<path_delta> &path2robot_array, unsigned int & search_start, path_point &gravation, bool &exist_gravaton)
{
	int num = 0;
	int index = 0;
	
	num = path2robot_array.size();

	for(index = search_start; index < num; index++)
	{
		if(path2robot_array[index].delta_dis > GRAVATON_RADIUS)
		{
			gravaton.x = path_array[index].x;
			gravaton.y = path_array[index].y;
			gravaton.yaw = path_array[index].yaw;
			
			exist_gravaton = true;
			
			break;
		}
		else
		{
			exist_gravaton = false;
		}
	}

	return index;

}

bool planner::ExecMonoPlanAndGravaton(planner&plannerObj,float* start_pos, float* goal_pos, unsigned int &start_index, unsigned int &gravaton_index)
{
	bool obtain_path = false;
	bool exist_init_gravaton = false;
	
	plannerObj.ObtainPathArray(plannerObj.serviceClient, plannerObj.path_srv, start_pos, goal_pos,&obtain_path);

	plannerObj.CalcPath2RobotDeltaDis(plannerObj.path_array, plannerObj.cur_robot_state);

	gravaton_index = plannerObj.CalcGravatonFromPath(plannerObj.path_array,plannerObj.path2robot_array, start_index, plannerObj.gravaton,exist_init_gravaton);

	cout<<"init gravaton.x : "<<plannerObj.gravaton.x<<endl;
	cout<<"init gravaton.y : "<<plannerObj.gravaton.y<<endl;
	cout<<"init array_num : "<<plannerObj.array_num<<endl;
	cout<<"init gravaton_index : "<<gravaton_index<<endl;

	return obtain_path;
}


void planner::Quaternion2Yaw(geometry_msgs::PoseStamped &pose, float &yaw)
{
	float x = 0.0;
	float y = 0.0;
	
	y = 2*(pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y);
	x = 1- 2*(pow(pose.pose.orientation.y, 2) + pow(pose.pose.orientation.z, 2));
	yaw = atan2(y,x) * RAD2DEG;
	
}


