#include "task_mgr.h"

task_mgr::task_mgr()
{


	task.abs_pos_flag = true;
	task.ref_frame = "/map";
	task.subtask_num = 0;
	task.tasklist_id = 0;

	task_sta_track.subtask_index = 0;
	task_sta_track.subtask_num = 0;
	task_sta_track.tasklist_id = 0;
	task_sta_track.task_sta = IDLE;
	
	ptr_subtask = NULL;

	obtain_goal_flag = false;
	record_goal_flag = false;

	cur_goal[0] = 0.0;
	cur_goal[1] = 0.0;
	cur_goal[2] = 0.0;

	sub_rviz_goal = nh_task.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &task_mgr::ObtainRvizGoalCallBack, this);

	pub_goal_maker = nh_task.advertise<visualization_msgs::Marker>("goal_markers", 10);

	pub_cancel_goal = nh_task.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);


	tasklist_finish = false;

	subtask_pause = false;
	subtask_discard = false;
	subtask_finish = false;
	
	subtask_record = false;	

}
task_mgr::~task_mgr()
{

}

void task_mgr::ReadTasklist()
{

	task.task_set.clear();

	nh_task.param(tasknode + "/task_identifier", task.tasklist_id, 0);
	nh_task.param(tasknode + "/goal_num", task.subtask_num, 0);
	nh_task.param<string>(tasknode + "/ref_frame", task.ref_frame, "map");
	nh_task.param(tasknode + "/abs_pos_flag", task.abs_pos_flag, true);

	geometry_msgs::Point point;
	subtask tmp_task;
	
	string goal_index[SUBTASK_NUM_MAX];
	char tmp_str[2];
	for(int i = 0; i < SUBTASK_NUM_MAX; i++)
	{
		sprintf(tmp_str, "%d", i);
		goal_index[i] = tasknode + "/goal_" + tmp_str;
	}

	vector<double> goal;
	
	if(task.abs_pos_flag == true)
	{
		for(int index=0; index < task.subtask_num; index++)
		{
			nh_task.getParam(goal_index[index], goal);
			
			tmp_task.tasklist_id = task.tasklist_id;
			tmp_task.subtask_index = index;
			tmp_task.x = goal[0];
			tmp_task.y = goal[1];
			tmp_task.yaw = goal[2];
			tmp_task.waiting = goal[3];

			task.task_set.push_back(tmp_task);

			point.x = goal[0];
			point.y = goal[1];
			point.z = 0.0;
			goalmark_list.points.push_back(point);
		}		

	}
	else
	{
		//to do
	} 
	

}
	
bool task_mgr::RecordRvizSubtask()
{

}

bool task_mgr::RecordRealEnvSubtask()
{

}
bool task_mgr::WriteTasklist()
{

}

void task_mgr::InitGoalMarkers(visualization_msgs::Marker *marker)
{
	marker->ns       = "waypoints";
	marker->id       = 0;
	marker->type     = visualization_msgs::Marker::CUBE_LIST;
	marker->action   = visualization_msgs::Marker::ADD;
	marker->lifetime = ros::Duration();//0 is forever
	marker->scale.x  = 0.1;
	marker->scale.y  = 0.1;
	marker->color.r  = 0.5;
	marker->color.g  = 0.7;
	marker->color.b  = 1.0;
	marker->color.a  = 1.0;

	marker->header.frame_id = "/odom";
	marker->header.stamp = ros::Time::now();

}	

void task_mgr::ObtainRvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& rviz_goal)
{

	float yaw = 0.0;
	
	cur_goal[0] = rviz_goal->pose.position.x;
	cur_goal[1] = rviz_goal->pose.position.y;
	
	Quaternion2Yaw(*rviz_goal, yaw);
	cur_goal[2] = yaw;

	obtain_goal_flag = true;

	pub_cancel_goal.publish(set_goal);	//if get the rviz goal , should remove it for no moving to it

}

void task_mgr::Rel2AbsTaskTransfor(void)
{

}


/*
void task_mgr::ObtainRvizGoalCallBack(const geometry_msgs::PoseStamped& rviz_goal,const int rec_num_max)
{
	float yaw = 0.0;
	static int index = 0;
	
	cur_goal[0] = rviz_goal.pose.position.x;
	cur_goal[1] = rviz_goal.pose.position.y;
	
	Quaternion2Yaw(rviz_goal, yaw);
	cur_goal[2] = yaw;

	subtask tmp_task;
		
	tmp_task.tasklist_id = task.tasklist_id;
	tmp_task.subtask_index = index;
	tmp_task.x = cur_goal[0];
	tmp_task.y = cur_goal[1];
	tmp_task.yaw = cur_goal[2];
	tmp_task.waiting = RVIZ_GOAL_WAITING;


	task.tasklist_id = 1;
	task.subtask_num = SUBTASK_NUM_MAX;
	task.ref_frame = "map";
	task.abs_pos_flag = "true";

	task.task_set.push_back(tmp_task);

	index++;

	obtain_goal_flag = true;

}
*/

void task_mgr::Quaternion2Yaw(const geometry_msgs::PoseStamped &pose, float &yaw)
{
	float x = 0.0;
	float y = 0.0;
	
	y = 2.0 * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y);
	x = 1.0 - 2 * (pow(pose.pose.orientation.y, 2) + pow(pose.pose.orientation.z, 2));
	yaw = atan2(y,x) * RAD2DEG;

}


