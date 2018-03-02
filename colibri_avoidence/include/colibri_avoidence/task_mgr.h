
#ifndef _TASK_MGR_H_
#define _TASK_MGR_H_

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <cstdlib>

#include <fstream>
#include <iomanip>
#include <ctime>

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>
#include "actionlib_msgs/GoalID.h"

#include "colibri_local_nav.h"
#include "colibri_ca.h"

#define SUBTASK_NUM_MAX 10
#define RVIZ_GOAL_WAITING 5.0

using namespace std;

typedef struct st_subtask 
{
	int tasklist_id;	//subtask belongs to which tasklist
	int subtask_index;	//below goal point in a special task_list index
	float x;
	float y;
	float yaw;
	float waiting;
	
}subtask;			// a subtask is a goal for operation


typedef struct st_tasklist 
{
	int tasklist_id;	//tasklist  identifier
	int subtask_num;
	string ref_frame;
	bool abs_pos_flag;	// the  subtask is the absolate coordinate info or not
	vector<subtask> task_set;
	
}tasklist;

typedef enum
{
	IDLE,
	EXECUTING,	
	WAITING,
	PAUSE,
	FINISHED,
	
}subtask_exec_sta;

typedef struct st_task_track 
{
	int tasklist_id;	//tasklist  identifier
	int subtask_num;
	int subtask_index;
	subtask_exec_sta task_sta;

}task_track;


typedef struct st_rec_point
{
	float x;
	float y;
	float yaw;

}record_point;

const string tasknode = "/import_task";

class task_mgr
{
	public:

		tasklist task;
		task_track task_sta_track;

		float *ptr_subtask;
		vector<record_point> rec_pos;

		float cur_goal[POS_DIM];

		bool obtain_goal_flag;
		bool record_goal_flag;
		
		ros::NodeHandle nh_task;	
		ros::Subscriber sub_rviz_goal;

		ros::Publisher pub_goal_maker;
		visualization_msgs::Marker	goalmark_list;

		ros::Publisher pub_cancel_goal;
		actionlib_msgs::GoalID set_goal;

		bool tasklist_finish;	// if the last subtask is complete

		bool subtask_pause;
		bool subtask_discard;
		bool subtask_finish;	//if a button is pressed  to true the task_finish ,means the rest subtask is also sw finished

		bool subtask_record;	//to make record the pose manually	
		
		task_mgr();
		~task_mgr();

		void ReadTasklist();
		
		bool RecordRvizSubtask();
		bool RecordRealEnvSubtask();

		bool WriteTasklist();
		
		void InitGoalMarkers(visualization_msgs::Marker *marker);

		void Rel2AbsTaskTransfor(void);

	private:
		
		void ObtainRvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& rviz_goal);

		//void ObtainRvizGoalCallBack(const geometry_msgs::PoseStamped& goal, const int rec_num_max);
		void Quaternion2Yaw(const geometry_msgs::PoseStamped &pose, float &yaw);

};


#endif

