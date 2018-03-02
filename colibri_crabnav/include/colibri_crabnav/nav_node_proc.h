#ifndef _NAV_NODE_PROC_H_
#define _NAV_NODE_PROC_H_

#include <fstream>
#include "yaml-cpp/yaml.h"

#include <math.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <string.h>
#include <map>
#include <utility>

#include <ros/ros.h>

#include "nav_msgs/Path.h"
#include "std_msgs/Bool.h"
#include "colibri_msgs/Coordinator.h"
#include "colibri_msgs/NavState.h"
#include "colibri_msgs/RobotCmd.h"
#include "geometry_msgs/PoseStamped.h"
#include "colibri_msgs/NavNode.h"

#include "global_planner.h"

#include "colibri_local_nav.h"

using namespace std;
extern string taskpath;

#define MAX_SEG_NUM 50

#define MANUAL_PATH
extern string routes_abs_path;
extern string spnodes_abs_path;
extern string branchmap_abs_path;



#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

template <class Type>  
Type stringToNum(const string& str)  
{  
    istringstream iss(str);  
    Type num;  
    iss >> num;  
    return num;      
}  

typedef struct st_point2D_int{
	int x;
	int y;
}point2d_pix;

typedef struct st_point2D_float{
	float x;
	float y;
}point2d_map;

typedef struct st_id_delta{
	int id;
	float delta_yaw;
}id_delta;


typedef struct st_pose{
	float x;
	float y;
	float yaw;
}pose;

typedef struct st_nav_state{
	int target_node;
	int target_heading;
	int cur_seg;
	bool at_target_flag;
	bool achieve_flag;
	int task_succ_flag;
	pose target;
	pose robot;
	int err_code;
}nav_state;

typedef struct st_seg_prop{
	int seg_id;
	int start_id;
	int end_id;
	point2d_pix start;
	point2d_pix end;
}seg_property;

typedef struct st_coordinator
{
	int basic_ctrl;
	int target_node;
	float target_heading;
	int route_seg_num;
	int seg_array[MAX_SEG_NUM];
}coordinator;

typedef struct st_robot_cmd
{
	int target_node;
	int clr_at_target;
	int clr_achieve_target;
	int basic_ctrl;
	int cur_seg;
	int pre_situated_node;
	int task_succ_flag;
	int music_mode;
	int screen_mode;
}handled_cmd;


class NavNodeProc{

	public:

		ros::NodeHandle nh_nav_node_;
		ros::Subscriber sub_robot_cmd_;
		ros::Subscriber sub_node_id_;
		ros::Publisher pub_nav_state_;
		ros::Subscriber sub_coordinator_;

		string map_name_;
		float map_origin_[POS_DIM];
		int map_size_[2];
		float map_resol_;
		int segs_num_;
		vector<seg_property> vec_seg_property_;
		map<int, int> node_seg_map_;
		map<int, int> seg_node_map_;
		map<int, float> node_head_map_;

		vector<float> seg_heading_;
		vector<int> branch_node_;
		map<int, float> nextnode_heading_map_;
		vector<int> total_route_segs_;
		int route_segs_;
		
		colibri_msgs::NavState robot_nav_state_;
		int cur_nav_node_;
		float cur_goal[POS_DIM];
		bool obtain_goal_flag;

		handled_cmd aiv_cmd_;
			

		vector<coordinator> exist_route_;
		int exist_route_num_;

		NavNodeProc();
		~NavNodeProc();

		void LoadExistedRoute(void);
		void InitNodeAndSegMap(float *head_array, int &array_size);
		void InitNodeAndSegMap(int &array_size);
		bool NavNode2NavPose();
		bool PubNavState();
		bool NavPose2NavNode(point2d_map & pose, int & rev_node_id);
		bool NavPixValid(point2d_pix &pix_uv);
		void LoadBranchNode(void);
		void SetupBranchMap(void);
		bool IsBranchNode(int &cur_node_id);
		bool ObtainNextNodeInRoute(int &cur_node_id, int & next_node);

		

	private:

		void RobotCmdCallBack(const colibri_msgs::RobotCmd::ConstPtr& cmd);
		void CoordinatorCallBack(const colibri_msgs::Coordinator::ConstPtr& coord);
		void NavNodeCallBack(const colibri_msgs::NavNode::ConstPtr& node);
		void Quaternion2Yaw(const geometry_msgs::PoseStamped &pose, float &yaw);

};

template <class T1, class T2>  
class FindX
{
	public:
         FindX(const T1 ref){ x_ = ref;}
         T1 GetX() {return x_;}

         bool operator()(T2 &seg)
		 {
	         if( abs(seg.end_id - x_) < 0.0001)

	              return true;
	         else
	              return false;
          }

	private: 
		 T1 x_;

};


#endif
