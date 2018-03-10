#ifndef _PATH_PROC_H_
#define _PATH_PROC_H_

#include <fstream>
#include "yaml-cpp/yaml.h"

#include <math.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <string.h>
#include <map>
#include <utility>
#include <sstream>
#include <numeric>
#include <queue>
#include <unistd.h>

#include <ros/ros.h>

#include "nav_msgs/Path.h"
#include "std_msgs/Bool.h"
#include "colibri_msgs/Coordinator.h"
#include "colibri_msgs/NavState.h"
#include "colibri_msgs/RobotCmd.h"
#include "colibri_msgs/TaskState.h"
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Pose.h"

#include <nav_msgs/GetPlan.h>

using namespace std;
extern string routes_path;
extern string sp_nodes_path;
extern ofstream  file1; 

extern bool get_coordinator_flag;
static int last_task_node = 127;

static int sub_seg_index = 0;
static bool new_seg_flag = false;
static bool lock_seg_flag = false;


#define MANUAL_PATH
//#define REC_PATH

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

typedef struct st_pose{
	float x;
	float y;
	float yaw;
}pose;


typedef struct st_segment{
	int seg_id;
	int start_id;
	int end_id;
	vector<point2d_pix> points_pix;
	vector<point2d_map> points_map;
}segment;

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

typedef struct st_route_list
{
	int target_id;
	float target_heading;
	vector<int> seg_list;
}route_list;

void int2str(int & i_val, string &str);
bool VerticalLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &ver_line);
bool BresenhamBasic(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &point_at_line);
bool CalcPixesInLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &point_at_line);


class PathProc{

	public:

		ros::NodeHandle nh_route_;
		ros::Subscriber sub_coodinator_;
		ros::Subscriber sub_nav_state_;
		ros::Publisher pub_route_;
		ros::Publisher pub_marker_;
		ros::Publisher pub_robot_cmd_;
		ros::Publisher pub_task_state_;
		
		ros::ServiceServer srv4getpath_;

		visualization_msgs::Marker  goalmark_list_;
		colibri_msgs::RobotCmd robot_cmd_;
		int  micro_seg_num_;
		
		string map_name_;
		float map_origin_[3];
		int map_size_[2];
		float map_resol_;
		int segs_num_;
		vector<seg_property> vec_seg_property_;
		vector<segment> vec_seg_;
		vector<point2d_map> route_map_;
		vector<point2d_pix> route_pix_;
		route_list cur_route_;
		vector<route_list> sub_route_vec_;
		map<int, int> node_seg_map_;
		map<int, int> seg_prenode_map_;
		map<int, int> seg_node_map_;
		map<int, float> node_heading_map_;
		map<int, int> seg_length_map_;

		vector<int> knee_nodes_;
		vector<int> updated_knee_nodes_;
		vector<float> nodes_heading_;
		vector<float> segs_heading_;

		nav_state robot_nav_state_;
		colibri_msgs::TaskState task_state_;
		int cur_seg_;
		
		int basic_ctrl_;
		nav_msgs::Path plan_path_;
		int parsed_node_;
		bool req4path_flag;
		
		bool task_switch_;

		int sub_seg_index_cache_;


		PathProc();
		~PathProc();
		void InitMarkers(void);
		void ConfigNodesHeading(float *head_array, int &array_size);
		void InitKneeNodes(int *node_array, int &array_size);
		void InitKneeNodes(void);
		bool AddTargetNode2KneeNodes(int &target_node);
		void CalcAllPointsInSegs(void);
		void CatSeg2Route(route_list &route);
		bool DecomposeRoute(vector<int> &seg_list, vector<int> &check_nodes, int &sub_route_num);
		void MakeNodeSegMap(vector<float> &vec_heading);
		void MakeNodeSegMap(void);
		bool StdNavPath(vector<point2d_map> &nav_path);
		bool ExecGetPathSrv(nav_msgs::GetPlan::Request & req, nav_msgs::GetPlan::Response & res);
		int FillMarkerPose(route_list & route);
		void FillRobotCmd(void);
		void HandleRecvRoute(void);
		int CalcRobotOnCurSeg(point2d_map & cur_pose, route_list &cur_route, vector<point2d_map> &straight_path);
		void Seg2LengthMap(void);
		//void FillTaskState(void);	
		void ClearFlags4NextTask(void);
		bool CalcNearestNode(float & robot_x, float &robot_y, int & nearest_node);

	private:
		void Pix2Map(vector<point2d_pix> &points_pix, vector<point2d_map> &points_map);

		void CoordinatorCallBack(const colibri_msgs::Coordinator::ConstPtr& coordinator);
		void NavStateCallBack(const colibri_msgs::NavState::ConstPtr& nav_state);
		bool NavPixValid(point2d_pix &pix_uv);
		bool MapPose2NavNode(point2d_map & pose, int & rev_node_id);
		void CalcLengthStairs(vector<int> & path_seg_id, vector<int> &len_stairs);			

};

template <class T1, class T2>  
class FindX
{
	public:
         FindX(const T1 ref){ x_ = ref;}
         T1 GetX() {return x_;}

         bool operator()(T2 &seg)
		 {
	         if( abs(seg.seg_id - x_) < 0.0001)

	              return true;
	         else
	              return false;
          }

	private: 
		 T1 x_;

};


#endif
