#ifndef ROUTES_PROC_H_
#define ROUTES_PROC_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <numeric>
#include <queue>
#include <sstream>
#include <string.h>
#include <utility>
#include <unistd.h>

#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/GetPlan.h>
#include "nav_msgs/Path.h"
#include "std_msgs/Bool.h"
#include <visualization_msgs/Marker.h>

#include "yaml-cpp/yaml.h"

#include "colibri_msgs/Coordinator.h"
#include "colibri_msgs/NavState.h"
#include "colibri_msgs/RobotCmd.h"
#include "colibri_msgs/TaskState.h"

using namespace std;
extern string routes_path;

extern bool get_coordinator_flag;
static int last_task_node = 255;

static bool new_seg_flag = false;
static bool lock_seg_flag = false;

#define MANUAL_PATH

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i) {
  i = node.as<T>();
}
#endif

template <class T>  
T stringToNum(const string& str) {  
  istringstream iss(str);  
  T num;  
  iss >> num;  
  return num;      
}  

template <typename T>
struct st_2d_point {
	T x;
	T y;
};

typedef st_2d_point<double> point2d_map;
typedef st_2d_point<unsigned int> point2d_pix;

template <typename T>
struct st_pose{
	T x;
	T y;
	T yaw;
};
typedef st_pose<float> pose;

struct st_seg_prop{
  st_seg_prop(): seg_id(1),
										 start_id(0),
										 end_id(255),
										 seg_type(0),
										 seg_dir(0),
										 arc_deg(0),
										 seg_heading(0.0),
										 start({0, 0}), 
										 ending({0, 0}) {

	};

	int seg_id;
	int start_id;
	int end_id;
	int seg_type;
	int seg_dir;
	float arc_deg;
	float seg_heading;
	point2d_pix start;
	point2d_pix ending;
};
typedef st_seg_prop seg_property;


typedef struct st_segment{
	int seg_id;
	int start_id;
	int end_id;
	vector<point2d_pix> points_pix;
	vector<point2d_map> points_map;
}segment;

struct st_nav_state{
	st_nav_state(int tgt_node = 0,
									float tgt_head = 0.0,
									int init_seg = 1,
									int init_node = 0,
									bool at_tgt_flag = false,
									bool achive = false,
									int succ_flag = 0,
									int err = 0) : target_node(tgt_node),
																 target_heading(tgt_head),
																 cur_seg(init_seg),
																 cur_node(init_node),
																 at_target_flag(at_tgt_flag),
																 achieve_flag(achive),
																 task_succ_flag(succ_flag),
																 err_code(err) {
		target.x = 0.0;
		target.y = 0.0;
		target.yaw = 0.0;
		
		robot.x = 0.0;
		robot.y = 0.0;
		robot.yaw = 0.0;																	
	}
	int target_node;
	float target_heading;
	int cur_seg;
	int cur_node;
	bool at_target_flag;
	bool achieve_flag;
	int task_succ_flag;
	pose target;
	pose robot;
	int err_code;
};
typedef st_nav_state nav_state;

struct st_map_basic {

	string name;
	float origin[3];
	unsigned int size[2];
	float resol;
};
typedef st_map_basic map_basic;

typedef struct st_route_list
{
	int target_id;
	float target_heading;
	vector<int> seg_list;
}route_list;


void Int2String(const int & i_val, string &str);
bool VerticalLine(const point2d_pix &start, const point2d_pix &end, vector<point2d_pix> &ver_line);
bool BresenhamBasic(const point2d_pix &start, const point2d_pix &end, vector<point2d_pix> &point_at_line);
bool CalcPixesInLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &point_at_line);

class Routes {
	public:

		Routes(void);
		~Routes(void);

		map_basic map_info_;

		int segs_num_;
		vector<seg_property> vec_seg_property_;
		vector<segment> vec_seg_;
		
		map<unsigned int, unsigned int> node_seg_map_;
		map<unsigned int, unsigned int> seg_prenode_map_;
		map<unsigned int, unsigned int> seg_postnode_map_;
		map<unsigned int, unsigned int> seg_length_map_;
		map<int, float> seg_heading_map_;

		vector<int> knee_nodes_;
		vector<int> updated_knee_nodes_;

		vector<route_list> sub_route_vec_;

		void InitKneeNodes(const int &array_size, int *node_array);
		void LoadRoutes(void);
		void SetupMapping(void);
		void CalcAllPointsInSegs(void); 
		bool AddTargetNode2KneeNodes(const int &target_node);
		bool DecomposeRoute(vector<int> &seg_list, vector<int> &check_nodes, int &sub_route_num);

	private:
		void Pix2Map(vector<point2d_pix> &points_pix, vector<point2d_map> &points_map);
		void Seg2LengthMap(void);


};

class PathProc {

	public:
				
		Routes *ptrRoutes_;

		ros::NodeHandle nh_route_;
		ros::Subscriber sub_coodinator_;
		ros::Subscriber sub_nav_state_;
		ros::Publisher pub_route_;
		ros::Publisher pub_marker_;

		visualization_msgs::Marker  goalmark_list_;
		colibri_msgs::RobotCmd robot_cmd_;
		route_list cur_route_;
		vector<seg_property> vec_seg_property_;
		vector<segment> vec_seg_;
		vector<point2d_map> route_map_;
		vector<point2d_pix> route_pix_;

		nav_state robot_nav_state_;
		int cur_seg_;
		
		int basic_ctrl_;
		nav_msgs::Path plan_path_;
		
		bool task_switch_;

		PathProc();
		~PathProc();
		void InitMarkers(void);
		void CatSeg2Route(route_list &route);
		bool StdNavPath(vector<point2d_map> &nav_path);
		int FillMarkerPose(route_list & route);
		void HandleRecvRoute(void);
		int CalcRobotOnCurSeg(point2d_map & cur_pose, route_list &cur_route, vector<point2d_map> &straight_path);

		bool CalcNearestNode(float & robot_x, float &robot_y, int & nearest_node);

	private:

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
         FindX(const T1 ref) { x_ = ref;}
         T1 GetX() {return x_;}

         bool operator()(T2 &seg) {
	         if( abs(seg.seg_id - x_) < 0.0001)
						 return true;
	         else
	           return false;
         }

	private: 
		 T1 x_;

};

#endif //	ROUTES_PROC_H_
