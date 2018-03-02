#include <string>
#include <vector>
#include <cmath>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <fstream>
#include "yaml-cpp/yaml.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h" 

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/GetPlan.h>

#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "map_search.h" // See header for copyright and usage information

using namespace std;
using namespace cv;

#ifndef _MAP_PROC_H_
#define _MAP_PROC_H_

extern int world_map[];
extern int MAP_WIDTH;
extern int MAP_HEIGHT;

extern AStarSearch<MapSearchNode> astarsearch;

extern string map_pgm_name;
extern string map_yaml_name;

#define DILATION_TYPE MORPH_ELLIPSE
#define DILATION_SIZE 7
#define GOAL_EDGE_MAX 15

#define RAD2DEG 57.296

#define SUBMAP_SEARCH
#define SUBMAP_RESOL 3

typedef struct st_pix_point
{
	int x;
	int y;

}pix_point;

typedef struct st_smpix_point
{
	float x;
	float y;
	
}smpix_point;

typedef struct st_map_point
{
	float x;
	float y;
	float yaw;
	
}map_point;

template<typename T>
vector<smpix_point> Smooth5p3t(vector<T> &input);

class map_proc
{
	public :
		ros::NodeHandle nh_img;
		ros::Publisher pub4path;
		ros::ServiceServer srv4fp;

		Mat map_image;
		Mat dilation_img;  
		Mat structe_element;
		Mat gray_img;
		
		sensor_msgs::ImagePtr msg;

		float cur_goal[3];		
		bool obtain_goal_flag;
		
		pix_point start;
		pix_point terminal;
		pix_point revised_terminal;

		string str_origin;
		float map_origin[3];
		float map_resol;

		int orimap_width;
		int orimap_height;
		int submap_width_comple;
		int submap_height_comple;

		vector<pix_point> nav_nodes;
		vector<map_point> nav_path;
		nav_msgs::Path plan_path;

		map_proc();
		~map_proc();
		
		bool CalcGoalEdgePoint(pix_point & end, pix_point & revised_end);
		bool SearchMapPreProc(void);
		
		bool ImgPix2NavPos(pix_point & pix, map_point & position);
		bool ImgPix2NavPos(smpix_point & pix, map_point & position);
		bool NavPos2ImgPix(map_point & position, pix_point & pix);
		
		bool PixNodes2NavPath(vector<pix_point> & nav_nodes, vector<map_point> &nav_path);
		bool PixNodes2NavPath(vector<smpix_point> & smnav_nodes, vector<map_point> &nav_path);

		void NavPath2PixNodes();
		bool StdNavPath(vector<map_point> &nav_path);

		bool LocalMapUpdate(void); //TODO
		bool LoadGoalFromTask(void); //TODO

		bool CalcSubMap(sensor_msgs::ImagePtr &msg);

		void ParseMapOrigin(void);

		bool PixBoundCheck(pix_point & pix);
		bool ExecPathFindingSrv(nav_msgs::GetPlan::Request & req, nav_msgs::GetPlan::Response & res);
		
		bool SearchNodeInit(map_point &start, map_point & goal, MapSearchNode &nodeStart, MapSearchNode &nodeEnd);
		bool SearchNodeInit(pix_point &start, pix_point & goal, MapSearchNode &nodeStart, MapSearchNode &nodeEnd);
		bool SearchAndObatainNodes(AStarSearch<MapSearchNode> &astarObj);


	private:
		
		void ObtainRvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& rviz_goal);
		void Quaternion2Yaw(const geometry_msgs::PoseStamped &pose, float &yaw);


};


#endif
