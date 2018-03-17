#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include "cartodom/Cartodom.h"

using namespace std; 

using namespace Eigen;


const float RAD2DEG = 57.2958;
const float DEG2RAD = 0.01745;
const float PI = 3.1415926;
const int DELAY_CNT_MAX = 10;


struct path_point
{
	float x;
	float y;
	float yaw;	// unit in degree
	
};

struct circle_params
{
	float circle_x;
	float circle_y;
	float radius;	// unit in degree
	
};


struct path_velocity
{
	float linear_vel;
	float angular_vel;
	
};


struct path_gen
{
	vector<path_point> path_array;
        vector<path_velocity> path_vel_array;
	
};


struct vehicle_params
{
	float vehicle_width;
	float wheel_radius;	
};

enum path_type{circle , line };

struct motion_params
{
	float a_max;
	float v_max;
	float ts;
};


struct path_params
{
	path_point start_point;
	path_point end_point;
	string rot_direction;
	float rot_radius;
	path_type path_typeT;
	
};

struct curvetimeT
{
	float ta;
	float tv;
	float t0;
	float t1;
	float t2;
	float t3;
	float total;	
};

struct velocity_params
{
	vector<path_velocity> path_vel_array;
	double max_anglular_vel;
	curvetimeT timeT;
};

struct CLF_coefficient
{	
	double k1;
	double k2;
	double k3;
	bool flag;
};



class Trajectory_tracker
{
	public:		
		vehicle_params vehicle_param;
		
		path_params path_param_input;		
		motion_params motion_param_input;
		
		circle_params circle_param;
		path_gen path_gen_output;
		CLF_coefficient CLF_coeff;
             
		path_point curent_pose_point;
		path_velocity CLF_vel;

		float curent_robot_vel[2];
		float curent_robot_pose[3];
				
		geometry_msgs::Quaternion start_quat;
		geometry_msgs::Quaternion goal_quat;
		geometry_msgs::PoseStamped tmp_point;
		//geometry_msgs::Twist curent_robot_vel;

		ros::Subscriber cartodom_sub;
		ros::Subscriber odom_sub;
		ros::Publisher cmd_vel_pub;
		
		
		//Constructor
		Trajectory_tracker();
		//Destructor
		~Trajectory_tracker();
		bool InitSubandPub();
		void CartodomCallback(const cartodom::Cartodom::ConstPtr & carto_odom);
		void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom); 
		
		void PathGeneration(path_params& path_param , motion_params& motion_param ,path_gen& path_get); 
		void ArcPathGeneration(path_params& path_param , motion_params& motion_param ,path_gen& path_get);
		void LinePathGeneration(path_params& path_param , motion_params& motion_param ,path_gen& path_get);
		void CalCircleCenter(path_params& path_param , circle_params& circle_param);
		void TCurveVelocityPlaning(motion_params& motion_param , path_params& path_param , float& distance, velocity_params& velocity_param);
		void LyapunovContorller(CLF_coefficient&  CLF_coeff , path_gen& path_in, path_point& curent_point, int& count , path_velocity& CLF_vel_get);

	private:		
		ros::Time last_time;
		ros::Time current_time;
};
