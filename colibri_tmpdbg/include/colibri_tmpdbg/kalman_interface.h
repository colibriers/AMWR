#ifndef KALMAN_INTERFACE_H_
#define KALMAN_INTERFACE_H_

#include <iostream>
#include "ros/ros.h"
#include "cartodom/Cartodom.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>

#define RAD2DEG 57.295
#define QUAT_NUM	4	//for x y z w;
#define POS_DIM		3	// for AIV : x, y, yaw;
#define VEL_DIM 	2	// for AIV: linear vel and anglar vel

using namespace std;

class KalmanInterface{
	
public:
		
		ros::NodeHandle nh_kalman;
		ros::Subscriber sub_carto_odom;
		ros::Subscriber sub_amcl_pose;
		ros::Subscriber sub_init_pose;
	
		float robot_init_state[POS_DIM];

		float cur_robot_state[POS_DIM];
		geometry_msgs::Quaternion cur_carto_quat;
		float cur_robot_vel[VEL_DIM];

		float amcl_cur_state[POS_DIM];
		geometry_msgs::Quaternion amcl_cur_quat;

		float set_init_state[POS_DIM];
		geometry_msgs::Quaternion set_init_quat;

		bool init_flag;
		bool amcl_flag;

		KalmanInterface();
		~KalmanInterface();

	private:
		
		void CartoOdomCallBack(const nav_msgs::Odometry::ConstPtr& carto_odom);
		void AmclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose);
		void InitPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& init_pose);
	
		
};

#endif
