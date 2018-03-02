#include "kalman_interface.h"

KalmanInterface::KalmanInterface(){

	memset(robot_init_state, 0, POS_DIM);
	memset(cur_robot_state, 0, POS_DIM);
	memset(amcl_cur_state, 0, POS_DIM);//store the linear vel and angular vel
	memset(set_init_state, 0, POS_DIM);//store the linear vel and angular vel
	memset(cur_robot_vel, 0, VEL_DIM);//store the linear vel and angular vel

	init_flag = false;
	amcl_flag = false;


	sub_carto_odom = nh_kalman.subscribe<nav_msgs::Odometry>("/odom", 1, &KalmanInterface::CartoOdomCallBack, this);
	sub_amcl_pose = nh_kalman.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &KalmanInterface::AmclPoseCallBack, this);
	sub_init_pose = nh_kalman.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initpose", 1, &KalmanInterface::AmclPoseCallBack, this);

}

void KalmanInterface::CartoOdomCallBack(const nav_msgs::Odometry::ConstPtr& carto_odom){

	float tmp_yaw_rad = 0.0;
	
	cur_robot_state[0] = carto_odom->pose.pose.position.x;
	cur_robot_state[1] = carto_odom->pose.pose.position.y;
	
	cur_carto_quat = carto_odom->pose.pose.orientation;
	tmp_yaw_rad = tf::getYaw(cur_carto_quat);
	
	cur_robot_state[2] = tmp_yaw_rad * RAD2DEG;

	cur_robot_vel[0] = carto_odom->twist.twist.linear.x;
	cur_robot_vel[1] = carto_odom->twist.twist.angular.z;
	
}

void KalmanInterface::AmclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose){

	float tmp_yaw_rad = 0.0;
	
	amcl_cur_state[0] = amcl_pose->pose.pose.position.x;
	amcl_cur_state[1] = amcl_pose->pose.pose.position.y;
	
	amcl_cur_quat = amcl_pose->pose.pose.orientation;
	tmp_yaw_rad = tf::getYaw(amcl_cur_quat);
	
	amcl_cur_state[2] = tmp_yaw_rad * RAD2DEG;

	amcl_flag = true;
	
}

void KalmanInterface::InitPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& init_pose){

	float tmp_yaw_rad = 0.0;
	
	set_init_state[0] = init_pose->pose.pose.position.x;
	set_init_state[1] = init_pose->pose.pose.position.y;
	
	set_init_quat = init_pose->pose.pose.orientation;
	tmp_yaw_rad = tf::getYaw(set_init_quat);
	
	set_init_state[2] = tmp_yaw_rad * RAD2DEG;

	init_flag = true;
	
}



