#include "colibri_ca.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "colibri_msgs/SafeVel.h"

#ifndef _COLIBRI_LOCAL_NAV_H_
#define _COLIBRI_LOCAL_NAV_H_

#define QUAT_NUM	4	//for x y z w;
#define POS_DIM		3	// for AIV : x, y, yaw;
#define VEL_DIM 	2	// for AIV: linear vel and anglar vel

#define TRANSLATION_TOLERANCE 	0.1		//+/- 10 cm
#define ROTATION_TOLERANCE 		8.0		//+/-5 degree
#define ROTATION_TOLERANCE4CL	6.0		//closed loop rotate control

#define OFFSET_LASER2ROBOT		0.3442	// in x dir

#define GOAL_NGHBORHD		0.8		//robot diagonal radius lenth = 0.5

#define	INFLATE_FACTOR		1.5

#define APPROACH_V_MAX		0.2
#define APPROACH_VTH_MAX	0.5

class local_nav
{
	public:
		
		ros::NodeHandle nh_nav;
		ros::Subscriber sub_carto_odom;
		ros::Publisher pub_apf_twist;

		ros::Subscriber sub_amcl_pose;
	
		float goal_state[POS_DIM];
		float robot_init_state[POS_DIM];

		float cur_robot_state[POS_DIM];
		geometry_msgs::Quaternion cur_carto_quat;
		float last_robot_state[POS_DIM];

		float amcl_cur_state[POS_DIM];
		geometry_msgs::Quaternion amcl_cur_quat;


		float cur_robot_vel[VEL_DIM];	//store the linear vel and angular vel
		float last_robot_vel[VEL_DIM];

		float linear_filter_aplha;		// filter factor in 1st filter
		float angular_filter_aplha;	

		float apf_ctrl_output[VEL_DIM];	
		geometry_msgs::Twist apf_cmd_vel;

		bool position_OK_flag;
		bool orintation_OK_flag;
		bool approaching_flag;
		
		colibri_msgs::SafeVel laser_safe_velocity;
		ros::Subscriber safe_vel_sub4laser;

		colibri_msgs::SafeVel ultra_safe_velocity;
		ros::Subscriber safe_vel_sub4ultra;
		
		local_nav();
		~local_nav();

		float LinearVelFilter(float* ctrl_linear_vel, float* last_linear_vel);
		float AngularVelFilter(float* ctrl_anglar_vel, float* last_angular_vel);
		void CalcOffsetOfGoalAndRobot(float* cur_robot_state, float* goal_state, float* delta_dis, float* delta_robot2goal_yaw,float* delta_laser2goal_yaw);

		bool ReachGoalPositionOK(float* delta_dis);
		bool ReachGoalOrientationOK(float* delta_yaw);

		bool ReachApprochingAreaOK(float* delta_dis);
    
		bool CalcGoalDirOfLaserView(float* dir_laser2goal, float* dir_laser, float* dir_goal_in_laser, float* self_rot_angle);

		void CalcEuclidDistance(float * pos_start, float * pos_end, float &dis);

		void SatuateCmdVel(float* cmd_linear_vel, float* cmd_angular_vel);
		bool EmergencyStop(float* cmd_vel);

		bool CalcSafeLinearVel(float &ctrl_vel, float &linear_thd, float* safe_linear_vel);
		bool CalcSafeAngularVel(float &ctrl_vel, int &steer, float &angular_thd, float* safe_angular_vel);
		void LimitPubTwist(geometry_msgs::Twist &ctrl_vel);
		

	private:
		
		void CartoOdomCallBack(const nav_msgs::Odometry::ConstPtr& carto_odom);
		void AmclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose);
		void RobotPos2LaserPos(float* robot_pos, float* laser_x, float* laser_y);
		int	SgnOfData(float* input);
		void LaserSafeVelCallBack(const colibri_msgs::SafeVel::ConstPtr& safe_vel);
		void UltraSafeVelCallBack(const colibri_msgs::SafeVel::ConstPtr& safe_vel);


};


#endif

