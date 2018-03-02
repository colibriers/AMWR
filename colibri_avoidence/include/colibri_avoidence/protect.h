#ifndef _PROTECT_H_
#define _PROTECT_H_

#include <iostream>
#include <cmath>
#include <string>
#include <vector>

#include <fstream>
#include <iomanip>
#include <ctime>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include "colibri_ca.h"

#include "colibri_aiv/Ultrasonic.h"
#include "colibri_aiv/Bumper.h"
#include "colibri_msgs/EnvSecurity.h"
#include "colibri_msgs/SafeVel.h"
#include "colibri_ca.h"

// 3 layers for running safty : laser / ultrosonic  /bumper


#define SCAN4SAFE_NUM 211	//obtain the  210 degree laser scan for resolution at 1 degree  from  (-15~195) degree
#define LASER_SAFE_MIN	0.06	//static safe limit for laser blind area
#define LASER_SAFE_MAX	2.0

#define ULTRA_NUM	  8		//front 4 ultrasonic, ignore the back 4 ultrasonic
#define ULTRA_SAFE_MIN	0.30
#define ULTRA_SAFE_MAX	2.0

#define UNSAFE_PROB		0.97 // >0.97 must stop
#define LEVEL_1_PROB 	0.95 // >0.95 advise stop
#define LEVEL_2_PROB 	0.85 // >0.85 turn for ca
#define LEVEL_3_PROB 	0.35 // > 0.35 dec vel  ; <0.35 hold on the current state

#define V_EXCPT_VAL   1.25
#define VTH_EXCPT_VAL 0.785	// 45 deg/sec

#define LINEAR_SAFE_MAX 	0.15
#define ANGULAR_SAFE_MAX 	0.2
#define LINEAR_STOP 		0.0
#define ANGULAR_STOP 		0.0

#define LASER_SAFE_DIS1		1.2
#define LASER_SAFE_ANG1		41	//asin(0.8/1.2)
#define LASER_SAFE_DIS2		0.8
#define LASER_SAFE_ANG2		30  //asin(0.4/0.8) 
#define LASER_SAFE_DIS3		0.4 //must not moving but allow rotate
#define LASER_SAFE_DIS4		0.3 //must stop


#define ULTRA_SAFE_DIS1		0.8
#define ULTRA_SAFE_DIS2		0.40

#define LASER_CA_WIDTH  1.0
#define LASER_CA_HEIGHT 0.8
#define LASER_ROT_RADIUS 0.4
#define LASER_STOP_RADIUS 0.3

#define ULTRA_CA_DEC  1.0
#define ULTRA_ROT_RADIUS 0.4
#define ULTRA_STOP_RADIUS 0.35


#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))


using namespace std;

typedef enum
{
	HOLD_STATE,
	DEC_VEL,
	TURN_LEFT,
	TURN_RIGHT,
	STILL_ROT,
	STOP,

}enum_act4safe;

typedef enum
{
	OMNI,	// omni-dir means no dir
	LEFT,
	RIGHT,
	FRONT,
	BACK,
	
}enum_bearing;

class protector
{
	public:
		
		float scan4safty[SCAN4SAFE_NUM];
		float ultra_vec[ULTRA_NUM];

		float min_scan;
		float min_ultra;

		unsigned int min_index_scan;
		unsigned int min_index_ultra;
		int min_scan_angle;
		
		bool bumper_signal;

		float v;
		float vth;

		enum_act4safe advise_action;

		bool collision_flag;
		float colision_prob;

		float laser_unsafe_prob;
		float ultra_unsafe_prob;
		
		ros::NodeHandle nh_safety;
		
		ros::Subscriber scan_sub4safe;
		ros::Subscriber	ultra_sub4safe;
		ros::Subscriber	bumper_sub4safe;
		ros::Subscriber	odom_sub4safe;

		ros::Publisher security_pub4env;
    	colibri_msgs::EnvSecurity env_secure;

		ros::Publisher security_pub4laser;
		ros::Publisher security_pub4ultra;
		colibri_msgs::SafeVel laser_safe_vel;
		colibri_msgs::SafeVel ultra_safe_vel;

		protector();
		
		~protector();

		void CalcMinDis4LaserScan(float* laser_vec);	
		void CalcMinDis4Ultrosonic(float* ultra_vec);
		
		float IntegrateMultiInfo4Safety(enum_act4safe* advise_action);	
		bool Detect4ExceptHighVel(float* v, float* vth);
		bool StopMovingInForce(void);
		
		void Intg4EnvSecure(void);
		bool CalcLaserSafeVelThd(float  &min_scan, int &min_scan_ang, int &steer, float *linear_safe, float* angular_safe);
		bool CalcUltraSafeVelThd(float &min_ultra, unsigned int &min_ultra_index, int &steer, float* linear_safe, float* angular_safe);

		bool CalcSafeLinearVel(float &ctrl_vel, float &linear_thd, float* safe_linear_vel);
		bool CalcSafeAngularVel(float &ctrl_vel, int &steer, float &angular_thd, float* safe_angular_vel);

		bool CalcLaserCA(float	&min_scan, int &min_scan_ang, int &steer, float *linear_safe, float* angular_safe, int &area_state);
		bool CalcUltraCA(float &min_ultra, unsigned int &min_ultra_index, int &steer, float* linear_safe, float* angular_safe, int &area_state);

	private:
		
		void ScanSafeCallBack(const sensor_msgs::LaserScan::ConstPtr& scan4safe);
		void UltraSafeCallBack(const colibri_aiv::Ultrasonic::ConstPtr& ultra4safe);
		void BumperSafeCallBack(const colibri_aiv::Bumper::ConstPtr& bumper4safe);
		void OdomSafeCallBack(const nav_msgs::Odometry::ConstPtr& odom4safe);
		void Polar2Decare(float  &min_scan, int &min_scan_ang,float &x, float &y);
		bool LocateInRecArea(float &rec_x, float &rec_y, float &x, float &y);

};

#endif

