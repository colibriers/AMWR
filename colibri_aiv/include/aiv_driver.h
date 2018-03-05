//include the system lib
#include <iostream>
#include <pthread.h>
#include <string>  
#include <vector>

//include the third party  lib
#include <boost/asio.hpp>
#include <boost/bind.hpp>

//include the ros  lib
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//include the proj  lib
#include "cartodom/Cartodom.h"
#include "cartodom_correct.h"
#include "colibri_aiv/Bumper.h"
#include "colibri_aiv/Ultrasonic.h"
#include "colibri_msgs/AuxInfo.h"
#include "colibri_msgs/MusicMode.h"
#include "colibri_msgs/NavState.h"


#ifndef _AIV_DRIVER_H_
#define _AIV_DRIVER_H_

using namespace std;
using namespace boost::asio;

#define CONST_PROTOCOL_LEN	32

// protocol bytes seperation
#define START_INDX_1  0
#define START_INDX_2	1     
#define RSVD_BYTE_INDX	2
#define REQ_RES_INDX	3
#define VALID_DATA_LEN_INDX		4
#define CMD_BYTE_INDX  5
#define CMD_CTRL_INDX	 6
#define VALID_DATA_START_INDX  7
#define CRC_INDX_1  28
#define CRC_INDX_2	29
#define END_INDX_1	30
#define END_INDX_2	31

//definition of the cmd byte
#define SEND_TWIST	0x01
#define REQ_VELOCITY	0x06
#define SEND_AUX	0X07

#define RSVD_VAL	0x00
#define POS_SIGN	0x00
#define NEG_SIGN	0xff

// definition of the common const bytes in protocol
#define FRAME_START_1  0xfa
#define FRAME_START_2  0x55
#define FRAME_RSVD	0x00
#define FRAME_REQ_RES  0x05
#define FRAME_RES_SUCC	0x06
#define FRAME_RES_FAIL	0x15
#define FRAME_CMD_START  0x00
#define FRAME_CMD_STOP	0xff
#define FRAME_END_1  0x55
#define FRAME_END_2  0xfa

//The definition of the timeout
#define TIMEOUT	 1.0
#define PI	3.14159265
#define WHEEL_TRACK  0.46
#define WHEEL_RADIUS	0.1
#define WHEEL_GEAR	25

//The definiotion of the offset 
#define OFFSET_LASER_X	0.352
#define RAD2DEG  57.296
#define DEG2RAD  0.01745

//secb_frame_offset
#define OFFSET_NX2GX  -15.8  //nav frame 2 google carto frame
#define OFFSET_NY2GY  -6.0

#define ODOM_EXCEPT_GAP  1.2
#define UPDATE_CNT  300
#define SW_AMCL_YAW_CNT  2

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


void *ReadDataThread(void *args);

class AIV_Driver
{
	private:
		void GenerateCmd(unsigned char *cmd_name,unsigned char cmd,unsigned char valid_data_len,unsigned char control,unsigned char *data);
		void TwistCallback(const geometry_msgs::Twist::ConstPtr & twist);
		void AuxInfoCallback(const colibri_msgs::AuxInfo::ConstPtr & aux_info);
		void DisplayFrame(unsigned char *cmd_list);
	public:
		unsigned int send_cnt;
		unsigned int recv_cnt;

		//cmd list
		unsigned char enable_motor[CONST_PROTOCOL_LEN];
		unsigned char disable_motor[CONST_PROTOCOL_LEN];
		unsigned char send_twist[CONST_PROTOCOL_LEN];
		unsigned char send_aux_info[CONST_PROTOCOL_LEN];
		
		unsigned char req_encoder_start[CONST_PROTOCOL_LEN];
		unsigned char req_imu_start[CONST_PROTOCOL_LEN];
		unsigned char req_ultra_start[CONST_PROTOCOL_LEN];
		unsigned char req_bumper_start[CONST_PROTOCOL_LEN];
		unsigned char req_vel_start[CONST_PROTOCOL_LEN];
		
		unsigned char req_encoder_stop[CONST_PROTOCOL_LEN];
		unsigned char req_imu_stop[CONST_PROTOCOL_LEN];
		unsigned char req_ultra_stop[CONST_PROTOCOL_LEN];
		unsigned char req_bumper_stop[CONST_PROTOCOL_LEN];
		unsigned char req_vel_stop[CONST_PROTOCOL_LEN];
		
		static volatile bool enable_motor_finish;
		static volatile bool disable_motor_finish;
		static volatile bool send_twist_finish;
		static volatile bool send_aux_finish;
		static volatile bool req_encoder_start_finish;
		static volatile bool req_imu_start_finish;
		static volatile bool req_ultra_start_finish;
		static volatile bool req_bumper_start_finish;
		static volatile bool req_vel_start_finish;
		
		static volatile bool req_encoder_stop_finish;
		static volatile bool req_imu_stop_finish;
		static volatile bool req_ultra_stop_finish;
		static volatile bool req_bumper_stop_finish;
		static volatile bool req_vel_stop_finish;		

		static unsigned char *send_cache;

		double cartodom_x;
		double cartodom_y;
		float cartodom_yaw;

		float cartodom_qx;
		float cartodom_qy;
		float cartodom_qz;
		float cartodom_qw;

		float cartodom_vx;
		float cartodom_vth;
		float cartodom_interval;

		int cur_music_mode;

		cartodom_correct carto;
		
		bool correct_cartodom_flag;
		double carto_odom_x;
		double carto_odom_y;
		double carto_odom_th;

		double wheel_odom_x;
		double wheel_odom_y;
		double wheel_odom_th;

		double wheel_odom_vx;
		double wheel_odom_vy;
		double wheel_odom_vth;

		bool correct_wheelodom_flag;
		double amcl_pose_x;
		double amcl_pose_y;

		float opt_odom_x;
		float opt_odom_y;
		bool odom_except_flag;

		geometry_msgs::Quaternion amcl_quat;
		float amcl_yaw_rad;

		float frame_delta_rad;  // nav frame and google carto frame rotation angle offset

		nav_state cur_nav_state;
		bool recv_nav_flag;
		//Constructor
		AIV_Driver();
		//Destructor
		~AIV_Driver();

		bool InitCom(const string port_name);
		bool CloseCom(const string port_name);
		bool InitSubandPub();
		//write data to serial port
		void WriteToCom(const unsigned char * data);
		//read data from serial port
		void ReadFromCom(void *args);
		
		//the asyanc callback function of asyanc_read
		void ReadInfoProc(unsigned char buf[],boost::system::error_code ec,std::size_t bytes_transferred);
		
		//to call io_service::run function to call asyanc callback function
		void ComCallHandle();

		void CreateThread(void *(*start_routine) (void *));

		void SendCmd(const unsigned char *cmd ,volatile bool &send_flag);
		
		void CartodomCallback(const cartodom::Cartodom::ConstPtr & carto_odom);

		void MusicCallback(const colibri_msgs::MusicMode::ConstPtr & music_info);
		void AmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_info);
		void CalcCartodomByAmcl(float & frame_diff_angle);
		void OdomException(bool & sys_stable);
		void NavStateCallback(const colibri_msgs::NavState::ConstPtr & nav_info);

		
	private:

		pthread_t thread_id;
		double time_period;

		ros::Time last_time;		
		ros::Time current_time;

		float left_rot_rate;
		float right_rot_rate;

		float left_last_vel;
		float left_cur_vel;

		float right_last_vel;
		float right_cur_vel;

		float left_avg_vel;
		float right_avg_vel;
		
		float left_avg_distance;
		float right_avg_distance;

		float aiv_dx;
		float aiv_vx;
		float aiv_dth;
		float aiv_vth; 
		
		ros::Subscriber twist_sub;
		ros::Publisher odom_pub;
		ros::Publisher ultrasonic_pub;
		ros::Publisher bumper_pub;
		ros::Subscriber cartodom_sub;
		ros::Subscriber aux_sub;
		ros::Subscriber music_sub;
		tf::TransformBroadcaster odom_broadcaster;

		ros::Publisher wheel_odom_pub;
		ros::Subscriber amcl_pose_sub;
		ros::Subscriber nav_state_sub;
		void ParseWheelRpm(const unsigned char *valid_data);
		void CartoReal2CartoIdeal(float & x_real, float & y_real , float & x_ideal, float & y_ideal , float & theta);


};

void SmoothFrameRotAngle(float & correct_yaw, float & carto_yaw, float & amcl_yaw);
void Quaternion2Yaw(geometry_msgs::Quaternion &quat, float &yaw);


#endif

