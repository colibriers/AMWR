#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <pthread.h>
#include <string>  
#include <vector>  
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "colibri_aiv/Ultrasonic.h"
#include "colibri_aiv/Bumper.h"
#include "cartodom/Cartodom.h"
#include "colibri_msgs/AuxInfo.h"
#include "colibri_msgs/MusicMode.h"
#include "cartodom_correct.h"

#ifndef _AIV_DRIVER_H_
#define _AIV_DRIVER_H_

using namespace std;
using namespace boost::asio;

#define CONST_PROTOCOL_LEN			32

// protocol bytes seperation
#define START_INDX_1			0
#define START_INDX_2			1     
#define RSVD_BYTE_INDX			2
#define REQ_RES_INDX        	3
#define VALID_DATA_LEN_INDX     4
#define CMD_BYTE_INDX           5
#define CMD_CTRL_INDX         	6
#define VALID_DATA_START_INDX   7
#define CRC_INDX_1				28
#define CRC_INDX_2				29
#define END_INDX_1           	30
#define END_INDX_2          	31

//definition of the cmd byte
#define ENABLE_DISABLE_MOTOR         0x00
#define SEND_TWIST                   0x01
#define REQ_ENCODER              	 0x02
#define REQ_IMU                 	 0x03
#define REQ_ULTRASONIC          	 0x04
#define REQ_BUMPER            	 	 0x05
#define REQ_VELOCITY             	 0x06
#define SEND_AUX				 	 0X07

#define ENABLE_MOTOR		0xff
#define DISABLE_MOTOR		0x55
#define RSVD_VAL			0x00
#define POS_SIGN			0x00
#define NEG_SIGN			0xff

// definition of the common const bytes in protocol
#define FRAME_START_1             0xfa
#define FRAME_START_2             0x55
#define FRAME_RSVD                0x00
#define FRAME_REQ_RES             0x05
#define FRAME_RES_SUCC            0x06
#define FRAME_RES_FAIL            0x15
#define FRAME_CMD_START           0x00
#define FRAME_CMD_STOP            0xff
#define FRAME_END_1               0x55
#define FRAME_END_2               0xfa

//The definition of the timeout
#define TIMEOUT                      1
#define PI                           3.141593
#define WHEEL_TRACK                  0.46
#define WHEEL_RADIUS                 0.1
#define WHEEL_GEAR                   25

//The definiotion of the offset 
#define OFFSET_LASER_X			0.352
#define RAD2DEG					57.296
#define DEG2RAD					0.01745

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

		void ParseWheelRpm(const unsigned char *valid_data);

};

#endif

