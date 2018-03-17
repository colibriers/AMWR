#ifndef APF_AVOIDING_H_
#define APF_AVOIDING_H_

#include <algorithm>
#include <ctime>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#include "colibri_msgs/AngPotnEngy.h"
#include "colibri_msgs/EnvSecurity.h"

#define RAD2DEG 	180.0/M_PI
#define DEG2RAD 	M_PI/180.

#define	DEG2RAD_PARAM(deg)	DEG2RAD * (deg)
#define	RAD2DEG_PARAM(rad)	RAD2DEG * (rad)

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

#define NUM_RAY4CA 				181			//every degree for front semi-cirle
#define ANGLE4CA_START 			0
#define ANGLE4CA_START_INDEX 	61		// 61=(0-(-30))/0.5+1
#define ANGLE4CA_END 			180
#define ANGLE4CA_END_INDEX 		421		// 421=(180-(-30))/0.5+1
#define LASER_BLIND 			0.06
#define LASER_RANGE				20.0
#define RAY_RESOL4CA			1.0			//laser ray resolution for colision avoidence

#define LASER4CA_FILL_DIS 2.0

#define K_SF 	1.25						//adj factor for latitude dir in polar frame
#define WIDTH 	0.56
#define D_SF	K_SF*WIDTH/2.0		//0.5*K_SF*WIDTH  lateral safe dis

#define K_SR 	1.1		//adj factor for longitude dir in polar frame
#define ACC_DEC -2.0		//accerlaration for decrease vel

#define D_M 	5.0		// local ca distance
#define KP_PHI_INF 65536.0

#define V_MAX 		0.65
#define V_MIN 		0.02
#define THETA_V_MAX DEG2RAD_PARAM(150)

#define PASSFCN_THD_RATIO 	0.2
#define MAX_PASSFCN_SCOPE		0.1
#define PASSVAL_TOLLERENCE  0.1

class APF {
	public:
		typedef Eigen::Array<float, 1, NUM_RAY4CA> Array_CA;
		
		Array_CA scan4ca_;

		float abstract_pf[NUM_RAY4CA];

		Array_CA delta_phi_vec_;
		Array_CA kp_phi_vec_;

		Array_CA krf_vec_;
		Array_CA kaf_vec_;
		Array_CA passfcn_vec_;
		
		Eigen::Array<int, 1, NUM_RAY4CA> phi_start_vec_;
		Eigen::Array<int, 1, NUM_RAY4CA> phi_end_vec_;

		float min_laser;
		int min_laser_dir;

		float dsr_v_;
		
		float max_passfcn_val_;
		int fwd_maxpass_cnt_ ;
		int bwd_maxpass_cnt_ ;
		int maxfcn_fwdbnd_;
		int maxfcn_bwdbnd_;

		float goal_dir_;	//goal direction in degree
		float ctrl_v_;
		float ctrl_vth_;
		
		float angle_adj_;
		float vel_adj;
		unsigned int apf_alarm_;
		
		ros::NodeHandle nh_ca;
		ros::Subscriber scan_sub4ca;
		ros::Subscriber env_sub4safe;
		ros::Publisher pf_Pub4dbg;

		colibri_msgs::AngPotnEngy pf_dbg;

		APF();
		APF(float init_goal_dir = 90.0);
		~APF();
		
		void CalcPhiRange(const Eigen::Array<int, 1, NUM_RAY4CA> &range_num);
		void CalcKpPhi(const float & vel);
		void CalcPhiParam(const float & velocity);
		
		void CalcKafTheta(const float& dir_goal_inlaser);		
		void CalcKrfTheta(void);
		void CalcCorrectedKrf(const float & krf_corrector);



		void CalcPassFcn(const int  & seq_flag, const bool & concern_apf);
		void CalcPassFcnWithoutRPF(void);

		void CalcAdjDir(void); 

		void CalcAlarmInAPF(void);
		void ResetMaxPassValCnt(void);

		void PubPfInfo4Dbg(void);
		void CalcCtrlCmd(const float & v_ref, const float & rot_coeff);

			
	private:

		void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_ca);
		void EnvSecurityCallBack(const colibri_msgs::EnvSecurity::ConstPtr& env);

		void CalcDsrVc(const float & vel);
		void LimitAngle(Array_CA & delta_ang);

};


#endif //	APF_AVOIDING_H_

