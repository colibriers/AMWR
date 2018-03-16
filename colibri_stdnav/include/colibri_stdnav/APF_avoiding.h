#ifndef APF_AVOIDING_H_
#define APF_AVOIDING_H_

#include <algorithm>
#include <ctime>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#include "colibri_msgs/AngPotnEngy.h"
#include "colibri_msgs/EnvSecurity.h"

using namespace std;

#define RAD2DEG 	180.0/M_PI
#define DEG2RAD 	M_PI/180.

#define	DEG2RAD_PARAM(deg)	DEG2RAD * (deg)
#define	RAD2DEG_PARAM(rad)	RAD2DEG * (rad)

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

#define NUM_RAY4CA 				181		//every degree for front semi-cirle
#define ANGLE4CA_START 			0
#define ANGLE4CA_START_INDEX 	61		// 61=(0-(-30))/0.5+1
#define ANGLE4CA_END 			180
#define ANGLE4CA_END_INDEX 		421		// 421=(180-(-30))/0.5+1
#define LASER_BLIND 			0.06
#define RAY_RESOL4CA			1.0		//laser ray resolution for colision avoidence

#define LASER4CA_FILL_DIS 2.0

#define K_SF 	1.25		//adj factor for latitude dir in polar frame
#define WIDTH 	0.56
#define D_SF	0.336	//0.5*K_SF*WIDTH  lateral safe dis

#define K_SR 	1.1		//adj factor for longitude dir in polar frame
#define ACC_DEC -2.0		//accerlaration for decrease vel
#define D_M 	5.0		// local ca distance
#define KP_PHI_INF 10000.0

#define V_MAX 		0.65
#define V_MIN 		0.02
#define THETA_V_MAX 0.45

#define KRF_CORRECTOR 0.1

#define PASSFCN_THD_RATIO 	0.2
#define MAX_PASSFCN_SCOPE	0.1
#define PASSVAL_TOLLERENCE  0.1

class APF
{
	public:
		
		float *ptrScan4ca;
		float scan4ca[NUM_RAY4CA];
		float ultra4ca[NUM_RAY4CA];

		float abstract_pf[NUM_RAY4CA];

		float min_laser;
		int min_laser_dir;

		float add_obs4ca[NUM_RAY4CA];
		
		float delta_phi_vec[NUM_RAY4CA];
		float kp_phi_vec[NUM_RAY4CA];

		float krf_vec[NUM_RAY4CA];
		float kaf_vec[NUM_RAY4CA];
		float passfcn_vec[NUM_RAY4CA];
		float max_passfcn_val;

		int fwd_maxpass_cnt ;
		int bwd_maxpass_cnt ;

		int maxfcn_fwdbnd;
		int maxfcn_bwdbnd;

		int phi_start_vec[NUM_RAY4CA];
		int phi_end_vec[NUM_RAY4CA];

		float goal_dir;	//goal direction in degree
		float v;
		float vth;
		
		float angle_adj;
		float vel_adj;
		unsigned int apf_alarm;
		unsigned int wander;
		
		ros::NodeHandle nh_ca;
		ros::Subscriber scan_sub4ca;
		ros::Subscriber env_sub4safe;
		ros::Publisher pf_Pub4dbg;

		colibri_msgs::AngPotnEngy pf_dbg;

		
		APF();
		~APF();
		
		float CalcKpPhi(float vel_center, float d_phi);
		void CalcKrfTheta(float* ptrKp_phi_vector, int* ptrPhi_range_start, int* ptrPhi_range_end);	
		void CalcPhiRange(int i, int range_num, int* ptrPhi_start, int* ptrPhi_end);
		void CalcPassFcnAndFwdBnd(unsigned int flag, float* max_passfcn_val, float* ptrK_pg);
		void CalcPassFcnWithoutRPF(float* max_passfcn_val, float* ptrK_pg, float* heading);
		void CalcPassFcnAndBwdBnd(unsigned int flag, float* max_passfcn_val, float* ptrK_pg);
		float CalcAdjDir(float* ptrPass_fcn_vector, float max_passfcn_val, int* fwd_bound, int* bwd_bound);
		void CalcCorrectedKrf(void);
		void CalcAlarmInAPF(void);
		void CalcPhiParam(float vel_center, float& dir_goal_inlaser);
		void ResetMaxPassValCnt(void);
		void LimitAngle(float& delta_ang);
		void TrimLaserRange4CA(float & compensate);
		void PubPfInfo4Dbg(void);

			
	private:

		void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_ca);
		void EnvSecurityCallBack(const colibri_msgs::EnvSecurity::ConstPtr& env);

		float CalcDsrVc(float vel_center);
		float CalcKrfCorrectFactor(int index);

};


#endif //	APF_AVOIDING_H_

