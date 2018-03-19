#ifndef	 ACTIONS_H_
#define  ACTIONS_H_

#include "colibri_ca.h"
#include "PID_controller.h"

#define STRAIGHT_MOVING_MAX 0.8

#define SIGMOID_AMP	1
#define SIGMOID_SLOPE_INC	-10
#define SIGMOID_SLOPE_DEC	10
#define	SIGMOID_OFFSET_X	0.5

#define SLOW_ROT_ANGLE 45

#define BELL_SIGMA	0.5

#define GRAVATON_NGHBORHD	0.6

using namespace std;

int SgnOfData(const float & data);
float SigmoidFunction(const int &fcn_dir, const float & input); 
float UpdownBellFunction(const float & input, const float & eps = 0.02);

class Actions {
	public:

		struct st_ctrl_cmd {
			st_ctrl_cmd():linear(0.), angular(0.) {

			}; 
			float linear;
			float angular;
		};
		typedef st_ctrl_cmd action_cmd;

		action_cmd ctrl_;
		
		float waiting_interval;
		
		float action4cmd_vel[VEL_DIM];

		PID_Controller ctrl4yawObj;

		const float angle_tolerance_;

		nav_action();
		~nav_action();

		float* WaitingAction(float waiting_time, unsigned int* finish_flag);
		float* StraightMovingAction(float* cur_vx, float* ref_vx, float proc_time);

		bool & StillRotatingAction(const float * ref_yaw, const float * cur_yaw, const float & rot_coeff = 1.2);
		bool & StillRotatingAction(const float & cur_yaw, const float & ref_yaw, const float & init_angular); 
		bool & StillRotatingActionClosedLoop(const float & ref_yaw, const float & cur_yaw);  

		
		float* AdjustMovingDirAction(float* cur_yaw, float* goal_in_laser, float* robot2goal, unsigned int* finish_flag);

		//float* ApproachingGoalAction(float* cur_pos, float* goal_pos,float* cur_laser2goal_angle, unsigned int* finish_flag);
		//float* ApproachingGoalAction(float* cur_pos, float* goal_pos, unsigned int* finish_flag);
		float* ApproachingGoalAction(float* cur_pos, float* goal_pos, float * cur_yaw, float & cur_vx, unsigned int* finish_flag);
		float* ApproachingGravatonAction(float* cur_pos, float* cur_vel, float* gravaton_pos,float* cur_laser2gravation_angle, unsigned int finish_flag);

		bool ReachGravatonOK(float *cur_pos, float *cur_gravaton,float &delta_dis);
		int CalcMicroRotAngle(float & r2g, float & heading, float & diff_angle);


	private:

		float delta_yaw_;

		ros::Time time_stamp;
		ros::Time time_stamp_start;
		void CalcCtrlDeltaYaw(const float &ref_yaw, const float cur_yaw);	
};


#endif

