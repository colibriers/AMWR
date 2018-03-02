#include "colibri_ca.h"
#include "colibri_local_nav.h"
#include "PID_controller.h"

#ifndef	 _COLIBRI_ACTION_H_
#define  _COLIBRI_ACTION_H_

#define STRAIGHT_MOVING_MAX 0.8

#define SIGMOID_AMP	1
#define SIGMOID_SLOPE_INC	-10
#define SIGMOID_SLOPE_DEC	10
#define	SIGMOID_OFFSET_X	0.5

#define BELL_SIGMA	0.5

#define GRAVATON_NGHBORHD	0.6

using namespace std;

class nav_action
{
	public:

		unsigned int waiting_type;
		float waiting_interval;
		
		float action4cmd_vel[VEL_DIM];

		PID_controller ctrl4yawObj;

		nav_action();
		~nav_action();

		float* WaitingAction(float waiting_time, unsigned int* finish_flag);
		float* StraightMovingAction(float* cur_vx, float* ref_vx, float proc_time);

		float* StillRotatingAction(float* cur_yaw, float* ref_yaw, unsigned int* finish_flag);
		float* CL4StillRotatingAction(float* cur_yaw, float* ref_yaw, unsigned int* finish_flag);
		
		float* AdjustMovingDirAction(float* cur_yaw, float* goal_in_laser, float* robot2goal, unsigned int* finish_flag);

		float* ApproachingGoalAction(float* cur_pos, float* goal_pos,float* cur_laser2goal_angle, unsigned int* finish_flag);
		float* ApproachingGravatonAction(float* cur_pos, float* cur_vel, float* gravaton_pos,float* cur_laser2gravation_angle, unsigned int finish_flag);

		bool ReachGravatonOK(float *cur_pos, float *cur_gravaton,float &delta_dis);

		float SigmoidFunction(int fcn_dir, float* input);	
		float UpdownBellFunction(float* input);

	private:

		ros::Time time_stamp;
		ros::Time time_stamp_start;

		
};


#endif

