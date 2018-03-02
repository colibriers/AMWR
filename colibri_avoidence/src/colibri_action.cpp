#include "colibri_action.h"


nav_action::nav_action()
{

	waiting_type = 0;

	waiting_interval = 0.0;
	
	time_stamp = ros::Time::now();
	time_stamp_start = time_stamp;

	action4cmd_vel[0] = 0.0;
	action4cmd_vel[1] = 0.0;

	ctrl4yawObj.InitRegulatorParam(0.1, 0.2, 0.0, 0.2);
}

nav_action::~nav_action()
{

}

float* nav_action::WaitingAction(float waiting_time, unsigned int* finish_flag)
{
	float waiting_delta_time = 0.0;
	static int time_record_flag = 0;

	if(time_record_flag == 0)
	{
		
		time_stamp = ros::Time::now();
		time_stamp_start = time_stamp;
		time_record_flag = 1;
	}
	else
	{
		time_stamp = ros::Time::now(); 
	}

	waiting_delta_time = (time_stamp - time_stamp_start).toSec();

	if(waiting_delta_time <= waiting_time)
	{
		action4cmd_vel[0] = 0.0;
		action4cmd_vel[1] = 0.0;
		*finish_flag = 0;
	}
	else
	{
		cout<<"WaitAction -- waiting_delta_time: "<<waiting_delta_time<<endl;
		time_record_flag = 0;
		time_stamp = ros::Time::now();
		time_stamp_start = time_stamp;
		*finish_flag = 1;
	}

	return action4cmd_vel;
}

float* nav_action::StraightMovingAction(float* cur_vx, float* ref_vx, float proc_time)
{
	float waiting_delta_time = 0.0;
	static float vx_delta = 0.0;
	static float vx_start = 0.0;
	float dynamic_time_interval = 0.0;
	float tmp_delta_out = 0.0;
	static int time_record_flag = 0;

	if(time_record_flag == 0)
	{
		
		time_stamp = ros::Time::now();
		time_stamp_start = time_stamp;
		time_record_flag = 1;
		vx_delta = *ref_vx - *cur_vx;
		vx_start = *cur_vx; 
	}
	else
	{
		time_stamp = ros::Time::now();
	}

	waiting_delta_time = (time_stamp - time_stamp_start).toSec();

	if(waiting_delta_time <= proc_time)
	{
		dynamic_time_interval = waiting_delta_time / proc_time;
		tmp_delta_out = dynamic_time_interval * vx_delta;

		action4cmd_vel[0] = vx_start + tmp_delta_out;
		action4cmd_vel[1] = 0.0;

	}
	else
	{
		action4cmd_vel[0] = *ref_vx;
		action4cmd_vel[1] = 0.0;
		vx_delta = 0.0;
		vx_start = 0.0;
		time_record_flag = 0;
		time_stamp = ros::Time::now();
		time_stamp_start = time_stamp;	
	}

	return action4cmd_vel;

}

float* nav_action::StillRotatingAction(float* cur_yaw, float* ref_yaw, unsigned int* finish_flag)
{

	float yaw_delta = 0.0;
	float yaw_delta_puv = 0.0;

	yaw_delta = (*ref_yaw - *cur_yaw);
	yaw_delta_puv = (*ref_yaw - *cur_yaw)/180.0;
	action4cmd_vel[0] = 0.0;
	action4cmd_vel[1] = 1.1 * yaw_delta_puv;	

	if(abs(yaw_delta) <= ROTATION_TOLERANCE)
	{
		*finish_flag = 1;
		action4cmd_vel[1] = 0.0;	
	}
	else
	{
		*finish_flag = 0;
	}

	return action4cmd_vel;
}

float* nav_action::CL4StillRotatingAction(float* cur_yaw, float* ref_yaw, unsigned int* finish_flag)
{

	float yaw_delta = 0.0;
	float yaw_delta_puv = 0.0;
	float ctrl_u_delta = 0.0;
	
	yaw_delta = (*ref_yaw - *cur_yaw);
	yaw_delta_puv = (*ref_yaw - *cur_yaw)/180.0;
	
	action4cmd_vel[0] = 0.0;
	if(abs(yaw_delta) <= 30)
	{
		ctrl4yawObj.Regulator(*ref_yaw, *cur_yaw, &ctrl_u_delta);
	}
	else
	{
		ctrl_u_delta = 0.0;
	}
	
	action4cmd_vel[1] = yaw_delta_puv + ctrl_u_delta;

	if(abs(yaw_delta) <= ROTATION_TOLERANCE4CL)
	{
		*finish_flag = 1;
		action4cmd_vel[1] = 0.0;	
	}
	else
	{
		*finish_flag = 0;
	}

	return action4cmd_vel;
}

float* nav_action::AdjustMovingDirAction(float* cur_yaw, float* goal_in_laser, float* robot2goal, unsigned int* finish_flag)
{

	float* tmp_action_cmd = NULL;
	static float tmp_target4adj = 0.0;
	static unsigned int rotating_flag = 0;
	static unsigned int rot4adj_finish_flag = 0;
	float tmp_r2g = 0.0;

	if((*goal_in_laser >= 0.0)&&(*goal_in_laser <= 180.0)&&(rotating_flag == 0))
	{
		*finish_flag = 1;
		action4cmd_vel[0] = 0.0;
		action4cmd_vel[1] = 0.0;

	}
	else
	{
		rotating_flag = 1;
		*finish_flag = 0;
		if(tmp_target4adj < 0.1)// this section used to handle the still rotation  causing robot2gravaton angle +/- 180 jump
		{
			tmp_target4adj = *robot2goal;
		}
		else
		{
			tmp_r2g = *robot2goal;
			if(abs(tmp_r2g - tmp_target4adj) > 60)
			{
				
			}
			else
			{
				tmp_target4adj = tmp_r2g;
			}


		}

		cout<<" >>.tmp_target4adj: "<<tmp_target4adj<<endl;
		cout<<" >>....cur_yaw: "<<*cur_yaw<<endl;

		tmp_action_cmd = StillRotatingAction(cur_yaw, &tmp_target4adj, &rot4adj_finish_flag);
		//tmp_action_cmd = CL4StillRotatingAction(cur_yaw, &tmp_target4adj, &rot4adj_finish_flag);
		cout<<" >>......rot4adj_finish_flag: "<<rot4adj_finish_flag<<endl;

		action4cmd_vel[0] = *tmp_action_cmd;
		action4cmd_vel[1] = *(tmp_action_cmd + 1);
		
		if(rot4adj_finish_flag == 1)		
		{
			*finish_flag = 1;
			rotating_flag = 0;
			tmp_target4adj = 0.0;
		}
	}

	return action4cmd_vel;
}


float* nav_action::ApproachingGoalAction(float* cur_pos, float* goal_pos,float* cur_laser2goal_angle, unsigned int* finish_flag)
{ 
	float delta_dis_puv = 1.0;		//distance per unit value
	float delta_yaw_puv = 0.0;
	float delta_x = *goal_pos - *cur_pos;
	float delta_y = *(goal_pos + 1) - *(cur_pos + 1);


	delta_dis_puv = sqrt(pow(delta_x, 2) + pow(delta_y, 2))/GOAL_NGHBORHD;
	delta_yaw_puv = (*cur_laser2goal_angle - 90.0) / 90.0;
	
	if((*cur_laser2goal_angle > 0.0)&&(*cur_laser2goal_angle <= 180.0))
	{
		action4cmd_vel[0] = APPROACH_V_MAX * SigmoidFunction(1, &delta_dis_puv);
		action4cmd_vel[1] = APPROACH_VTH_MAX * SigmoidFunction(1, &delta_yaw_puv);
	}
	else
	{
		cout<<"Before ApproachGoalAction, Robot should Execute AdjMovingdirAction ..."<<endl;
		action4cmd_vel[0] = 0.0;
		action4cmd_vel[1] = 0.0;
		*finish_flag = 0;
	}
	
	return action4cmd_vel;

}

float* nav_action::ApproachingGravatonAction(float* current_pos, float* current_vel, float* gravaton_pos, float* current_laser2gravaton_angle, unsigned int complete_flag)
{

}

bool nav_action::ReachGravatonOK(float *cur_pos, float *cur_gravaton, float &delta_dis)
{
	float tmp_delta_x = 0.0;
	float tmp_delta_y = 0.0;
	
	tmp_delta_x = *cur_pos - *cur_gravaton;
	tmp_delta_y = *(cur_pos + 1) - *(cur_gravaton + 1);
	
	delta_dis = sqrt(pow(tmp_delta_x, 2) + pow(tmp_delta_y, 2));

	if(delta_dis <= GRAVATON_NGHBORHD)
	{
		return true;
	}
	else
	{
		return false;
	}

}



float nav_action::SigmoidFunction(int fcn_dir, float* input)	
{
	float output;
	/*
	*  logistic function: y = k / (1+exp(a*(x-b))) 
	*/

	if(fcn_dir == 1)		// up direction : S shape
	{
		output = SIGMOID_AMP / (1 + exp(SIGMOID_SLOPE_INC * (*input - SIGMOID_OFFSET_X)));
	}
	else if(fcn_dir == -1)	//down direction: mirror S shape
	{
		output = SIGMOID_AMP / (1 + exp(SIGMOID_SLOPE_DEC * (*input - SIGMOID_OFFSET_X)));

	}
	else
	{
		cout<<"sigmoid function direction exception"<<endl;
		output = 0.0;
	}

	return output;
}

float nav_action::UpdownBellFunction(float* input)
{
	float output = 0.0;
	float tmp = 0.0;
	float corrector = 0.6899;	//to make output between 0~1; 

	if(abs(*input) <= 0.02)		// 0.0278*180 = 5 degree
	{
		output = 0.0;

	}
	else if(abs(*input) <= 1.0)
	{
		tmp = 1 / (sqrt(2 * PI) * BELL_SIGMA);
		output = tmp - tmp * exp(-1.0 * pow(*input, 2) / (2.0 * pow(BELL_SIGMA, 2)));
		output = output / corrector;
	}
	else
	{
		output = 1.0;

	}

	return output;
	
}



