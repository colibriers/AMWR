#include "actions.h"

int SgnOfData(const float & data) {
	float epsilon = 0.000001;
	if(data > epsilon) {
		return 1;
		
	} else if(data < -epsilon) {
		return -1;
		
	} else {
		return 0;
		
	}		
}

float SigmoidFunction(const int & fcn_dir, const float & input)	
{
	float output;
	/*
	*  logistic function: y = k / (1+exp(a*(x-b))) 
	*/

	if(fcn_dir == 1)		// up direction : S shape
	{
		output = SIGMOID_AMP / (1 + exp(SIGMOID_SLOPE_INC * (input - SIGMOID_OFFSET_X)));
	}
	else if(fcn_dir == -1)	//down direction: mirror S shape
	{
		output = SIGMOID_AMP / (1 + exp(SIGMOID_SLOPE_DEC * (input - SIGMOID_OFFSET_X)));

	}
	else
	{
		cout<<"sigmoid function direction exception"<<endl;
		output = 0.0;
	}

	return output;
}

float UpdownBellFunction(const float & input, const float & eps) {
	float output = 0.0;
	float tmp = 0.0;
	float corrector = 0.6899;	//to make output between 0~1; 

	if(abs(input) <= eps)	 {
		output = 0.0; 	// 0.0278*180 = 5 degree
	} else if(abs(input) <= 1.0) {
		tmp = 1 / (sqrt(2 * PI) * BELL_SIGMA);
		output = tmp - tmp * exp(-1.0 * pow(*input, 2) / (2.0 * pow(BELL_SIGMA, 2)));
		output = output / corrector;
	} else {
		output = 1.0;
	}
	return output;
}


Actions::Actions():ctrl4yawObj(0.1, 0.2, 0.0, 0.2)
{
	waiting_interval = 0.0;
	
	time_stamp = ros::Time::now();
	time_stamp_start = time_stamp;

	action4cmd_vel[0] = 0.0;
	action4cmd_vel[1] = 0.0;
}

Actions::~Actions()
{

}

const Actions::float angle_tolerance_ = 4.0;

bool & Actions::WaitingAction(const float & waiting_time) {
	float waiting_delta_time = 0.0;
	static bool time_record_flag = false;
	bool wait_finish_flag = false;

	if(time_record_flag == false) {
		time_stamp = ros::Time::now();
		time_stamp_start = time_stamp;
		time_record_flag = 1;
	} else {
		time_stamp = ros::Time::now(); 
	}

	waiting_delta_time = (time_stamp - time_stamp_start).toSec();

	if(waiting_delta_time <= waiting_time) {
		ctrl_.linear = 0.0;
		ctrl_.angular = 0.0;
		wait_finish_flag = false;
	} else {
		time_record_flag = 0;
		time_stamp = ros::Time::now();
		time_stamp_start = time_stamp;
		wait_finish_flag = true;
	}

	return wait_finish_flag;
}

void Actions::MoveForwardAction(const float & ref_vx, const float & cur_vx, const float & proc_time) {
	float waiting_delta_time = 0.0;
	static float vx_delta = 0.0;
	static float vx_start = 0.0;
	float dynamic_time_interval = 0.0;
	float tmp_delta_out = 0.0;
	static int time_record_flag = 0;

	if(time_record_flag == 0) {
		time_stamp = ros::Time::now();
		time_stamp_start = time_stamp;
		time_record_flag = 1;
		vx_delta = ref_vx - cur_vx;
		vx_start = cur_vx; 
	} else {
		time_stamp = ros::Time::now();
	}

	waiting_delta_time = (time_stamp - time_stamp_start).toSec();

	if(waiting_delta_time <= proc_time) {
		dynamic_time_interval = waiting_delta_time / proc_time;
		tmp_delta_out = dynamic_time_interval * vx_delta;

		ctrl_.linear = vx_start + tmp_delta_out;
		ctrl_.angular = 0.0;
	} else {
		ctrl_.linear = ref_vx;
		ctrl_.angular = 0.0;
		vx_delta = 0.0;
		vx_start = 0.0;
		time_record_flag = 0;
		time_stamp = ros::Time::now();
		time_stamp_start = time_stamp;	
	}
}

bool & Actions::StillRotatingAction(const float * ref_yaw, const float * cur_yaw, const float & rot_coeff) {

	float yaw_delta = 0.0;
	float yaw_delta_puv = 0.0;
	bool rot_finish_flag = false;

	yaw_delta = (*ref_yaw - *cur_yaw);
	yaw_delta_puv = (*ref_yaw - *cur_yaw)/180.0;
	ctrl_.linear = 0.0;
	ctrl_.angular = rot_coeff * yaw_delta_puv;	

	if(abs(yaw_delta) <= angle_tolerance_)
	{
		rot_finish_flag = true;
		ctrl_.angular = 0.0;	
	}
	else
	{
		rot_finish_flag = false;
	}

	return rot_finish_flag;
}

bool & Actions::StillRotatingAction(const float & ref_yaw, const float & cur_yaw, const float & init_angular) {
	float delta_puv = 1;
	bool rot_finish_flag = false;
	
	ctrl_.linear = 0.;

	CalcCtrlDeltaYaw(ref_yaw, cur_yaw);

	if(abs(delta_yaw_) > SLOW_ROT_ANGLE) {
		ctrl_.angular = init_angular * SgnOfData(delta_yaw_);
		
	} else {
		delta_puv = abs(delta_yaw_/SLOW_ROT_ANGLE);
		ctrl_.angular = init_angular * SgnOfData(delta_yaw_) * SigmoidFunction(1, &delta_puv);	
	}

	if(abs(delta_yaw_) <= angle_tolerance_)
	{
		rot_finish_flag = true;
		ctrl_.angular = 0.;
	}
	else
	{
		rot_finish_flag = false;

	}

	return rot_finish_flag;
}


bool & Actions::StillRotatingActionClosedLoop(const float & ref_yaw, const float & cur_yaw) {
	float yaw_delta = 0.0;
	float yaw_delta_puv = 0.0;
	float ctrl_u_delta = 0.0;
	bool rot_finish_flag = false;
	
	yaw_delta = (ref_yaw - cur_yaw);
	yaw_delta_puv = yaw_delta / 180.0;
	
	ctrl_.linear = 0.0;
	if(abs(yaw_delta) <= 30) {
		ctrl4yawObj.Regulator(ref_yaw, cur_yaw);
		ctrl_u_delta = ctrl4yawObj.ctrl_param_.u_out;
	} else {
		ctrl_u_delta = 0.0;
	}
	
	ctrl_.angular = yaw_delta_puv + ctrl_u_delta;

	if(abs(yaw_delta) <= angle_tolerance_) {
		rot_finish_flag = true;
		ctrl_.angular = 0.0;	
	} else {
		rot_finish_flag = false;
	}

	return rot_finish_flag;
}

void Actions::CalcCtrlDeltaYaw(const float &ref_yaw, const float cur_yaw) {
	float yaw_delta = 0.0;
	int rot_dir = 1;

	float tmp_diff = ref_yaw - cur_yaw;
	
	if(tmp_diff > 180.) {
		yaw_delta = 360. - tmp_diff;
		rot_dir = -1;
		
	} else if(tmp_diff < -180.) {
		yaw_delta = 360. + tmp_diff;
		rot_dir = 1;
		
	} else {
		yaw_delta = tmp_diff;
		rot_dir = 1;
	}
	delta_yaw_ = yaw_delta * rot_dir;
	
}

float* Actions::AdjustMovingDirAction(float* cur_yaw, float* goal_in_laser, float* robot2goal, unsigned int* finish_flag)
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

/*
float* Actions::ApproachingGoalAction(float* cur_pos, float* goal_pos,float* cur_laser2goal_angle, unsigned int* finish_flag)
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


float* Actions::ApproachingGoalAction(float* cur_pos, float* goal_pos, unsigned int* finish_flag)
{ 
	float delta_dis_puv = 1.0;		//distance per unit value
	float delta_yaw_puv = 0.0;
	float delta_x = *goal_pos - *cur_pos;
	float delta_y = *(goal_pos + 1) - *(cur_pos + 1);

	delta_dis_puv = sqrt(pow(delta_x, 2) + pow(delta_y, 2))/GOAL_NGHBORHD;
	delta_yaw_puv = 0.0;

	//action4cmd_vel[0] = APPROACH_V_MAX * SigmoidFunction(1, &delta_dis_puv);
	//action4cmd_vel[1] = APPROACH_VTH_MAX * SigmoidFunction(1, &delta_yaw_puv);
	if(delta_dis_puv > 0.5)
	{
		action4cmd_vel[0] = APPROACH_V_MAX * delta_dis_puv;
	}
	else
	{
		action4cmd_vel[0] = APPROACH_V_MAX * SigmoidFunction(1, &delta_dis_puv);
	}

	action4cmd_vel[1] = APPROACH_VTH_MAX * delta_yaw_puv;
	
	if(delta_dis_puv > 0.06)	
	{
		*finish_flag = 0;
	}
	else
	{
		cout<<"Complete the approaching process"<<endl;
		action4cmd_vel[0] = 0.0;
		action4cmd_vel[1] = 0.0;
		*finish_flag = 1;
	}

	if(action4cmd_vel[0] < 0.009) //if vel so small should stop it 
	{
		action4cmd_vel[0] = 0.0;
		action4cmd_vel[1] = 0.0;
		*finish_flag = 1;
	}

	
	return action4cmd_vel;

}

*/

float* Actions::ApproachingGoalAction(float* cur_pos, float* goal_pos, float * cur_yaw, float & cur_vx, unsigned int* finish_flag)
{ 
	float delta_dis_puv = 1.0;		//distance per unit value
	float delta_yaw = 0.0;
	float delta_x = *goal_pos - *cur_pos;
	float delta_y = *(goal_pos + 1) - *(cur_pos + 1);
	float delta_gap = 0.4;
	static bool lock_approx_vel = false;
	static float aiv_vx = 0.2;
	float angle_diff = 0;
	float tmp_r2g = 0.0;
	static float last_dis_puv = 10.0;
	static int pose_diverge_cnt = 0;
	int rot_proterty = 0;
	float stop_ratio = 0.08;

	if(lock_approx_vel == false)
	{
		if(cur_vx < 0.2)
		{
			aiv_vx = 0.2;
		}
		else
		{
			aiv_vx = cur_vx;			
		}

		lock_approx_vel = true;
	}
	else
	{
		
	}


	delta_dis_puv = sqrt(pow(delta_x, 2) + pow(delta_y, 2))/GOAL_NGHBORHD;
	tmp_r2g = atan2(delta_y, delta_x) * RAD2DEG;

	rot_proterty = CalcMicroRotAngle(tmp_r2g, *cur_yaw, angle_diff);


	if(delta_dis_puv > delta_gap)
	{
		action4cmd_vel[0] = aiv_vx;
		if(abs(angle_diff) > 3.0)
		{
			if(rot_proterty != 0)
			{
				action4cmd_vel[1] = rot_proterty * angle_diff / 100;

			}
			else
			{
				action4cmd_vel[1] = angle_diff / 100;
			}

		}
		else
		{
			action4cmd_vel[1] = 0.0;
		}
	}
	else
	{
		action4cmd_vel[0] = aiv_vx * delta_dis_puv / delta_gap;
		action4cmd_vel[1] = 0.0;
	}

	//cout<<"------xxx----- Approaching action angle_diff: "<< angle_diff <<endl;
	cout<<"tmp_r2g: "<< tmp_r2g << " *cur_yaw: "<< *cur_yaw <<endl;
	
	if(delta_dis_puv > stop_ratio)	
	{
		*finish_flag = 0;
		
		if(delta_dis_puv - last_dis_puv > 0 && delta_dis_puv < delta_gap)
		{
			pose_diverge_cnt++;
		}
		else
		{
			
		}
		
		if(pose_diverge_cnt > 1)
		{
			action4cmd_vel[0] = 0.0;
			action4cmd_vel[1] = 0.0;
			pose_diverge_cnt = 0;
			last_dis_puv = 10.0;
			*finish_flag = 1;
		}
		else
		{
			
		}
	
	}
	else
	{
		cout<<"Complete the approaching process: delta_dis: "<< delta_dis_puv*GOAL_NGHBORHD<<endl;
		action4cmd_vel[0] = 0.0;
		action4cmd_vel[1] = 0.0;
		pose_diverge_cnt = 0;
		last_dis_puv = 10.0;

		*finish_flag = 1;

	}

	if(delta_dis_puv != last_dis_puv)
	{
		last_dis_puv = delta_dis_puv;
	}

	if(	*finish_flag == 1)
	{
		lock_approx_vel = false;
	}
	
	
	return action4cmd_vel;

}



float* Actions::ApproachingGravatonAction(float* current_pos, float* current_vel, float* gravaton_pos, float* current_laser2gravaton_angle, unsigned int complete_flag)
{

}

int Actions::CalcMicroRotAngle(float & r2g, float & heading, float & diff_angle)
{
	int diff_angle_property = 0;
	float tmp_diff = r2g - heading;
	diff_angle = 0.0;
	if(tmp_diff > 180)
	{
		diff_angle = 360 - tmp_diff;
		diff_angle_property = -1;

	}
	else if(tmp_diff < -180)
	{
		diff_angle = 360 + tmp_diff;
		diff_angle_property = 1;

	}
	else
	{
		diff_angle = tmp_diff;
		diff_angle_property = 0;
	}

	return diff_angle_property;
	
}

bool Actions::ReachGravatonOK(const float *cur_pos, const float *cur_gravaton) {
	float tmp_delta_x = 0.0;
	float tmp_delta_y = 0.0;
	
	tmp_delta_x = *cur_pos - *cur_gravaton;
	tmp_delta_y = *(cur_pos + 1) - *(cur_gravaton + 1);
	
	robot2gravaton_dis_ = sqrt(pow(tmp_delta_x, 2) + pow(tmp_delta_y, 2));

	if(robot2gravaton_dis_ <= GRAVATON_NGHBORHD) {
		return true;
	} else {
		return false;
	}

}

