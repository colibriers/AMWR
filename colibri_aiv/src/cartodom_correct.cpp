#include "cartodom_correct.h" 

cartodom_correct::cartodom_correct()
{
	cur_cartodom.x = -OFFSET_LASER_X;
	cur_cartodom.y = 0.0;
	cur_cartodom.yaw = 0.0;

	last_cartodom.x = -OFFSET_LASER_X;
	last_cartodom.y = 0.0;
	last_cartodom.yaw = 0.0;
	
	cartodom_dr.x = 0.0;
	cartodom_dr.y = 0.0;
	cartodom_dr.yaw= cur_cartodom.yaw;
	
	delta_cartodom = 0.0;
	
	cur_vx = 0.0;
	last_vx = cur_vx;

	delta_dead_reckon = 0.0;
	
	delta_odom_error = 0.0;
	
	epsilon = 0.16; //exp obtain
	
	corrected_odom.x = 0.0;
	corrected_odom.y = 0.0;
	corrected_odom.yaw = 0.0;

}

cartodom_correct::~cartodom_correct()
{

}

void cartodom_correct::CalcDeadReckonDeltaDis(float delta_time)
{
	delta_dead_reckon = (cur_vx + last_vx) / 2.0 * delta_time;
	delta_dead_reckon = abs(delta_dead_reckon);
}
void cartodom_correct::CalcCartodomOriDeltaDis()
{
	float tmp_delta_x = 0.0;
	float tmp_delta_y = 0.0;
	tmp_delta_x = cur_cartodom.x - last_cartodom.x;
 	tmp_delta_y = cur_cartodom.y - last_cartodom.y;
	delta_cartodom = sqrt(pow(tmp_delta_x, 2) + pow(tmp_delta_y, 2));
}

robot_pos cartodom_correct::CorrectCartodom()
{

	if((cur_cartodom.x > 7.5) && abs(delta_cartodom - delta_dead_reckon) > epsilon) //7.5 = 8.28 - 2*0.4 long line cartodom_x - offset
	{
		cartodom_dr.x = cartodom_dr.x + delta_dead_reckon * cos(cur_cartodom.yaw); // belive cartodom yaw or amcl yaw ?
		cartodom_dr.y = cartodom_dr.y + delta_dead_reckon * sin(cur_cartodom.yaw);
	}
	else
	{

	}

	corrected_odom.x =  cartodom_dr.x;
	corrected_odom.y =  cartodom_dr.y;
	corrected_odom.yaw = cur_cartodom.yaw;

	ROS_INFO("cur_cartodom.x cur_cartodom.y: %0.3f %0.3f", cur_cartodom.x, cur_cartodom.y);
	ROS_INFO("delta_cartodom delta_dead_reckon: %0.3f %0.3f", delta_cartodom, delta_dead_reckon);
	ROS_INFO("corrected_odom: %0.3f %0.3f", corrected_odom.x, corrected_odom.y);

	return corrected_odom;

}



