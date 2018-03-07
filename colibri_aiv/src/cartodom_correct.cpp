#include "cartodom_correct.h" 

cartodom_correct::cartodom_correct() {
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
	
	epsilon = 0.17; //exp obtain
	normal_thd = 0.11;
	carto_except = false;
	except_phase = 0;
	
	corrected_odom.x = 0.0;
	corrected_odom.y = 0.0;
	corrected_odom.yaw = 0.0;

}

cartodom_correct::~cartodom_correct() {

}

void cartodom_correct::CalcDeadReckonDeltaDis(float delta_time) {
	if(cur_vx > 0.65) //avoid except vel
	{
		cur_vx = last_vx;
	}
	delta_dead_reckon = (cur_vx + last_vx) / 2.0 * delta_time;
	delta_dead_reckon = abs(delta_dead_reckon);
}
void cartodom_correct::CalcCartodomOriDeltaDis() {
	float tmp_delta_x = 0.0;
	float tmp_delta_y = 0.0;
	tmp_delta_x = cur_cartodom.x - last_cartodom.x;
 	tmp_delta_y = cur_cartodom.y - last_cartodom.y;
	delta_cartodom = sqrt(pow(tmp_delta_x, 2) + pow(tmp_delta_y, 2));
}

robot_pos cartodom_correct::CorrectCartodom() {
	if((cur_cartodom.x > 7.5) && (cur_cartodom.y < 2.5) && (cur_vx > 0.3) && abs(delta_cartodom - delta_dead_reckon) > epsilon) //7.5 = 8.28 - 2*0.4 long line cartodom_x - offset
	{
		cartodom_dr.x = 0.0;
		cartodom_dr.y = cartodom_dr.y + delta_dead_reckon;
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

bool cartodom_correct::IsCartoException() {
	static float cur_delta_carto2dr = 0.0;
	static float last_delta_carto2dr = 0.0;
	static float lastlast_delta_carto2dr = 0.0;
	static bool carto_except_flag = false;

	bool compen_yaw_flag = (cur_cartodom.yaw > 0.785) && (cur_cartodom.yaw < 2.355);
	cur_delta_carto2dr = abs(delta_cartodom - delta_dead_reckon);
	
	if((cur_cartodom.x > 7.5) && (cur_cartodom.y < 2.5) && (compen_yaw_flag) && (cur_vx > 0.4) && (cur_delta_carto2dr > epsilon))
	{
		carto_except_flag = true;
	}

	if((cur_delta_carto2dr < normal_thd) && (last_delta_carto2dr < normal_thd) && (lastlast_delta_carto2dr < normal_thd))
	{
		carto_except_flag = false;	
	}

	last_delta_carto2dr = cur_delta_carto2dr;
	lastlast_delta_carto2dr = last_delta_carto2dr;

	carto_except = carto_except_flag;

	return carto_except_flag;

}

void cartodom_correct::CalcCurExceptState() {
	bool cur_isExcept = false;
	static bool last_isExcept = false;
	cur_isExcept = IsCartoException();

	if(cur_isExcept==false && last_isExcept==false)
	{
		except_phase = 0;//normal
		last_isExcept = cur_isExcept;
		return;
	}

	if(cur_isExcept==true && last_isExcept==false)
	{
		except_phase = 1; //inc edge
		last_isExcept = cur_isExcept;
		return;
	}

	if(cur_isExcept==true && last_isExcept==true)
	{
		except_phase = 2;//correct
		last_isExcept = cur_isExcept;
		return;
	}

	if(cur_isExcept==false && last_isExcept==true)
	{
		except_phase = 3; //dec edge
		last_isExcept = cur_isExcept;
		return;
	}


}



