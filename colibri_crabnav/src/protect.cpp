#include "protect.h"

protector::protector()
{
	for(int i = 0; i < SCAN4SAFE_NUM; i++)
	{
		scan4safty[i] = 20.0;
	}
	
	for(int j = 0; j < ULTRA_NUM; j++)
	{
		ultra_vec[j] = 3.0;
	}

	bumper_signal = false;
	
	v = 0.0;
	vth = 0.0;
	advise_action = HOLD_STATE;

	collision_flag = false;
	colision_prob = 0.0;
	
	laser_unsafe_prob = 0.0;
	ultra_unsafe_prob = 0.0;
	
	min_scan = 20.0;
	min_ultra = 3.0;

	min_index_scan = 105;
	min_scan_angle = 90;
	min_index_ultra = 0;	

	vec_scan4safe.reserve(SCAN4SAFE_NUM);
	vec_rect_polar.reserve(SAFE_RECT_NUM);

	rectangle[0].width = 0.6;
	rectangle[0].height = 0.5;
	rectangle[1].width = 0.6;
	rectangle[1].height = 1.6;
	rectangle[2].width = 0.6;
	rectangle[2].height = 3.0;

	scan_sub4safe = nh_safety.subscribe<sensor_msgs::LaserScan>("/scan", 1, &protector::CrabScanSafeCallBack, this);

	ultra_sub4safe = nh_safety.subscribe<colibri_ultra::Ultrasonic>("/ultra_front", 1, &protector::UltraSafeCallBack, this);
	bumper_sub4safe = nh_safety.subscribe<colibri_aiv::Bumper>("/bumper", 1, &protector::BumperSafeCallBack, this);
	odom_sub4safe = nh_safety.subscribe<nav_msgs::Odometry>("/odom", 1, &protector::OdomSafeCallBack, this);

	security_pub4env = nh_safety.advertise<colibri_msgs::EnvSecurity>("/env_secure", 1);
	security_pub4laser = nh_safety.advertise<colibri_msgs::SafeVel>("/laser_safe_vel", 1);
	security_pub4ultra = nh_safety.advertise<colibri_msgs::SafeVel>("/ultra_safe_vel", 1);

}
protector::~protector()
{

}

/*	
*   void CalcMinDis4LaserScan(float* laser_vec) 
*   Description: Calc the min dis , index min_scan and colision prob in the scan laser data
*   Results in min_scan, min_index_scan
*/
void protector::CalcMinDis4LaserScan(float* laser_vec)
{
	float tmp_range = *laser_vec;
	int index_mindis = 0;
	
	for(int i = 0; i < SCAN4SAFE_NUM; i++)
	{
		if(*(laser_vec + i) <= tmp_range)
		{
			tmp_range = *(laser_vec + i);
			index_mindis = i;
		}
	}
	
	min_scan = tmp_range;
	min_index_scan = index_mindis;
	min_scan_angle = index_mindis - 15; //-15  is the laser safty start angle

	if(tmp_range <= LASER_SAFE_MIN)
	{
		laser_unsafe_prob = 1.0;
	}
	else if(tmp_range <= LASER_SAFE_MAX)
	{
		laser_unsafe_prob = (LASER_SAFE_MAX - tmp_range) / LASER_SAFE_MAX;
	}
	else
	{
		laser_unsafe_prob = 0.0;
	}

}

void protector::CalcMinDis4LaserScan(void)
{
	
	vector<float>::iterator min_it = min_element(vec_scan4safe.begin(), vec_scan4safe.end());
	
	min_scan = *min_it;
	min_index_scan = distance(vec_scan4safe.begin(), min_it);
	min_scan_angle = min_index_scan - 15; //-15  is the laser safty start angle

	if((*min_it) <= LASER_SAFE_MIN)
	{
		laser_unsafe_prob = 1.0;
	}
	else if((*min_it) <= LASER_SAFE_MAX)
	{
		laser_unsafe_prob = (LASER_SAFE_MAX - (*min_it)) / LASER_SAFE_MAX;
	}
	else
	{
		laser_unsafe_prob = 0.0;
	}

}


/*	
*   void CalcMinDis4Ultrosonic(float* ultra_vec) 
*   Description: Calc the min dis , index min_ultra and colision prob in the ultra data
*   results in min_ultra, min_index_ultra
*/
void protector::CalcMinDis4Ultrosonic(float* ultra_vec)
{
	float tmp_range = *ultra_vec;
	unsigned int index_mindis = 0;
	
	for(int i = 0; i < ULTRA_NUM/2; i++)		// for ca the front 4 ultra should be concerned
	{
		if(*(ultra_vec + i) <= tmp_range)
		{
			tmp_range = *(ultra_vec + i);
			index_mindis = i;
		}
	}
	
	min_ultra = tmp_range;
	min_index_ultra = index_mindis + 1;		// +1 for vec from 0 but ultra start from 1 for phiscically

	if(tmp_range <= ULTRA_SAFE_MIN)
	{
		ultra_unsafe_prob = 1.0;
	}
	else if(tmp_range <= ULTRA_SAFE_MAX)
	{
		ultra_unsafe_prob = (ULTRA_SAFE_MAX - tmp_range) / (ULTRA_SAFE_MAX - ULTRA_SAFE_MIN);
	}
	else
	{
		ultra_unsafe_prob = 0.0;
	}

}

bool protector::CalcLaserSafeVelThd(float &min_scan, int &min_scan_ang, int &steer, float* linear_safe, float* angular_safe)
{
	if(min_scan > LASER_SAFE_DIS1)
	{
		*linear_safe = V_MAX;
		*angular_safe = THETA_V_MAX;
		steer = 0;	//0 means to steer any dir
		return false;
	}
	else if(min_scan > LASER_SAFE_DIS2)
	{
	
		if(abs(min_scan_ang - 90) <= LASER_SAFE_ANG1) //limit the vel in the angle 50 scope
		{
			*angular_safe = THETA_V_MAX;
			*linear_safe = LINEAR_SAFE_MAX + (V_MAX - LINEAR_SAFE_MAX) * (min_scan - LASER_SAFE_DIS2)/(LASER_SAFE_DIS1 - LASER_SAFE_DIS2);
			steer = 0;
		}
		else
		{
			if(min_scan_ang > 90)
			{
				*angular_safe = ANGULAR_SAFE_MAX;
				*linear_safe = V_MAX;
				steer = 0;
			}
			else if(min_scan_ang < 90)
			{
				*angular_safe = ANGULAR_SAFE_MAX;
				*linear_safe = V_MAX;
				steer = 0;
			}
			else
			{
			
			}
			
		}

	}
	else if(min_scan > LASER_SAFE_DIS3)
	{	

		if(abs(min_scan_ang - 90) <= LASER_SAFE_ANG2)
		{
			*angular_safe = THETA_V_MAX;
			*linear_safe = LINEAR_SAFE_MAX * (min_scan - LASER_SAFE_DIS3)/(LASER_SAFE_DIS2 - LASER_SAFE_DIS3);
			steer = 0;	
		}
		else
		{
			if(min_scan_ang > 90)
			{
				*angular_safe = -THETA_V_MAX;
				*linear_safe = LINEAR_SAFE_MAX;
				steer = -1;	
			}
			else if(min_scan_ang < 90)
			{
				*angular_safe = THETA_V_MAX;
				*linear_safe = LINEAR_SAFE_MAX;
				steer = 1;	
			}
			else
			{

			}
			
		}		
		
	}
	else if(min_scan > LASER_SAFE_DIS4)
	{
		*angular_safe = ANGULAR_SAFE_MAX;
		*linear_safe = LINEAR_STOP;
		steer = 0;	
	}
	else
	{
		*angular_safe = ANGULAR_STOP;
		*linear_safe = LINEAR_STOP;
		steer = 0;	

	}

	return true;	
		
}

bool protector::CalcUltraSafeVelThd(float &min_ultra, unsigned int &min_ultra_index, int &steer, float* linear_safe, float* angular_safe)
{
	if(min_ultra > ULTRA_SAFE_DIS1)
	{
		*linear_safe = V_MAX;
		*angular_safe = THETA_V_MAX;
		steer = 0;
		return false;
	}
	else if(min_ultra > ULTRA_SAFE_DIS2)
	{
		*linear_safe = LINEAR_SAFE_MAX * (min_ultra - ULTRA_SAFE_DIS2) / (ULTRA_SAFE_DIS1 - ULTRA_SAFE_DIS2);
		if(min_ultra_index == 4)
		{
			*angular_safe = -THETA_V_MAX;
			steer = -1;
		}
		else if(min_ultra_index == 1)
		{
			*angular_safe = THETA_V_MAX;	
			steer = 1;
		}
		else
		{
			*angular_safe = THETA_V_MAX;	
			steer = 0;
		}	

	}
	else // ultra dis < 0.36 must stop moving
	{	
		*angular_safe = THETA_V_MAX;
		*linear_safe = LINEAR_STOP;
		steer = 0;
	}

	return true;	

}

bool protector::CalcCrabSafeVelThd(int &laser_encoder,float  &min_scan, int &min_scan_ang, float *linear_safe, float* angular_safe)
{

	switch (laser_encoder)
	{
		case 4:		// in 3m rectangle
			*linear_safe = CRAB_MID_LINEAR_VEL;
			*angular_safe = CRAB_MAX_ANGULAR_VEL;				
			break;
			
		case 6:		//
			*linear_safe = CRAB_MIN_LINEAR_VEL;
			*angular_safe = CRAB_MAX_ANGULAR_VEL;	
			break;
			
		case 7:		// in 0.5 rectangle
			*linear_safe = CRAB_STOP_LINEAR_VEL;
			*angular_safe = CRAB_MAX_ANGULAR_VEL;	
			break;	
			
		case 0:
			if(min_scan < 0.35 && (min_scan_ang > 180 || min_scan_ang < 0))
			{
				*linear_safe = CRAB_STOP_LINEAR_VEL;
				*angular_safe = CRAB_STOP_ANGULAR_VEL;
			}
			else
			{
				*linear_safe = CRAB_MAX_LINEAR_VEL;
				*angular_safe = CRAB_MAX_ANGULAR_VEL;
			}
			break;
			
		default:
			*linear_safe = CRAB_MID_LINEAR_VEL;
			*angular_safe = CRAB_MID_ANGULAR_VEL;	
			break;

	}

	return true;
		
}


bool protector::CalcLaserCA(float	&min_scan, int &min_scan_ang, int &steer, float *linear_safe, float* angular_safe, int &area_state)
{
	float tmp_x = 0.0;
	float tmp_y = 0.0;
	bool inRecFlag = false;
	float window_x = LASER_CA_WIDTH;
	float window_y = LASER_CA_HEIGHT;
	float dec_radius = 0.94; //sqrt(pow(LASER_CA_WIDTH/2.0, 2) + pow(LASER_CA_HEIGHT, 2));
	Polar2Decare(min_scan, min_scan_ang, tmp_x, tmp_y);
	inRecFlag = LocateInRecArea(window_x, window_y, tmp_x, tmp_y);

	if(inRecFlag == true)
	{
		if(min_scan >= LASER_ROT_RADIUS)
		{
			*linear_safe = V_MAX * (min_scan - (LASER_ROT_RADIUS+LASER_STOP_RADIUS)/2.0)/(dec_radius - LASER_ROT_RADIUS);
			*angular_safe = THETA_V_MAX;
			steer = 0;
			area_state = 1;
		}
		else if(min_scan > LASER_STOP_RADIUS)
		{
			*linear_safe = LINEAR_STOP;
			*angular_safe = THETA_V_MAX;
			if(min_scan_ang > 135)
			{
				steer = -1;
			}
			else if(min_scan_ang < 45)
			{
				steer = 1;
			}
			else
			{
				steer = 0;
			}
			area_state = 2;
		}
		else
		{
			*linear_safe = LINEAR_STOP;
			*angular_safe = ANGULAR_STOP;
			steer = 0;
			area_state = 3;

		}

		return true;

	}
	else
	{
		*linear_safe = V_MAX;
		*angular_safe = THETA_V_MAX;
		steer = 0;
		area_state = 0;
		return false;
	}
	

}
bool protector::CalcUltraCA(float &min_ultra, unsigned int &min_ultra_index, int &steer, float* linear_safe, float* angular_safe, int &area_state)
{
	if(min_ultra > ULTRA_CA_DEC)
	{
		*linear_safe = V_MAX;
		*angular_safe = THETA_V_MAX;
		steer = 0;
		area_state = 0;
		return false;	
	}
	else if(min_ultra > ULTRA_ROT_RADIUS)
	{
		*linear_safe = V_MAX * (min_ultra - ULTRA_ROT_RADIUS)/(ULTRA_CA_DEC - ULTRA_ROT_RADIUS);
		*angular_safe = THETA_V_MAX;
		steer = 0;
		area_state = 1;
	}
	else if(min_ultra > ULTRA_STOP_RADIUS)
	{
		*linear_safe = LINEAR_STOP;
		*angular_safe = THETA_V_MAX;
		if(min_ultra_index == 1)
		{
			steer = 1;
		}
		else if(min_ultra_index == 4)
		{
			steer = -1;
		}
		else
		{
			steer = 0;
		}
			
		area_state = 2;
	}
	else
	{
		*linear_safe = LINEAR_STOP;
		*angular_safe = ANGULAR_STOP;
		steer = 0;
		area_state = 3;

	}


}

bool protector::CalcCrabUltraCA(range_finder & ultra, safe_state & safe_ultra)
{
	if(ultra.min_dis > ULTRA_CA_DEC)
	{
		safe_ultra.linear_up_vel = CRAB_MAX_LINEAR_VEL;
		safe_ultra.angular_up_vel = CRAB_MAX_ANGULAR_VEL;
		safe_ultra.steer = 0;
		safe_ultra.area_state = 0;
		return false;	
	}
	else if(ultra.min_dis > ULTRA_ROT_RADIUS)
	{
		safe_ultra.linear_up_vel = CRAB_MIN_LINEAR_VEL;
		safe_ultra.angular_up_vel = CRAB_MAX_ANGULAR_VEL;
		safe_ultra.steer= 0;
		safe_ultra.area_state = 1;
	}
	else if(ultra.min_dis > ULTRA_STOP_RADIUS)
	{
		safe_ultra.linear_up_vel = CRAB_STOP_LINEAR_VEL;
		safe_ultra.angular_up_vel = CRAB_STOP_ANGULAR_VEL;
		if(ultra.min_index == 1)
		{
			safe_ultra.steer = 1;
		}
		else if(ultra.min_index == 4)
		{
			safe_ultra.steer = -1;
		}
		else
		{
			safe_ultra.steer = 0;
		}
			
		safe_ultra.area_state = 2;
	}
	else
	{
		safe_ultra.linear_up_vel = CRAB_STOP_LINEAR_VEL;
		safe_ultra.angular_up_vel = CRAB_STOP_ANGULAR_VEL;
		safe_ultra.steer = 0;
		safe_ultra.area_state = 3;

	}


}

bool protector::CalcCrabUltraCA(safe_state & safe_ultra)
{
	static int confirm_cnt = 0;
	float tmp_ultra_min = (ultra_vec[2]>ultra_vec[1]) ? ultra_vec[1]:ultra_vec[2];
		
	if(tmp_ultra_min < ULTRA_STOP_DIS)
	{
		confirm_cnt++;
	}
	else
	{
		confirm_cnt--;
		if(confirm_cnt < 0)
		{
			confirm_cnt = 0;
		}
	}

	if(confirm_cnt > 2)
	{
		confirm_cnt = 2;
		safe_ultra.linear_up_vel = 0;
		safe_ultra.angular_up_vel = 0;
		safe_ultra.steer = 0;
		safe_ultra.area_state = 0;
		return true;
	}
	else
	{
		safe_ultra.linear_up_vel = CRAB_MAX_LINEAR_VEL;
		safe_ultra.angular_up_vel = CRAB_MAX_ANGULAR_VEL;
		safe_ultra.steer = 0;
		safe_ultra.area_state = 0;
		return false;	
	}
	
	
}


void protector::PubLaserSafeVel(safe_state & laser_safe, int &laser_rect_encoder)
{
	// publish the laser module safe vel
	laser_safe_vel.header.stamp = ros::Time::now();
	laser_safe_vel.header.frame_id = "laser";

	if((laser_safe.linear_up_vel == LINEAR_STOP) && (laser_safe.angular_up_vel == ANGULAR_STOP) && (laser_safe.steer == 0))
	{
		laser_safe_vel.stop.data = true;
	}
	else
	{
		laser_safe_vel.stop.data = false;
	}				
	laser_safe_vel.linear_safe_thd = laser_safe.linear_up_vel ;
	laser_safe_vel.area_status = laser_safe.area_state;
	laser_safe_vel.steer = 0;
	laser_safe_vel.angular_safe_thd = laser_safe.angular_up_vel ;
	laser_safe_vel.rsvd = laser_rect_encoder;

	security_pub4laser.publish(laser_safe_vel);

}


void protector::PubUltraSafeVel(safe_state & ultra_safe)
{
	// publish the ultra module safe vel
	ultra_safe_vel.header.stamp = ros::Time::now();
	ultra_safe_vel.header.frame_id = "ultra";
	
	if((ultra_safe.linear_up_vel == LINEAR_STOP) && (ultra_safe.angular_up_vel == ANGULAR_STOP) && (ultra_safe.steer == 0))
	{
		ultra_safe_vel.stop.data = true;
	}
	else
	{
		ultra_safe_vel.stop.data = false;
	}				
	ultra_safe_vel.linear_safe_thd = ultra_safe.linear_up_vel;
	ultra_safe_vel.area_status = ultra_safe.area_state;
	ultra_safe_vel.steer = ultra_safe.steer;
	ultra_safe_vel.angular_safe_thd = ultra_safe.angular_up_vel;
	ultra_safe_vel.rsvd = 0.0;
	
	security_pub4ultra.publish(ultra_safe_vel);

}


bool protector::CalcCrabLaserCA(int &laser_encoder, range_finder & laser, safe_state & safe_laser)
{

	if(laser_encoder != 0)
	{
		switch(laser_encoder)
		{
			case 4: 	// in 3m rectangle
				safe_laser.linear_up_vel= CRAB_MID_LINEAR_VEL;
				safe_laser.angular_up_vel = CRAB_MAX_ANGULAR_VEL;
				safe_laser.area_state = 1;
				safe_laser.steer = 0;
				break;
				
			case 6: 	// in 1.6m 
				safe_laser.linear_up_vel= CRAB_MIN_LINEAR_VEL;
				safe_laser.angular_up_vel = CRAB_MAX_ANGULAR_VEL;
				safe_laser.area_state = 2;
				safe_laser.steer = 0;
				break;
				
			case 7: 	// in 0.5 rectangle
				safe_laser.linear_up_vel= CRAB_STOP_LINEAR_VEL;
				safe_laser.angular_up_vel = CRAB_STOP_ANGULAR_VEL;
				safe_laser.area_state = 3;
				safe_laser.steer = 0;
				break;
			case 0: 	// out of the rectangle
				safe_laser.linear_up_vel= CRAB_MAX_LINEAR_VEL;
				safe_laser.angular_up_vel = CRAB_MAX_ANGULAR_VEL;
				safe_laser.area_state = 0;
				safe_laser.steer = 0;
				break;	
			default:
				safe_laser.linear_up_vel= CRAB_STOP_LINEAR_VEL;
				safe_laser.angular_up_vel = CRAB_STOP_ANGULAR_VEL;
				safe_laser.area_state = 100;
				safe_laser.steer = 0;			
				break;

		}

		return true;

	}
	else
	{
		if(laser.min_dis < 0.35 && (laser.min_index > 180 || laser.min_index < 0))
		{
			safe_laser.linear_up_vel= CRAB_STOP_LINEAR_VEL;
			safe_laser.angular_up_vel = CRAB_STOP_ANGULAR_VEL;
			safe_laser.area_state = 4;
			safe_laser.steer = 0;
			return true;
		}
		else
		{
			safe_laser.linear_up_vel= CRAB_MAX_LINEAR_VEL;
			safe_laser.angular_up_vel = CRAB_MAX_ANGULAR_VEL;
			safe_laser.area_state = 0;
			if(laser.min_index > 135)
			{
				safe_laser.steer = -1;
			}
			else if(laser.min_index < 45)
			{
				safe_laser.steer = 1;

			}
			else
			{			
				safe_laser.steer = 0;
			}

			return false;
		}

	}

}


void protector::Polar2Decare(float  &min_scan, int &min_scan_ang,float &x, float &y)
{
	x = min_scan * cos(min_scan_ang * DEG2RAD);
	y = min_scan * sin(min_scan_ang * DEG2RAD);
}

bool protector::LocateInRecArea(float &rec_width, float &rec_height, float &x, float &y)
{
	if((abs(x) < rec_width/2.0) && (y <= rec_height) && (y > 0.0))
	{
		return true;
	}
	else
	{
		return false;
	}
}

//Calc the rectangle bound map into a polar frame theta-rho relation for 180 points
map<int, float> protector::Rect2Polar(float &width, float &height)
{
	map<int, float> rho;
	float tmp_slope_dis = 0.0;
	float tmp_w = width / 2.0;
	float tmp_h = height;
	int front_ang = floor(RAD2DEG * atan2(tmp_w, tmp_h));
	for(int i = 0; i <= (90 - front_ang); i++)
	{
		tmp_slope_dis = (width / 2.0) / cos(i * DEG2RAD);
		rho.insert(pair<int, float>(i, tmp_slope_dis)); 
	}
	
	for(int j = 90 - front_ang + 1; j <= 90; j++)
	{
		tmp_slope_dis = height / cos(PI / 2.0 - j * DEG2RAD);
		rho.insert(pair<int, float>(j, tmp_slope_dis)); 
	}

	for(int k = 91; k <= 180; k++)
	{
		tmp_slope_dis = rho[180 - k];
		rho.insert(pair<int, float>(k, tmp_slope_dis)); 
	}

	return rho;

}


bool protector::PointInRect(map<int, float> &rec2polar)
{
	float tmp_diff = 0.0;
	vector<float> differ;
	for(int i = 0 ; i <= 180; i++)
	{
		tmp_diff = vec_scan4safe.at(i+15) - rec2polar[i];
		differ.push_back(tmp_diff);
	}

	vector<float>::const_iterator min_it = min_element(differ.begin(), differ.end());

	if(*min_it < - 0.001)
	{
		return true;
	}
	else
	{
		return false;
	}


}


bool protector::CalcSafeLinearVel(float &ctrl_vel, float &linear_thd, float* safe_linear_vel)
{
	if(0 == linear_thd)
	{
		*safe_linear_vel = 0.0;
	}
	else
	{
		if(ctrl_vel > linear_thd)
		{
			*safe_linear_vel = linear_thd;
		}
		else
		{
			*safe_linear_vel = ctrl_vel;
			return false;
		}
			
	}
	
	return true;
}

bool protector::CalcSafeAngularVel(float &ctrl_vel, int &steer, float &angular_thd, float* safe_angular_vel)
{
	if(steer == 0)
	{
		if(abs(ctrl_vel) <= abs(angular_thd))
		{
			*safe_angular_vel = ctrl_vel;
			return false;
		}
		else if(ctrl_vel > abs(angular_thd))
		{
			*safe_angular_vel = abs(angular_thd);
		}
		else if(ctrl_vel < (-1.0 * abs(angular_thd)))
		{
			*safe_angular_vel = -1.0 * abs(angular_thd);

		}
		else
		{
			*safe_angular_vel = 0.0;
		}
	}
	else
	{
		if(angular_thd > 0.0)
		{
			if(ctrl_vel > angular_thd)
			{
				*safe_angular_vel = angular_thd;
			}
			else if(ctrl_vel < 0.0)
			{
				*safe_angular_vel = 0.0;
			}
			else
			{
				*safe_angular_vel = ctrl_vel;
				return false;
			}
		
		}
		else if(angular_thd < 0.0)
		{
			if(ctrl_vel < angular_thd)
			{
				*safe_angular_vel = angular_thd;
			}
			else if(ctrl_vel > 0.0)
			{
				*safe_angular_vel = 0.0;
			}
			else
			{
				*safe_angular_vel = ctrl_vel;
				return false;
			}
				
		}
		else
		{
			*safe_angular_vel = 0.0;
		}

	}
	
	return true;

}

/*	
*   float IntegrateMultiInfo4Safety(enum_act4safe* advise_action)
*   Description: Using MultiInfo: laser prob, ultra prob and bumper to obtain comprehensive colision prob and remmend action
*   return  colision_prob  and results in advise_action
*/
float protector::IntegrateMultiInfo4Safety(enum_act4safe* advise_action)
{

	enum_bearing obs_dir = OMNI;
	
	if(bumper_signal == true || ultra_unsafe_prob > LEVEL_1_PROB || laser_unsafe_prob > LEVEL_1_PROB)
	{
		collision_flag = true;
		colision_prob = 1.0;
		*advise_action = STOP;
	}
	else
	{
		collision_flag = false;
		if(bumper_signal == false)
		{
			colision_prob = MAX(laser_unsafe_prob, ultra_unsafe_prob);
			if(colision_prob == laser_unsafe_prob)
			{
				if(min_index_scan <= (SCAN4SAFE_NUM - 1)/2)
				{
					obs_dir = RIGHT;
				}
				else
				{
					obs_dir = LEFT;
				}			
			}
			else
			{
				if(min_index_ultra<= ULTRA_NUM / 4)	// for front collision safty ,shoule be 4/2 ->8/4 ,right hand law for order
				{
					obs_dir = RIGHT;
				}
				else
				{
					obs_dir = LEFT;
				}	
			}
		}
		
		if(colision_prob <= LEVEL_3_PROB)
		{
			*advise_action = HOLD_STATE;
		}
		else if(colision_prob <= LEVEL_2_PROB)
		{
			*advise_action = DEC_VEL;
		}
		else
		{
			if(obs_dir == RIGHT)
			{
				*advise_action = TURN_LEFT;
			}
			else if(obs_dir == LEFT)
			{
				*advise_action = TURN_RIGHT;
			}
			else
			{
				*advise_action = STOP;
			}			
		}

	}

	return colision_prob;
}

/*	
*   bool StopMovingInForce(void)
*   Description: Using IntergrateMultInfo 's comprehensive colision prob to output stop flag
*   return  stop logic true or false
*/
bool protector::StopMovingInForce(void)
{
	if(colision_prob >= UNSAFE_PROB)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}

/*	
*   bool Detect4ExceptHighVel(float* v, float* vth)
*   Description: Detect for odom v and vth for exception
*   return  vel exception flag
*/
bool protector::Detect4ExceptHighVel(float* v, float* vth)
{
	if(*v >= V_EXCPT_VAL || *vth >= VTH_EXCPT_VAL)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void protector::Intg4EnvSecure(void)
{

	env_secure.header.stamp = ros::Time::now();
	env_secure.header.frame_id = "robot";
	
	env_secure.collision.data = collision_flag;
	env_secure.collision_prob = colision_prob;
	
	env_secure.laser_min_dis = min_scan;
	env_secure.laser_min_angle = min_scan_angle;
	env_secure.laser_prob = laser_unsafe_prob;
	
	env_secure.ultra_min_dis = min_ultra;
	env_secure.ultra_min_index = min_index_ultra;
	env_secure.ultra_prob = ultra_unsafe_prob;

	if(bumper_signal == true)
	{
		env_secure.bumper_min_dis = 0.0;
		env_secure.bumper_prob = 1.0 ;
	}
	else
	{
		env_secure.bumper_min_dis = 3.0;
		env_secure.bumper_prob = 0.0;
	}
			
	env_secure.bumper_min_index = 1;

}



void protector::ScanSafeCallBack(const sensor_msgs::LaserScan::ConstPtr& scan4safe)
{
	int j = 31;	//from laser ray at  -15deg starts : 31 = 15/0.5+1;
	for(int i = 0; i < SCAN4SAFE_NUM; i++)
	{
		scan4safty[i] = scan4safe->ranges[j];
		
		if(scan4safty[i] <= 0.08)
		{
			scan4safty[i] = (0.2*scan4safe->ranges[j-4] + 0.3*scan4safe->ranges[j-1] + 0.3*scan4safe->ranges[j+1] + 0.2*scan4safe->ranges[j+4]) ;
		}
		
		j = j + 2; 
	}

}

int protector::LaserRectEncoder(void)
{
	int encoder = 0;

	for(int i = 0; i < SAFE_RECT_NUM; i++)
	{
		map<int, float> tmp = vec_rect_polar[i];
		bool in_rect_flag = PointInRect(tmp);
		if(in_rect_flag)
		{
			rect_encoder.set(i);
		}
		else
		{
			rect_encoder.reset(i);
		}
		tmp.clear();
	}

	encoder = int (rect_encoder.to_ulong());

	return encoder;

}

void protector::InitRectPolarVec(void)
{
	map<int, float> tmp_r2p;

	for(int i = 0; i < SAFE_RECT_NUM; i++)
	{
		tmp_r2p = Rect2Polar(rectangle[i].width, rectangle[i].height);
		vec_rect_polar.push_back(tmp_r2p);
		tmp_r2p.clear();
		map<int, float> ().swap(tmp_r2p);
	}

}

/*
void protector::CrabScanSafeCallBack(const sensor_msgs::LaserScan::ConstPtr& scan4safe)
{
	float tmp_scan = 20.0;
	vec_scan4safe.clear();
	vector<float> ().swap(vec_scan4safe);
	
	int j = 31;	//from laser ray at  -15deg starts : 31 = 15/0.5+1;
	for(int i = 0; i < SCAN4SAFE_NUM; i++)
	{
		tmp_scan = scan4safe->ranges[j];
		
		if(tmp_scan <= 0.08)
		{
			tmp_scan = (0.2*scan4safe->ranges[j-4] + 0.3*scan4safe->ranges[j-1] + 0.3*scan4safe->ranges[j+1] + 0.2*scan4safe->ranges[j+4]) ;
		}

		vec_scan4safe.push_back(tmp_scan);
		
		j = j + 2; 
	}

}
*/

void protector::CrabScanSafeCallBack(const sensor_msgs::LaserScan::ConstPtr& scan4safe)
{
	float tmp_scan = 20.0;
	vec_scan4safe.clear();
	vector<float> ().swap(vec_scan4safe);
	
	int j = 31;	//from laser ray at  -15deg starts : 31 = 15/0.5+1;
	for(int i = 0; i < SCAN4SAFE_NUM; i++)
	{
		tmp_scan = scan4safe->ranges[j];
		
		if(tmp_scan <= 0.06)
		{
			tmp_scan = 20.0;
		}

		vec_scan4safe.push_back(tmp_scan);
		
		j = j + 2; 
	}

}



void protector::UltraSafeCallBack(const colibri_ultra::Ultrasonic::ConstPtr& ultra4safe)
{
	ultra_vec[0] = (ultra4safe->ultra_1); //for front ultra unit cm to m
	ultra_vec[1] = (ultra4safe->ultra_2);
	ultra_vec[2] = (ultra4safe->ultra_3);
	ultra_vec[3] = (ultra4safe->ultra_4);

}

void protector::BumperSafeCallBack(const colibri_aiv::Bumper::ConstPtr& bumper4safe)
{	
	bumper_signal = bumper4safe->bumper.data;	
}

void protector::OdomSafeCallBack(const nav_msgs::Odometry::ConstPtr& odom4safe)
{
	v = odom4safe->twist.twist.linear.x;
	vth = odom4safe->twist.twist.angular.z;	
}



