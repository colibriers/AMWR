#include "protect.h"

protector::protector()
{
	memset(scan4safty, 1.0, SCAN4SAFE_NUM);
	memset(ultra_vec, 1.0, ULTRA_NUM);
	bumper_signal = false;
	
	v = 0.0;
	vth = 0.0;
	advise_action = HOLD_STATE;

	collision_flag = false;
	colision_prob = 0.0;
	
	laser_unsafe_prob = 0.0;
	ultra_unsafe_prob = 0.0;
	
	min_scan = 20.0;
	min_ultra = 6.5;

	min_index_scan = 105;
	min_scan_angle = 90;
	min_index_ultra = 0;	
	
	scan_sub4safe = nh_safety.subscribe<sensor_msgs::LaserScan>("/scan", 1, &protector::ScanSafeCallBack, this);

	ultra_sub4safe = nh_safety.subscribe<colibri_aiv::Ultrasonic>("/ultrasonic", 1, &protector::UltraSafeCallBack, this);
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
		env_secure.bumper_min_dis = 1.0;
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

void protector::UltraSafeCallBack(const colibri_aiv::Ultrasonic::ConstPtr& ultra4safe)
{
	ultra_vec[0] = (ultra4safe->ultrasonic1) / 100.0; //for front ultra unit cm to m
	ultra_vec[1] = (ultra4safe->ultrasonic2) / 100.0;
	ultra_vec[2] = (ultra4safe->ultrasonic3) / 100.0;
	ultra_vec[3] = (ultra4safe->ultrasonic4) / 100.0;

	ultra_vec[4] = (ultra4safe->ultrasonic5) / 100.0;	// for rear ultra
	ultra_vec[5] = (ultra4safe->ultrasonic6) / 100.0;
	ultra_vec[6] = (ultra4safe->ultrasonic7) / 100.0;
	ultra_vec[7] = (ultra4safe->ultrasonic8) / 100.0;
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



