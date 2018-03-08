#include "protect.h"

#define LASER_SAFE
//#define ULTRA_SAFE

bool node_shutdown = false;
void SigintHandler(int sig) {
	node_shutdown = true;
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "Anti_Collision_Node");
		
	int delay_cnt = 0;
	int CNT = 5;

	float laser_linear_vel_safe = 0.0;
	float laser_angular_vel_safe = 0.0;
	int laser_steer = 0;
	int laser_rect_encoder = 0;
	int laser_area_sts = 0;
	
	int ultra_area_sts = 0;
	float ultra_linear_vel_safe = 0.0;
	float ultra_angular_vel_safe = 0.0;
	int ultra_steer = 0;
	float coli_prob = 0.0;
	
	protector protectObj;		// init laser ultra and bumper anti collision obj
	protectObj.InitRectPolarVec();	// init the 3 rectangle area for laser obstacle encoder

	ros::Rate loop_rate(10);
	ROS_INFO("Start to detect the safety...");

	range_finder laser_property, ultra_property;
	safe_state laser_safe, ultra_safe;

	while (ros::ok())
	{	

		if(delay_cnt < CNT)
		{
			delay_cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		if(delay_cnt >= CNT)
		{
#ifdef LASER_SAFE		
			protectObj.CalcMinDis4LaserScan();	
 			laser_property.min_dis =  protectObj.min_scan;
			laser_property.min_index =  protectObj.min_scan_angle;

			laser_rect_encoder = protectObj.LaserRectEncoder();

			protectObj.CalcCrabLaserCA(laser_rect_encoder,laser_property, laser_safe);
			protectObj.PubLaserSafeVel(laser_safe, laser_rect_encoder);
#endif

#ifdef ULTRA_SAFE

			protectObj.CalcMinDis4Ultrosonic(protectObj.ultra_vec);
 			ultra_property.min_dis =  protectObj.min_ultra;
			ultra_property.min_index =  protectObj.min_index_ultra;

			protectObj.CalcCrabUltraCA(ultra_safe);
			protectObj.PubUltraSafeVel(ultra_safe);
#endif


			coli_prob = protectObj.IntegrateMultiInfo4Safety(&protectObj.advise_action);
			//cout<<"Intg coli_prob: "<< coli_prob<< endl;
			protectObj.Intg4EnvSecure();	
			protectObj.security_pub4env.publish(protectObj.env_secure);

			if(node_shutdown == true)
			{
				ros::shutdown();
			}

			ros::spinOnce();
			loop_rate.sleep();

		}
	

	}

	return 0;
}

