#include "protect.h"

#define LASER_SAFE
#define ULTRA_SAFE

bool node_shutdown = false;
void SigintHandler(int sig) {
	node_shutdown = true;
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "Anti_Collision_Node");
		
	int laser_rect_encoder = 0;
	
	protector protectObj(20.0, 3.0);		// init laser ultra and bumper anti collision obj
	protectObj.InitRectPolarVec();	// init the 3 rectangle area for laser obstacle encoder

	ros::Rate loop_rate(10);
	ROS_INFO("Start to detect the safety...");

	range_finder laser_property, ultra_property;
	safe_state laser_safe, ultra_safe;

	while (ros::ok()) {			
		if(protectObj.ObjLaser_.update_flag_) {
			
#ifdef LASER_SAFE
			protectObj.ObjLaser_.update_flag_ = false;

			protectObj.CalcMinDis4LaserScan();	
 			laser_property.min_dis =  protectObj.ObjLaser_.min_data_;
			laser_property.min_index =  protectObj.ObjLaser_.min_index_;

			laser_rect_encoder = protectObj.LaserRectEncoder();

			protectObj.CalcCrabLaserCA(laser_rect_encoder,laser_property, laser_safe);
			protectObj.PubLaserSafeVel(laser_safe, laser_rect_encoder);
#endif
			}
		
		if(protectObj.ObjUltra_.update_flag_) {

#ifdef ULTRA_SAFE
			protectObj.ObjUltra_.update_flag_ = false;

			protectObj.CalcMinDis4Ultrosonic();
 			ultra_property.min_dis =  protectObj.ObjUltra_.min_data_;
			ultra_property.min_index =  protectObj.ObjUltra_.min_index_;

			protectObj.CalcCrabUltraCA(ultra_safe);
			protectObj.PubUltraSafeVel(ultra_safe);
#endif
			}

			protectObj.IntegrateMultiInfo4Safety();
			cout<<"Intg laser_rect_encoder: "<< laser_rect_encoder<< endl;
			protectObj.Intg4EnvSecure();	
			protectObj.security_pub4env_.publish(protectObj.env_secure_);

			if(node_shutdown == true) {
				ros::shutdown();
			}

			ros::spinOnce();
			loop_rate.sleep();

	}

	return 0;
}

