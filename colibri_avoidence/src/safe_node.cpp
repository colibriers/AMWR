#include "protect.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Anti_collision_node");
	
	protector protectObj;
	
	ros::Rate loop_rate(10);
	
	int delay_cnt = 0;
	float coli_prob = 0.0;

	ROS_INFO("Start to detect the safety...");

	float laser_linear_vel_safe = 0.0;
	float laser_angular_vel_safe = 0.0;
	int laser_steer = 0;

	float ultra_linear_vel_safe = 0.0;
	float ultra_angular_vel_safe = 0.0;
	int ultra_steer = 0;

	float ultra_unit[4] = {0,0,0,0};

	//ofstream  file1; 
	//file1.open ("ultra.txt");
	//int rec_max = 1800;
	//int cnt = 0;


	int laser_area_sts = 0;
	int ultra_area_sts = 0;
	while (ros::ok())
	{	
		if(delay_cnt < DELAY_CNT_MAX)
		{
			delay_cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		if(delay_cnt >= DELAY_CNT_MAX)
		{
			protectObj.CalcMinDis4LaserScan(protectObj.scan4safty);	
			cout<<"protectObj.min_scan: "<< protectObj.min_scan << endl;
			cout<<"protectObj.min_scan_angle: "<< protectObj.min_scan_angle<< endl;
			cout<<"protectObj.laser_prob: "<< protectObj.laser_unsafe_prob<< endl;
			
			protectObj.CalcMinDis4Ultrosonic(protectObj.ultra_vec);
			cout<<"protectObj.min_ultra: "<< protectObj.min_ultra<< endl;
			cout<<"protectObj.min_index_ultra: "<< protectObj.min_index_ultra<< endl;
			cout<<"protectObj.ultra_unsafe_prob: "<< protectObj.ultra_unsafe_prob<< endl;
 			 
			
			coli_prob = protectObj.IntegrateMultiInfo4Safety(&protectObj.advise_action);
			cout<<"Intg coli_prob: "<< coli_prob<< endl;

			protectObj.Intg4EnvSecure();	
			protectObj.security_pub4env.publish(protectObj.env_secure);

	/*		
			protectObj.CalcLaserSafeVelThd(protectObj.min_scan , protectObj.min_scan_angle, laser_steer, &laser_linear_vel_safe, &laser_angular_vel_safe);
			cout<<"laser_linear_vel_safe_thd: "<< laser_linear_vel_safe<< endl;
			cout<<"laser_angular_vel_safe_thd: "<< laser_angular_vel_safe<< endl;
			cout<<"laser_steer_thd: "<< laser_steer<< endl;

			protectObj.CalcUltraSafeVelThd(protectObj.min_ultra, protectObj.min_index_ultra, ultra_steer, &ultra_linear_vel_safe, &ultra_angular_vel_safe);
			cout<<"ultra_linear_vel_safe_thd: "<< ultra_linear_vel_safe << endl;
			cout<<"ultra_angular_vel_safe_thd: "<< ultra_angular_vel_safe << endl;
			cout<<"ultra_steer_thd: "<< ultra_steer << endl;
	*/


			protectObj.CalcLaserCA(protectObj.min_scan , protectObj.min_scan_angle, laser_steer, &laser_linear_vel_safe, &laser_angular_vel_safe,laser_area_sts);
			cout<<"laser_linear_vel_safe_thd: "<< laser_linear_vel_safe<< endl;
			cout<<"laser_angular_vel_safe_thd: "<< laser_angular_vel_safe<< endl;
			cout<<"laser_steer_thd: "<< laser_steer<< endl;
			cout<<"laser_area_sts: "<< laser_area_sts<< endl;

			protectObj.CalcUltraCA(protectObj.min_ultra, protectObj.min_index_ultra, ultra_steer, &ultra_linear_vel_safe, &ultra_angular_vel_safe,ultra_area_sts);
			cout<<"ultra_linear_vel_safe_thd: "<< ultra_linear_vel_safe << endl;
			cout<<"ultra_angular_vel_safe_thd: "<< ultra_angular_vel_safe << endl;
			cout<<"ultra_steer_thd: "<< ultra_steer << endl;
			cout<<"ultra_area_sts: "<< ultra_area_sts<< endl;
			


/*
			file1 << fixed << setprecision(4) << protectObj.ultra_vec[0];
			file1 << '\t';
			file1 << fixed << setprecision(4) << protectObj.ultra_vec[1];
			file1 << '\t';	
			file1 << fixed << setprecision(4) << protectObj.ultra_vec[2];
			file1 << '\t';
			file1 << fixed << setprecision(4) << protectObj.ultra_vec[3];
			file1 << '\n';

			cnt ++;
			if(cnt == rec_max)
			{
				file1.close();	//record laser dis completed
			}
*/
			// publish the laser module safe vel
			protectObj.laser_safe_vel.header.stamp = ros::Time::now();
			protectObj.laser_safe_vel.header.frame_id = "laser";

			if((laser_linear_vel_safe == LINEAR_STOP) && (laser_angular_vel_safe == ANGULAR_STOP) && (laser_steer == 0))
			{
				protectObj.laser_safe_vel.stop.data = true;
			}
			else
			{
				protectObj.laser_safe_vel.stop.data = false;
			}				
			protectObj.laser_safe_vel.linear_safe_thd = laser_linear_vel_safe;
			protectObj.laser_safe_vel.area_status = laser_area_sts;
			protectObj.laser_safe_vel.steer = laser_steer;
			protectObj.laser_safe_vel.angular_safe_thd = laser_angular_vel_safe;
			protectObj.laser_safe_vel.rsvd = 0.0;
			
			protectObj.security_pub4laser.publish(protectObj.laser_safe_vel);

			// publish the ultra module safe vel
			protectObj.ultra_safe_vel.header.stamp = ros::Time::now();
			protectObj.ultra_safe_vel.header.frame_id = "ultra";

			if((ultra_linear_vel_safe == LINEAR_STOP) && (ultra_angular_vel_safe == ANGULAR_STOP) && (ultra_steer == 0))
			{
				protectObj.ultra_safe_vel.stop.data = true;
			}
			else
			{
				protectObj.ultra_safe_vel.stop.data = false;
			}				
			protectObj.ultra_safe_vel.linear_safe_thd = ultra_linear_vel_safe;
			protectObj.ultra_safe_vel.area_status = ultra_area_sts;
			protectObj.ultra_safe_vel.steer = ultra_steer;
			protectObj.ultra_safe_vel.angular_safe_thd = ultra_angular_vel_safe;
			protectObj.ultra_safe_vel.rsvd = 0.0;
			
			protectObj.security_pub4ultra.publish(protectObj.ultra_safe_vel);

			ros::spinOnce();
			loop_rate.sleep();
			ROS_INFO("end ...");
		}
	

	}

	return 0;
}

