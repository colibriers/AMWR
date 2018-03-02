#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "colibri_msgs/EnvSecurity.h"
#include "colibri_msgs/AuxInfo.h"
#include "cartodom/Cartodom.h"

#define LIGHT_OFF 0
#define LIGHT_ON 255
#define LIGHT_SLOW_BLINKING 15	//0x0f 1s flash
#define LIGHT_FAST_BLINKING 240	//0xf0 0.5s flash 

#define HORN_OFF 0
#define HORN_ON 255		// normal music
#define HORN_ALARM 240	// fault music

#define ULTRA_OFF 0
#define ULTRA_ON 255
#define ROS_OK 0
#define ROS_SYS_FATAL 255

geometry_msgs::Twist aux_twist;
colibri_msgs::EnvSecurity aux_envsec;
cartodom::Cartodom aux_cartodom;

void SubTwistCallBack(const geometry_msgs::Twist::ConstPtr & cmd_vel);
void SubEnvSecurityCallBack(const colibri_msgs::EnvSecurity::ConstPtr & env_sec);
void StateFbCallback(const cartodom::Cartodom::ConstPtr & carto_odom);


int main(int argc, char* argv[])
{	

	// ROS nav node initial
	ros::init(argc, argv, "Auxiliary_Info_Pub");
	ROS_INFO("Start to Pub the Auxinfo ... ");
	
	ros::NodeHandle nh_aux;
	ros::Subscriber twist_sub;
	ros::Subscriber env_sub;
	ros::Subscriber state_fb_sub;
	ros::Publisher aux_pub;

	ros::Rate loop_rate(10);	
	
	colibri_msgs::AuxInfo aux_info;
	aux_info.header.stamp = ros::Time::now();
	aux_info.header.frame_id = "robot";
	
	aux_info.lf_light = LIGHT_SLOW_BLINKING;
	aux_info.lr_light = LIGHT_SLOW_BLINKING;
	aux_info.rf_light = LIGHT_SLOW_BLINKING;
	aux_info.rr_light = LIGHT_SLOW_BLINKING;

	aux_info.horn = HORN_OFF;
	aux_info.laser_mindis = 1.0;
	aux_info.laser_mindir = 90.0;
	aux_info.ultra_switch = ULTRA_ON;
	aux_info.ros_fault = ROS_OK;

	twist_sub = nh_aux.subscribe<geometry_msgs::Twist>("/t_cmd_vel", 1, &SubTwistCallBack);
	env_sub = nh_aux.subscribe<colibri_msgs::EnvSecurity>("/env_secure", 1, &SubEnvSecurityCallBack);
	state_fb_sub = nh_aux.subscribe<cartodom::Cartodom>("/cartodom", 1, &StateFbCallback);

	aux_pub = nh_aux.advertise<colibri_msgs::AuxInfo>("/aux_info", 1);

	aux_envsec.laser_min_dis = 20.0;
	aux_envsec.ultra_min_dis = 6.5;

	while(ros::ok())
	{
		aux_info.header.stamp = ros::Time::now();
		aux_info.header.frame_id = "robot";

		//backward running
		if(aux_twist.linear.x < -0.05)
		{
			aux_info.lf_light = LIGHT_ON;
			aux_info.lr_light = LIGHT_FAST_BLINKING;
			aux_info.rf_light = LIGHT_ON;
			aux_info.rr_light = LIGHT_FAST_BLINKING;
		}
		else if(aux_twist.linear.x >= 0.02) 	// left or right  turning
		{
			if(aux_twist.angular.z > 0.05)
			{
				aux_info.lf_light = LIGHT_FAST_BLINKING;
				aux_info.lr_light = LIGHT_SLOW_BLINKING;
				aux_info.rf_light = LIGHT_ON;
				aux_info.rr_light = LIGHT_SLOW_BLINKING;
			}
			else if(aux_twist.angular.z < -0.05)
			{
				aux_info.lf_light = LIGHT_ON;
				aux_info.lr_light = LIGHT_SLOW_BLINKING;
				aux_info.rf_light = LIGHT_FAST_BLINKING;
				aux_info.rr_light = LIGHT_SLOW_BLINKING;
			}
			else
			{
				aux_info.lf_light = LIGHT_ON;
				aux_info.lr_light = LIGHT_ON;
				aux_info.rf_light = LIGHT_ON;
				aux_info.rr_light = LIGHT_ON;
			}
			
		}
		else //still
		{
			aux_info.lf_light = LIGHT_SLOW_BLINKING;
			aux_info.lr_light = LIGHT_SLOW_BLINKING;
			aux_info.rf_light = LIGHT_SLOW_BLINKING;
			aux_info.rr_light = LIGHT_SLOW_BLINKING;

			if(aux_twist.angular.z > 0.02)
			{
				aux_info.lf_light = LIGHT_SLOW_BLINKING;
				aux_info.lr_light = LIGHT_ON;
				aux_info.rf_light = LIGHT_SLOW_BLINKING;
				aux_info.rr_light = LIGHT_SLOW_BLINKING;
			}
			
			if(aux_twist.angular.z < -0.02)
			{
				aux_info.lf_light = LIGHT_ON;
				aux_info.lr_light = LIGHT_SLOW_BLINKING;
				aux_info.rf_light = LIGHT_SLOW_BLINKING;
				aux_info.rr_light = LIGHT_SLOW_BLINKING;
			}
		}

		if(aux_envsec.laser_min_dis < 0.25 || aux_envsec.ultra_min_dis < 0.4)
		{
			if(abs(aux_cartodom.vx) <= 0.02 && abs(aux_cartodom.vth) <= 0.02)
			{
				aux_info.horn = HORN_ALARM;
			}
			else
			{
				aux_info.horn = HORN_OFF;
			}

		}
		else
		{
			aux_info.horn = HORN_ON;
		}

		aux_info.horn = HORN_OFF;

		aux_info.laser_mindis = aux_envsec.laser_min_dis;
		aux_info.laser_mindir = aux_envsec.laser_min_angle;
		aux_info.ultra_switch = ULTRA_ON;
		aux_info.ros_fault = ROS_OK;

		aux_pub.publish(aux_info);
		
		ros::spinOnce();
		loop_rate.sleep();


	}

	
	for(int j = 5; j > 0; j--)
	{
		aux_info.lf_light = LIGHT_ON;
		aux_info.lr_light = LIGHT_ON;
		aux_info.rf_light = LIGHT_ON;
		aux_info.rr_light = LIGHT_ON;
		aux_info.horn = HORN_OFF;
		aux_info.laser_mindis = 20;
		aux_info.laser_mindir = 6.5;
		aux_info.ultra_switch = ULTRA_ON;
		aux_info.ros_fault = ROS_OK;

		aux_pub.publish(aux_info);	
	}

	return 0;
}

void SubTwistCallBack(const geometry_msgs::Twist::ConstPtr & cmd_vel)
{
	aux_twist.linear.x = cmd_vel->linear.x;
	aux_twist.angular.z = cmd_vel->angular.z;
}

void SubEnvSecurityCallBack(const colibri_msgs::EnvSecurity::ConstPtr & env_sec)
{
	aux_envsec.laser_min_dis = env_sec->laser_min_dis;
	aux_envsec.laser_min_angle = env_sec->laser_min_angle;
	aux_envsec.ultra_min_dis = env_sec->ultra_min_dis;
	aux_envsec.ultra_min_index = env_sec->ultra_min_index;
}

void StateFbCallback(const cartodom::Cartodom::ConstPtr & carto_odom)
{
	aux_cartodom.x = carto_odom->x;
	aux_cartodom.y = carto_odom->y;
	aux_cartodom.yaw = carto_odom->yaw;
	
	aux_cartodom.vx = carto_odom->vx;
	aux_cartodom.vth = carto_odom->vth;
}



