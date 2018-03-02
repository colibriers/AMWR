#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cartodom/Cartodom.h"
#include <cmath>
#include <tf/transform_listener.h>
#include <iostream>

#define LASER_OFFSET_X 0.3442
#define RAD2DEG	57.29578
#define DEG2RAD	0.017453

#define DEADZONE_X 0.005
#define DEADZONE_Y 0.005
#define DEADZONE_YAW_DEG	0.1

using namespace std;

void InitCartodom(cartodom::Cartodom &cartodom);

int main(int argc, char **argv)
{

	 ros::init(argc, argv, "pub_cartodom_node");

	 ros::NodeHandle nh;
	 ros::Publisher cartodom_pub = nh.advertise<cartodom::Cartodom>("/cartodom", 10);

	 ros::Rate loop_rate(10);	// 20ms obtain a pos
	
	 cartodom::Cartodom cartodom;
	
     InitCartodom(cartodom);
	 
	 ros::Time last_time;
	 ros::Time current_time;

	 current_time = ros::Time::now();
	 last_time = current_time;
	
	 double time_period = (current_time - last_time).toSec();
	
	 double yaw = 0.0;
	 double pitch = 0.0;
	 double roll = 0.0;

	 double x = 0.0;
	 double y = 0.0;

	 double last_x = 0.0;
	 double last_y = 0.0;
	 double last_yaw = 0.0;

	 double delta_carto_x = 0.0;
	 double delta_carto_y = 0.0;

	 tf::TransformListener listener;
	 tf::StampedTransform transform;

	 ROS_INFO("Start pub a custom cartodom info");

	 while (ros::ok())
	 {
		current_time = ros::Time::now();

		cartodom.header.stamp = current_time ;

		time_period = (current_time - last_time).toSec();
		
		try
		{
			listener.lookupTransform("odom_virtual", "base_footprint_virtual", ros::Time(0), transform);
		}
		catch(tf::TransformException ex)
		{
			ROS_WARN("%s",ex.what());
			ros::Duration(0.5).sleep();
		}

		x = transform.getOrigin().x();
		y = transform.getOrigin().y();

		transform.getBasis().getEulerYPR(yaw, pitch, roll);		

		if(abs(x - last_x) < DEADZONE_X)
		{
			x = last_x;
		}
		if(abs(y - last_y) < DEADZONE_Y)
		{
			y = last_y;
		}	
		if(abs((yaw - last_yaw) * RAD2DEG) < DEADZONE_YAW_DEG)
		{
			yaw = last_yaw;
		}					

		cartodom.x = x;
		cartodom.y = y;
		cartodom.yaw = yaw;

		cartodom.qx = transform.getRotation().getX();
		cartodom.qy = transform.getRotation().getY();
		cartodom.qz = transform.getRotation().getZ();	
		cartodom.qw = transform.getRotation().getW();

		delta_carto_x = x - last_x;
		delta_carto_y = y - last_y;

		cartodom.vx = sqrt(pow(delta_carto_x, 2) + pow(delta_carto_y, 2)) / time_period; //vx calc is always positive(no dir) 
		cartodom.vth = (yaw - last_yaw) / time_period;
	    cartodom.interval = time_period;	

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);	
		cartodom.header.stamp = ros::Time::now();

		cartodom_pub.publish(cartodom);

		//ROS_INFO("base_footprint_virtual x: %lf", cartodom.x);
		//ROS_INFO("base_footprint_virtual y: %lf", cartodom.y);
		//ROS_INFO("base_footprint_virtual yaw: %lf degree", cartodom.yaw * RAD2DEG);
		//ROS_INFO("base_footprint_virtual vx: %lf", cartodom.vx);
		//ROS_INFO("base_footprint_virtual vth: %lf", cartodom.vth);

		last_x = x;
		last_y = y;
		last_yaw = yaw;
		last_time = current_time;

		ros::spinOnce();

		loop_rate.sleep();

	 }

	return 0;
}

void InitCartodom(cartodom::Cartodom &cartodom)
{
	 cartodom.header.stamp = ros::Time::now();
	 cartodom.header.frame_id = "odom_virtual";
	 cartodom.x = -LASER_OFFSET_X;
	 cartodom.y = 0.0;
	 cartodom.yaw = 0.0;
	 cartodom.qx = 0.0;
	 cartodom.qy = 0.0;
	 cartodom.qz = 0.0;
	 cartodom.qw = 1.0;
	 cartodom.vx = 0.0;
	 cartodom.vth = 0.0;
	 cartodom.interval = 0.02;
}
