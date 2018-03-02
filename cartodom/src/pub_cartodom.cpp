#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cartodom/Cartodom.h"

#include <sstream>

int main(int argc, char **argv)
{

	 ros::init(argc, argv, "pub_cartodom_node");

	 ros::NodeHandle nh;
	 ros::Publisher cartodom_pub = nh.advertise<cartodom::Cartodom>("/cartodom", 10);

	 ros::Rate loop_rate(25);
	
	 cartodom::Cartodom cartodom;

	 cartodom.header.stamp = ros::Time::now();
	 cartodom.header.frame_id = "odom_virtual";
	 cartodom.x = 1.1;
	 cartodom.y = 2.2;
	 cartodom.yaw = 30.5;
	 cartodom.qx = 0.1;
	 cartodom.qy = 0.2;
	 cartodom.qz = 0.3;
	 cartodom.qw = 1.0;
	 cartodom.vx = 0.2;
	 cartodom.vth = 0.0;
	 cartodom.interval = 0.035;

	 int count = 0;
	 
	 ROS_INFO("Start pub a custom cartodom info");
	 while (ros::ok())
	 {
	
		cartodom.header.stamp = ros::Time::now();

		cartodom_pub.publish(cartodom);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	 }

	return 0;
}
