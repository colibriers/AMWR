#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "pub_const_anglar");


  ros::NodeHandle nh;

  geometry_msgs::Twist const_vel;

  const_vel.linear.x = 0.0;
  const_vel.linear.y = 0.0;
  const_vel.linear.z = 0.0;
  const_vel.angular.x = 0.0;
  const_vel.angular.y = 0.0;
  const_vel.angular.z = 0.157;

  ros::Publisher constvel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  ros::Rate loop_rate(10);



 ROS_INFO("Start to publish a const rotation thetadot: %f for 10 secs", const_vel.angular.z);

  int count = 0;
  while (ros::ok())
  {

    constvel_pub.publish(const_vel);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
	
	if(count == 100)
	{
		constvel_pub.publish(geometry_msgs::Twist());
		ROS_INFO("Pub const for 10 sec Over!!!");
		break;	
	}
  }

  return 0;
}
