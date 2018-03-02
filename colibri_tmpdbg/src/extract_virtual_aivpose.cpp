#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <iostream>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "extract_aivpose");
	ros::NodeHandle nh;

	tf::TransformListener listener;
	double x = 0.0;
	double y = 0.0;
	double yaw = 0.0;
	double pitch = 0.0;
	double roll = 0.0;
	double qx = 0;
	double qy = 0;
	double qz = 0;
	double qw = 0;


	ros::Rate rate(1.0);
	while(nh.ok())
	{
		tf::StampedTransform transform;
		try
		{
			listener.lookupTransform("map", "base_link_virtual", ros::Time(0), transform);
			
		}
		catch(tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		
		
		x = transform.getOrigin().x() + 0.352;
		y = transform.getOrigin().y();

		qw = transform.getRotation().getW();
		qx = transform.getRotation().getX();
		qy = transform.getRotation().getY();
		qz = transform.getRotation().getZ();	
		
		transform.getBasis().getEulerYPR(yaw, pitch, roll);		

		ROS_INFO("baselink_virtual x: %lf", x);
		ROS_INFO("baselink_virtual y: %lf", y);
		ROS_INFO("baselink_virtual yaw: %lf degree", yaw*57.3);

		rate.sleep();
	}
	
	return 0;
}
