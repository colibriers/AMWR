#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "extract_aivpose");
	ros::NodeHandle nh;

	ros::NodeHandle nh_odom;
    ros::Publisher odom_pub;
	odom_pub = nh_odom.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

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


	ros::Rate rate(10.0);
	while(nh.ok())
	{
		tf::StampedTransform transform;
		try
		{
			listener.lookupTransform("odom_virtual", "base_link_virtual", ros::Time(0), transform);
			
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

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.header.frame_id = "odom";
		//odom_trans.child_frame_id = "base_link";
		odom_trans.child_frame_id = "base_footprint";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		//publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = "odom";
		
		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		
		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = 0.0;
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.angular.z = 0.0;
		
		//publish the message
		odom_pub.publish(odom);

		rate.sleep();
	}
	
	return 0;
}
