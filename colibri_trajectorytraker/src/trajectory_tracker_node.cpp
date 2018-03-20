#include "trajectory_tracker.h"

#include <dynamic_reconfigure/server.h>
#include <colibri_trajectorytraker/trajectory_tracker_Config.h>

path_point start_point ,end_point;
float rotation_radius ,v_max ,a_max;
string rot_direction;

void dynamic_callback(colibri_trajectorytraker::trajectory_tracker_Config &config, uint32_t level)
{
	ROS_INFO("\r\n Reconfigure Request: \r\n start_point_x = %f\r\n start_point_y = %f\r\n end_point_x = %f\r\n end_point_y = %f\r\n circle_radius = %f\r\n rotation_direction = %s\r\n v_max = %f\r\n a_max = %f\r\n", 	
			   config.start_point_x, 	
			   config.start_point_y, 			   
			   config.end_point_x, 	
			   config.end_point_y, 			   
			   config.circle_radius,	
			   config.rotation_direction.c_str(),
			   config.v_max,
			   config.a_max
			   );
	start_point.x= config.start_point_x;
	start_point.y= config.start_point_y;
	end_point.x= config.end_point_x;
	end_point.y= config.end_point_y;
	rotation_radius = config.circle_radius;
	rot_direction = config.rotation_direction.c_str();
	v_max = config.v_max;
	a_max = config.a_max;
	
}

int main(int argc, char* argv[])
{	
	ros::init(argc, argv, "trajectory_tracker_node");
	
	ROS_INFO("Start to trajectory_tracking ... ");
	Trajectory_tracker tracker;

	dynamic_reconfigure::Server<colibri_trajectorytraker::trajectory_tracker_Config> server;
        dynamic_reconfigure::Server<colibri_trajectorytraker::trajectory_tracker_Config>::CallbackType f;
  	f = boost::bind(&dynamic_callback, _1, _2);
  	server.setCallback(f);

	/*start_point.x= 5.0;
	start_point.y= -5.0;
	end_point.x= 7.0;
	end_point.y= -3.0;
	rotation_radius = 5.0;
	rot_direction = "counterclockwise";
	v_max = 1.0;
	a_max = 1.0;
	*/
      
	tracker.vehicle_param.vehicle_width = 0.482;
	tracker.vehicle_param.wheel_radius = 0.20;
			
	tracker.CLF_coeff.k1 = 5.0;
	tracker.CLF_coeff.k2 = 0.5;
	tracker.CLF_coeff.k3 = 5.0;
	tracker.CLF_coeff.flag = false;
		 	
	tracker.InitSubandPub();
	ros::Rate loop_rate(10);
	int delay_count = 0;

	int number_count = 0;
		
	while (ros::ok())
	{
		if(delay_count < DELAY_CNT_MAX+10)
		{
			delay_count++;
			ros::spinOnce();
			loop_rate.sleep();
		}
	
		if(delay_count >= DELAY_CNT_MAX)
		{

		  	ros::spinOnce();
			tracker.path_param_input.start_point = start_point;
			tracker.path_param_input.end_point = end_point;
			tracker.path_param_input.path_typeT = circle;
			tracker.path_param_input.rot_direction = rot_direction;
			tracker.path_param_input.rot_radius = rotation_radius;
			tracker.motion_param_input.a_max = a_max;
			tracker.motion_param_input.v_max = v_max;
			tracker.motion_param_input.ts = 0.1;
			
			tracker.PathGeneration(tracker.path_param_input,tracker.motion_param_input,tracker.path_gen_output);
			tracker.Output(tracker.path_gen_output);
			
			path_point curent_pose_point;
			curent_pose_point.x = tracker.curent_robot_pose[0];
			curent_pose_point.y = tracker.curent_robot_pose[1];
			curent_pose_point.yaw = tracker.curent_robot_pose[2];

			float delta_x ;
			
			int final_index = tracker.path_gen_output.path_vel_array.size();
			
		       	if(number_count < final_index)
			{
				tracker.LyapunovContorller(tracker.CLF_coeff, tracker.path_gen_output, curent_pose_point, number_count, tracker.CLF_vel);
				number_count++;
					
				geometry_msgs::Twist vel;
				vel.linear.x = tracker.CLF_vel.linear_vel;
		    		vel.angular.z = tracker.CLF_vel.angular_vel;
				tracker.cmd_vel_pub.publish(vel);

				//ros::spinOnce();
				loop_rate.sleep();			
			}
			else
			{
			
			}

		}
				
	}
	
		
	return 0;
	
}	






































