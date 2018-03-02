#include <iostream>
#include <string>  
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "cartodom/Cartodom.h"
#include "colibri_msgs/AuxInfo.h"
#include "colibri_msgs/MusicMode.h"

#ifndef _CARTODOM_CORRECT_H_
#define _CARTODOM_CORRECT_H_
using namespace std;

#define RAD2DEG 57.296
#define OFFSET_LASER_X	0.352

typedef struct st_robot_pos{
	float x;
	float y;
	float yaw;	//rad
}robot_pos;


class cartodom_correct
{

	public:

		ros::Time cur_correct_time;
		ros::Time last_correct_time;
			
		robot_pos cur_cartodom;
		robot_pos last_cartodom;
		
		float delta_cartodom;
		
		float cur_vx;
		float last_vx;
		
		float delta_dead_reckon;
	
		float delta_odom_error;

		float epsilon;
		float normal_thd;
		bool carto_except;
		int except_phase;

		robot_pos cartodom_dr;
		robot_pos corrected_odom;
		

	public:
		cartodom_correct();
		~cartodom_correct();
		void CalcDeadReckonDeltaDis(float delta_time);
		void CalcCartodomOriDeltaDis();
		bool IsCartoException();		
		robot_pos CorrectCartodom();
		void CalcCurExceptState();

};

#endif

