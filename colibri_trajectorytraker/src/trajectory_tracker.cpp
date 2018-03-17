#include "trajectory_tracker.h"

Trajectory_tracker::Trajectory_tracker()
{

	vehicle_param.vehicle_width = 0.482;
	vehicle_param.wheel_radius = 0.20;
	motion_param_input.a_max = 1.0;
	motion_param_input.v_max = 1.0;
	motion_param_input.ts = 0.10;

	CLF_coeff.k1 = 5.0;
	CLF_coeff.k2 = 0.5;
	CLF_coeff.k3 = 5.0;
	CLF_coeff.flag = false;
	
	last_time = ros::Time::now();
	current_time = ros::Time::now();

}

Trajectory_tracker::~Trajectory_tracker()
{

}

bool Trajectory_tracker::InitSubandPub()
{
	ros::NodeHandle nh_cartodom;	
	cartodom_sub= nh_cartodom.subscribe<cartodom::Cartodom>("cartodom", 10, boost::bind(&Trajectory_tracker::CartodomCallback, this, _1));

	ros::NodeHandle nh_odom;			
	cartodom_sub= nh_odom.subscribe<nav_msgs::Odometry>("odom", 10, boost::bind(&Trajectory_tracker::OdomCallback, this, _1));
		
	ros::NodeHandle nh_cmd_vel;
	cmd_vel_pub = nh_cmd_vel.advertise<geometry_msgs::Twist>("t_cmd_vel", 10);
	
}



void Trajectory_tracker::CartodomCallback(const cartodom::Cartodom::ConstPtr & carto_odom)
{
	curent_robot_pose[0] = carto_odom->x;
	curent_robot_pose[1] = carto_odom->y;
	curent_robot_pose[2] = carto_odom->yaw * RAD2DEG;

	ROS_INFO("\r\n Robot current position:\r\n current_pose.x = %f\r\n current_pose.y = %f\r\n current_pose.yaw = %f\r\n",
		curent_robot_pose[0],
		curent_robot_pose[1],
		curent_robot_pose[2]
		);
	
	
}


void Trajectory_tracker::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	curent_robot_vel[0] = odom->twist.twist.linear.x;
	curent_robot_vel[1] = odom->twist.twist.angular.z;
}


void Trajectory_tracker::PathGeneration(path_params& path_param , motion_params& motion_param ,path_gen& path_get)
{
	path_params path_param_in = path_param;
	motion_params motion_param_in = motion_param;
	path_gen  path_get_out;
	path_type type = path_param_in.path_typeT;

	switch(type)
		{
		 	case circle:
				ArcPathGeneration(path_param_in , motion_param_in , path_get_out);
				break;

			case line:
				LinePathGeneration(path_param_in , motion_param_in , path_get_out);
				break;
				
			//default:
			
				//break;				
		}
	//output
	path_get = path_get_out;

}


void Trajectory_tracker::ArcPathGeneration(path_params& path_param , motion_params& motion_param ,path_gen& path_get)
{
	path_params path_param_in = path_param;
	motion_params motion_param_in = motion_param;
	circle_params circle_param_gen;
	velocity_params velocity_param_gen;

	path_point point_S = path_param.start_point;
	path_point point_E = path_param.end_point;
	string rotation_direction = path_param.rot_direction;
	float R = path_param.rot_radius;
	
	CalCircleCenter(path_param_in , circle_param_gen);

	float SE[2] = {point_E.x-point_S.x , point_E.y-point_S.y};

	float theta_CS = atan2(point_S.y - circle_param_gen.circle_y,point_S.x - circle_param_gen.circle_x)*RAD2DEG;
	float theta_CE = atan2(point_E.y - circle_param_gen.circle_y,point_E.x - circle_param_gen.circle_x)*RAD2DEG;
	float theta_SE = atan2(SE[1],SE[0])*RAD2DEG;

	if ((theta_SE > 0)&&(theta_SE < 90) )
	{
		if(rotation_direction == "clockwise")
		{
			if (theta_CS < theta_CE)
			{
				theta_CS = theta_CS+360;
			}
          			
		}
	}

	else if ((theta_SE > 90)&&(theta_SE < 180))
	{
		if(rotation_direction == "clockwise")
		{
			if (theta_CS < theta_CE)
          		{
          			theta_CS = theta_CS+360;
			}
		}    
	}
    
	else if ((theta_SE > -180)&&(theta_SE < -90))
	{
		 if(rotation_direction == "counterclockwise")
		 {
		 	if (theta_CS > theta_CE)
		 	{
		 		theta_CE = theta_CE+360;
		 	}      
		 }
	}
 
	else if ((theta_SE > -90)&&(theta_SE < 0))
	{
		if(rotation_direction == "counterclockwise")
		{
			 if (theta_CS > theta_CE)
			 {
			 	theta_CE = theta_CE+360;
			 }
          
		} 
	}
    
	else if (theta_SE == 0) 
	{
		if(rotation_direction =="counterclockwise")
		{
			if (theta_CS > theta_CE)
			{
			 	theta_CS = -theta_CS; 
			}
		}
	}
    
	else if (theta_SE == 90)
	{
		if(rotation_direction == "clockwise")
		{
			theta_CS = theta_CS+360;	
		}
	}
    
	else if (theta_SE == 180)
	{
		if(rotation_direction == "clockwise")
		{
			 if( theta_CS < theta_CE)
			 {
			 	theta_CE = -theta_CE; 
			 }     
		}	
	}
  
	else if (theta_SE == -90)
	{
		if(rotation_direction == "counterclockwise")
		{
			theta_CE = theta_CE+360; 
		}
	}

	float delta_theta = abs(theta_CE -theta_CS)*DEG2RAD;
	float len_arc = 2*PI*R*delta_theta/(2*PI);
        float s = len_arc; 

	TCurveVelocityPlaning(motion_param_in , path_param_in , s, velocity_param_gen);

	path_get.path_array.clear();
	path_get.path_vel_array.clear();

	//output
        path_get.path_vel_array = velocity_param_gen.path_vel_array;
		
	int k_sim = velocity_param_gen.path_vel_array.size();

	float theta_arc[k_sim];
	theta_arc[0]=0;
	float theta_st=theta_CS*DEG2RAD;
	
        float 	time[k_sim];
	
	curvetimeT timeT = velocity_param_gen.timeT;
	float omega_max = velocity_param_gen.max_anglular_vel;
	float ts = motion_param_in.ts;
	float omega_p[k_sim];
	path_point tmp_path_point;

	for (int k = 0 ; k < k_sim; k++)
	{
		time[k] = k*ts;
		path_velocity temp_vel = velocity_param_gen.path_vel_array.at(k);
		omega_p[k] = temp_vel.angular_vel;
			
		if ((time[k] >= 0) && (time[k] <= timeT.t1))
		{
                        
			theta_arc[k] = 1/2*time[k]*omega_p[k] + theta_arc[0];
		}
		       
		else if ((time[k] > timeT.t1) && (time[k]  < timeT.t2))
		{
			theta_arc[k] = 1/2*(2*time[k] - timeT.t1)*omega_p[k] + theta_arc[0];
		}
		       
		else if ((time[k] >= timeT.t2) && (time[k] <= timeT.t3))
		{
			 theta_arc[k] = (timeT.t3 + timeT.t2 - timeT.t1)*omega_max/2-omega_p[k]*(timeT.t3 - (time[k]))/2 + theta_arc[0];
		}
		      

		 tmp_path_point.x = R*cos(theta_arc[k] + theta_st) + circle_param_gen.circle_x;
		 tmp_path_point.y = R*sin(theta_arc[k] + theta_st) + circle_param_gen.circle_y;
                 //output
		 path_get.path_array.push_back(tmp_path_point);
	
	}

}

void Trajectory_tracker::LinePathGeneration(path_params& path_param , motion_params& motion_param ,path_gen& path_get)
{

}

void Trajectory_tracker::CalCircleCenter(path_params& path_param , circle_params& circle_param)
{
        path_point circle_center1;
	path_point circle_center2;
	path_point point_S = path_param.start_point;
	path_point point_E = path_param.end_point;
	string rotation_direction = path_param.rot_direction;
	float R = path_param.rot_radius;

	float Circle_Center1[2];
	float Circle_Center2[2];

	float distance=sqrt(pow(point_E.x-point_S.x,2.0)+pow(point_E.y-point_S.y,2.0));

	if  (R >= (distance/2))
	{
	 	if (point_S.x == point_E.x)
	 	{
	 		circle_center1.y=(point_S.y+point_E.y)/2;
        		circle_center2.y=(point_S.y+point_E.y)/2;
        		float dis=sqrt(pow(R,2.0)-pow((abs((point_S.y-point_E.y)/2)),2.0));
        		circle_center1.x=point_S.x-dis;
        		circle_center2.x=point_S.x+dis;
	 	}
		else
		{
			float c1=(pow(point_E.x,2.0)-pow(point_S.x,2.0)+pow(point_E.y,2.0)-pow(point_S.y,2.0))/(2*(point_E.x-point_S.x));
       			float c2=(point_E.y-point_S.y)/(point_E.x-point_S.x);

        		float A=(pow(c2,2.0)+1);
        		float B=(2*point_S.x*c2-2*c1*c2-2*point_S.y);
        		float C=(pow(point_S.x,2.0)-2*point_S.x*c1+pow(c1,2.0)+pow(point_S.y,2.0)-pow(R,2.0));

        		float delta=pow(B,2.0)-4*A*C;

			if (delta >= 0)
			{
				circle_center1.y=(-B+sqrt(delta))/(2*A);
            			circle_center2.y=(-B-sqrt(delta))/(2*A);
            			circle_center1.x=c1-c2*circle_center1.y;
            			circle_center2.x=c1-c2*circle_center2.y;	
			}
				 
		}
		
		Circle_Center1[0] = circle_center1.x;
		Circle_Center1[1] = circle_center1.y;
		Circle_Center2[0] = circle_center2.x;
		Circle_Center2[1] = circle_center2.y;
		
	}
        else
	{
	 	cout << "Cann't Calculate Circle Center!Radius should larger than distance/2!";
	}

	float SE[2] = {point_E.x-point_S.x , point_E.y-point_S.y};
	float SC1[2] = {Circle_Center1[0]-point_S.x , Circle_Center1[1]-point_S.y};
	float SC2[2] = {Circle_Center2[0]-point_S.x , Circle_Center2[1]-point_S.y};

	float Sign1 = SE[0]*SC1[1]-SE[1]*SC1[0]; 
	float Sign2 = SE[0]*SC2[1]-SE[1]*SC2[0];	

	if (rotation_direction == "clockwise")
	{
		if (Sign1 < 0)
		{
			circle_param.circle_x= Circle_Center1[0];
			circle_param.circle_y = Circle_Center1[1];
			circle_param.radius = R;
		}
		else if(Sign2 < 0)
		{
			circle_param.circle_x = Circle_Center2[0];
			circle_param.circle_y = Circle_Center2[1];
			circle_param.radius = R;
		}
		else if((Sign2 == 0)&&(Sign2 == 0))
		{
			circle_param.circle_x = Circle_Center1[0];
			circle_param.circle_y = Circle_Center1[1];
			circle_param.radius = R;
		}
				
	}
	else
	{
		if (Sign1 > 0)
		{
			circle_param.circle_x = Circle_Center1[0];
			circle_param.circle_y = Circle_Center1[1];
			circle_param.radius = R;
		}
		else if(Sign2 > 0)
		{
			circle_param.circle_x = Circle_Center2[0];
			circle_param.circle_y = Circle_Center2[1];
			circle_param.radius = R;
		}
		else if((Sign2 == 0)&&(Sign2 == 0))
		{
			circle_param.circle_x = Circle_Center1[0];
			circle_param.circle_y = Circle_Center1[1];
			circle_param.radius = R;
		}

	}
}

void Trajectory_tracker::TCurveVelocityPlaning(motion_params& motion_param , path_params& path_param , float& distance, velocity_params& velocity_param)
{
	//path_point point_S = path_param.start_point;
	//path_point point_E = path_param.end_point;
	string rotation_direction = path_param.rot_direction;
	float R = path_param.rot_radius;

	float v_max = motion_param.v_max;
	float a_max = motion_param.a_max;
	float omega_max = 0;
	float ts = motion_param.ts;
	curvetimeT timeT;
	float dis = distance;
	
        timeT.ta = v_max/a_max;
	timeT.t0 = 0;
	timeT.t1 = (timeT.t0 + timeT.ta);
	timeT.tv = (dis - v_max*timeT.t1)/v_max;
	timeT.t2 = timeT.t1 + timeT.tv;
	timeT.t3 = timeT.t2 + timeT.ta;
	timeT.total = timeT.t3;
	
	int k_sim  = floor(timeT.total/ts);
	float temp_v; 
	float temp_omega; 
        path_velocity tem_vel;
	float 	time[k_sim];
			
	for (int k = 0;k < k_sim; k++)
	{
		time[k] = k*ts;
		if (time[k] >= 0 && time[k] <= timeT.t1)
		{
			temp_v = a_max*time[k];
			tem_vel.linear_vel = temp_v;
		}
		
    		else if (time[k] > timeT.t1 && time[k] < timeT.t2)
    		{
    			temp_v = v_max;
    			tem_vel.linear_vel = temp_v;
    		}
     
     		else if (time[k] >= timeT.t2)
     		{
     			temp_v = v_max-a_max*(time[k]-timeT.t2);
			tem_vel.linear_vel = temp_v;
     		}
			

		if (rotation_direction == "clockwise") 
		{
		        temp_omega = -temp_v/R;
			omega_max = -v_max/R;
			tem_vel.angular_vel = temp_omega;
		}
                else
                {
                	temp_omega = temp_v/R;
			omega_max = v_max/R;
			tem_vel.angular_vel = temp_omega;
                }

		velocity_param.path_vel_array.push_back(tem_vel);

	}

	velocity_param.timeT = timeT;
	velocity_param.max_anglular_vel = omega_max;
    	
}


void Trajectory_tracker::LyapunovContorller(CLF_coefficient&  CLF_coeff , path_gen& path_in, path_point& curent_point , int& count, path_velocity& CLF_vel_get)
{
	float k1 = CLF_coeff.k1;  //  yError
	float k2 = CLF_coeff.k2;  //  xError
	float k3 = CLF_coeff.k3;  //  ThetaError
        
	path_point reference_pose_point;
	path_point current_pose_point;
	path_velocity reference_velocity;

	float v_center;
	float w_center;

	float x_c;
	float y_c;
	float theta_c;

	float x_r;
	float y_r;
	float theta_r;

	float xError;
	float yError;
	float thetaError;

	float xError_CLF;
	float yError_CLF;
	float thetaError_CLF;

	float v_CLF;
	float omega_CLF;

	RowVector3f p_c, p_r;
	Matrix3f T_e;
	Vector3f p_e;
		
        int k =count;
	int k_sim = path_in.path_vel_array.size();

	reference_pose_point = path_in.path_array.at(k);
	reference_velocity = path_in.path_vel_array.at(k);
	current_pose_point = curent_point;

	if(k <= k_sim)
	{
		v_center = reference_velocity.linear_vel;
                w_center = reference_velocity.angular_vel;
				
		x_c = current_pose_point.x;
		y_c = current_pose_point.y;
		theta_c = current_pose_point.yaw;

		x_r = reference_pose_point.x;
		y_r = reference_pose_point.y;
		//theta_r = reference_pose_point.yaw;
				
		xError = x_r -x_c;
		yError = y_r -y_c;		
		theta_r = atan2(yError,xError);
		thetaError = reference_pose_point.yaw - current_pose_point.yaw;

		p_c(0, 0) = current_pose_point.x;
		p_c(0, 1) = current_pose_point.y;
		p_c(0, 2) = current_pose_point.yaw;

		p_r(0, 0) = reference_pose_point.x;
		p_r(0, 1) = reference_pose_point.y;
		p_r(0, 2) = theta_r;

		T_e(0, 0) = cos(theta_c);
		T_e(0, 1) = sin(theta_c);
		T_e(0, 2) = 0;		
		T_e(1, 0) = -sin(theta_c);
		T_e(1, 1) = cos(theta_c);
		T_e(1, 2) = 0;
		T_e(2, 0) = 0;
		T_e(2, 1) = 0;
		T_e(2, 2) = 1;

		p_e=T_e*(p_r-p_c).transpose();

		xError_CLF = p_e(0,0);
    		yError_CLF = p_e(1,0);
		thetaError_CLF =  p_e(2,0); 

		v_CLF = v_center*cos(thetaError_CLF)+k2*xError_CLF;
    		omega_CLF  = w_center+k1*v_center*yError_CLF+k3*sin(thetaError_CLF);
		

		
									 
	}



}





































