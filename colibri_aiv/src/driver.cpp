#include <stdlib.h>
#include "aiv_driver.h" 

static unsigned char recv_cache[CONST_PROTOCOL_LEN];
static io_service AIV_ios;
static serial_port pserialport(AIV_ios);
static boost::system::error_code ec;

volatile bool AIV_Driver::send_twist_finish_ = false;
volatile bool AIV_Driver::send_aux_finish_ = false;
volatile bool AIV_Driver::req_vel_start_finish_ = false;
volatile bool AIV_Driver::req_vel_stop_finish_ = false;

unsigned char *AIV_Driver::send_cache_ = NULL;
unsigned int dbg_info_cnt = 0;
unsigned int lost_twist_res_cnt = 0;

void* ReadDataThread(void *args)
{
	AIV_Driver obj_driver;
	obj_driver.InitSubandPub();
	obj_driver.ReadFromCom(NULL);
}

AIV_Driver::AIV_Driver()
{
	send_cnt_ = 0;
	recv_cnt_ = 0;

	last_time_ = ros::Time::now();
	cur_time_ = ros::Time::now();

	cur_wheels_vel_.left = 0.0;
	cur_wheels_vel_.right = 0.0;
	
	last_wheels_vel_.left = 0.0;
	last_wheels_vel_.right = 0.0;
	
	avg_wheels_vel_.left = 0.0;
	avg_wheels_vel_.right = 0.0;
	
	avg_wheels_dis_.left = 0.0;
	avg_wheels_dis_.right = 0.0;
	
	left_rot_rate_ = 0.0;
	right_rot_rate_ = 0.0;
	
	aiv_dx_ = 0.0;	
	aiv_dth_ = 0.0;	
	aiv_vx_ = 0.0;	
	aiv_vth_ = 0.0;

	cur_music_mode_ = 0;

	unsigned char cmd_data[21];	//used to store for valid data in defined protocol
	memset(cmd_data,0,21);
		
	GenerateCmd(send_twist_, SEND_TWIST, 0x04,RSVD_VAL, cmd_data);
	GenerateCmd(send_aux_info_, SEND_AUX, 0x0B,RSVD_VAL, cmd_data);

	GenerateCmd(req_vel_start_, REQ_VELOCITY, RSVD_VAL, FRAME_CMD_START, cmd_data);
	GenerateCmd(req_vel_stop_, REQ_VELOCITY, RSVD_VAL, FRAME_CMD_STOP, cmd_data);
		
	//DisplayFrame(send_twist_);

	cartodom_.x = -OFFSET_LASER_X;
	cartodom_.y = 0.0;
	cartodom_.yaw = 0.0;

	
	cartodom_quat_.qx = 0.0;
	cartodom_quat_.qy = 0.0;
	cartodom_quat_.qz = 0.0;
	cartodom_quat_.qw = 1.0;

	cartodom_vel_.vx = 0.0;
	cartodom_vel_.vy = 0.0;
	cartodom_vel_.vth = 0.0;

	cartodom_interval_ = 0.02;

	carto_odom_.x = -0.352;
	carto_odom_.y = 0.0;
	carto_odom_.yaw = 0.0;

	wheel_odom_.x = -0.352;
	wheel_odom_.y = 0.0;
	wheel_odom_.yaw = 0.0;

	correct_wheelodom_flag_ = false;
	correct_cartodom_flag_ = false;

	amcl_pose_.x = OFFSET_NX2GX; //for secb 
	amcl_pose_.y = OFFSET_NY2GY;
	amcl_pose_.yaw = 1.57;

	
	frame_delta_rad_ = 0.0;
	
	opt_odom_x_ = carto_odom_.x;
	opt_odom_y_ = carto_odom_.y;
	odom_except_flag_ = false;

	cur_nav_state_.target_node = 127;
	cur_nav_state_.target_heading = 0.0;
	cur_nav_state_.cur_seg = 44;
	cur_nav_state_.at_target_flag = false;
	cur_nav_state_.achieve_flag = false;
	cur_nav_state_.task_succ_flag = true;
	cur_nav_state_.target.x = 0.0;
	cur_nav_state_.target.y = 0.0;
	cur_nav_state_.target.yaw = 0.0;
	cur_nav_state_.robot.x = 0.0;
	cur_nav_state_.robot.y = 0.0;
	cur_nav_state_.robot.yaw = 0.0;
	cur_nav_state_.err_code = 127;


}

AIV_Driver::~AIV_Driver()
{

}

bool AIV_Driver::InitCom(const string port_name)
{
	/*
	pserialport = serial_port(AIV_ios);
	if(!pserialport){
		cout << "pserialport is NULL !"
		return false;
	}
	*/

	//todo:check if it is open successfully

	pserialport.open(port_name,ec);
	pserialport.set_option( serial_port::baud_rate( 115200 ), ec );  
	pserialport.set_option( serial_port::flow_control( serial_port::flow_control::none ), ec );  
	pserialport.set_option( serial_port::parity( serial_port::parity::none ), ec );  
	pserialport.close();
	sleep(0.2);
	
	pserialport.open(port_name,ec);

	pserialport.set_option( serial_port::baud_rate( 115200 ), ec );  
	pserialport.set_option( serial_port::flow_control( serial_port::flow_control::none ), ec );  
	pserialport.set_option( serial_port::parity( serial_port::parity::none ), ec );  
	pserialport.set_option( serial_port::stop_bits( serial_port::stop_bits::one ), ec);  
	pserialport.set_option( serial_port::character_size( 8 ), ec);  

	cout<<port_name<<" serial port init complete"<<endl;

	return true;
}

bool AIV_Driver::CloseCom(const string port_name)
{

	pserialport.open(port_name,ec);
	pserialport.set_option( serial_port::baud_rate( 115200 ), ec );  
	pserialport.set_option( serial_port::flow_control( serial_port::flow_control::none ), ec );  
	pserialport.set_option( serial_port::parity( serial_port::parity::none ), ec );  
	pserialport.close();
	
	return true;
}

void AIV_Driver::GenerateCmd(unsigned char *cmd_name,unsigned char cmd,unsigned char valid_data_len,unsigned char control,unsigned char *data)
{
	memset(cmd_name,0,CONST_PROTOCOL_LEN);

	*(cmd_name + START_INDX_1) = FRAME_START_1;
	*(cmd_name + START_INDX_2) = FRAME_START_2;	
	*(cmd_name + RSVD_BYTE_INDX) = FRAME_RSVD;
	*(cmd_name + REQ_RES_INDX) = FRAME_REQ_RES;
	*(cmd_name + VALID_DATA_LEN_INDX) = valid_data_len;
	*(cmd_name + CMD_BYTE_INDX) = cmd;
	*(cmd_name + CMD_CTRL_INDX) = control;	

	if(valid_data_len != 0)
	{
		for(unsigned char i = 0; i < valid_data_len; i++)
		{
			*(cmd_name + VALID_DATA_START_INDX + i) = *(data + i);
		}
	}

	*(cmd_name + CRC_INDX_1) = RSVD_VAL;		// TODO : CRC SUM
	*(cmd_name + CRC_INDX_2) = RSVD_VAL;
	*(cmd_name + END_INDX_1) = FRAME_END_1;
	*(cmd_name + END_INDX_2) = FRAME_END_2;

}


void AIV_Driver::WriteToCom(const unsigned char * data)
{
	size_t len = write(pserialport, buffer(data, CONST_PROTOCOL_LEN), ec);
	/*	
	cout <<"send "<<len<<" Bytes:";
	int i;
	for(i = 0; i < CONST_PROTOCOL_LEN; i ++)
	{
		printf(" %x", data[i]);	
	}
	cout << endl;
	*/
}


void AIV_Driver::ReadInfoProc(unsigned char buf[], boost::system::error_code ec, std::size_t bytes_transferred)
{	
	static int ctrl_couter = 0;	//ctrl correct cartodom period
	static bool lock_dec_flag = false;
	static bool lock_inc_flag = false;
	static float tmp_cartodom_dr_x = cartodom_.x;
	static float tmp_cartodom_dr_y = cartodom_.y;
	static float compen_y = 0.0;
	static float inc_edge_y = 0.0;
	float dec_edge_delta = 0.0;
	static float lastlast_tmp_cartodom_y = 0.0;
	static float last_tmp_cartodom_y = 0.0;

	static bool sw_amcl_yaw_flag = false;
	static int rec_amcl_cnt = 0;
	double delta_x = 0.0;
	double delta_y = 0.0;
	
	//cout<<"callback read "<<bytes_transferred<<" bytes:";
	unsigned char recv_data[CONST_PROTOCOL_LEN];
    memset(recv_data,0,CONST_PROTOCOL_LEN);
    /*
	cout<<"recv orignal data: ";
	for(int i = 0; i < CONST_PROTOCOL_LEN; i ++)
	{
		printf(" %x", recv_cache[i]);	
	}	
	cout<<endl;
	*/
    if((recv_cache[31] == FRAME_START_1) && (recv_cache[0] == FRAME_START_2) && (recv_cache[1] == FRAME_RSVD))
	{
        memcpy(recv_data, &recv_cache[31], 1);
        memcpy(&recv_data[1], recv_cache, 31);
    }
	else
	{   
		for(int i = 0;i < bytes_transferred - 1; i ++) 
		{   
			if((recv_cache[i] == FRAME_START_1) && (recv_cache[i + 1] == FRAME_START_2) && (recv_cache[i + 2] == FRAME_RSVD))
			{
			    memcpy(recv_data, &recv_cache[i], bytes_transferred - i); 
			    memcpy(&recv_data[bytes_transferred - i], recv_cache, i);
			}
		}   
	}	
	
	if((recv_data[START_INDX_1] != FRAME_START_1)
		|| (recv_data[START_INDX_2] != FRAME_START_2)
		|| (recv_data[END_INDX_1] != FRAME_END_1)
		|| (recv_data[END_INDX_2] != FRAME_END_2))
	{
		cout<<"Head and tail in received frame is not correct !"<<endl;
		for(int j = 0; j < bytes_transferred - 1; j++)
		{
			printf(" %x", recv_cache[j]);
		}
		cout << endl;

		for(int k = 0; k < bytes_transferred - 1; k++)
		{
			printf(" %x", recv_data[k]);
		}
		cout << endl;
		
		return;
	}

	//TODO:CRC verify

	if(recv_data[REQ_RES_INDX] != FRAME_RES_SUCC)
	{
		if(recv_data[REQ_RES_INDX] == FRAME_RES_FAIL)
		{
			cout<<"TRIO respones failed !"<<endl;
			return;
		}
		else
		{
			cout<<"TRIO returned a unknown byte in 4rd Byte:"<<recv_data[REQ_RES_INDX]<<endl;
			return;
		}
	}

	switch(recv_data[CMD_BYTE_INDX])
	{		
		case SEND_TWIST:
			if(AIV_Driver::send_twist_finish_ == true)
			{
				if(recv_data[VALID_DATA_LEN_INDX] != 0x04)
				{
					cout<<"Data count for respones of send_twist_ cmd shoud be 0x04,but is: "<<recv_data[VALID_DATA_LEN_INDX]<<endl;
					return;
				}

				if( recv_data[VALID_DATA_START_INDX + 0] != AIV_Driver::send_cache_[VALID_DATA_START_INDX + 0]
					|| recv_data[VALID_DATA_START_INDX + 1] != AIV_Driver::send_cache_[VALID_DATA_START_INDX + 1]
					|| recv_data[VALID_DATA_START_INDX + 2] != AIV_Driver::send_cache_[VALID_DATA_START_INDX + 2]
					|| recv_data[VALID_DATA_START_INDX + 3] != AIV_Driver::send_cache_[VALID_DATA_START_INDX + 3] )
				{
					cout<<"Received data bytes is not same with the send data byte"<<endl;
					return;
				}
				else
				{
					AIV_Driver::send_twist_finish_ = false;

					//cout<<"send_twist_ cmd is executed successfully !"<<endl;
				}
			
			}
			else
			{
				cout<<"ROS does not send send_twist_ cmd,but recv a response cmd !"<<endl;
				return;
			}
			
			break;
				
		case REQ_VELOCITY:
			
				if(recv_data[VALID_DATA_LEN_INDX] != 0x06)
				{
					cout<<"Recved valid data lenth for request_vel shoud be 0x06,but returned is: "<<recv_data[VALID_DATA_LEN_INDX]<<endl;
					return;
				}
				
				if(recv_data[CMD_CTRL_INDX] == FRAME_CMD_START)
				{
					
					ParseWheelRpm(&recv_data[VALID_DATA_START_INDX]);

					//cout<<"left_rot_rate:"<<left_rot_rate<<"   right_rot_rate:"<<right_rot_rate<<endl;

					cur_time_ = ros::Time::now();
					time_period_ = (cur_time_ - last_time_).toSec();
					last_time_ = cur_time_;

					CalcWheelParameters();

					aiv_dx_ = (avg_wheels_dis_.left + avg_wheels_dis_.right) / 2.0;
					aiv_dth_ = (avg_wheels_dis_.right - avg_wheels_dis_.left) / WHEEL_TRACK;
					aiv_vx_ = (avg_wheels_vel_.left + avg_wheels_vel_.right) / 2.0;
					//aiv_vth_ = aiv_dth_ / time_period;
					
					/*/mask the patch
					ctrl_couter++;
					if(ctrl_couter > 9)
					{
						carto_.cur_correct_time = ros::Time::now();
						float correct_delta_time = (carto_.cur_correct_time - carto_.last_correct_time).toSec();					

						carto_.cur_cartodom.x = cartodom_.x;
						carto_.cur_cartodom.y = cartodom_.y;
						carto_.cur_cartodom.yaw = cartodom_.yaw;
						carto_.cur_vx = aiv_vx_;
						
						carto_.CalcCartodomOriDeltaDis();
						carto_.CalcDeadReckonDeltaDis(correct_delta_time);
						carto_.CalcCurExceptState();


						carto_.last_cartodom.x = carto_.cur_cartodom.x;
						carto_.last_cartodom.y = carto_.cur_cartodom.y;
						carto_.last_cartodom.yaw = carto_.cur_cartodom.yaw;
						carto_.last_vx = carto_.cur_vx;
						carto_.last_correct_time = carto_.cur_correct_time;
						
						ctrl_couter = 0;

						lastlast_tmp_cartodom_y = last_tmp_cartodom_y;
						last_tmp_cartodom_y = tmp_cartodom_dr_y;

						ROS_INFO("cur_cartodom_.x / y: %0.3f %0.3f", cartodom_.x, cartodom_.y);
						ROS_INFO("tmp_cartodom_dr_x tmp_cartodom_dr_y: %0.3f %0.3f", tmp_cartodom_dr_x, tmp_cartodom_dr_y);
						ROS_INFO("compen_y carto.except_phase carto.carto_except: %0.3f %d %d", compen_y, carto_.except_phase, carto_.carto_except);

					}
				
					if(carto_.except_phase==0)
					{
						tmp_cartodom_dr_x = cartodom_.x;
						tmp_cartodom_dr_y = cartodom_.y + compen_y;
						lock_dec_flag = false;
						lock_inc_flag = false;
					}
					else if(carto_.except_phase==1)
					{
						if(lock_inc_flag==false)
						{
							inc_edge_y = lastlast_tmp_cartodom_y;
							lock_inc_flag = true;
							compen_y = 0.4;
						}
						
						compen_y += aiv_vx_ * time_period_;
						tmp_cartodom_dr_x = cartodom_.x;
						tmp_cartodom_dr_y = inc_edge_y + compen_y;
						lock_dec_flag = false;
					}
					else if(carto_.except_phase==2)
					{
						compen_y += aiv_vx_ * time_period_;
						tmp_cartodom_dr_x = cartodom_.x;
						tmp_cartodom_dr_y = inc_edge_y + compen_y;
						lock_dec_flag = false;
						lock_inc_flag = false;
					}
					else
					{
						if(lock_dec_flag==false)
						{
							dec_edge_delta = tmp_cartodom_dr_y - cartodom_.y;
							compen_y = dec_edge_delta;
							lock_dec_flag = true;
						}

						tmp_cartodom_dr_x = cartodom_.x;
						tmp_cartodom_dr_y = cartodom_.y + compen_y;
						lock_inc_flag = false;
					}
					*/
					
					geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(cartodom_.yaw);

					SmoothFrameRotAngle(frame_delta_rad_, cartodom_.yaw, amcl_pose_.yaw);
					//ROS_INFO("frame_delta_rad / cartodom_yaw / amcl_yaw_rad: %0.2lf %0.2lf %0.2lf rad", frame_delta_rad, cartodom_yaw, amcl_yaw_rad);					
					frame_delta_rad_ = 0.0;
					CalcCartodomByAmcl(frame_delta_rad_);

					// calc wheel odom 
					if(sw_amcl_yaw_flag == false)
					{
						delta_x = aiv_vx_ * cos(cartodom_.yaw) * time_period_;
						delta_y = aiv_vx_ * sin(cartodom_.yaw) * time_period_;
					}
					else
					{
						//delta_x = aiv_vx_ * sin(amcl_yaw_rad) * time_period;
						//delta_y = -1.0 * aiv_vx_ * cos(amcl_yaw_rad) * time_period;
						//ROS_INFO("delta_x / delta_y / amcl_yaw_rad: %0.2lf %0.2lf %0.2lf rad", delta_x, delta_y, amcl_yaw_rad);

						delta_x = aiv_vx_ * cos(cartodom_.yaw) * time_period_;
						delta_y = aiv_vx_ * sin(cartodom_.yaw) * time_period_;
					}
					
					wheel_odom_.x += delta_x;
					wheel_odom_.y += delta_y;
					wheel_odom_.yaw = cartodom_.yaw;

					if(correct_wheelodom_flag_)
					{
						correct_wheelodom_flag_ = false;
						wheel_odom_.x = amcl_pose_.y - OFFSET_NY2GY;
						wheel_odom_.y = -1.0 * (amcl_pose_.x - OFFSET_NX2GX);
						rec_amcl_cnt++;
						if(rec_amcl_cnt > SW_AMCL_YAW_CNT)
						{
							rec_amcl_cnt = SW_AMCL_YAW_CNT+1;
							sw_amcl_yaw_flag = true;
						}

					}

					nav_msgs::Odometry odom_wheel;
					odom_wheel.header.stamp = cur_time_;
					odom_wheel.header.frame_id = "odom_virtual";

					odom_wheel.pose.pose.position.x = wheel_odom_.x;
					odom_wheel.pose.pose.position.y = wheel_odom_.y;
					odom_wheel.pose.pose.position.z = 0.0;
					odom_wheel.pose.pose.orientation = odom_quat;

					odom_wheel.child_frame_id = "base_footprint_virtual";
					odom_wheel.twist.twist.linear.x = aiv_vx_;
					odom_wheel.twist.twist.linear.y = 0;
					odom_wheel.twist.twist.angular.z = cartodom_vel_.vth;

					wheel_odom_pub.publish(odom_wheel);

					OdomException(sw_amcl_yaw_flag);

					ROS_INFO("carto_odom_.x/y / wheel_odom_.x/y cartodom_.x/y : %0.3lf %0.3lf ++ %0.3lf %0.3lf ++ %0.3lf %0.3lf", carto_odom_.x, carto_odom_.y, wheel_odom_.x,wheel_odom_.y,cartodom_.x,cartodom_.y);					
					ROS_INFO("opt_odom_x/y_ odom_except_flag_ sw_amcl_yaw_flag: %0.3lf %0.3lf ++ %d ++ %d", opt_odom_x_, opt_odom_y_, odom_except_flag_,sw_amcl_yaw_flag);

					geometry_msgs::TransformStamped odom_trans;
					odom_trans.header.stamp = cur_time_;
					odom_trans.header.frame_id = "odom";
					odom_trans.child_frame_id = "base_footprint";					

					odom_trans.transform.translation.x = cartodom_.x;	// this tf value from the cartodom
					odom_trans.transform.translation.y = cartodom_.y;
					odom_trans.transform.translation.z = 0.0;
					odom_trans.transform.rotation = odom_quat;

					//send the transform
					odom_broadcaster.sendTransform(odom_trans);
					
					//publish the odometry message over ROS
					nav_msgs::Odometry odom;
					odom.header.stamp = cur_time_;
					odom.header.frame_id = "odom";
					
					//set the position
					odom.pose.pose.position.x = cartodom_.x;
					odom.pose.pose.position.y = cartodom_.y;
					odom.pose.pose.position.z = 0.0;
					odom.pose.pose.orientation = odom_quat;
					
					//set the vel
					odom.child_frame_id = "base_footprint";
					odom.twist.twist.linear.x = aiv_vx_;		//in topic /odom ,the aiv total v from encoder
					odom.twist.twist.linear.y = 0;
					odom.twist.twist.angular.z = cartodom_vel_.vth;
					
					//publish the message
					odom_pub.publish(odom);

					dbg_info_cnt++;
					if(dbg_info_cnt>7)
					{
						ROS_DEBUG("virtual x/y/yaw: %0.3lfm %0.3lfm %0.2lfdeg", cartodom_.x, cartodom_.y, cartodom_.yaw * RAD2DEG);
						//ROS_INFO("virtual vx/vth: %0.3lfm/s  %0.2lfrad/s",aiv_vx_,cartodom_vth);
						dbg_info_cnt = 0;
					}
					
					AIV_Driver::req_vel_start_finish_ = false; 
				}
				else if(recv_data[CMD_CTRL_INDX] == FRAME_CMD_STOP)
				{
					AIV_Driver::req_vel_stop_finish_ = false;
				} 
			
			break;

		case SEND_AUX:
			if(AIV_Driver::send_aux_finish_ == true)
			{
				if(recv_data[VALID_DATA_LEN_INDX] != 0x0B)
				{
					cout<<"The data count byte of the respones of the send_aux cmd shoud be 0x0B,but returned is: "<<recv_data[VALID_DATA_LEN_INDX]<<endl;
					return;
				}

				if( recv_data[VALID_DATA_START_INDX + 0] != AIV_Driver::send_cache_[VALID_DATA_START_INDX + 0]
					|| recv_data[VALID_DATA_START_INDX + 1] != AIV_Driver::send_cache_[VALID_DATA_START_INDX + 1]
					|| recv_data[VALID_DATA_START_INDX + 2] != AIV_Driver::send_cache_[VALID_DATA_START_INDX + 2]
					|| recv_data[VALID_DATA_START_INDX + 3] != AIV_Driver::send_cache_[VALID_DATA_START_INDX + 3] )
				{
					cout<<"The received data bytes is not same with the send data byte"<<endl;
					return;
				}
				else
				{
					AIV_Driver::send_aux_finish_ = false;

					//cout<<"send_aux cmd is executed successfully !"<<endl;
				}
			
			}
			else
			{
				cout<<"ROS does not send send_aux cmd,but recv a response cmd !"<<endl;
				return;
			}
			
			break;
			
		default :

			cout<<"received unknown cmd_byte:"<<recv_data[CMD_BYTE_INDX]<<endl;
			break;
			
	}
		
	return;
}

void AIV_Driver::ReadFromCom(void *args)
{

	while(ros::ok())
	{
		async_read(pserialport,buffer(recv_cache,CONST_PROTOCOL_LEN),boost::bind(&AIV_Driver::ReadInfoProc,this,recv_cache,_1,_2));

		ComCallHandle();	
		recv_cnt_ ++;
		//cout <<"recv times: "<<recv_cnt_<<endl;

	}

}

void AIV_Driver::CalcWheelParameters(void) {

	cur_wheels_vel_.left = (2 * PI * WHEEL_RADIUS * left_rot_rate_)/(60 * WHEEL_GEAR);	//left speed:m/s
	cur_wheels_vel_.right = (2 * PI * WHEEL_RADIUS * right_rot_rate_)/(60 * WHEEL_GEAR); //right speed:m/s

	avg_wheels_vel_.left = (cur_wheels_vel_.left + last_wheels_vel_.left) / 2.0;
	avg_wheels_vel_.right = (cur_wheels_vel_.right + last_wheels_vel_.right) / 2.0;
	
	last_wheels_vel_.left = cur_wheels_vel_.left ;
	last_wheels_vel_.right = cur_wheels_vel_.right ;

	avg_wheels_dis_.left = avg_wheels_vel_.left * time_period_;
	avg_wheels_dis_.right = avg_wheels_vel_.right * time_period_;
		
}


void AIV_Driver::ComCallHandle()
{
  //	AIV_ios.poll();
	AIV_ios.run();
	AIV_ios.reset();
}

void AIV_Driver::CreateThread(void *(*start_routine) (void *))
{	
	int ret = pthread_create(&thread_id_,NULL,start_routine,NULL);
	if( ret )
	{
	   cout << "pthread_create error,error_code: "<< ret <<endl;
	   return;
	}
}

bool AIV_Driver::InitSubandPub(void)
{

	ros::NodeHandle global_nh;	
	twist_sub= global_nh.subscribe<geometry_msgs::Twist>("t_cmd_vel", 10, boost::bind(&AIV_Driver::TwistCallback, this, _1));
	aux_sub= global_nh.subscribe<colibri_msgs::AuxInfo>("aux_info", 10, boost::bind(&AIV_Driver::AuxInfoCallback, this, _1));
	music_sub= global_nh.subscribe<colibri_msgs::MusicMode>("music", 10, boost::bind(&AIV_Driver::MusicCallback, this, _1));

	ros::NodeHandle nh_cartodom;
	cartodom_sub= global_nh.subscribe<cartodom::Cartodom>("cartodom", 10, boost::bind(&AIV_Driver::CartodomCallback, this, _1));
	
	ros::NodeHandle nh_odom;
	odom_pub = nh_odom.advertise<nav_msgs::Odometry>("odom", 10);
	wheel_odom_pub = nh_odom.advertise<nav_msgs::Odometry>("wheel_odom", 10);
	//amcl_pose_sub = global_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, boost::bind(&AIV_Driver::AmclPoseCallback, this, _1));
	//nav_state_sub = global_nh.subscribe<colibri_msgs::NavState>("/nav_state", 10, boost::bind(&AIV_Driver::NavStateCallback, this, _1));

}

void AIV_Driver::TwistCallback(const geometry_msgs::Twist::ConstPtr & twist)
{

	static int twist_seq = 0;
	if(twist->linear.x < 0 )
	{
		send_twist_[VALID_DATA_START_INDX + 0] = NEG_SIGN;
		send_twist_[VALID_DATA_START_INDX + 1] = abs(100*twist->linear.x);
	}
	else
	{
		send_twist_[VALID_DATA_START_INDX + 0] = POS_SIGN;
		send_twist_[VALID_DATA_START_INDX + 1] = 100*twist->linear.x;
	}

	if(twist->angular.z < 0 )
	{
		send_twist_[VALID_DATA_START_INDX + 2] = NEG_SIGN;
		send_twist_[VALID_DATA_START_INDX + 3] = abs(100*twist->angular.z);
	}
	else
	{
		send_twist_[VALID_DATA_START_INDX + 2] = POS_SIGN;
		send_twist_[VALID_DATA_START_INDX + 3] = 100*twist->angular.z;
	}

	send_twist_[VALID_DATA_START_INDX + 8] = cur_music_mode_; //add music ctrl
	twist_seq++;
	if(twist_seq > 255)
	{
		twist_seq = 0;
	}
	send_twist_[VALID_DATA_START_INDX + 9] = twist_seq;
	
	send_cache_ = send_twist_;
	SendCmd(send_twist_, send_twist_finish_);
	
	send_cnt_++;
	//cout <<"send times: "<<send_cnt_<<endl;
}

void AIV_Driver::AuxInfoCallback(const colibri_msgs::AuxInfo::ConstPtr & aux_info)
{

	send_aux_info_[VALID_DATA_START_INDX + 0] = aux_info->lf_light;
	send_aux_info_[VALID_DATA_START_INDX + 1] = aux_info->lr_light;
	send_aux_info_[VALID_DATA_START_INDX + 2] = aux_info->rf_light;
	send_aux_info_[VALID_DATA_START_INDX + 3] = aux_info->rr_light;
	send_aux_info_[VALID_DATA_START_INDX + 4] = aux_info->horn;
	send_aux_info_[VALID_DATA_START_INDX + 5] = (int (aux_info->laser_mindis * 100))/256;
	send_aux_info_[VALID_DATA_START_INDX + 6] = (int (aux_info->laser_mindis * 100))%256;
	if(aux_info->laser_mindir < 0)
	{
		send_aux_info_[VALID_DATA_START_INDX + 7] = NEG_SIGN;
	}
	else
	{
		send_aux_info_[VALID_DATA_START_INDX + 7] = POS_SIGN;
	}
	send_aux_info_[VALID_DATA_START_INDX + 8] = int (aux_info->laser_mindir);
	send_aux_info_[VALID_DATA_START_INDX + 9] = aux_info->ultra_switch;
	send_aux_info_[VALID_DATA_START_INDX + 10] = aux_info->ros_fault;

	send_cache_ = send_aux_info_;
	SendCmd(send_aux_info_, send_aux_finish_);
	
	send_cnt_++;
	//cout <<"send times: "<<send_cnt_<<endl; 
}

void AIV_Driver::AmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_info)
{
	static int recv_amcl_cnt = 0;
	static bool lock_amcl = false;
	/*	
	recv_amcl_cnt++;

	if(recv_amcl_cnt > UPDATE_CNT)
	{
		correct_wheelodom_flag = true;
		correct_cartodom_flag = true;
		amcl_pose_x = pose_info->pose.pose.position.x;
		amcl_pose_y = pose_info->pose.pose.position.y;
		recv_amcl_cnt = 0;	
	}
	*/

	bool pose_rec_flag = (pose_info->pose.pose.position.x > -0.35) && (pose_info->pose.pose.position.x < 0.35) 
						 && (pose_info->pose.pose.position.y < 0.5) && (pose_info->pose.pose.position.y > -0.5);
	if(pose_rec_flag == true && lock_amcl == false)
	{
		correct_wheelodom_flag_ = true;
		correct_cartodom_flag_ = true;
		amcl_pose_.x = pose_info->pose.pose.position.x;
		amcl_pose_.y = pose_info->pose.pose.position.y;
		lock_amcl = true;	
	}

	if(pose_info->pose.pose.position.x > 3.5 || pose_info->pose.pose.position.x < 0.0)
	{
		lock_amcl = false;
	}

	amcl_quat_.x = pose_info->pose.pose.orientation.x;
	amcl_quat_.y = pose_info->pose.pose.orientation.y;
	amcl_quat_.z = pose_info->pose.pose.orientation.z;
	amcl_quat_.w = pose_info->pose.pose.orientation.w;

	amcl_pose_.yaw = tf::getYaw(amcl_quat_);
	ROS_INFO("correct_wheelodom_flag_: %d ", correct_wheelodom_flag_);

}

void AIV_Driver::CartoReal2CartoIdeal(float & x_real, float & y_real , float & x_ideal, float & y_ideal , float & theta)
{
	float trans_x = 0.0;
	float trans_y = 0.0;

	x_ideal = x_real * cos(theta) - y_real * sin(theta) + trans_x;
	y_ideal = x_real * sin(theta) + y_real * cos(theta) + trans_y;
}


void AIV_Driver:: NavStateCallback(const colibri_msgs::NavState::ConstPtr & nav_info)
{

	static bool lock_pose = false;

	cur_nav_state_.target_node = nav_info->target_node;
	cur_nav_state_.target_heading = nav_info->cur_seg;
	cur_nav_state_.cur_seg = nav_info->cur_seg;
	cur_nav_state_.at_target_flag = nav_info->at_target_flag;
	cur_nav_state_.achieve_flag = nav_info->achieve_flag;
	cur_nav_state_.task_succ_flag = nav_info->task_succ_flag;
	cur_nav_state_.target.x = nav_info->target_x;
	cur_nav_state_.target.y = nav_info->target_y;
	cur_nav_state_.target.yaw = nav_info->target_yaw;
	cur_nav_state_.robot.x = nav_info->cur_x;
	cur_nav_state_.robot.y = nav_info->cur_y;
	cur_nav_state_.robot.yaw = nav_info->cur_yaw;

	if(cur_nav_state_.cur_seg==0 && cur_nav_state_.task_succ_flag==1 && lock_pose==false)
	{
		amcl_pose_.x = cur_nav_state_.robot.x;
		amcl_pose_.y = cur_nav_state_.robot.y;
		correct_wheelodom_flag_ = true;
		correct_cartodom_flag_ = true;		
		lock_pose = true;
	}

	if(cur_nav_state_.cur_seg!=0)
	{
		lock_pose = false;
	}
	ROS_INFO("correct_cartodom_flag_: %d ", correct_cartodom_flag_);


}

void AIV_Driver::CalcCartodomByAmcl(float & frame_diff_angle)
{
	// calc carto delta odom 
	float cur_carto_x = cartodom_.x;
	float cur_carto_y = cartodom_.y;

	static float last_carto_x_ideal = 0.0;
	static float last_carto_y_ideal = 0.0;
	float cur_carto_x_ideal = 0.0;
	float cur_carto_y_ideal = 0.0;

	float delta_x = 0.0;
	float delta_y = 0.0;
	
	CartoReal2CartoIdeal(cur_carto_x, cur_carto_y, cur_carto_x_ideal, cur_carto_y_ideal , frame_diff_angle);
	
	delta_x = cur_carto_x_ideal - last_carto_x_ideal;
	delta_y = cur_carto_y_ideal - last_carto_y_ideal;

	ROS_DEBUG("cur_carto_ideal x/y last_carto_ideal x/y: %0.3lf %0.3lf %0.3lf %0.3lf",cur_carto_x_ideal,cur_carto_y_ideal,last_carto_x_ideal,last_carto_y_ideal);
		
	carto_odom_.x += delta_x;
	carto_odom_.y += delta_y;
	carto_odom_.yaw = cartodom_.yaw;

	if(correct_cartodom_flag_)
	{
		correct_cartodom_flag_ = false;
		carto_odom_.x = amcl_pose_.y - OFFSET_NY2GY;
		carto_odom_.y = -1.0 * (amcl_pose_.x - OFFSET_NX2GX);
	}

	last_carto_x_ideal = cur_carto_x_ideal;
	last_carto_y_ideal = cur_carto_y_ideal;	

}


void AIV_Driver::OdomException(bool & sys_stable)
{
	float odom_delta_x = carto_odom_.x - wheel_odom_.x;
	float odom_delta_y = carto_odom_.y - wheel_odom_.y;

	static int delay_cnt = 0;
	static bool start_opt_odom_flag = false;

	if(sys_stable && (start_opt_odom_flag == false))
	{
		delay_cnt++;
		if(delay_cnt > 100)
		{
			delay_cnt = 101;
			start_opt_odom_flag = true;
		}
		
	}

	if(start_opt_odom_flag)
	{
		if(abs(odom_delta_y) > ODOM_EXCEPT_GAP) //compare axis pose diff
		{
			opt_odom_x_ = wheel_odom_.x;
			opt_odom_y_ = wheel_odom_.y;
			odom_except_flag_ = true;
		}
		else
		{
			opt_odom_x_ = carto_odom_.x;
			opt_odom_y_ = carto_odom_.y;
			odom_except_flag_ = false;
		}
	}
	else
	{
		opt_odom_x_ = cartodom_.x;
		opt_odom_y_ = cartodom_.y;
	}

	ROS_INFO("start_opt_odom_flag: %d ", start_opt_odom_flag);

}


void AIV_Driver::CartodomCallback(const cartodom::Cartodom::ConstPtr & carto_odom)
{
	cartodom_.x = carto_odom->x;
	cartodom_.y = carto_odom->y;
	cartodom_.yaw = carto_odom->yaw;
	
	cartodom_quat_.qx = carto_odom->qx;
	cartodom_quat_.qy = carto_odom->qy;
	cartodom_quat_.qz = carto_odom->qz;
	cartodom_quat_.qw = carto_odom->qw;
	
	cartodom_vel_.vx = carto_odom->vx;
	cartodom_vel_.vth = carto_odom->vth;
	cartodom_interval_ = carto_odom->interval;
}

void AIV_Driver::MusicCallback(const colibri_msgs::MusicMode::ConstPtr & music_info)
{
	cur_music_mode_ = music_info->music_mode;
}

void AIV_Driver::SendCmd(const unsigned char *cmd ,volatile bool &send_flag)
{
	WriteToCom(cmd);
	send_flag = true;
	ros::Time start_time = ros::Time::now();

	while(send_flag == true)
	{
		if(ros::Time::now() - start_time > ros::Duration(TIMEOUT, 0))
		{
			switch(cmd[CMD_BYTE_INDX])
			{					
				case SEND_TWIST:
					cout<<"TRIO respones the send_twist_ cmd timeout !"<<endl;
					lost_twist_res_cnt++;
					if(lost_twist_res_cnt > 30)
					{
						system("roslaunch colibri_crabnav kill_aiv.launch");
					}
					break;	

				case REQ_VELOCITY:
					cout<<"TRIO respones the request_vel cmd timeout !"<<endl;
					break;
				case SEND_AUX:
					cout<<"TRIO respones the send_aux cmd timeout !"<<endl;
					break;

				default :
					break;
			}
			
			break;
		}
	 }

}

void AIV_Driver::DisplayFrame(const unsigned char *cmd)
{
	int i;
	
	for(i = 0;i < CONST_PROTOCOL_LEN; i ++)
	{
		printf(" %x",cmd[i]);		
	}
	
	cout <<endl;
}


void AIV_Driver::ParseWheelRpm(const unsigned char *valid_data)
{
	if(*(valid_data  + 0)== 0x00)
	{
		left_rot_rate_ = (float)((*(valid_data + 1) << 8) | (*(valid_data + 2)));
	}
	else if(*(valid_data  + 0)== 0xff)
	{
		left_rot_rate_ =(-1.0)*(float)((*(valid_data + 1) << 8) | (*(valid_data + 2)));
	}
	else
	{
		cout<<"[VALID_DATA_START_INDX + 0] of request_vel cmd is illegal: data sign is not correct!"<<endl;
		return;
	}

	
	
	if(*(valid_data + 3)== 0x00)
	{
		right_rot_rate_ = (float)((*(valid_data + 4) << 8) | (*(valid_data + 5)));

	}
	else if(*(valid_data + 3)== 0xff)
	{
		right_rot_rate_ =(-1.0)*(float)((*(valid_data + 4) << 8) | (*(valid_data + 5)));

	}
	else
	{
		cout<<"[VALID_DATA_START_INDX + 3] of request_vel cmd is illegal: data sign is not correct!"<<endl;
		return;
	}

}

void SmoothFrameRotAngle(float & correct_yaw, float & carto_yaw, float & amcl_yaw)
{
	static float rot_diff[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	float tmp_angle_diff = amcl_yaw - carto_yaw ;
	float angle_offset = 0.0;
	if(abs(tmp_angle_diff) >= 3.14)
	{
		tmp_angle_diff = 6.28 + tmp_angle_diff;
	}

	if(tmp_angle_diff <= 3.14)
	{

		if(abs(tmp_angle_diff - 1.57) > 0.785) // angle diff should < 45 degree
		{
			rot_diff[6] = correct_yaw;
		}
		else
		{
			rot_diff[6] = tmp_angle_diff - 1.57;
		}


		angle_offset = 0.05*rot_diff[0] + 0.1*rot_diff[1] + 0.2*rot_diff[2] + 0.3*rot_diff[3] + 0.2*rot_diff[4] + 0.1*rot_diff[5] + 0.05*rot_diff[6];
		rot_diff[0] = rot_diff[1];
		rot_diff[1] = rot_diff[2];
		rot_diff[2] = rot_diff[3];
		rot_diff[3] = rot_diff[4];
		rot_diff[4] = rot_diff[5];
		rot_diff[5] = rot_diff[6];
	}
		
	correct_yaw = angle_offset;

}

void Quaternion2Yaw(geometry_msgs::Quaternion &quat, double &yaw) {
	double x = 0.0;
	double y = 0.0;
	
	y = 2.0 * (quat.w * quat.z + quat.x * quat.y);
	x = 1.0 - 2.0 * (pow(quat.y, 2) + pow(quat.z, 2));
	yaw = atan2(y,x);	//return a radium angle
	
}


