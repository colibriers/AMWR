#include "colibri_ca.h"
#include "colibri_local_nav.h"

int main(int argc, char* argv[])
{	
	ros::init(argc, argv, "Fixedpoint_nav_node");
	scan_ca scan4caObj;
	local_nav local4navObj;
	
	ros::Rate loop_rate(10);

	ROS_INFO("Start to Nav in Local Area ... ");

	float v_0 = 0.0;
	int range_num = 0;

	float tmp_theta_obs = 0.0;	//obstacle 's theta angle
	float tmp_passfcn_value = 0.0;
	
	int delay_cnt = 0;
	
	float tmp_delta_dis = 0.0;
	float tmp_robot2goal_yaw = 0.0;
	float tmp_laser2goal_yaw = 0.0;
	bool goal_inlaser_flag = true;
	float dir_goal_in_laser = 0.0;
	float self_rotation_angle = 0.0;

	while (ros::ok())
	{	
			
		if(delay_cnt < DELAY_CNT_MAX)
		{
			delay_cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		if(delay_cnt >= DELAY_CNT_MAX)
		{
			ROS_INFO("---start ... ...");
			
			local4navObj.CalcOffsetOfGoalAndRobot(local4navObj.cur_robot_state, local4navObj.goal_state, &tmp_delta_dis, &tmp_robot2goal_yaw, &tmp_laser2goal_yaw);

			goal_inlaser_flag = local4navObj.CalcGoalDirOfLaserView(&tmp_laser2goal_yaw, &local4navObj.cur_robot_state[2], &dir_goal_in_laser, &self_rotation_angle);

			for(int i = 0; i < NUM_RAY4CA; i++)
			{
				scan4caObj.delta_phi_vec[i] = asin(D_SF / (*(scan4caObj.ptrScan4ca + i))) * RAD2DEG;
				scan4caObj.kp_phi_vec[i] = scan4caObj.CalcKpPhi(v_0, *(scan4caObj.ptrScan4ca + i));
				range_num = floor(scan4caObj.delta_phi_vec[i] / RAY_RESOL4CA);
				
				scan4caObj.CalcPhiRange(i,range_num,&scan4caObj.phi_start_vec[i-1],&scan4caObj.phi_end_vec[i-1]);

				tmp_theta_obs = i; 			// unit in degree
				scan4caObj.kaf_vec[i] = cos((tmp_theta_obs - dir_goal_in_laser) * DEG2RAD);
				scan4caObj.krf_vec[i] = scan4caObj.kp_phi_vec[i];

			}

			scan4caObj.CalcKrfTheta(scan4caObj.kp_phi_vec, scan4caObj.phi_start_vec, scan4caObj.phi_end_vec);
		
			scan4caObj.CalcPassFcnAndFwdBnd(scan4caObj.wander, &scan4caObj.max_passfcn_val, scan4caObj.passfcn_vec);
			scan4caObj.CalcPassFcnAndBwdBnd(scan4caObj.wander, &scan4caObj.max_passfcn_val, scan4caObj.passfcn_vec);

			scan4caObj.angle_adj = scan4caObj.CalcAdjDir(scan4caObj.passfcn_vec,scan4caObj.max_passfcn_val, &scan4caObj.maxfcn_fwdbnd,&scan4caObj.maxfcn_bwdbnd);
				
			scan4caObj.CalcAlarmInAPF();

			cout<<"current_robot_state[0]"<<local4navObj.cur_robot_state[0]<<endl;
			cout<<"current_robot_state[1]"<<local4navObj.cur_robot_state[1]<<endl;
			cout<<"current_robot_state[2]"<<local4navObj.cur_robot_state[2]<<endl;
			
			cout<<"adj_angle: "<< scan4caObj.angle_adj <<endl;
			cout<<"passfcn_max_value: "<< scan4caObj.max_passfcn_val <<endl;
			cout<<"apf_alarm: "<< scan4caObj.apf_alarm <<endl;

			cout<<"tmp_delta_dis: "<<tmp_delta_dis<<endl;
			cout<<"tmp_robot2goal_yaw: "<<tmp_robot2goal_yaw<<endl;
			cout<<"tmp_laser2goal_yaw: "<<tmp_laser2goal_yaw<<endl;
	
			cout<<"dir_goal_in_laser: "<<dir_goal_in_laser<<endl;
		
			if(goal_inlaser_flag == true)
			{
				local4navObj.apf_ctrl_output[0] = (V_MAX - V_MIN) * (scan4caObj.max_passfcn_val / D_M) + V_MIN;
				local4navObj.apf_ctrl_output[1] = scan4caObj.angle_adj / 300.0;	
			}
			else
			{
				local4navObj.apf_ctrl_output[0] = 0.0;
				local4navObj.apf_ctrl_output[1] = 0.0;	
			}


			local4navObj.SatuateCmdVel(local4navObj.apf_ctrl_output,local4navObj.apf_ctrl_output+1);
			local4navObj.approaching_flag = local4navObj.ReachApprochingAreaOK(&tmp_delta_dis);

			if(local4navObj.approaching_flag == false)
			{
				local4navObj.apf_cmd_vel.linear.x = local4navObj.apf_ctrl_output[0];
				local4navObj.apf_cmd_vel.angular.z = local4navObj.apf_ctrl_output[1];
			}
			else
			{
				local4navObj.apf_cmd_vel.linear.x = 0.0;
				local4navObj.apf_cmd_vel.angular.z = 0.0;
			}
			
			cout<<"pub vel	x: "<<local4navObj.apf_cmd_vel.linear.x<<endl;
			cout<<"pub angular z: : "<<local4navObj.apf_cmd_vel.angular.z<<endl;

			local4navObj.pub_apf_twist.publish(local4navObj.apf_cmd_vel);

			scan4caObj.ResetMaxPassValCnt();

		
			ros::spinOnce();
			loop_rate.sleep();
			ROS_INFO("---end ...");
		}
	}

	return 0;	
}



