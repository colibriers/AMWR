#include "colibri_ca.h"
#include "colibri_local_nav.h"
#include "colibri_action.h"
#include "PID_controller.h"
#include "global_planner.h"

#include <boost/bind.hpp>

void PlannerCallback(planner *plannerObj, float* start_pos, float* goal_pos, bool *finish_flag);

int main(int argc, char* argv[])
{	
	ros::init(argc, argv, "Follow_path_node");
	
	scan_ca scan4caObj;	
	local_nav local4navObj;
	nav_action actionObj;
	planner plannerObj;
		
	ros::Rate loop_rate(10);

	ROS_INFO("Start to Follow a path ... ");

	unsigned int delay_cnt = 0;
	
	int range_num = 0;
	float tmp_theta_laseray = 0.0;	//obstacle 's theta angle
	

	float tmp_delta_dis = 0.0;
	float tmp_robot2goal_yaw = 0.0;
	float tmp_laser2goal_yaw = 0.0;
	bool goal_inlaser_flag = true;
	float dir_goal_in_laser = 0.0;
	float self_rotation_angle = 0.0;

	unsigned int turn_adj_flag = 1;

	float tmp_action_cmd_t[2] = {0.0, 0.0};
	float* ptr_action_cmd_t = tmp_action_cmd_t;
	float newyaw = 145.0;
	unsigned int micro_adj_flag = 0;

	unsigned int adj_flag = 0;
	
	bool obtain_flag = false;
	unsigned int search_start = 0;
	
	ros::NodeHandle nh_fp;
	ros::Timer planner_timer;
	bool finish_plan = false;

	bool tmp_timer_finish =false;
	bool *timer_finish = &tmp_timer_finish;

	static unsigned int index4gravaton = 0; 

	float delta_robot2gravaton = 0.0;
	bool at_gravaton_flag = false;
	bool exist_gravaton_flag = false;
	bool replan_flag = false;

	float rt_r2g_dis = 100.0;

	while (!ros::service::waitForService(plannerObj.srv4make_plan, ros::Duration(0.2)))
	{
		ROS_INFO("Waiting for service move_base/make_plan to become available");
	}
	
	ROS_INFO("Service move_base/make_plan prepared OK...");

	finish_plan = plannerObj.ExecMonoPlanAndGravaton(plannerObj,&local4navObj.cur_robot_state[0],&local4navObj.goal_state[0], search_start,index4gravaton);
	
	planner_timer = nh_fp.createTimer(ros::Duration(PLAN_INTERVAL), boost::bind(&PlannerCallback, &plannerObj, &local4navObj.cur_robot_state[0],&local4navObj.goal_state[0], timer_finish));

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
			ROS_INFO("------ start ------");
			if(*timer_finish == true)
			{
				*timer_finish = false;
				index4gravaton = 0;
				replan_flag = true;

			}else
			{
						
				if((at_gravaton_flag == true && local4navObj.approaching_flag == false)||(replan_flag == true))
				{
					plannerObj.CalcPath2RobotDeltaDis(plannerObj.path_array, local4navObj.cur_robot_state);
					index4gravaton = plannerObj.CalcGravatonFromPath(plannerObj.path_array, plannerObj.path2robot_array, index4gravaton, plannerObj.gravaton,exist_gravaton_flag);
					at_gravaton_flag = false;
					
					replan_flag = false;

				}

				cout<<" Waiting for next path plan..."<<endl;
				
				cout<<" plannerObj.gravation.x = "<<plannerObj.gravaton.x<<endl;
				cout<<" plannerObj.gravation.y = "<<plannerObj.gravaton.y<<endl;		
				
				at_gravaton_flag = actionObj.ReachGravatonOK(&local4navObj.cur_robot_state[0],&plannerObj.gravaton.x, delta_robot2gravaton);
				cout<<" delta_robot2gravaton = "<<delta_robot2gravaton<<endl;
				cout<<" at_gravaton_flag : "<<at_gravaton_flag<<endl;
				
				if(local4navObj.approaching_flag == true)
				{
					plannerObj.gravaton.x = local4navObj.goal_state[0];
					plannerObj.gravaton.y = local4navObj.goal_state[1];
					plannerObj.gravaton.yaw = local4navObj.goal_state[2];

				}
			}

		
			local4navObj.CalcOffsetOfGoalAndRobot(local4navObj.cur_robot_state, &plannerObj.gravaton.x, &tmp_delta_dis, &tmp_robot2goal_yaw, &tmp_laser2goal_yaw);

			goal_inlaser_flag = local4navObj.CalcGoalDirOfLaserView(&tmp_laser2goal_yaw, &local4navObj.cur_robot_state[2], &dir_goal_in_laser, &self_rotation_angle);

			for(int i = 0; i < NUM_RAY4CA; i++)
			{
				scan4caObj.delta_phi_vec[i] = asin(D_SF / (*(scan4caObj.ptrScan4ca + i))) * RAD2DEG;
				scan4caObj.kp_phi_vec[i] = scan4caObj.CalcKpPhi(local4navObj.cur_robot_vel[0], *(scan4caObj.ptrScan4ca + i));
				range_num = floor(scan4caObj.delta_phi_vec[i] / RAY_RESOL4CA);
				
				scan4caObj.CalcPhiRange(i,range_num,&scan4caObj.phi_start_vec[i-1],&scan4caObj.phi_end_vec[i-1]);

				tmp_theta_laseray = i; 			// unit in degree
				scan4caObj.kaf_vec[i] = cos((tmp_theta_laseray - dir_goal_in_laser) * DEG2RAD);  //dir_goal_in_laser is 0~180 degree from right side
				scan4caObj.krf_vec[i] = scan4caObj.kp_phi_vec[i];
			}

			scan4caObj.CalcKrfTheta(scan4caObj.kp_phi_vec, scan4caObj.phi_start_vec, scan4caObj.phi_end_vec);
			scan4caObj.CalcCorrectedKrf();
			scan4caObj.CalcPassFcnAndFwdBnd(scan4caObj.wander, &scan4caObj.max_passfcn_val, scan4caObj.passfcn_vec);
			scan4caObj.CalcPassFcnAndBwdBnd(scan4caObj.wander, &scan4caObj.max_passfcn_val, scan4caObj.passfcn_vec);

			scan4caObj.angle_adj = scan4caObj.CalcAdjDir(scan4caObj.passfcn_vec,scan4caObj.max_passfcn_val, &scan4caObj.maxfcn_fwdbnd,&scan4caObj.maxfcn_bwdbnd);
				
			scan4caObj.CalcAlarmInAPF();
			
			if(goal_inlaser_flag == true)
			{
				local4navObj.apf_ctrl_output[0] = (V_MAX - V_MIN) * (scan4caObj.max_passfcn_val / D_M) + V_MIN;
				local4navObj.apf_ctrl_output[1] = scan4caObj.angle_adj / 200.0;	
			}
			else
			{
				local4navObj.apf_ctrl_output[0] = 0.0;
				local4navObj.apf_ctrl_output[1] = 0.0;	
			}
							
			local4navObj.SatuateCmdVel(local4navObj.apf_ctrl_output,local4navObj.apf_ctrl_output+1);

			local4navObj.CalcEuclidDistance(local4navObj.cur_robot_state, local4navObj.goal_state, rt_r2g_dis);
			
			local4navObj.approaching_flag = local4navObj.ReachApprochingAreaOK(&rt_r2g_dis);
			
			if(tmp_delta_dis >= GOAL_NGHBORHD)
			{
				ptr_action_cmd_t = actionObj.AdjustMovingDirAction(&local4navObj.cur_robot_state[2], &dir_goal_in_laser, &tmp_robot2goal_yaw, &turn_adj_flag);
			}
			else
			{

			}

//			cout<<"turn_adj_flag: " << turn_adj_flag <<endl;
			
			if(turn_adj_flag == 1)
			{
				if(local4navObj.approaching_flag == false)
				{
					*ptr_action_cmd_t = local4navObj.apf_ctrl_output[0];
					*(ptr_action_cmd_t + 1) = local4navObj.apf_ctrl_output[1];
				}
				else
				{
					ptr_action_cmd_t = actionObj.ApproachingGoalAction(&local4navObj.cur_robot_state[0],&local4navObj.goal_state[0],&dir_goal_in_laser,&micro_adj_flag);
				}

			}
			
			cout<<"current_robot_state[0]"<<local4navObj.cur_robot_state[0]<<endl;
			cout<<"current_robot_state[1]"<<local4navObj.cur_robot_state[1]<<endl;
			cout<<"current_robot_state[2]"<<local4navObj.cur_robot_state[2]<<endl;
			
			cout<<"adj_angle: "<< scan4caObj.angle_adj <<endl;

			cout<<"tmp_delta_dis: "<<tmp_delta_dis<<endl;
			cout<<"tmp_robot2goal_yaw: "<<tmp_robot2goal_yaw<<endl;
			cout<<"tmp_laser2goal_yaw: "<<tmp_laser2goal_yaw<<endl;
	
			cout<<"dir_goal_in_laser: "<<dir_goal_in_laser<<endl;
			
			//local4navObj.position_OK_flag = local4navObj.ReachGoalPositionOK(&tmp_delta_dis);

			local4navObj.SatuateCmdVel(ptr_action_cmd_t,ptr_action_cmd_t + 1);

			local4navObj.apf_cmd_vel.linear.x = *ptr_action_cmd_t;
			local4navObj.apf_cmd_vel.angular.z = *(ptr_action_cmd_t + 1);
			
			if(local4navObj.position_OK_flag == true)
			{
				local4navObj.apf_cmd_vel.linear.x = 0.0;
				local4navObj.apf_cmd_vel.angular.z = 0.0;
			}
			
			cout<<"approaching_flag: " << local4navObj.approaching_flag <<endl;
			cout<<"position_OK_flag: " << local4navObj.position_OK_flag <<endl;

			local4navObj.pub_apf_twist.publish(local4navObj.apf_cmd_vel); 
			cout<<"pub_linear_x: " << local4navObj.apf_cmd_vel.linear.x <<endl;
			cout<<"pub_angular_z: " << local4navObj.apf_cmd_vel.angular.z <<endl;

			scan4caObj.ResetMaxPassValCnt();
		
			ros::spinOnce();
			loop_rate.sleep();
			ROS_INFO("------- end --------");
		}
	}

	local4navObj.apf_cmd_vel.linear.x = 0.0;
	local4navObj.apf_cmd_vel.angular.z = 0.0;

	local4navObj.pub_apf_twist.publish(local4navObj.apf_cmd_vel); 

	return 0;	
}


void PlannerCallback(planner *plannerObj, float* start_pos, float* goal_pos, bool *finish_flag)
{		
	plannerObj->ObtainPathArray(plannerObj->serviceClient, plannerObj->path_srv,start_pos,goal_pos,finish_flag);
}



