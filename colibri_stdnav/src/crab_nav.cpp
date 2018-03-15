#include "colibri_ca.h"
#include "colibri_local_nav.h"
#include "colibri_action.h"
#include "PID_controller.h"
#include "global_planner.h"
#include "task_mgr.h"
#include "nav_node_proc.h"

#include <boost/bind.hpp>

#include "geometry_msgs/PoseStamped.h"

#include<signal.h>

#define LASER_CA_LIMIT
//#define ULTRA_CA_LIMIT
//#define CA_LIMIT
//#define NO_LIMIT

#define LOW_VEL 0.24
#define NORMAL_VEL 0.5
#define HIGH_VEL 0.9

bool node_shutdown  = false;
int cnt_null_cmdvel = 0;
static int rot_escape_flag = 0;
static int rec_obs_dir	= 90;

unsigned int micro_adj_flag = 0;
unsigned int adjdir_flag = 0;
float route_end[2] = {10.0, 10.0};

string routes_abs_path;
string spnodes_abs_path;
string branchmap_abs_path;

float CalcVel(float & cmd_vel, float & fb_vel);
void PlannerCallback(planner *plannerObj, float* start_pos, float* goal_pos, bool *finish_flag);
void MySigintHandler(int sig);
int main(int argc, char* argv[])
{	

	// ROS nav node initial
	ros::init(argc, argv, "Nav_Mult_Goal_Node");
	ROS_INFO("Start to Go to Mult Goals in Rviz ... ");

#ifndef MANUAL_PATH
	routes_abs_path.assign(argv[1]);
	spnodes_abs_path.assign(argv[2]);
	branchmap_abs_path.assign(argv[3]);
	cout<<"Load Routes YAML Name: "<<routes_abs_path<<endl;
	cout<<"Load Special Nodes YAML Name: "<<spnodes_abs_path<<endl;
	cout<<"Load Branch Map YAML Name: "<<branchmap_abs_path<<endl;
#endif

	// Auto nav obj initial
	scan_ca scan4caObj;	
	local_nav local4navObj;
	nav_action actionObj;
	planner plannerObj;
	//task_mgr taskObj;
	NavNodeProc navNodeObj;

	// Local key points relation param initial
	float tmp_delta_dis = 0.0;
	float tmp_robot2goal_yaw = 0.0;
	float tmp_laser2goal_yaw = 0.0;
	float dir_goal_in_laser = 0.0;
	float self_rotation_angle = 0.0;
	bool goal_inlaser_flag = true;

	float tmp_action_cmd_t[2] = {0.0, 0.0};
	float* ptr_action_cmd_t = tmp_action_cmd_t;


	unsigned int adj_flag = 0;
	
	bool obtain_flag = false;
	unsigned int search_start = 0;
	
	ros::NodeHandle nh_pp;
	ros::Timer planner_timer;
	bool finish_plan = false;


	static unsigned int index4gravaton = 0; 

	float delta_robot2gravaton = 0.0;
	bool at_gravaton_flag = false;
	bool exist_gravaton_flag = false;
	bool replan_flag = false;
	
	float rt_r2g_dis = 100.0;

	ros::Rate loop_rate(10);		// Set control  freq at 10 hz
	unsigned int delay_cnt = 0;		// Init delay conter for scanObj 

	float ori_apf_linear = 0.0;
	float ori_apf_angular = 0.0;
	signal(SIGINT, MySigintHandler);

	point2d_map route_terminator = {0.0, 0.0};
	int route_terminator_node = 0;
	navNodeObj.InitNodeAndSegMap(navNodeObj.segs_num_);
	navNodeObj.LoadBranchNode();
	navNodeObj.SetupBranchMap();
	float tmp_pub_vx = 0.0;
	float ref_vel = 0.6;
	int length_path = 0;
	float diff_angle = 0.0;
	
	bool start_rot_flag = true;
	float start_clock_rot = -40.0;
	float start_anticlock_rot = 40.0;
	float start_rot_angular = PI / 4.0;	
	unsigned int clock_rot_flag = 0;
	unsigned int anticlock_rot_flag = 0;
	static bool clock_rot_finish = false;
	static bool anticlock_rot_finish = false;
	float start_heading = 0.0;
	unsigned int start_rot_finish = 0;
	float ang_tolerance = 6.0;

	while (ros::ok())
	{	
			
		if(delay_cnt < DELAY_CNT_MAX)	// Waiting for scan data
		{
			delay_cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}

		if(delay_cnt >= DELAY_CNT_MAX && start_rot_flag)
		{
			ptr_action_cmd_t = actionObj.StillRotatingAction(&local4navObj.cartodom_yaw,&start_clock_rot, start_rot_angular, ang_tolerance, &clock_rot_flag);
			if(clock_rot_flag == 1)
			{
				clock_rot_finish = true;
			}
			if(clock_rot_finish)
			{
				ptr_action_cmd_t = actionObj.StillRotatingAction(&local4navObj.cartodom_yaw,&start_anticlock_rot, start_rot_angular,ang_tolerance, &anticlock_rot_flag);
			}
			if(anticlock_rot_flag)
			{
				anticlock_rot_finish = true;
			}
			if(anticlock_rot_finish)
			{
				ptr_action_cmd_t = actionObj.StillRotatingAction(&local4navObj.cartodom_yaw,&start_heading, start_rot_angular, &start_rot_finish);
			}

			if(start_rot_finish == 1)
			{
				start_rot_flag = false;
			}
			ROS_DEBUG("--local4navObj.cartodom_yaw: %0.2lf deg", local4navObj.cartodom_yaw);
			
			local4navObj.apf_cmd_vel.linear.x = 0.0;
			local4navObj.apf_cmd_vel.angular.z = *(ptr_action_cmd_t + 1);
			local4navObj.LimitPubTwist(local4navObj.apf_cmd_vel);
			
			local4navObj.pub_apf_twist.publish(local4navObj.apf_cmd_vel);
						
			ros::spinOnce();
			loop_rate.sleep();
			
			if(node_shutdown == true)
			{
				local4navObj.apf_cmd_vel.linear.x = 0.0;
				local4navObj.apf_cmd_vel.angular.z = 0.0;
				++cnt_null_cmdvel;
				if(cnt_null_cmdvel > 5)
				{
					ros::shutdown();
				}
			}

		}	
		
		if(start_rot_flag == false)
		{

			//--------------------------  Calc gravaton for nav ------------------------------
			if(plannerObj.time_to_refresh == true)	// Path plan timer OK
			{
				plannerObj.time_to_refresh = false;
				index4gravaton = 0;
				replan_flag = true;				
			}
			else
			{	
				plannerObj.PrunePath(plannerObj.path_pruned_array, plannerObj.path_array, local4navObj.amcl_cur_state);	
				length_path = plannerObj.path_array.size() ;
				if(length_path < 36)
				{
					ref_vel = LOW_VEL;
				}
				else if(length_path < 96)
				{
					ref_vel = NORMAL_VEL;
				}
				else
				{
					ref_vel = HIGH_VEL;
				}	
				// If path replan or robot at gravaton but not in approaching target goal, calc a new gravaton in the existed planned path
				if((at_gravaton_flag == true && local4navObj.approaching_flag == false)||(replan_flag == true))	
				{
			
					plannerObj.CalcPath2RobotDeltaDis(plannerObj.path_pruned_array, local4navObj.amcl_cur_state);
					index4gravaton = plannerObj.CalcGravatonFromPath(plannerObj.path_pruned_array, plannerObj.path2robot_array, index4gravaton, plannerObj.gravaton, exist_gravaton_flag);
					at_gravaton_flag = false;
					replan_flag = false;
				}

				//judge the amcl pos and gravaton distance relation			
				at_gravaton_flag = actionObj.ReachGravatonOK(&local4navObj.amcl_cur_state[0],&plannerObj.gravaton.x, delta_robot2gravaton);

				// if robot goes to the setting target goal approching radius, set gravaton same as target goal 
				if(local4navObj.approaching_flag == true)
				{
					plannerObj.gravaton.x = route_end[0];
					plannerObj.gravaton.y = route_end[1];
					plannerObj.gravaton.yaw = 0.0;
				}
			}

			//---------------------------------------------------------------------
			
			local4navObj.CalcOffsetOfGoalAndRobot(local4navObj.amcl_cur_state, &plannerObj.gravaton.x, &tmp_delta_dis, &tmp_robot2goal_yaw, &tmp_laser2goal_yaw);

			goal_inlaser_flag = local4navObj.CalcGoalDirOfLaserView(&tmp_laser2goal_yaw, &local4navObj.amcl_cur_state[2], &dir_goal_in_laser, &self_rotation_angle);

			scan4caObj.CalcPhiParam(local4navObj.cur_robot_vel[0], dir_goal_in_laser);
			//scan4caObj.PubPfInfo4Dbg();

			scan4caObj.CalcKrfTheta(scan4caObj.kp_phi_vec, scan4caObj.phi_start_vec, scan4caObj.phi_end_vec);
			scan4caObj.CalcPassFcnWithoutRPF(&scan4caObj.max_passfcn_val, scan4caObj.passfcn_vec, &scan4caObj.angle_adj);

			ROS_INFO("Gravaton x/y: %0.2lf %0.2lf", plannerObj.gravaton.x, plannerObj.gravaton.y);
			ROS_INFO("Cur_state x/y/yaw: %0.2lf %0.2lf %0.1lf", local4navObj.amcl_cur_state[0],local4navObj.amcl_cur_state[1],local4navObj.amcl_cur_state[2]);

			scan4caObj.CalcAlarmInAPF();
			
			if(goal_inlaser_flag == true)
			{
				ori_apf_linear = (ref_vel - V_MIN) * (scan4caObj.max_passfcn_val / D_M) + V_MIN;

				//actionObj.CalcMicroRotAngle(tmp_robot2goal_yaw, local4navObj.amcl_cur_state[2],diff_angle);
				
				if(abs(scan4caObj.angle_adj) <= 2)
				{
					scan4caObj.angle_adj = 0; //clear the quake
				}
				ori_apf_angular = 1.32 * scan4caObj.angle_adj / 180.0;
				
				//ori_apf_angular = diff_angle / 150.0;

				local4navObj.apf_ctrl_output[0] = local4navObj.LinearVelFilter(&ori_apf_linear, &local4navObj.cur_robot_vel[0]);
				local4navObj.apf_ctrl_output[1] = local4navObj.AngularVelFilter(&ori_apf_angular, &local4navObj.cur_robot_vel[1]);
			}
			else	//if gravaton is not in front of  laser , should exec the still rot 
			{
				local4navObj.apf_ctrl_output[0] = 0.0;
				local4navObj.apf_ctrl_output[1] = 0.0;	
			}
							
			local4navObj.SatuateCmdVel(local4navObj.apf_ctrl_output, local4navObj.apf_ctrl_output+1);

			local4navObj.CalcEuclidDistance(local4navObj.amcl_cur_state, route_end, rt_r2g_dis);	// rt_r2g_dis(robot2goal) is different from tmp_delta_dis(robot2gravaton)	
			local4navObj.approaching_flag = local4navObj.ReachApprochingAreaOK(&rt_r2g_dis);


			if(local4navObj.approaching_flag == false)
			{
				*ptr_action_cmd_t = local4navObj.apf_ctrl_output[0];
				*(ptr_action_cmd_t + 1) = local4navObj.apf_ctrl_output[1];
			}
			else
			{
				if(micro_adj_flag == 0)
				{
					ptr_action_cmd_t = actionObj.ApproachingGoalAction(&local4navObj.amcl_cur_state[0], &route_end[0], &local4navObj.amcl_cur_state[2], local4navObj.cur_robot_vel[0], &micro_adj_flag);
				}
			}
						
			float terminal_angle;
			route_terminator.x = route_end[0];
			route_terminator.y = route_end[1];
			int branch_next_node = 127;
			if(navNodeObj.NavPose2NavNode(route_terminator, route_terminator_node))
			{
				if(navNodeObj.IsBranchNode(route_terminator_node))
				{
					navNodeObj.ObtainNextNodeInRoute(route_terminator_node, branch_next_node);
					terminal_angle = navNodeObj.nextnode_heading_map_[branch_next_node];	
				}
				else
				{
					terminal_angle = navNodeObj.node_head_map_[route_terminator_node];
				}

				navNodeObj.robot_nav_state_.target_node = route_terminator_node;
			}
			else
			{
				terminal_angle = 0.0;
				navNodeObj.robot_nav_state_.target_node = 100;
			}

			float angle_vel = PI / 2.0;		
			if(micro_adj_flag == 1)
			{

				ptr_action_cmd_t = actionObj.StillRotatingAction(&local4navObj.amcl_cur_state[2],&terminal_angle, angle_vel,  &adjdir_flag);
				if(adjdir_flag == 1)
				{
					*ptr_action_cmd_t = 0.0;
					*(ptr_action_cmd_t + 1) = 0.0;		
					navNodeObj.robot_nav_state_.achieve_flag = true;			
				}
				else
				{
					navNodeObj.robot_nav_state_.achieve_flag = false;	
				}
				navNodeObj.robot_nav_state_.at_target_flag = true;
			}
			else
			{
				navNodeObj.robot_nav_state_.at_target_flag = false;
			}

			local4navObj.SatuateCmdVel(ptr_action_cmd_t, ptr_action_cmd_t + 1);

			local4navObj.apf_cmd_vel.linear.x = *ptr_action_cmd_t;
			local4navObj.apf_cmd_vel.angular.z = *(ptr_action_cmd_t + 1);

			ROS_INFO("Tmp_delta_dis: %0.2lf m", tmp_delta_dis);			

			float tmp_linear = local4navObj.apf_cmd_vel.linear.x;
			float tmp_angluar = local4navObj.apf_cmd_vel.angular.z;

#ifdef LASER_CA_LIMIT

			float laser_safe_linear_vel = 0.0;
			float laser_safe_angular_vel = 0.0;
			int laser_steer = local4navObj.laser_safe_velocity.steer;
			local4navObj.CalcSafeLinearVel(tmp_linear, local4navObj.laser_safe_velocity.linear_safe_thd, &laser_safe_linear_vel);
			local4navObj.CalcSafeAngularVel(tmp_angluar, laser_steer, local4navObj.laser_safe_velocity.angular_safe_thd, &laser_safe_angular_vel);

			local4navObj.apf_cmd_vel.linear.x = laser_safe_linear_vel;
			local4navObj.apf_cmd_vel.angular.z = laser_safe_angular_vel;
#endif
			
#ifdef ULTRA_CA_LIMIT
			float ultra_safe_linear_vel = 0.0;
			float ultra_safe_angular_vel = 0.0;
			int ultra_steer = local4navObj.ultra_safe_velocity.steer;
			local4navObj.CalcSafeLinearVel(tmp_linear, local4navObj.ultra_safe_velocity.linear_safe_thd, &ultra_safe_linear_vel);
			local4navObj.CalcSafeAngularVel(tmp_angluar, ultra_steer, local4navObj.ultra_safe_velocity.angular_safe_thd, &ultra_safe_angular_vel);

			local4navObj.apf_cmd_vel.linear.x = ultra_safe_linear_vel;
			local4navObj.apf_cmd_vel.angular.z = ultra_safe_angular_vel;
#endif

#ifdef CA_LIMIT

			local4navObj.apf_cmd_vel.linear.x = MIN(laser_safe_linear_vel, ultra_safe_linear_vel);
			local4navObj.apf_cmd_vel.angular.z = MIN(laser_safe_angular_vel, ultra_safe_angular_vel);
#endif

/*
#ifdef NO_LIMIT
			if((local4navObj.position_OK_flag == true))
#endif
*/
#ifdef LASER_CA_LIMIT
			if((local4navObj.position_OK_flag == true)||(local4navObj.laser_safe_velocity.stop.data == true))
#endif

#ifdef ULTRA_CA_LIMIT
			if((local4navObj.position_OK_flag == true)||(local4navObj.ultra_safe_velocity.stop.data == true))
#endif


#ifdef CA_LIMIT
			if((local4navObj.position_OK_flag == true)||(local4navObj.laser_safe_velocity.stop.data == true)||(local4navObj.ultra_safe_velocity.stop.data == true))
#endif

			{
				local4navObj.apf_cmd_vel.linear.x = 0.0;
				local4navObj.apf_cmd_vel.angular.z = 0.0;
			}

			if(node_shutdown == true)
			{
				local4navObj.apf_cmd_vel.linear.x = 0.0;
				local4navObj.apf_cmd_vel.angular.z = 0.0;
				++cnt_null_cmdvel;
				if(cnt_null_cmdvel > 5)
				{
					ros::shutdown();
				}
			}

			float cur_cmd_vel = local4navObj.apf_cmd_vel.linear.x;
			local4navObj.apf_cmd_vel.linear.x = CalcVel(cur_cmd_vel, local4navObj.cur_robot_vel[0]);

			local4navObj.LimitPubTwist(local4navObj.apf_cmd_vel);

			navNodeObj.robot_nav_state_.target_x = route_end[0]; 
			navNodeObj.robot_nav_state_.target_y = route_end[1];
			navNodeObj.robot_nav_state_.target_yaw = local4navObj.goal_state[2];
			navNodeObj.robot_nav_state_.cur_x = local4navObj.amcl_cur_state[0];
			navNodeObj.robot_nav_state_.cur_y = local4navObj.amcl_cur_state[1];
			navNodeObj.robot_nav_state_.cur_yaw = local4navObj.amcl_cur_state[2];


			navNodeObj.PubNavState();
			local4navObj.pub_apf_twist.publish(local4navObj.apf_cmd_vel);
			
			scan4caObj.ResetMaxPassValCnt();
			
			ROS_INFO("cmd_vel: %0.2lf m/s  %0.2lf rad/s", local4navObj.apf_cmd_vel.linear.x, local4navObj.apf_cmd_vel.angular.z);
			ROS_INFO("micro_adj_flag: %d --adjdir_flag: %d", micro_adj_flag, adjdir_flag);
			
			tmp_pub_vx = local4navObj.apf_cmd_vel.linear.x;

			
			if(navNodeObj.aiv_cmd_.clr_achieve_target == 1)
			{
				micro_adj_flag = 0;
				adjdir_flag = 0;
				navNodeObj.robot_nav_state_.achieve_flag = false;			
			}
			
			ros::spinOnce();
			loop_rate.sleep();
			
		}
		
	}

	local4navObj.apf_cmd_vel.linear.x = 0.0;
	local4navObj.apf_cmd_vel.linear.z = 0.0;
	local4navObj.pub_apf_twist.publish(local4navObj.apf_cmd_vel);

	return 0;	
}

void PlannerCallback(planner *plannerObj, float* start_pos, float* goal_pos, bool *finish_flag)
{		
	plannerObj->ObtainPathArray(plannerObj->serviceClient, plannerObj->path_srv, start_pos, goal_pos, finish_flag);
	route_end[0] = plannerObj->path_array.back().x;
	route_end[1] = plannerObj->path_array.back().y;	
	cout<<"Timer to call planner..."<<endl;
}

void MySigintHandler(int sig)
{
	node_shutdown = true;
}

float CalcVel(float & cmd_vel, float & fb_vel)
{
	float output = cmd_vel;
	const float acc_thd = 0.25;
	const float dec_thd = -0.12;
	
	if(cmd_vel - fb_vel > acc_thd)
	{
		output = fb_vel + acc_thd/2.0;
	}

	if(cmd_vel - fb_vel < dec_thd)
	{
		output = cmd_vel + 1.5*dec_thd;
	}

	if(output < 0)
	{
	   output = 0;
	}
	else if(output > V_MAX)
	{
	   output = V_MAX;
	}
	else
	{

	}	
	
	return output;

}

