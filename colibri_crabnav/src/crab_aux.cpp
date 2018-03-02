#include "colibri_ca.h"
#include "colibri_local_nav.h"
#include "colibri_action.h"
#include "PID_controller.h"
#include "global_planner.h"
#include "task_mgr.h"
#include "nav_node_proc.h"

#include <math.h>
#include <vector>
#include <algorithm>

#include "colibri_msgs/MusicMode.h"
#include "colibri_msgs/NavState.h"
#include "colibri_battery/Battery.h"
#include "colibri_msgs/SafeVel.h"

bool get_safevel_flag = false;
int obs_rec_coder = 0;
bool get_bat_flag = false;
float bat_vol_ratio = 1.0;
bool get_navsta_flag = false;
nav_state cur_nav_state;

enum music_mode{SILENCE, ALARM, RUNNING, AVOIDENCE, CHARGING, DOWNLOAD, UPLOAD};

music_mode cur_mode = SILENCE;

void SafeVelCallBack(const colibri_msgs::SafeVel::ConstPtr& safe_vel)
{
	obs_rec_coder = safe_vel->rsvd;
	get_safevel_flag = true;

};
void BatteryCallBack(const colibri_battery::Battery::ConstPtr& bat_info)
{
	bat_vol_ratio = bat_info->SOC/100.0;
	get_bat_flag = true;

};
void NavStateCallBack(const colibri_msgs::NavState::ConstPtr& nav_sts)
{

	cur_nav_state.target_node = nav_sts->target_node;
	cur_nav_state.target_heading = nav_sts->target_heading;
	cur_nav_state.cur_seg = nav_sts->cur_seg;
	cur_nav_state.at_target_flag = nav_sts->at_target_flag;
	cur_nav_state.achieve_flag = nav_sts->achieve_flag;
	cur_nav_state.task_succ_flag = nav_sts->task_succ_flag;
	cur_nav_state.target.x = nav_sts->target_x;
	cur_nav_state.target.y = nav_sts->target_y;
	cur_nav_state.target.yaw = nav_sts->target_yaw;
	cur_nav_state.robot.x = nav_sts->cur_x;
	cur_nav_state.robot.y = nav_sts->cur_y;
	cur_nav_state.robot.yaw = nav_sts->cur_yaw;
	cur_nav_state.err_code = nav_sts->err_code;
	
	get_navsta_flag = true;
};

bool IsElemInVector(vector<int> & vec, int &elem)
{
    vector<int>::iterator it = find(vec.begin(), vec.end(), elem);
    if(it == vec.end())
   	{
		return false;
	}
    else
   	{
		return true;
	}
};

int main(int argc, char* argv[])
{	

	// ROS nav node initial
	ros::init(argc, argv, "Nav_Aux_Node");
	ros::NodeHandle nh_aux;
	ros::Subscriber sub_safe_vel;
	ros::Subscriber sub_bat_info;
	ros::Subscriber sub_nav_state;
	ros::Publisher pub_music_mode;

	ros::Rate loop_rate(5);
	/* //sz929map
	int upload_nodes[] = {5, 6, 23, 24, 25, 26, 27, 28};
	int download_nodes[] = {31, 32, 33, 34, 35, 36, 37, 13, 14, 15};
	int charging_nodes[] = {41, 42, 43, 44};
	*/

	//ych1008map
	int upload_nodes[] = {6, 21, 22, 23, 24, 25, 26, 27};
	int download_nodes[] = {30, 31, 32, 34, 35, 13};
	int charging_nodes[] = {40, 41, 42, 43};

	size_t tmp_len = sizeof(upload_nodes)/sizeof(int);
	vector<int> vec_upload(upload_nodes, upload_nodes + tmp_len);
	tmp_len = sizeof(download_nodes)/sizeof(int);
	vector<int> vec_download(download_nodes, download_nodes + tmp_len);

	sub_safe_vel = nh_aux.subscribe<colibri_msgs::SafeVel>("/laser_safe_vel", 1, &SafeVelCallBack);
	sub_bat_info = nh_aux.subscribe<colibri_battery::Battery>("/battery_info", 1, &BatteryCallBack);
	sub_nav_state = nh_aux.subscribe<colibri_msgs::NavState>("/nav_state", 1, &NavStateCallBack);
	pub_music_mode = nh_aux.advertise<colibri_msgs::MusicMode>("/music", 1);

	colibri_msgs::MusicMode cur_music_mode;

	static int last_target_node = 0;
	while (ros::ok())
	{	
	
		switch(cur_mode)
		{
			case SILENCE:
				if(cur_nav_state.target_node != 100)
				{
					if(obs_rec_coder == 6 || obs_rec_coder == 7 )
					{
						cur_mode = AVOIDENCE;
					}
					else
					{
						if(bat_vol_ratio < 0.15)
						{
							cur_mode = SILENCE; // to mask the Charging
						}
						else
						{
							if(cur_nav_state.task_succ_flag == 1)
							{
								if(IsElemInVector(vec_upload, cur_nav_state.target_node))
								{
									cur_mode = UPLOAD;
								}
					
								if(IsElemInVector(vec_download, cur_nav_state.target_node))
								{
									cur_mode = DOWNLOAD;
								}
								
							}
						}
					
					}

				}
				else
				{
					cur_mode = SILENCE;
				}
				
				break;

			case RUNNING:
				break;

			case UPLOAD:
				if(cur_nav_state.task_succ_flag == 0)
				{
					if(last_target_node != cur_nav_state.target_node)
					{
						cur_mode = SILENCE;
					}
				}

				break;	
				
			case DOWNLOAD:
				if(cur_nav_state.task_succ_flag == 0)
				{
					if(last_target_node != cur_nav_state.target_node)
					{
						cur_mode = SILENCE;
					}
				}
			
				break;
			case CHARGING:
				cur_mode = SILENCE;				
				break;	
			case ALARM:
				break;	
			case AVOIDENCE:
				if(obs_rec_coder == 0)
				{
					cur_mode = SILENCE;
				}
				if(bat_vol_ratio < 0.15)
				{
					cur_mode = CHARGING;
				}

				
				break;	

			default :
				cur_mode = SILENCE;
				break;		
	
		}


		cur_music_mode.music_mode = cur_mode;
		pub_music_mode.publish(cur_music_mode);

		last_target_node = cur_nav_state.target_node;
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;	
}

