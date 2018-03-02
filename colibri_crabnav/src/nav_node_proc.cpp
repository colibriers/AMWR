#include "nav_node_proc.h"

NavNodeProc::NavNodeProc()
{
	
#ifdef MANUAL_PATH

	string path_name;

	char user_name[10];
	getlogin_r(user_name, 10);
	string str_username = user_name;
	path_name.assign("/home/" + str_username + "/colibri_ws/src/colibri_crabnav/path/sh1029_mdf_cut_routes.yaml");

#else
	string path_name(routes_abs_path);
#endif
    cout <<"path_name: "<<path_name<<endl;
	ifstream fin_path(path_name.c_str());
	if(fin_path.fail())
	{
		cout<<"yaml file can not open in parse the yaml argv in proc"<<endl;
		exit(-1);
	}

	YAML::Node doc_path = YAML::Load(fin_path);
	try 
	{ 

		doc_path["path"]["image"] >> map_name_;
		doc_path["path"]["resolution"] >> map_resol_;
		doc_path["path"]["origin"][0] >> map_origin_[0];
		doc_path["path"]["origin"][1] >> map_origin_[1];
		doc_path["path"]["origin"][2] >> map_origin_[2];
		doc_path["path"]["map_size"][0] >> map_size_[0];
		doc_path["path"]["map_size"][1] >> map_size_[1];
		doc_path["path"]["seg_num"] >> segs_num_;

		seg_heading_.reserve(segs_num_);
		vec_seg_property_.reserve(segs_num_);
		
		string seg_prop_name;
		string seg_terminal_name;
		string seg_heading_name;
		stringstream sstr_num; 
		string num2str;
		float tmp_heading = 0.0;
		seg_property tmp_seg_prop;
		for(int seg_index = 1; seg_index <= segs_num_; seg_index++)
		{
			sstr_num << seg_index;
		    num2str = sstr_num.str();
			seg_prop_name = "seg" + num2str+ "_property";
			seg_terminal_name = "seg" + num2str+ "_vector";
			seg_heading_name =  "seg" + num2str+ "_heading";
				
			doc_path["path"][seg_prop_name][0] >> tmp_seg_prop.seg_id;
			doc_path["path"][seg_prop_name][1] >> tmp_seg_prop.start_id;
			doc_path["path"][seg_prop_name][2] >> tmp_seg_prop.end_id;
			
			doc_path["path"][seg_terminal_name][0] >> tmp_seg_prop.start.x;		
			doc_path["path"][seg_terminal_name][1] >> tmp_seg_prop.start.y;
			doc_path["path"][seg_terminal_name][2] >> tmp_seg_prop.end.x;
			doc_path["path"][seg_terminal_name][3] >> tmp_seg_prop.end.y;
			
			doc_path["path"][seg_heading_name] >> tmp_heading;

			seg_heading_.push_back(tmp_heading);
			vec_seg_property_.push_back(tmp_seg_prop);
			sstr_num.str("");
		}
	
	}
	catch (YAML::InvalidScalar) 
	{ 
		cout<<"The yaml does not contain an origin tag or it is invalid."<<endl;
		exit(-1);
	}


	cur_nav_node_ = 0;	//if node_id ==127 means no nav node to go shoule stop immediately
	obtain_goal_flag = false;

	robot_nav_state_.header.stamp = ros::Time::now();
	robot_nav_state_.header.frame_id = "robot";
	robot_nav_state_.target_node = 0;
	robot_nav_state_.target_heading = 0.0;
	robot_nav_state_.cur_seg = 44;
	robot_nav_state_.at_target_flag = false;
	robot_nav_state_.achieve_flag = false;
	robot_nav_state_.target_x = 0.0;
	robot_nav_state_.target_y = 0.0;
	robot_nav_state_.target_yaw = 0.0;
	robot_nav_state_.cur_x = 0.0;
	robot_nav_state_.cur_y = 0.0;
	robot_nav_state_.cur_yaw = 0.0;
	robot_nav_state_.err_code = 0;

	aiv_cmd_.target_node = 0;
	aiv_cmd_.clr_at_target = 0;
	aiv_cmd_.clr_achieve_target = 1;
	aiv_cmd_.basic_ctrl = 0;
	aiv_cmd_.cur_seg = 44;
	aiv_cmd_.pre_situated_node = 0;
	aiv_cmd_.task_succ_flag = 0;
	aiv_cmd_.music_mode = 127;
	aiv_cmd_.screen_mode = 127;

	

	pub_nav_state_ = nh_nav_node_.advertise<colibri_msgs::NavState>("/nav_state", 1);
	sub_robot_cmd_ = nh_nav_node_.subscribe<colibri_msgs::RobotCmd>("/robot_cmd", 1, &NavNodeProc::RobotCmdCallBack, this);
	sub_coordinator_ = nh_nav_node_.subscribe<colibri_msgs::Coordinator>("/coordinator", 1, &NavNodeProc::CoordinatorCallBack, this);
	//sub_node_id_ = nh_nav_node_.subscribe<colibri_msgs::NavNode>("/nav_node", 1, &NavNodeProc::NavNodeCallBack, this);


}

NavNodeProc::~NavNodeProc()
{

}

void NavNodeProc::LoadExistedRoute(void)
{

	string route_name;
	route_name.assign("/home/aiv-4/colibri_ws/src/colibri_crabnav/config/tasklist/hf910_dbg_route.yaml");

	ifstream fin_path(route_name.c_str());
	if(fin_path.fail())
	{
		cout<<"existed route yaml file can not open in parse the yaml argv in proc"<<endl;
		exit(-1);
	}

	YAML::Node doc_existed_route = YAML::Load(fin_path);
	try 
	{ 

		doc_existed_route["route"]["route_num"] >> exist_route_num_;
		exist_route_.reserve(exist_route_num_);
		
		string route_target_name;
		string route_seg_name;
		stringstream sstr_num; 
		string num2str;

		for(int route_index = 0; route_index < exist_route_num_; route_index++)
		{
			coordinator tmp;
			tmp.basic_ctrl = 0;
			sstr_num << route_index;
		    num2str = sstr_num.str();
			route_target_name = "route" + num2str+ "_target_property";
			route_seg_name = "route" + num2str+ "_seg_vec";
			
			doc_existed_route["route"][route_target_name][0] >> tmp.target_node;
			doc_existed_route["route"][route_target_name][1] >> tmp.route_seg_num;
			doc_existed_route["route"][route_target_name][2] >> tmp.target_heading;

			int i = 0;
			while(i < MAX_SEG_NUM)
			{
				if(i < tmp.route_seg_num)
				{
					doc_existed_route["route"][route_seg_name][i] >> tmp.seg_array[i];
				}
				else
				{
					tmp.seg_array[i] = 255;
				}
				i++;
			}
		
			exist_route_.push_back(tmp);
			sstr_num.str("");
		}
	
	}
	catch (YAML::InvalidScalar) 
	{ 
		cout<<"The existed route yaml does not contain an origin tag or it is invalid."<<endl;
		exit(-1);
	}



}

void NavNodeProc::LoadBranchNode(void)
{

		
#ifdef MANUAL_PATH
		string path_name;
		char user_name[10];
		getlogin_r(user_name, 10);
		string str_username = user_name;
		path_name.assign("/home/" + str_username + "/colibri_ws/src/colibri_crabnav/path/sh1029_mdf_cut_sp_nodes.yaml");
#else
		string path_name(spnodes_abs_path);
#endif


	branch_node_.clear();	
	vector<int> ().swap(branch_node_);

	ifstream fin_path(path_name.c_str());
	if(fin_path.fail())
	{
		cout<<"sp_nodes yaml file can not open in parse the yaml argv in proc"<<endl;
		exit(-1);
	}

	YAML::Node doc_path = YAML::Load(fin_path);
	try 
	{ 
		int branch_nodes_num = 0;
		int tmp_node_id = 127;
		doc_path["special_nodes"]["branch_nodes_num"] >> branch_nodes_num;
		branch_node_.reserve(branch_nodes_num);
		
		for(int node_index = 0; node_index < branch_nodes_num; node_index++)
		{
			doc_path["special_nodes"]["branch_nodes_id"][node_index] >> tmp_node_id;
			branch_node_.push_back(tmp_node_id);
		}
	
	}
	catch (YAML::InvalidScalar) 
	{ 
		cout<<"The sp_nodes yaml does not contain an origin tag or it is invalid."<<endl;
		exit(-1);
	}
	

}

void NavNodeProc::SetupBranchMap(void)
{

#ifdef MANUAL_PATH
	string path_name;
	char user_name[10];
	getlogin_r(user_name, 10);
	string str_username = user_name;
	path_name.assign("/home/" + str_username + "/colibri_ws/src/colibri_crabnav/path/sh1029_mdf_cut_branch_map.yaml");	
#else
	string path_name(branchmap_abs_path);
#endif

	nextnode_heading_map_.clear();	
	
	ifstream fin_path(path_name.c_str());
	if(fin_path.fail())
	{
		cout<<"branch_map yaml file can not open in parse the yaml argv in proc"<<endl;
		exit(-1);
	}

	YAML::Node doc_path = YAML::Load(fin_path);
	try 
	{ 
		int branch_maps_num = 0;
		int branch_nextnode = 127;
		float branch_heading = 0.0;
		doc_path["branch_map"]["map_num"] >> branch_maps_num;

		string branch_name;
		string route_seg_name;
		stringstream sstr_num; 
		string num2str;

		for(int branch_index = 1; branch_index <= branch_maps_num; branch_index++)
		{
			sstr_num << branch_index;
		    num2str = sstr_num.str();
			branch_name = "branch" + num2str+ "_map";

			doc_path["branch_map"][branch_name][1] >> branch_nextnode;
			doc_path["branch_map"][branch_name][2] >> branch_heading;
			nextnode_heading_map_.insert(make_pair(branch_nextnode, branch_heading));
			sstr_num.str("");
		}

	
	}
	catch (YAML::InvalidScalar) 
	{ 
		cout<<"The branch_map yaml does not contain an origin tag or it is invalid."<<endl;
		exit(-1);
	}
	

}


bool NavNodeProc::IsBranchNode(int &cur_node_id)
{
	if(find(branch_node_.begin(), branch_node_.end(), cur_node_id) != branch_node_.end())
	{
		return true;
	}
	else
	{
		return false;
	}

}


void NavNodeProc::NavNodeCallBack(const colibri_msgs::NavNode::ConstPtr& node)
{
	cur_nav_node_ = int (node->node_id);
	NavNode2NavPose();

}


void NavNodeProc::InitNodeAndSegMap(float *head_array, int &array_size)
{

	int j = 0;
	for(vector<seg_property>::iterator it = vec_seg_property_.begin(); it != vec_seg_property_.end(); ++it)
	{
		node_seg_map_.insert(make_pair((*it).end_id, (*it).seg_id));
		node_head_map_.insert(pair<int, float>((*it).end_id, *(head_array + j)));
		seg_node_map_.insert(make_pair((*it).seg_id, (*it).end_id));
		j++;
		if(j > array_size)
		{
			cout<<"Horriable, the pointer go out of the range..."<<endl;
			break;
		}
		
	}

}

void NavNodeProc::InitNodeAndSegMap(int &array_size)
{

	int j = 0;
	for(vector<seg_property>::iterator it = vec_seg_property_.begin(); it != vec_seg_property_.end(); ++it)
	{
		node_seg_map_.insert(make_pair((*it).end_id, (*it).seg_id));
		node_head_map_.insert(pair<int, float>((*it).end_id, seg_heading_[j]));
		seg_node_map_.insert(make_pair((*it).seg_id, (*it).end_id));
		j++;
		if(j > array_size)
		{
			cout<<"Horriable, the pointer go out of the range..."<<endl;
			break;
		}
		
	}

}


bool NavNodeProc::PubNavState(void)
{

	colibri_msgs::NavState nav_sta;
	nav_sta.header.stamp = ros::Time::now();
	nav_sta.header.frame_id = "robot";
	nav_sta.target_node = robot_nav_state_.target_node;
	nav_sta.target_heading = robot_nav_state_.target_heading;
	nav_sta.cur_seg = aiv_cmd_.cur_seg;
	nav_sta.at_target_flag = robot_nav_state_.at_target_flag;
	nav_sta.achieve_flag = robot_nav_state_.achieve_flag;
	nav_sta.task_succ_flag = aiv_cmd_.task_succ_flag;
	nav_sta.target_x = robot_nav_state_.target_x;
	nav_sta.target_y = robot_nav_state_.target_y;
	nav_sta.target_yaw = robot_nav_state_.target_yaw;
	nav_sta.cur_x = robot_nav_state_.cur_x;
	nav_sta.cur_y = robot_nav_state_.cur_y;
	nav_sta.cur_yaw = robot_nav_state_.cur_yaw;
	nav_sta.err_code = robot_nav_state_.err_code;
	
	pub_nav_state_.publish(nav_sta);

}

bool NavNodeProc::NavPose2NavNode(point2d_map & pose, int & rev_node_id)
{

	point2d_pix tmp_uv;
	vector<seg_property> tmp(vec_seg_property_);

	tmp_uv.x =  (pose.x - map_origin_[0]) / map_resol_;
	tmp_uv.y =  map_size_[1] - (pose.y - map_origin_[1]) / map_resol_;

	if(NavPixValid(tmp_uv))
	{
		
		for(vector<seg_property>::iterator it = tmp.begin(); it != tmp.end(); it++)
		{

			int delta_u = abs((*it).end.x - tmp_uv.x);
			int delta_v = abs((*it).end.y - tmp_uv.y);
			if( (delta_u < 3) && (delta_v < 3))
			{
				rev_node_id = (*it).end_id;
				return true;
			}

		}

		cout <<" Can not find the nav node from the request info"<<endl;
		rev_node_id = 255;
		return false;
		
	}
	else
	{
		rev_node_id = 255;
		return false;
	}

	
}

bool NavNodeProc::NavPixValid(point2d_pix &pix_uv)
{
	if(pix_uv.x >= map_size_[0] || pix_uv.x <= 0 || pix_uv.y >= map_size_[1] || pix_uv.y <= 0 )
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool NavNodeProc::ObtainNextNodeInRoute(int &cur_node_id, int & next_node)
{
	int cur_seg = node_seg_map_[cur_node_id];
	int next_seg = cur_seg;
	vector<int>::iterator it = find(total_route_segs_.begin(), total_route_segs_.end(), cur_seg);
	if(it != total_route_segs_.end())
	{
		it++;
		next_seg = *it;
		next_node = seg_node_map_[next_seg];
		return true;
	}
	else
	{
		next_node = cur_node_id;
		return false;
	}
	
}


bool NavNodeProc::NavNode2NavPose()
{	
	if(cur_nav_node_ != 255)
	{
		point2d_pix tmp_end_pix;
		vector<seg_property>::iterator it = find_if(vec_seg_property_.begin(), vec_seg_property_.end(),FindX<int,seg_property>(cur_nav_node_));
		if(it != vec_seg_property_.end())
		{
			tmp_end_pix = (*it).end;
			int tmp_x = tmp_end_pix.x;
			int tmp_y = map_size_[1] - tmp_end_pix.y;
			cur_goal[0] = (float) (map_origin_[0] + tmp_x * map_resol_);
			cur_goal[1] = (float) (map_origin_[1] + tmp_y * map_resol_);
			cur_goal[2] = node_head_map_[(*it).end_id];
			obtain_goal_flag = true;
		}
		else
		{
			cout<<"the input nav node not in the path nodes"<<endl;
			obtain_goal_flag = false;
			return false;
		}
	
	}
	else
	{
		cout<<"the input nav node is 255 for stop"<<endl;
		obtain_goal_flag = false;
		return false;
	}

}


void NavNodeProc::RobotCmdCallBack(const colibri_msgs::RobotCmd::ConstPtr& cmd)
{
	aiv_cmd_.target_node = int (cmd->target_node);
	aiv_cmd_.clr_at_target = int (cmd->clr_at_target);
	aiv_cmd_.clr_achieve_target = int (cmd->clr_achieve_target);
	aiv_cmd_.basic_ctrl = int (cmd->basic_ctrl);

	aiv_cmd_.cur_seg = int (cmd->cur_seg);
	aiv_cmd_.pre_situated_node = int (cmd->pre_situated_node);
	aiv_cmd_.task_succ_flag = int (cmd->task_succ_flag);
	aiv_cmd_.music_mode = int (cmd->music_mode);
	aiv_cmd_.screen_mode = int (cmd->screen_mode);

}

void NavNodeProc::CoordinatorCallBack(const colibri_msgs::Coordinator::ConstPtr& coord)
{
	total_route_segs_.clear();
	vector<int> ().swap(total_route_segs_);
	
	route_segs_ = coord->route_segs_num;
	total_route_segs_.reserve(route_segs_);
	for(int i = 0; i < route_segs_; i++)
	{
		total_route_segs_.push_back(coord->segs_vector[i]);
	}

}

void NavNodeProc::Quaternion2Yaw(const geometry_msgs::PoseStamped &pose, float &yaw)
{
	float x = 0.0;
	float y = 0.0;
	
	y = 2.0 * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y);
	x = 1.0 - 2 * (pow(pose.pose.orientation.y, 2) + pow(pose.pose.orientation.z, 2));
	yaw = atan2(y,x) * RAD2DEG;

}


