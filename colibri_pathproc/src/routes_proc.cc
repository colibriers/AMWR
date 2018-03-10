#include "path_proc.h"

bool get_coordinator_flag = false;

PathProc::PathProc()
{

#ifdef MANUAL_PATH
	string path_name;

	char user_name[10];
	getlogin_r(user_name, 10);
	string str_username = user_name;
	path_name.assign("/home/" + str_username + "/colibri_ws/src/colibri_pathproc/routes/ygl1012_mdf_cut_routes.yaml");

#else
	string path_name(routes_path);
#endif

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
		doc_path["path"]["origin"] >> tt;
		doc_path["path"]["origin"][0] >> map_origin_[0];
		doc_path["path"]["origin"][1] >> map_origin_[1];
		doc_path["path"]["origin"][2] >> map_origin_[2];
		doc_path["path"]["map_size"][0] >> map_size_[0];
		doc_path["path"]["map_size"][1] >> map_size_[1];
		doc_path["path"]["seg_num"] >> segs_num_;
		
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
			seg_prop_name = "seg" + num2str + "_property";
			seg_terminal_name = "seg" + num2str + "_vector";
			seg_heading_name = "seg" + num2str + "_heading";
		
			doc_path["path"][seg_prop_name][0] >> tmp_seg_prop.seg_id;
			doc_path["path"][seg_prop_name][1] >> tmp_seg_prop.start_id;
			doc_path["path"][seg_prop_name][2] >> tmp_seg_prop.end_id;
			
			doc_path["path"][seg_terminal_name][0] >> tmp_seg_prop.start.x;		
			doc_path["path"][seg_terminal_name][1] >> tmp_seg_prop.start.y;
			doc_path["path"][seg_terminal_name][2] >> tmp_seg_prop.end.x;
			doc_path["path"][seg_terminal_name][3] >> tmp_seg_prop.end.y;

			doc_path["path"][seg_heading_name] >> tmp_heading;

			segs_heading_.push_back(tmp_heading);

			vec_seg_property_.push_back(tmp_seg_prop);
			sstr_num.str("");
		}
	
	}
	catch (YAML::InvalidScalar) 
	{ 
		cout<<"The yaml does not contain an origin tag or it is invalid."<<endl;
		exit(-1);
	}

	parsed_node_ = 255;
	req4path_flag = false;

	robot_nav_state_.target_node = 0;
	robot_nav_state_.target_heading = 0.0;
	robot_nav_state_.cur_seg = 0;
	robot_nav_state_.at_target_flag = false;
	robot_nav_state_.achieve_flag = false;
	robot_nav_state_.target.x = 0.0;
	robot_nav_state_.target.y = 0.0;
	robot_nav_state_.target.yaw = 0.0;
	robot_nav_state_.robot.x = 0.0;
	robot_nav_state_.robot.y = 0.0;
	robot_nav_state_.robot.yaw = 0.0;
	robot_nav_state_.err_code = 0;

	robot_cmd_.target_node = 0;
	robot_cmd_.clr_at_target = 1;
	robot_cmd_.clr_achieve_target = 1;
	robot_cmd_.basic_ctrl = 0;
	robot_cmd_.cur_seg = 44;
	robot_cmd_.pre_situated_node = 0;
	robot_cmd_.task_succ_flag = 1;
	robot_cmd_.music_mode = 127;
	robot_cmd_.screen_mode = 127;


	cur_route_.target_id = 0;
	cur_route_.target_heading = 0.0;
	cur_seg_ = 48;
	micro_seg_num_ = 1;
	task_switch_ = false;

	sub_seg_index_cache_ = 1;


	//cout<<"Load map: "<<map_name_<<endl;
	//cout<<"Path Segment Num: "<<segs_num_<<endl;

	pub_route_ = nh_route_.advertise<nav_msgs::Path>("/nav_path", 1);
	sub_coodinator_ = nh_route_.subscribe<colibri_msgs::Coordinator>("/coordinator", 1, &PathProc::CoordinatorCallBack, this);
	sub_nav_state_ = nh_route_.subscribe<colibri_msgs::NavState>("/nav_state", 1, &PathProc::NavStateCallBack, this);
	pub_marker_ = nh_route_.advertise<visualization_msgs::Marker>("waypoint_markers", 1);
 	//srv4getpath_ = nh_route_.advertiseService("/move_base/make_plan", &PathProc::ExecGetPathSrv, this);
	pub_robot_cmd_ = nh_route_.advertise<colibri_msgs::RobotCmd>("/robot_cmd", 1);
	//pub_task_state_ = nh_route_.advertise<colibri_msgs::TaskState>("/task_state", 1);




}

PathProc::~PathProc()
{

}

void PathProc::InitMarkers(void)
{
	goalmark_list_.ns       = "waypoints";
	goalmark_list_.id       = 0;
	goalmark_list_.type     = visualization_msgs::Marker::CUBE_LIST;
	goalmark_list_.action   = visualization_msgs::Marker::ADD;
	goalmark_list_.lifetime = ros::Duration();//0 is forever
	goalmark_list_.scale.x  = 0.2;
	goalmark_list_.scale.y  = 0.2;
	goalmark_list_.color.r  = 1.0;
	goalmark_list_.color.g  = 0.0;
	goalmark_list_.color.b  = 0.0;
	goalmark_list_.color.a  = 1.0;

	goalmark_list_.header.frame_id = "map";
	goalmark_list_.header.stamp = ros::Time::now();

}

int PathProc::FillMarkerPose(route_list & route)
{

	goalmark_list_.points.clear();
	geometry_msgs::Pose subgoal_list;

	for(vector<int>::iterator it = route.seg_list.begin(); it != route.seg_list.end(); ++it)
	{
		vector<segment>::iterator tmp_it = find_if(vec_seg_.begin(), vec_seg_.end(),FindX<int,segment>(*it));
		if(tmp_it != vec_seg_.end())
		{
			subgoal_list.position.x = (*tmp_it).points_map.back().x;
			subgoal_list.position.y = (*tmp_it).points_map.back().y;
			subgoal_list.orientation.w = 1.0;
			goalmark_list_.points.push_back(subgoal_list.position);
		}
	}
	return goalmark_list_.points.size();
}

/*
void PathProc::FillTaskState(void)
{

	task_state_.cur_seg = cur_seg_;
	task_state_.pre_node = 255;
	task_state_.nav_task_node = cur_route_.target_id;
	if(sub_seg_index == (micro_seg_num_ - 1) && robot_nav_state_.achieve_flag == true)
	{
		task_state_.task_succ_flag = 1;
	}
	else
	{
		task_state_.task_succ_flag = 0;
	}

	task_state_.rsvd = 255;
}
*/

bool PathProc::CalcNearestNode(float & robot_x, float &robot_y, int & nearest_node)
{
	vector<float> delta_r2node;
	float tmp_delta = 0.0;
	float tmp_node_x = 0.0;
	float tmp_node_y = 0.0;
	int index = 0;
	for(vector<segment>::iterator it = vec_seg_.begin(); it != vec_seg_.end(); ++it)
	{
		tmp_node_x = it->points_map.back().x;
		tmp_node_y = it->points_map.back().y;
		tmp_delta = sqrt(pow(tmp_node_x - robot_x, 2) + pow(tmp_node_y - robot_y, 2));
		delta_r2node.push_back(tmp_delta);
	}

	vector<float>::iterator shortest = min_element(delta_r2node.begin(), delta_r2node.end());   

	index = distance(delta_r2node.begin(), shortest);
	nearest_node = vec_seg_[index].end_id;

	if(*shortest < 0.5)
	{
		return true;
	}
	else
	{
		return false;
	}

}

void PathProc::FillRobotCmd(void)
{
	static int confirm_cnt = 0;
	int near_node = 0;
	float robot_x = 0.0;
	float robot_y = 0.0;

	robot_cmd_.target_node = cur_route_.target_id ;
	
	if(robot_nav_state_.at_target_flag == true)
	{
		robot_cmd_.clr_at_target = 1;
	}
	else
	{
		robot_cmd_.clr_at_target = 0;

	}

	robot_cmd_.basic_ctrl = basic_ctrl_;

	robot_cmd_.pre_situated_node = 127;
	
	if(sub_seg_index_cache_ == (micro_seg_num_ - 1)) 
	{
		if(robot_nav_state_.achieve_flag == true)
		{
			confirm_cnt ++;
		}
		
		if(confirm_cnt > 25)
		{
			robot_cmd_.task_succ_flag = 1;
			confirm_cnt = 0;
		}
		else
		{
		
		}
		
	}
	else
	{
		robot_cmd_.task_succ_flag = 0;
	}

	if(robot_cmd_.task_succ_flag == 1)
	{
		robot_x = robot_nav_state_.robot.x;
		robot_y = robot_nav_state_.robot.y;
		CalcNearestNode(robot_x, robot_y, near_node);
		robot_cmd_.cur_seg  = near_node;
	}
	else
	{
		robot_cmd_.cur_seg = seg_prenode_map_[cur_seg_];
	}

	robot_cmd_.music_mode = 127;
	robot_cmd_.screen_mode = 127; 
}



/* Calc the bresham line pix coordinate from all existed segs to fill in segment struct vec_seg_*/
void PathProc::CalcAllPointsInSegs(void) 
{
	vec_seg_.clear();
	vector<segment> ().swap(vec_seg_);
	segment tmp;
	
	for (vector<seg_property>::iterator it = vec_seg_property_.begin(); it!=vec_seg_property_.end(); ++it)
	{

		segment tmp;
		tmp.seg_id = (*it).seg_id;
		tmp.start_id = (*it).start_id;
		tmp.end_id = (*it).end_id;
		CalcPixesInLine((*it).start, (*it).end, tmp.points_pix);
		Pix2Map(tmp.points_pix, tmp.points_map);
		vec_seg_.push_back(tmp);

	}
	
}

void PathProc::Seg2LengthMap(void)
{
	int tmp_length = 0;
	seg_length_map_.clear();
	
	for(vector<segment>::iterator it = vec_seg_.begin(); it != vec_seg_.end(); ++it)
	{
		tmp_length = (*it).points_map.size() - 1;
		seg_length_map_.insert(pair<int, int>((*it).seg_id, tmp_length));
	}
}

void PathProc::Pix2Map(vector<point2d_pix> &points_pix, vector<point2d_map> &points_map)
{
	point2d_map tmp;
	points_map.clear();
	vector<point2d_map> ().swap(points_map);
	for (vector<point2d_pix>::iterator it = points_pix.begin(); it!=points_pix.end(); ++it)
	{
		int tmp_x = (*it).x;
		int tmp_y = map_size_[1] - (*it).y;
		tmp.x = (float) (map_origin_[0] + tmp_x * map_resol_);
		tmp.y = (float) (map_origin_[1] + tmp_y * map_resol_);
		points_map.push_back(tmp);
		
	}
}

/* Calc the whole  map and pix route removing the seg start from the route_list struct */
void PathProc::CatSeg2Route(route_list &route)
{
	route_pix_.clear();
	vector<point2d_pix> ().swap(route_pix_);
	route_map_.clear();
	vector<point2d_map> ().swap(route_map_);
	for (vector<int>::iterator it = route.seg_list.begin(); it!=route.seg_list.end(); ++it)
	{
		vector<segment>::iterator tmp = find_if(vec_seg_.begin(), vec_seg_.end(),FindX<int,segment>(*it));
		segment tmp_rm_start(*tmp);
		tmp_rm_start.points_pix.erase(tmp_rm_start.points_pix.begin());	// remove the start point
		tmp_rm_start.points_map.erase(tmp_rm_start.points_map.begin());
		route_pix_.insert(route_pix_.end(), tmp_rm_start.points_pix.begin(), tmp_rm_start.points_pix.end());
		route_map_.insert(route_map_.end(), tmp_rm_start.points_map.begin(), tmp_rm_start.points_map.end());
	}

}

/* Calc the sub segs from a known seg_list which is the whole route from hostpc and decompose it into sub_route_vec_*/
bool PathProc::DecomposeRoute(vector<int> &seg_list, vector<int> &check_nodes, int &sub_route_num)
{
	sub_route_vec_.clear();
	vector<route_list> ().swap(sub_route_vec_);

	vector<int> remain_segs(seg_list);
	route_list tmp_sub_route;
	sub_route_num = 0;
	for(vector<int>::iterator it = remain_segs.begin(); it != remain_segs.end(); ++it) //get every segs in the whole route
	{
		int tmp_node = seg_node_map_[*it];
		vector<int>::iterator iElement = find(check_nodes.begin(), check_nodes.end(), tmp_node);// check the node in route is in check_nodes
		if(iElement != check_nodes.end())	//node in the check_nodes
		{
			for(vector<int>::iterator sub_it = remain_segs.begin(); sub_it <= it; ++sub_it)
			{
				tmp_sub_route.seg_list.push_back(*sub_it);
			}
			tmp_sub_route.target_id = *iElement;
			tmp_sub_route.target_heading = node_heading_map_[*iElement];
			sub_route_vec_.push_back(tmp_sub_route);
			sub_route_num++;
		}
		tmp_sub_route.seg_list.clear();
		vector<int> ().swap(tmp_sub_route.seg_list);		
	}


	int sub_num = sub_route_vec_.size(); 
	for(int i = 1; i < sub_num; i++)
	{
		for(vector<int>::iterator index = sub_route_vec_[sub_num-1-i].seg_list.begin(); index < sub_route_vec_[sub_num-1-i].seg_list.end(); ++index)
		{
			vector<int>::iterator iElem = find(sub_route_vec_[sub_num-i].seg_list.begin(), sub_route_vec_[sub_num-i].seg_list.end(), (*index));
			sub_route_vec_[sub_num-i].seg_list.erase(iElem);
		}
	}


	return true;
	
}
/*
bool PathProc::ExecGetPathSrv(nav_msgs::GetPlan::Request & req, nav_msgs::GetPlan::Response & res)
{

	point2d_map tmp_start,tmp_goal;
	bool parse_node_flag = false;
	
	tmp_start.x = req.start.pose.position.x;
	tmp_start.y = req.start.pose.position.y;
	tmp_goal.x = req.goal.pose.position.x;
	tmp_goal.y = req.goal.pose.position.y;
	cout<<"req start.x: "<<tmp_start.x<<endl;
	cout<<"req start.y: "<<tmp_start.y<<endl;
	cout<<"req goal.x: "<<tmp_goal.x<<endl;
	cout<<"req goal.y: "<<tmp_goal.y<<endl;	
	
	parse_node_flag = MapPose2NavNode(tmp_goal, parsed_node_);
	
	if(parse_node_flag == true)
	{
		route_list route;
		route.target_id = parsed_node_;
		route.target_heading = 0.0;
		int micro_seg_num = 1;

		for(int j= 0; j < parsed_node_; j++)
		{
			route.seg_list.push_back(j);
		}
		
		AddTargetNode2KneeNodes(route.target_id);
		DecomposeRoute(route.seg_list, knee_nodes_, micro_seg_num);
		
		if(micro_seg_num != 1)
		{
			if(robot_nav_state_.achieve_flag && (inc_seg_flag == false))
			{
				sub_seg_index++;
				inc_seg_flag = true;
				robot_cmd_.clr_achieve_target = 1;
				robot_nav_state_.achieve_flag = false;
			}
			else
			{
				robot_cmd_.clr_achieve_target = 0;
				
			}
			if(sub_seg_index >= micro_seg_num)
			{
				sub_seg_index = micro_seg_num - 1; 
				cout<<"Exception sub_seg_index "<<endl;
			}

			CatSeg2Route(sub_route_vec_[sub_seg_index]);
			
		}
		else
		{
			if(robot_nav_state_.achieve_flag)
			{
				robot_cmd_.clr_achieve_target = 1;
			}
			else
			{
				robot_cmd_.clr_achieve_target = 0;
			}

			CatSeg2Route(route);
			
		}
		StdNavPath(route_map_);
		FillMarkerPose(route);


#ifdef REC_PATH
		for(vector<point2d_map>::iterator it = route_map_.begin(); it != route_map_.end(); ++it)
		{
			file1 << fixed << setprecision(4) << (*it).x;
			file1 << '\t';
			file1 << fixed << setprecision(4) << (*it).y;
			file1 << endl;	
		}

		file1.close();	
#endif

		res.plan = this->plan_path_;
		req4path_flag = true;

	}
	else
	{
		return false;
	}

	return true;
	
}
*/

bool PathProc::StdNavPath(vector<point2d_map> &nav_path)
{
	if(nav_path.empty())
	{
		return false;
	}
	else
	{
		plan_path_.poses.clear();
		geometry_msgs::PoseStamped tmp_pose_stamped;
		
		plan_path_.header.stamp = ros::Time::now();
		plan_path_.header.frame_id = "map";
		tmp_pose_stamped.header.stamp = ros::Time::now();
		tmp_pose_stamped.header.frame_id = "map";

		size_t len = nav_path.size();
		for (size_t i = 0; i < len; i++)
		{
			tmp_pose_stamped.pose.position.x = nav_path[i].x;			
			tmp_pose_stamped.pose.position.y = nav_path[i].y;
			tmp_pose_stamped.pose.position.z = 0.0;
			tmp_pose_stamped.pose.orientation.x = 0.0;
			tmp_pose_stamped.pose.orientation.y = 0.0;
			tmp_pose_stamped.pose.orientation.z = 0.0;
			tmp_pose_stamped.pose.orientation.w = 1.0;
			plan_path_.poses.push_back(tmp_pose_stamped);
		}
		
		return true;
	}
	
}

bool PathProc::MapPose2NavNode(point2d_map & pose, int & rev_node_id)
{

	point2d_pix tmp_uv;
	vector<seg_property> tt(vec_seg_property_);

	tmp_uv.x =  (pose.x - map_origin_[0]) / map_resol_;
	tmp_uv.y =  map_size_[1] - (pose.y - map_origin_[1]) / map_resol_;

	if(NavPixValid(tmp_uv))
	{
		
		for(vector<seg_property>::iterator it = tt.begin(); it != tt.end(); it++)
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

bool PathProc::NavPixValid(point2d_pix &pix_uv)
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

int PathProc::CalcRobotOnCurSeg(point2d_map & cur_pose, route_list &cur_route, vector<point2d_map> & straight_path)
{
	int path_total_len = straight_path.size();
	vector<float> delta_dis;
	delta_dis.reserve(path_total_len);
	static int cur_seg = 48;
	static int last_cur_seg = 48;
	float delta_x, delta_y, delta_distance;
	
	if(cur_route.seg_list.empty())
	{
		cur_seg_ = last_cur_seg;
		return cur_seg_;	
	}
	else
	{
	
	}

	for(vector<point2d_map>::iterator it = straight_path.begin(); it != straight_path.end(); ++it)
	{
		delta_x = it->x - cur_pose.x;
		delta_y = it->y - cur_pose.y;
		delta_distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
		delta_dis.push_back(delta_distance);
	}

	vector<float>::iterator it_min = min_element(delta_dis.begin(), delta_dis.end());
	int gap = distance(delta_dis.begin(), it_min);

	vector<int> stairs_len;
	int seg_index_cnt = 0;

	CalcLengthStairs(cur_route.seg_list, stairs_len);

	for(vector<int>::iterator it = stairs_len.begin(); it != stairs_len.end(); ++it)
	{
		if(gap < *it)
		{
			cur_seg = cur_route.seg_list.at(seg_index_cnt);
			break;
		}
		
		seg_index_cnt++;
	}
	cur_seg_ = cur_seg;
	last_cur_seg = cur_seg_;

	return cur_seg_;
}

void PathProc::CalcLengthStairs(vector<int> & path_seg_id, vector<int> &len_stairs)
{
	int path_seg_num = path_seg_id.size();
	len_stairs.reserve(path_seg_num);
	int acc_len = 0;
	vector<int> tmp_len_vec;
	tmp_len_vec.reserve(path_seg_num);

	for(vector<int>::iterator it = path_seg_id.begin(); it != path_seg_id.end(); ++it)
	{
		tmp_len_vec.push_back(seg_length_map_[*it]);	
	}

	for(vector<int>::iterator it_len = tmp_len_vec.begin(); it_len != tmp_len_vec.end(); ++it_len)
	{
		acc_len = accumulate(tmp_len_vec.begin(), it_len, tmp_len_vec.front());
		len_stairs.push_back(acc_len);
	}

}


void PathProc::MakeNodeSegMap(vector<float> &vec_heading)
{
	int i = 0;
	for(vector<seg_property>::iterator it = vec_seg_property_.begin(); it != vec_seg_property_.end(); ++it)
	{
		node_seg_map_.insert(pair<int, int>((*it).end_id, (*it).seg_id));
		seg_node_map_.insert(pair<int, int>((*it).seg_id, (*it).end_id));
		seg_prenode_map_.insert(pair<int, int>((*it).seg_id, (*it).start_id));
		node_heading_map_.insert(pair<int, float>((*it).end_id, vec_heading[i]));
		i++;
	}
}

void PathProc::MakeNodeSegMap(void)
{
	int i = 0;
	for(vector<seg_property>::iterator it = vec_seg_property_.begin(); it != vec_seg_property_.end(); ++it)
	{
		node_seg_map_.insert(pair<int, int>((*it).end_id, (*it).seg_id));
		seg_node_map_.insert(pair<int, int>((*it).seg_id, (*it).end_id));
		seg_prenode_map_.insert(pair<int, int>((*it).seg_id, (*it).start_id));
		node_heading_map_.insert(pair<int, float>((*it).end_id, segs_heading_[i]));
		i++;
	}
}


void PathProc::ConfigNodesHeading(float *head_array, int &array_size)
{
	nodes_heading_.clear();
	vector<float> ().swap(nodes_heading_);
	for(int i = 0; i < array_size; i++)
	{
		nodes_heading_.push_back(*(head_array + i));
	}

}

void PathProc::InitKneeNodes(int *node_array, int &array_size)
{
	knee_nodes_.clear();
	vector<int> ().swap(knee_nodes_);
	for(int i = 0; i < array_size; i++)
	{
		knee_nodes_.push_back(*(node_array + i));
	}
}

void PathProc::InitKneeNodes(void)
{
	
#ifdef MANUAL_PATH
	string path_name;

	char user_name[10];
	getlogin_r(user_name, 10);
	string str_username = user_name;
	path_name.assign("/home/" + str_username + "/colibri_ws/src/colibri_pathproc/routes/ygl1012_mdf_cut_sp_nodes.yaml");

#else
	string path_name(sp_nodes_path);
#endif

	knee_nodes_.clear();	
	vector<int> ().swap(knee_nodes_);

	ifstream fin_path(path_name.c_str());
	if(fin_path.fail())
	{
		cout<<"sp_nodes yaml file can not open in parse the yaml argv in proc"<<endl;
		exit(-1);
	}

	YAML::Node doc_path = YAML::Load(fin_path);
	try 
	{ 
		int sp_nodes_num = 0;
		int tmp_node_id = 127;
		doc_path["special_nodes"]["sp_nodes_num"] >> sp_nodes_num;
		
		for(int node_index = 0; node_index < sp_nodes_num; node_index++)
		{
			doc_path["special_nodes"]["sp_nodes_id"][node_index] >> tmp_node_id;
			knee_nodes_.push_back(tmp_node_id);
			updated_knee_nodes_.push_back(tmp_node_id);
		}
	
	}
	catch (YAML::InvalidScalar) 
	{ 
		cout<<"The sp_nodes yaml does not contain an origin tag or it is invalid."<<endl;
		exit(-1);
	}



}


bool PathProc::AddTargetNode2KneeNodes(int &target_node)
{
	updated_knee_nodes_.clear();
	vector<int> ().swap(updated_knee_nodes_);
	
	vector<int> tmp_knee(knee_nodes_);

	for(vector<int>::iterator it = tmp_knee.begin(); it != tmp_knee.end(); ++it)
	{
		updated_knee_nodes_.push_back(*it);
	}
		
	vector<int>::iterator iElement = find(knee_nodes_.begin(), knee_nodes_.end(), target_node);	
	if(iElement == knee_nodes_.end())
	{
		updated_knee_nodes_.push_back(target_node);	
		return true;
	}
	else
	{
		return false;
	}

}


void PathProc::NavStateCallBack(const colibri_msgs::NavState::ConstPtr& nav_state)
{
	static int last_target_node = 0;
	static int rec_flag = false;
	
	robot_nav_state_.target_node = nav_state->target_node;
	robot_nav_state_.target_heading = nav_state->cur_seg;
	robot_nav_state_.cur_seg = nav_state->cur_seg;
	robot_nav_state_.at_target_flag = nav_state->at_target_flag;
	robot_nav_state_.achieve_flag = nav_state->achieve_flag;
	robot_nav_state_.task_succ_flag = nav_state->task_succ_flag;
	robot_nav_state_.target.x = nav_state->target_x;
	robot_nav_state_.target.y = nav_state->target_y;
	robot_nav_state_.target.yaw = nav_state->target_yaw;
	robot_nav_state_.robot.x = nav_state->cur_x;
	robot_nav_state_.robot.y = nav_state->cur_y;
	robot_nav_state_.robot.yaw = nav_state->cur_yaw;
	robot_nav_state_.err_code = nav_state->err_code;

	if(last_target_node != robot_nav_state_.target_node)
	{
		new_seg_flag = true;
	}

	if(new_seg_flag == true && robot_nav_state_.achieve_flag == false)
	{
		lock_seg_flag = false;
	}

	last_target_node = robot_nav_state_.target_node;
	
}

void PathProc::CoordinatorCallBack(const colibri_msgs::Coordinator::ConstPtr& coordinator)
{
	int seg_num = 0;
	cur_route_.seg_list.clear();
	vector<int> ().swap(cur_route_.seg_list);
	basic_ctrl_ = coordinator->basic_ctrl;
	cur_route_.target_id = coordinator->target_node;
	cur_route_.target_heading = coordinator->target_heading;
	seg_num = coordinator->route_segs_num;
	
	if(seg_num == 0)
	{
		get_coordinator_flag = false;
		return;
	}
	else
	{
		for(int i = 0; i < seg_num; i++)
		{
			cur_route_.seg_list.push_back(coordinator->segs_vector[i]);
		}
		get_coordinator_flag = true;
		
		if(last_task_node != cur_route_.target_id)
		{
			task_switch_ = true;
		}
		else
		{
			task_switch_ = false;
		}
		last_task_node =  cur_route_.target_id;

	}

	//cout<<"seg_num in Coordinator callback: "<<seg_num<<endl;

	
}

void PathProc::ClearFlags4NextTask(void)
{
	static int delay_cnt = 0;
	static bool lock_flag = false;
	if(task_switch_ == true)
	{
		lock_flag = true;
	}

	if((lock_flag == true) && (delay_cnt < 10))
	{
		sub_seg_index = 0;
		new_seg_flag = false;
		lock_seg_flag = false;
		micro_seg_num_ = 1;
		task_switch_ = false;
		robot_cmd_.clr_at_target = 0;
		robot_cmd_.clr_achieve_target = 1;
		robot_cmd_.task_succ_flag = 0;
		sub_seg_index_cache_ = 1;
		delay_cnt++;
	}
	else
	{
		delay_cnt = 0;
		lock_flag = false;

	}
	
	
}

void PathProc::HandleRecvRoute(void)
{
	AddTargetNode2KneeNodes(cur_route_.target_id);
	DecomposeRoute(cur_route_.seg_list, updated_knee_nodes_, micro_seg_num_);
	//cout<<"+++ micro_seg_num_ : "<<micro_seg_num_<<endl;
	//cout<<"--- sub_seg_index : "<<sub_seg_index<<endl;
	
	if(micro_seg_num_ > 1)
	{
		if(robot_nav_state_.achieve_flag && (lock_seg_flag == false))
		{
			sub_seg_index++;
			if(sub_seg_index == micro_seg_num_)
			{
				sub_seg_index = micro_seg_num_ - 1;
				robot_cmd_.clr_achieve_target = 0;
			}
			else
			{
				robot_cmd_.clr_achieve_target = 1;
			}
			lock_seg_flag = true;
		}
		else
		{
			robot_cmd_.clr_achieve_target = 0;			
		}

		if(sub_seg_index < micro_seg_num_)
		{
			CatSeg2Route(sub_route_vec_[sub_seg_index]);
		}
		
	}
	else
	{
		sub_seg_index = 0;
		/*
		if(robot_nav_state_.achieve_flag)
		{
			robot_cmd_.clr_achieve_target = 1;
		}
		else
		{
			robot_cmd_.clr_achieve_target = 0;
		}
		*/
		CatSeg2Route(sub_route_vec_[sub_seg_index]);
		
	}

	sub_seg_index_cache_ = sub_seg_index;
	StdNavPath(route_map_);


}

bool VerticalLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &ver_line)
{
	
	point2d_pix tmp_point;
	
	int index = start.y;
	if(start.y < end.y)
	{
		do
		{
			tmp_point.x = start.x;
			tmp_point.y = index;
			ver_line.push_back(tmp_point);
			index++;
		}while(index <= end.y); 	
	}
	else
	{
		do
		{
			tmp_point.x = start.x;
			tmp_point.y = index;
			ver_line.push_back(tmp_point);
			index--;
		}while(index >= end.y); 	
	}

	return true;

}

bool BresenhamBasic(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &point_at_line) 
{  
	// Calc the slop [0, 1] bresenham
	int dx = fabs(end.x - start.x);
	int dy = fabs(end.y - start.y);  
    int p = 2 * dy - dx;  
    int twoDy = 2 * dy;
	int twoDyMinusDx = 2 * (dy - dx);
    int x,y;
	int x_limit = end.x;
	
	point2d_pix tmp_point;

	bool reverse_flag = false;
	if(start.x > end.x)  
	{  
	  x = end.x;  
	  y = end.y;  
	  x_limit = start.x;
	  reverse_flag = true;
	}  
	else	
	{  
	  x = start.x;	
	  y = start.y;	
	}  
	tmp_point.x = x;
	tmp_point.y = y;
	point_at_line.push_back(tmp_point);

	while(x < x_limit)	
	{  
	  x++;	
	  if(p<0)
	  {
		  p+=twoDy; 
	  }
	  else	
	  {  
		  y++;	
		  p+=twoDyMinusDx;	
	  }  
	  tmp_point.x = x;
	  tmp_point.y = y;
	  point_at_line.push_back(tmp_point); 
	} 

	if(reverse_flag == true)
	{
	  reverse(point_at_line.begin(), point_at_line.end()); 
	}
  
} 

bool CalcPixesInLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &point_at_line)
{

	point_at_line.clear();
	vector<point2d_pix> ().swap(point_at_line);

	if (start.x==end.x && start.y==end.y)
		return false;
	
	if (start.x == end.x)
	{
		VerticalLine(start, end, point_at_line);

		return true;
	}

	float k = (float)(end.y-start.y)/(end.x-start.x);

	if (k >= 0 && k <= 1)
	{

		BresenhamBasic(start, end, point_at_line);

	}
	else if (k > 1)
	{
		int tmp = start.x;
		start.x = start.y;
		start.y = tmp;
		tmp = end.x;
		end.x = end.y;
		end.y = tmp;
		BresenhamBasic(start, end, point_at_line);
		for (vector<point2d_pix>::iterator it = point_at_line.begin(); it!=point_at_line.end(); ++it)
		{
			int tmp = (*it).x;
			(*it).x = (*it).y;
			(*it).y = tmp;
		}

	}
	else if (k >= -1 && k < 0)
	{
		start.y = -1 * start.y;
		end.y = -1 * end.y;

		BresenhamBasic(start, end, point_at_line);
		for (vector<point2d_pix>::iterator it = point_at_line.begin(); it!=point_at_line.end(); ++it)
		{
			(*it).y = -(*it).y;
		}

	}
	else if (k < -1)
	{

		int tmp = start.x;
		start.x = -1 * start.y;
		start.y = tmp;
		tmp = end.x;
		end.x = -1 * end.y;
		end.y = tmp;

		BresenhamBasic(start, end, point_at_line);
		for (vector<point2d_pix>::iterator it = point_at_line.begin(); it!=point_at_line.end(); ++it)
		{
			int tmp = (*it).x;
			(*it).x = (*it).y;
			(*it).y = tmp;
			(*it).y = -(*it).y;
		}

	}

	return true;

}

void int2str(int & i_val, string & str)
{
	stringstream stream;
	stream << i_val;
	str = stream.str();
}



