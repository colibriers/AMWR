#include "routes_proc.h"

bool get_coordinator_flag = false;

Routes::Routes() {
	LoadRoutes();
}

Routes::~Routes() {

}

void Routes::InitKneeNodes(const int &array_size, int *node_array) {
	vector<int> ().swap(knee_nodes_);
	for(int i = 0; i < array_size; i++) {
		knee_nodes_.push_back(*(node_array + i));
	}
}

void Routes::LoadRoutes(void) {
	
#ifdef MANUAL_PATH
		string path_name;
	
		char user_name[10];
		getlogin_r(user_name, 10);
		string str_username = user_name;
		path_name.assign("/home/" + str_username + "/colibri_ws/src/colibri_pathproc/routes/ygl0101_mdf_routes.yaml");
	
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
	
			doc_path["routes"]["image"] >> map_info_.name;
			doc_path["routes"]["resolution"] >> map_info_.resol;
			doc_path["routes"]["origin"][0] >> map_info_.origin[0];
			doc_path["routes"]["origin"][1] >> map_info_.origin[1];
			doc_path["routes"]["origin"][2] >> map_info_.origin[2];
			doc_path["routes"]["map_size"][0] >> map_info_.size[0];
			doc_path["routes"]["map_size"][1] >> map_info_.size[1];
			doc_path["routes"]["seg_num"] >> segs_num_;
			
			string seg_prop_name;
			string seg_terminal_name;
			string seg_heading_name;
			stringstream sstr_num; 
			string num2str;

			seg_property tmp_seg_prop;
			for(int seg_index = 1; seg_index <= segs_num_; seg_index++)
			{
				sstr_num << seg_index;
				num2str = sstr_num.str();
				seg_prop_name = "seg" + num2str + "_property";
				seg_terminal_name = "seg" + num2str + "_vector";
				seg_heading_name = "seg" + num2str + "_heading";
			
				doc_path["routes"][seg_prop_name][0] >> tmp_seg_prop.seg_id;
				doc_path["routes"][seg_prop_name][1] >> tmp_seg_prop.start_id;
				doc_path["routes"][seg_prop_name][2] >> tmp_seg_prop.end_id;
				doc_path["routes"][seg_prop_name][3] >> tmp_seg_prop.seg_type;
				doc_path["routes"][seg_prop_name][4] >> tmp_seg_prop.seg_dir;
				doc_path["routes"][seg_prop_name][5] >> tmp_seg_prop.arc_deg;

				doc_path["routes"][seg_terminal_name][0] >> tmp_seg_prop.start.x; 	
				doc_path["routes"][seg_terminal_name][1] >> tmp_seg_prop.start.y;
				doc_path["routes"][seg_terminal_name][2] >> tmp_seg_prop.ending.x;
				doc_path["routes"][seg_terminal_name][3] >> tmp_seg_prop.ending.y;
	
				doc_path["routes"][seg_heading_name] >> tmp_seg_prop.seg_heading;
	
				vec_seg_property_.push_back(tmp_seg_prop);
				sstr_num.str("");
			}
		
		}
		catch (YAML::InvalidScalar) 
		{ 
			cout<<"The yaml does not contain an origin tag or it is invalid."<<endl;
			exit(-1);
		}
}

void Routes::SetupMapping(void) {
	if(vec_seg_property_.empty()) {
		cout<<"Null vec_seg_property_"<<endl;
		return;
	}
	for(vector<seg_property>::iterator it = vec_seg_property_.begin(); it != vec_seg_property_.end(); ++it) {
		seg_postnode_map_.insert(pair<int, int>((*it).seg_id, (*it).end_id));
		seg_prenode_map_.insert(pair<int, int>((*it).seg_id, (*it).start_id));
		node_seg_map_.insert(pair<int, int>((*it).end_id, (*it).seg_id));
		seg_heading_map_.insert(pair<int, float>((*it).seg_id, (*it).seg_heading));
	}
	CalcAllPointsInSegs();
	Seg2LengthMap();
}

void Routes::Pix2Map(vector<point2d_pix> &points_pix, vector<point2d_map> &points_map) {
	point2d_map tmp;
	for (vector<point2d_pix>::iterator it = points_pix.begin(); it!=points_pix.end(); ++it) {
		int tmp_x = (*it).x;
		int tmp_y = map_info_.size[1] - (*it).y;
		tmp.x = (float) (map_info_.origin[0] + tmp_x * map_info_.resol);
		tmp.y = (float) (map_info_.origin[1] + tmp_y * map_info_.resol);
		points_map.push_back(tmp);
	}
}

/* Calc the bresham line pix coordinate from all existed segs to fill in segment struct vec_seg_*/
void Routes::CalcAllPointsInSegs(void) {
	vector<segment> ().swap(vec_seg_);
	for (vector<seg_property>::iterator it = vec_seg_property_.begin(); it!=vec_seg_property_.end(); ++it) {
		segment tmp;
		tmp.seg_id = (*it).seg_id;
		tmp.start_id = (*it).start_id;
		tmp.end_id = (*it).end_id;
		CalcPixesInLine((*it).start, (*it).ending, tmp.points_pix);
		Pix2Map(tmp.points_pix, tmp.points_map);
		vec_seg_.push_back(tmp);
	}
}

void Routes::Seg2LengthMap(void) {
	int tmp_length = 0;
	seg_length_map_.clear();
	
	for(vector<segment>::iterator it = vec_seg_.begin(); it != vec_seg_.end(); ++it) {
		tmp_length = (*it).points_map.size() - 1;
		seg_length_map_.insert(pair<int, int>((*it).seg_id, tmp_length));
	}
}

bool Routes::AddTargetNode2KneeNodes(const int &target_node) {
	vector<int> ().swap(updated_knee_nodes_);
	vector<int> tmp_knee(knee_nodes_);

	for(vector<int>::iterator it = tmp_knee.begin(); it != tmp_knee.end(); ++it) {
		updated_knee_nodes_.push_back(*it);
	}
		
	vector<int>::iterator iElement = find(knee_nodes_.begin(), knee_nodes_.end(), target_node);	
	if(iElement == knee_nodes_.end()) {
		updated_knee_nodes_.push_back(target_node);	
		return true;
	} else {
		return false;
	}

}

/* Calc the sub segs from a known seg_list which is the whole route from hostpc and decompose it into sub_route_vec_*/
bool Routes::DecomposeRoute(vector<int> &seg_list, vector<int> &check_nodes, int &sub_route_num) {
	vector<route_list> ().swap(sub_route_vec_);
	vector<int> remain_segs(seg_list);
	route_list tmp_sub_route;
	sub_route_num = 0;
	for(vector<int>::iterator it = remain_segs.begin(); it != remain_segs.end(); ++it) {
		int tmp_node = this->seg_postnode_map_[*it];
		vector<int>::iterator iElement = find(check_nodes.begin(), check_nodes.end(), tmp_node);// check the node in route is in check_nodes
		if(iElement != check_nodes.end())	{
			for(vector<int>::iterator sub_it = remain_segs.begin(); sub_it <= it; ++sub_it) {
				tmp_sub_route.seg_list.push_back(*sub_it);
			}
			tmp_sub_route.target_id = *iElement;
			tmp_sub_route.target_heading = seg_heading_map_[*iElement];
			sub_route_vec_.push_back(tmp_sub_route);
			sub_route_num++;
		}
		tmp_sub_route.seg_list.clear();
		vector<int> ().swap(tmp_sub_route.seg_list);		
	}

	int sub_num = sub_route_vec_.size(); 
	for(int i = 1; i < sub_num; i++) {
		for(vector<int>::iterator index = sub_route_vec_[sub_num-1-i].seg_list.begin(); index < sub_route_vec_[sub_num-1-i].seg_list.end(); ++index) {
			vector<int>::iterator iElem = find(sub_route_vec_[sub_num-i].seg_list.begin(), sub_route_vec_[sub_num-i].seg_list.end(), (*index));
			sub_route_vec_[sub_num-i].seg_list.erase(iElem);
		}
	}

	return true;
	
}

PathProc::PathProc() {
	cur_route_.target_id = 0;
	cur_route_.target_heading = 0.0;
	cur_seg_ = 48;
	task_switch_ = false;

	pub_route_ = nh_route_.advertise<nav_msgs::Path>("/nav_path", 1);
	sub_coodinator_ = nh_route_.subscribe<colibri_msgs::Coordinator>("/coordinator", 1, &PathProc::CoordinatorCallBack, this);
	sub_nav_state_ = nh_route_.subscribe<colibri_msgs::NavState>("/nav_state", 1, &PathProc::NavStateCallBack, this);
	pub_marker_ = nh_route_.advertise<visualization_msgs::Marker>("waypoint_markers", 1);

}

PathProc::~PathProc() {

}

void PathProc::InitMarkers(void) {
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

int PathProc::FillMarkerPose(route_list & route) {
	goalmark_list_.points.clear();
	geometry_msgs::Pose subgoal_list;

	for(vector<int>::iterator it = route.seg_list.begin(); it != route.seg_list.end(); ++it) {
		vector<segment>::iterator tmp_it = find_if(vec_seg_.begin(), vec_seg_.end(),FindX<int,segment>(*it));
		if(tmp_it != vec_seg_.end()) {
			subgoal_list.position.x = (*tmp_it).points_map.back().x;
			subgoal_list.position.y = (*tmp_it).points_map.back().y;
			subgoal_list.orientation.w = 1.0;
			goalmark_list_.points.push_back(subgoal_list.position);
		}
	}
	return goalmark_list_.points.size();
}

bool PathProc::CalcNearestNode(float & robot_x, float &robot_y, int & nearest_node) {
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


/* Calc the whole  map and pix route removing the seg start from the route_list struct */
void PathProc::CatSeg2Route(route_list &route)
{
	vector<point2d_pix> ().swap(route_pix_);
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

	tmp_uv.x =  (pose.x - ptrRoutes_->map_info_.origin[0]) / ptrRoutes_->map_info_.resol;
	tmp_uv.y =  ptrRoutes_->map_info_.size[1] - (pose.y - ptrRoutes_->map_info_.origin[1]) / ptrRoutes_->map_info_.resol;

	if(NavPixValid(tmp_uv))
	{
		
		for(vector<seg_property>::iterator it = tt.begin(); it != tt.end(); it++)
		{

			int delta_u = abs((*it).ending.x - tmp_uv.x);
			int delta_v = abs((*it).ending.y - tmp_uv.y);
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
	if(pix_uv.x >= ptrRoutes_->map_info_.size[0] || pix_uv.x <= 0 || pix_uv.y >= ptrRoutes_->map_info_.size[1] || pix_uv.y <= 0 )
	{
		return false;
	}
	else
	{
		return true;
	}
}

int PathProc::CalcRobotOnCurSeg(point2d_map & cur_pose, route_list &cur_route, vector<point2d_map> &straight_path)
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
		tmp_len_vec.push_back(ptrRoutes_->seg_length_map_[*it]);	
	}

	for(vector<int>::iterator it_len = tmp_len_vec.begin(); it_len != tmp_len_vec.end(); ++it_len)
	{
		acc_len = accumulate(tmp_len_vec.begin(), it_len, tmp_len_vec.front());
		len_stairs.push_back(acc_len);
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

void PathProc::HandleRecvRoute(void)
{

	CatSeg2Route(cur_route_);
	StdNavPath(route_map_);

}

bool VerticalLine(const point2d_pix &start, const point2d_pix &end, vector<point2d_pix> &ver_line)
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

bool BresenhamBasic(const point2d_pix &start, const point2d_pix &end, vector<point2d_pix> &point_at_line) 
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

bool CalcPixesInLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &point_at_line) {
	point_at_line.clear();
	vector<point2d_pix> ().swap(point_at_line);

	if (start.x==end.x && start.y==end.y)
		return false;
	
	if (start.x == end.x) {
		VerticalLine(start, end, point_at_line);
		return true;
	}

	float k = (float)(end.y-start.y)/(end.x-start.x);

	if (k >= 0 && k <= 1) {
		BresenhamBasic(start, end, point_at_line);

	}
	else if (k > 1) {
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

void Int2String(const int & i_val, string & str) {
	stringstream stream;
	stream << i_val;
	str = stream.str();
}

