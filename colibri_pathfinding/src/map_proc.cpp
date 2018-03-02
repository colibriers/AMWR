#include "map_proc.h"

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif


template <class Type>  
Type stringToNum(const string& str)  
{  
    istringstream iss(str);  
    Type num;  
    iss >> num;  
    return num;      
}  

template<typename T>
vector<smpix_point> Smooth5p3t(vector<T> &input)
{
	vector<smpix_point> output;
	vector<T> mid_val(input);
	smpix_point tmp_val;
	size_t len = input.size();
	if(len < 5)
	{
		cout<<"can not exec the 5 points 3 time smooth filter!"<<endl;
		return output;
	}
	
	tmp_val.x = (69 * mid_val[0].x + 4 * (mid_val[1].x + mid_val[3].x) - 6 * mid_val[2].x - mid_val[4].x) / 70.0;
	tmp_val.y = (69 * mid_val[0].y + 4 * (mid_val[1].y + mid_val[3].y) - 6 * mid_val[2].y - mid_val[4].y) / 70.0;
	output.push_back(tmp_val);
	
	tmp_val.x = (2 * (mid_val[0].x + mid_val[4].x) + 27 * mid_val[1].x + 12 * mid_val[2].x - 8 * mid_val[3].x) / 35.0;
	tmp_val.y = (2 * (mid_val[0].y + mid_val[4].y) + 27 * mid_val[1].y + 12 * mid_val[2].y - 8 * mid_val[3].y) / 35.0;
	output.push_back(tmp_val);

	for(size_t j = 2; j < (len - 2); j++)
	{
		tmp_val.x = (-3 * (mid_val[j-2].x + mid_val[j + 2].x) + 12 * (mid_val[j -1].x + mid_val[j + 1].x) + 17 * mid_val[j].x) / 35.0;
		tmp_val.y = (-3 * (mid_val[j-2].y + mid_val[j + 2].y) + 12 * (mid_val[j -1].y + mid_val[j + 1].y) + 17 * mid_val[j].y) / 35.0;
		output.push_back(tmp_val);
	}
	
	tmp_val.x = (2 * (mid_val[len - 1].x + mid_val[len - 5].x) + 27 * mid_val[len - 2].x + 12 * mid_val[len - 3].x - 8 * mid_val[len - 4].x) / 35.0;
	tmp_val.y = (2 * (mid_val[len - 1].y + mid_val[len - 5].y) + 27 * mid_val[len - 2].y + 12 * mid_val[len - 3].y - 8 * mid_val[len - 4].y) / 35.0;
	output.push_back(tmp_val);
	tmp_val.x = (69 * mid_val[len -1].x + 4 * (mid_val[len - 2].x + mid_val[len - 4].x) - 6 * mid_val[len - 3].x - mid_val[len - 4].x) / 70.0;
	tmp_val.y = (69 * mid_val[len -1].y + 4 * (mid_val[len - 2].y + mid_val[len - 4].y) - 6 * mid_val[len - 3].y - mid_val[len - 4].y) / 70.0;
	output.push_back(tmp_val);

	return output;
}
	
map_proc::map_proc()
{
	cur_goal[0] = 0.0;
	cur_goal[1] = 0.0;
	cur_goal[2] = 0.0;
	obtain_goal_flag = false;
	
	start.x = 1;
	start.y = 1;
	terminal.x = 1;
	terminal.y = 1;
	revised_terminal.x = terminal.x;
	revised_terminal.y = terminal.y;

	orimap_width = 0;
	orimap_height = 0;	
	submap_width_comple = 0;
	submap_height_comple = 0;

	ifstream fin(map_yaml_name.c_str());
	if(fin.fail())
	{
		ROS_ERROR("map yaml can not open in parse the map yaml argv in proc");
		exit(-1);
	}

#ifdef HAVE_NEW_YAMLCPP
		// The document loading process changed in yaml-cpp 0.5.
		YAML::Node doc = YAML::Load(fin);
#else
		YAML::Parser parser(fin);
		YAML::Node doc;
		parser.GetNextDocument(doc);
#endif


	try 
	{ 
		doc["origin"][0] >> map_origin[0]; 
		doc["origin"][1] >> map_origin[1]; 
		doc["origin"][2] >> map_origin[2];
		cout<<map_origin[0]<<endl;
	} catch (YAML::InvalidScalar) 
	{ 
		ROS_ERROR("The map does not contain an origin tag or it is invalid.");
		exit(-1);
	}
	
	nh_img.param("/fp_node/resolution", map_resol, (float)0.05);

	pub4path = nh_img.advertise<nav_msgs::Path>("/nav_path", 1);

	//map_image = imread("/home/colibri/colibri_ws/src/colibri_pathfinding/maps/626_mdf.pgm", CV_LOAD_IMAGE_COLOR);
	map_image = imread(map_pgm_name, CV_LOAD_IMAGE_COLOR);

	if(map_image.empty())
	{
	 	cout<<"open existed map error!"<<endl;
	}

	structe_element = cv::getStructuringElement( DILATION_TYPE, Size( 2*DILATION_SIZE + 1, 2*DILATION_SIZE+1 ),
										 		Point( DILATION_SIZE, DILATION_SIZE ) );

 	srv4fp = nh_img.advertiseService("/move_base/make_plan", &map_proc::ExecPathFindingSrv, this);

	
}

map_proc::~map_proc()
{

}

bool map_proc::CalcGoalEdgePoint(pix_point & end, pix_point & revised_end)
{

	if(world_map[end.y * MAP_WIDTH + end.x] < 255) // if the set goal is not in black area
	{
		revised_end.x = end.x;
		revised_end.y = end.y;	
	}
	else
	{

#ifdef SUBMAP_SEARCH
		for(int i = 1; i <= 2; i++) // only concern 2*SUBMAP_RESOL*mapresol
		{
			if(world_map[end.y * MAP_WIDTH + end.x + i] < 255)
			{
				revised_end.x = end.x + i;
				revised_end.y = end.y;	

				break;
			}
			if(world_map[(end.y - i) * MAP_WIDTH + end.x] < 255)
			{
				revised_end.x = end.x ;
				revised_end.y = end.y - i;	

				break;
			}

			if(world_map[end.y * MAP_WIDTH + end.x - i] < 255)
			{
				revised_end.x = end.x - i;
				revised_end.y = end.y;	

				break;
			}

			if(world_map[(end.y + i) *MAP_WIDTH + end.x] < 255)
			{
				revised_end.x = end.x;
				revised_end.y = end.y + i;

				break;
			}
			
			if( GOAL_EDGE_MAX == i)
			{
				cout <<"No nearest edge point to approx the goal!"<<endl;
				revised_end.x = end.x;
				revised_end.y = end.y;

				return false;
			}

		}

#else
		for(int i = 1; i <= GOAL_EDGE_MAX; i++) // should conce
		{
			if(world_map[end.y * MAP_WIDTH + end.x + i] < 255)
			{
				revised_end.x = end.x + i;
				revised_end.y = end.y;	

				break;
			}
			if(world_map[(end.y - i) * MAP_WIDTH + end.x] < 255)
			{
				revised_end.x = end.x ;
				revised_end.y = end.y - i;	

				break;
			}

			if(world_map[end.y * MAP_WIDTH + end.x - i] < 255)
			{
				revised_end.x = end.x - i;
				revised_end.y = end.y;	

				break;
			}

			if(world_map[(end.y + i) *MAP_WIDTH + end.x] < 255)
			{
				revised_end.x = end.x;
				revised_end.y = end.y + i;

				break;
			}
			
			if( GOAL_EDGE_MAX == i)
			{
				cout <<"No nearest edge point to approx the goal!"<<endl;
				revised_end.x = end.x;
				revised_end.y = end.y;

				return false;
			}

		}

#endif

	}

	return true;

}


bool map_proc::SearchMapPreProc(void)
{
	erode( map_image, dilation_img, structe_element );		
	cvtColor(dilation_img, gray_img, CV_BGR2GRAY);
	msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", gray_img).toImageMsg();

#ifdef SUBMAP_SEARCH
	orimap_width = (int) msg->width;
	orimap_height = (int) msg->height;

	MAP_WIDTH = int (orimap_width / SUBMAP_RESOL);
	MAP_HEIGHT = int (orimap_height / SUBMAP_RESOL);

	submap_width_comple = msg->width % SUBMAP_RESOL;
	submap_height_comple = msg->height % SUBMAP_RESOL;

	CalcSubMap(msg);

#else
	MAP_WIDTH = msg->width;
	MAP_HEIGHT = msg->height;
	for (int pix_index = 0; pix_index < MAP_WIDTH*MAP_HEIGHT; pix_index++)
	{
		world_map[pix_index] = 255 - msg->data[pix_index];

	}

#endif

	return true;	
}

bool map_proc::CalcSubMap(sensor_msgs::ImagePtr &msg)
{
	int blank_matrix[SUBMAP_RESOL*SUBMAP_RESOL];
	memset(blank_matrix, 1, sizeof(blank_matrix));

	vector<int> square_matrix(blank_matrix, blank_matrix + sizeof(blank_matrix)/sizeof(blank_matrix[0]));
	int tmp_val = 254;
	
	for(int y = 0; y < MAP_HEIGHT; y++)
	{
		for(int x = 0; x < MAP_WIDTH; x++)
		{
			square_matrix.clear();
			for(int index = 0; index < SUBMAP_RESOL*SUBMAP_RESOL; index++)
			{
				int sub_row = index / SUBMAP_RESOL;
				int sub_col = index % SUBMAP_RESOL;
				square_matrix.push_back(msg->data[x * SUBMAP_RESOL + (SUBMAP_RESOL*y + sub_row) * orimap_width + sub_col]);
			}
			tmp_val = *min_element(square_matrix.begin(),square_matrix.end());
			world_map[x + y*MAP_WIDTH] = 255 - tmp_val;		
		}
	}

	return true;
}


bool map_proc::ImgPix2NavPos(pix_point & pix, map_point & position)
{
	bool transfer_flag = false;
	transfer_flag = PixBoundCheck(pix);
	if(false == transfer_flag)
	{
		return false;
	}
	else
	{

#ifdef SUBMAP_SEARCH
	position.x = map_origin[0] + map_resol * (SUBMAP_RESOL * pix.x + 1);
	position.y = map_origin[1] + submap_height_comple*map_resol + (MAP_HEIGHT - pix.y) * map_resol * SUBMAP_RESOL - map_resol;
	position.yaw = 0.0;
	
#else
	position.x = map_origin[0] + map_resol * pix.x;
	position.y = map_origin[1] + (MAP_HEIGHT - pix.y) * map_resol;
	position.yaw = 0.0;

#endif
		return true;
	}
	
}

bool map_proc::ImgPix2NavPos(smpix_point & pix, map_point & position)
{
	bool transfer_flag = false;

#ifdef SUBMAP_SEARCH
		position.x = map_origin[0] + map_resol * (SUBMAP_RESOL * pix.x + 1);
		position.y = map_origin[1] + submap_height_comple*map_resol + (MAP_HEIGHT - pix.y) * map_resol * SUBMAP_RESOL - map_resol;
		position.yaw = 0.0;
		
#else
		position.x = map_origin[0] + map_resol * pix.x;
		position.y = map_origin[1] + (MAP_HEIGHT - pix.y) * map_resol;
		position.yaw = 0.0;
	
#endif

	
	return true;
	
}


bool map_proc::NavPos2ImgPix(map_point & position, pix_point & pix)
{
	bool transfer_flag = false;

#ifdef SUBMAP_SEARCH
	pix.x = floor((position.x - map_origin[0]) / (map_resol * SUBMAP_RESOL));
	pix.y = floor(MAP_HEIGHT - ((position.y - map_origin[1] - submap_height_comple*map_resol) / (map_resol * SUBMAP_RESOL)));

#else
	pix.x = floor((position.x - map_origin[0]) / map_resol);
	pix.y = floor(MAP_HEIGHT - ((position.y - map_origin[1]) / map_resol));

#endif

	cout<<"MAP_WIDTH: "<<MAP_WIDTH<<endl;
	cout<<"MAP_HEIGHT: "<<MAP_HEIGHT<<endl;
	cout<<"map_origin[0]: "<<map_origin[0]<<endl;
	cout<<"map_origin[1]: "<<map_origin[1]<<endl;
	cout<<"map_resol: "<<map_resol<<endl;

	transfer_flag = PixBoundCheck(pix);

	return transfer_flag;
}

bool map_proc::PixNodes2NavPath(vector<pix_point> & nav_nodes, vector<map_point> &nav_path)
{
	map_point tmp_pos;
	nav_path.clear();
	size_t len = nav_nodes.size();
    for (size_t i = 0; i < len; i++)
    {

		if(ImgPix2NavPos(nav_nodes[i], tmp_pos))
		{
			nav_path.push_back(tmp_pos);
		}
		else
		{
			return false;
		}
		
    }
	return true;
}

bool map_proc::PixNodes2NavPath(vector<smpix_point> & smnav_nodes, vector<map_point> &nav_path)
{
	map_point tmp_pos;
	nav_path.clear();
	size_t len = smnav_nodes.size();
    for (size_t i = 0; i < len; i++)
    {

		if(ImgPix2NavPos(smnav_nodes[i], tmp_pos))
		{
			nav_path.push_back(tmp_pos);
		}
		else
		{
			return false;
		}
		
    }
	return true;

}

void map_proc::NavPath2PixNodes()
{

}

bool map_proc::LocalMapUpdate()
{

}

bool map_proc::LoadGoalFromTask()
{

}

bool map_proc::StdNavPath(vector<map_point> &nav_path)
{
	if(nav_path.empty())
	{
		return false;
	}
	else
	{
		plan_path.poses.clear();
		geometry_msgs::PoseStamped tmp_pose_stamped;
		
		plan_path.header.stamp = ros::Time::now();
		plan_path.header.frame_id = "map";
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
			plan_path.poses.push_back(tmp_pose_stamped);
		}
		
		return true;
	}
	
}

void map_proc::ObtainRvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& rviz_goal)
{

	float yaw = 0.0;
	
	cur_goal[0] = rviz_goal->pose.position.x;
	cur_goal[1] = rviz_goal->pose.position.y;
	
	Quaternion2Yaw(*rviz_goal, yaw);
	cur_goal[2] = yaw;

	obtain_goal_flag = true;

}

void map_proc::Quaternion2Yaw(const geometry_msgs::PoseStamped &pose, float &yaw)
{
	float x = 0.0;
	float y = 0.0;
	
	y = 2.0 * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y);
	x = 1.0 - 2 * (pow(pose.pose.orientation.y, 2) + pow(pose.pose.orientation.z, 2));
	yaw = atan2(y,x) * RAD2DEG;

}

bool map_proc::PixBoundCheck(pix_point & pix)
{

	if(pix.x < 1 || pix.x > MAP_WIDTH || pix.y < 1 || pix.y > MAP_HEIGHT)
	{
		cout<<"The pix u/v in Img is out of the img itself"<<endl;
		return false;
	}
	else
	{
		return true;
	}

}

void map_proc::ParseMapOrigin(void)
{
	vector<string> vStr;
	boost::split( vStr, str_origin, boost::is_any_of( ", " ), boost::token_compress_on );
	for( vector<string>::iterator it = vStr.begin(); it != vStr.end(); ++it )
	{
	  //cout << *it << endl;
	}
	map_origin[0] = stringToNum<float>(vStr[0].substr(1));
	map_origin[1] = stringToNum<float>(vStr[1]);
	map_origin[2] = 0.0;
}

bool map_proc::ExecPathFindingSrv(nav_msgs::GetPlan::Request & req, nav_msgs::GetPlan::Response & res)
{

	map_point tmp_start,tmp_goal;
	
	tmp_start.x = req.start.pose.position.x;
	tmp_start.y = req.start.pose.position.y;
	tmp_goal.x = req.goal.pose.position.x;
	tmp_goal.y = req.goal.pose.position.y;
	cout<<"req start.x: "<<tmp_start.x<<endl;
	cout<<"req start.y: "<<tmp_start.y<<endl;
	cout<<"req goal.x: "<<tmp_goal.x<<endl;
	cout<<"req goal.y: "<<tmp_goal.y<<endl;	

	MapSearchNode nodeStart;
	MapSearchNode nodeEnd;

	bool isOK = this->SearchNodeInit(tmp_start, tmp_goal, nodeStart, nodeEnd);

	if(true == isOK)
	{
		astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );
		this->SearchAndObatainNodes(astarsearch);
		astarsearch.EnsureMemoryFreed();
	}
	else
	{
		cout<<"The search point is not normal!"<< endl;
	}

	vector<smpix_point> tmp_smnodes; 

	tmp_smnodes	= Smooth5p3t(this->nav_nodes);
	tmp_smnodes	= Smooth5p3t(tmp_smnodes);
	this->PixNodes2NavPath(tmp_smnodes, this->nav_path );
	this->StdNavPath(this->nav_path);

	res.plan = this->plan_path;

	return true;
	
}

bool map_proc::SearchAndObatainNodes(AStarSearch<MapSearchNode> &astarObj)
{
	unsigned int SearchState;
	unsigned int SearchSteps = 0;
	
	do
	{
		SearchState = astarObj.SearchStep();
		SearchSteps++;
	}
	while(SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);
	
	if(SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED)
	{
			cout << "Search found goal state\n";
	
			pix_point tmp_nav_node;
			nav_nodes.clear();
	
			MapSearchNode *node = astarObj.GetSolutionStart();
			int steps = 0;
			node->PrintNodeInfo();
			for( ;; )
			{
				node = astarObj.GetSolutionNext();
	
				if( !node )
				{
					break;
				}
	
				node->PrintNodeInfo();
				steps ++;
				
				tmp_nav_node.x = node->x;
				tmp_nav_node.y = node->y;				
				nav_nodes.push_back(tmp_nav_node);	
	
			};
			cout << "Solution steps " << steps << endl;
			// Once you're done with the solution you can free the nodes up
			astarObj.FreeSolutionNodes();	
	
	}
	else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED )
	{	
		cout << "Search terminated. Did not find goal state\n";
		return false;
	}
	
	// Display the number of loops the search went through
	cout << "SearchSteps : " << SearchSteps << "\n";
	return true;

}

bool map_proc::SearchNodeInit(map_point &start, map_point & goal, MapSearchNode &nodeStart, MapSearchNode &nodeEnd)
{
	// pos transfer to pix info
	pix_point start_node = {1, 1};
	pix_point goal_node = {1, 1};
	pix_point revised_node = {1, 1};
	bool trans_start_flag = false;
	bool trans_goal_flag = false;
	
	trans_start_flag = NavPos2ImgPix(start, start_node);
	trans_goal_flag = NavPos2ImgPix(goal, goal_node);

	if(trans_start_flag && trans_goal_flag)
	{
		
		nodeStart.x = start_node.x;
		nodeStart.y = start_node.y;

		bool isgoal = CalcGoalEdgePoint(goal_node, revised_node);
		if(true == isgoal)
		{
			nodeEnd.x = revised_node.x;
			nodeEnd.y = revised_node.y;
		}
		else
		{
			nodeEnd.x = goal.x;
			nodeEnd.y = goal.y;	
			return false;
		}
	}
	else
	{
		return false;
	}

	return true;

}

bool map_proc::SearchNodeInit(pix_point &start, pix_point & goal, MapSearchNode &nodeStart, MapSearchNode &nodeEnd)
{
	// pos transfer to pix info
	pix_point start_node = {1, 1};
	pix_point goal_node = {1, 1};
	pix_point revised_node = {1, 1};
	bool start_in_flag = false;
	bool goal_in_flag = false;
	
	start_in_flag = PixBoundCheck(start);
	goal_in_flag = PixBoundCheck(goal);

	if(start_in_flag && goal_in_flag)
	{
		
		nodeStart.x = start.x;
		nodeStart.y = start.y;

		bool isgoal = CalcGoalEdgePoint(goal, revised_node);
		if(true == isgoal)
		{
			nodeEnd.x = revised_node.x;
			nodeEnd.y = revised_node.y;
		}
		else
		{
			nodeEnd.x = goal.x;
			nodeEnd.y = goal.y;		
			return false;
		}
	}
	else
	{
		return false;
	}

	return true;

}



