
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <ctime>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "stlastar.h" 
#include "map_search.h"

using namespace std;
using namespace cv;

#define DEBUG_LISTS 1
#define DEBUG_LIST_LENGTHS_ONLY 0
#define DILATION_TYPE MORPH_ELLIPSE
#define DILATION_SIZE 7 

// Global data
const int MAP_WIDTH_MAX = 1000;
const int MAP_HEIGHT_MAX = 1000; // mean 50m*50m at resolution 0.05m/grid
int world_map[ MAP_WIDTH_MAX * MAP_HEIGHT_MAX ] = {};
int MAP_WIDTH = 1;
int MAP_HEIGHT = 1;

static const std::string OUTPUT = "Output";
bool CalcGoalEdgePoint(int &node_x, int &node_y,int &edge_x, int &edge_y)
{

	if(world_map[node_y * MAP_WIDTH + node_x] < 255) // if the set goal is not in black area
	{
		edge_x = node_x;
		edge_y = node_y;	
	}
	else
	{
		for(int i = 1; i <= 2*DILATION_SIZE; i++)	// should conce
		{
			if(world_map[node_y * MAP_WIDTH + node_x + i] < 255)
			{
				edge_x = node_x + i;
				edge_y = node_y;
				break;
			}
			if(world_map[(node_y - i) * MAP_WIDTH + node_x] < 255)
			{
				edge_x = node_x ;
				edge_y = node_y - i;
				break;
			}

			if(world_map[node_y * MAP_WIDTH + node_x - i] < 255)
			{
				edge_x = node_x - i;
				edge_y = node_y;
				break;
			}

			if(world_map[(node_y + i) *MAP_WIDTH + node_x] < 255)
			{
				edge_x = node_x;
				edge_y = node_y + i;
				break;
			}
			
			if( 2 * DILATION_SIZE == i)
			{
				cout <<"No nearest edge point to approx the goal!"<<endl;
				edge_x = node_x;
				edge_y = node_y;
				return false;
			}

		}
	}

	return true;

}
// The world map

int main( int argc, char *argv[] )
{

	ros::init(argc, argv, "find_route");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);
	
	ofstream  path_node; 
	path_node.open ("/home/colibri/colibri_ws/src/colibri_pathfinding/path/nodes.txt");

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("maps/image", 1);	
	Mat image = imread("/home/colibri/colibri_ws/src/colibri_pathfinding/maps/626_mdf.pgm", CV_LOAD_IMAGE_COLOR);
	if(image.empty())
	{
	 	cout<<"open error!"<<endl;
	}

	namedWindow(OUTPUT, CV_WINDOW_AUTOSIZE); 
	Mat dilation_img;  
	Mat element = cv::getStructuringElement( DILATION_TYPE, Size( 2*DILATION_SIZE + 1, 2*DILATION_SIZE+1 ),
										     Point( DILATION_SIZE, DILATION_SIZE ) );
	/// Apply the dilation operation as the free space is at gray value 254, we use erode function
	erode( image, dilation_img, element );		
	Mat gray_img;
	cvtColor(dilation_img, gray_img, CV_BGR2GRAY);
	imwrite("/home/colibri/colibri_ws/src/colibri_pathfinding/maps/Dilate_Img.pgm", gray_img);

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", gray_img).toImageMsg();
	sensor_msgs::ImagePtr ori_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dilation_img).toImageMsg();

	MAP_WIDTH = msg->width;
	MAP_HEIGHT = msg->height;
	
	for (int pix_index = 0; pix_index < MAP_WIDTH*MAP_HEIGHT; pix_index++)
	{
		world_map[pix_index] = 255 - msg->data[pix_index];

	}
	
	AStarSearch<MapSearchNode> astarsearch;

	unsigned int SearchCount = 0;

	const unsigned int NumSearches = 1;

	while(SearchCount < NumSearches)
	{
		// Create a start state
		MapSearchNode nodeStart;
		nodeStart.x = 390;
		nodeStart.y = 150;

		// Define the goal state
		MapSearchNode nodeEnd;
		nodeEnd.x = 600;
		nodeEnd.y = 97;
		// Set Start and goal states

		int tmp_goal_x, tmp_goal_y;
		bool isgoal = CalcGoalEdgePoint(nodeEnd.x, nodeEnd.y, tmp_goal_x, tmp_goal_y);
		if(true == isgoal)
		{
			nodeEnd.x = tmp_goal_x;
			nodeEnd.y = tmp_goal_y;
		}

		astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

		unsigned int SearchState;
		unsigned int SearchSteps = 0;

		do
		{
			SearchState = astarsearch.SearchStep();
			SearchSteps++;
		}
		while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

		if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
		{
			cout << "Search found goal state\n";

				MapSearchNode *node = astarsearch.GetSolutionStart();
				int steps = 0;

				node->PrintNodeInfo();

				path_node << node->x;
				path_node << '\t';
				path_node << node->y;
				path_node << '\n';
				
				for( ;; )
				{
					node = astarsearch.GetSolutionNext();

					if( !node )
					{
						break;
					}

					node->PrintNodeInfo();
					steps ++;
					
					path_node << node->x;
					path_node << '\t';
					path_node << node->y;
					path_node << '\n';

				};

				cout << "Solution steps " << steps << endl;

				// Once you're done with the solution you can free the nodes up
				astarsearch.FreeSolutionNodes();


		}
		else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED )
		{
			cout << "Search terminated. Did not find goal state\n";

		}

		// Display the number of loops the search went through
		cout << "SearchSteps : " << SearchSteps << "\n";

		SearchCount ++;
		astarsearch.EnsureMemoryFreed();
	}

	path_node.close();	//record laser dis completed

	while (nh.ok()) 
	{
	  pub.publish(ori_msg);
	  ros::spinOnce();
	
	  imshow(OUTPUT, dilation_img);  
	  waitKey(1000);
	
	  loop_rate.sleep();
	}
	
	destroyWindow(OUTPUT);

	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
