////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "map_search.h" // See header for copyright and usage information

// Definitions

// map helper functions

int GetMap( int x, int y)
{
	if( x < 0 ||
	    x >= MAP_WIDTH ||
		 y < 0 ||
		 y >= MAP_HEIGHT
	  )
	{
		return 254;
	}

	return world_map[(y*MAP_WIDTH)+x];
}

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{

	// same state in a maze search is simply when (x,y) are the same
	if( (x == rhs.x) &&
		(y == rhs.y) )
	{
		return true;
	}
	else
	{
		return false;
	}

}

void MapSearchNode::PrintNodeInfo()
{
	char str[100];
	sprintf( str, "Node position : (%d,%d)\n", x,y );

	cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal.

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
	//return fabsf(x - nodeGoal.x) + fabsf(y - nodeGoal.y);
	float h_dis =  fabsf(x - nodeGoal.x) + fabsf(y - nodeGoal.y);	//init using manhaton dis
	h_dis = sqrt(pow((nodeGoal.x - x), 2) + pow((nodeGoal.y - y), 2));
	return h_dis;
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

	if( (x == nodeGoal.x) &&
		(y == nodeGoal.y) )
	{
		return true;
	}

	return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{

	int parent_x = -1;
	int parent_y = -1;

	if( parent_node )
	{
		parent_x = parent_node->x;
		parent_y = parent_node->y;
	}


	MapSearchNode NewNode;

	// push each possible move except allowing the search to go backwards

	if( (GetMap( x-1, y) < 255)
		&& !((parent_x == x-1) && (parent_y == y))
	  )
	{
		NewNode = MapSearchNode( x-1, y );
		astarsearch->AddSuccessor( NewNode );
	}

	if( (GetMap( x, y-1) < 255)
		&& !((parent_x == x) && (parent_y == y-1))
	  )
	{
		NewNode = MapSearchNode( x, y-1 );
		astarsearch->AddSuccessor( NewNode );
	}

	if( (GetMap( x+1, y) < 255)
		&& !((parent_x == x+1) && (parent_y == y))
	  )
	{
		NewNode = MapSearchNode( x+1, y );
		astarsearch->AddSuccessor( NewNode );
	}


	if( (GetMap( x, y+1) < 255)
		&& !((parent_x == x) && (parent_y == y+1))
		)
	{
		NewNode = MapSearchNode( x, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}

	if( (GetMap( x+1, y+1) < 255)
		&& !((parent_x == x+1) && (parent_y == y+1))
	  )
	{
		NewNode = MapSearchNode( x+1, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}

	if( (GetMap( x-1, y+1) < 255)
		&& !((parent_x == x-1) && (parent_y == y+1))
	  )
	{
		NewNode = MapSearchNode( x-1, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}

	if( (GetMap( x-1, y-1) < 255)
		&& !((parent_x == x-1) && (parent_y == y-1))
	  )
	{
		NewNode = MapSearchNode( x-1, y-1 );
		astarsearch->AddSuccessor( NewNode );
	}


	if( (GetMap( x+1, y-1) < 255)
		&& !((parent_x == x+1) && (parent_y == y-1))
		)
	{
		NewNode = MapSearchNode( x+1, y-1 );
		astarsearch->AddSuccessor( NewNode );
	}

	return true;
}

float MapSearchNode::GetCost( MapSearchNode &successor )
{
//	return (float) GetMap( x, y);
	float g_dis = 1.0;	//using current node's successor to current nodes euculide dis works as the current move cost instead of current nodes's val
	g_dis = sqrt(pow((successor.x - x), 2) + pow((successor.y - y), 2));

	return g_dis;
}

