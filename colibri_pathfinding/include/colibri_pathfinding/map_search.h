#include "stlastar.h" // See header for copyright and usage information
#include <math.h>
#ifndef _MAP_SEARCH_H_
#define _MAP_SEARCH_H_

extern int world_map[];
extern int MAP_WIDTH;
extern int MAP_HEIGHT;

int GetMap( int x, int y);

class MapSearchNode
{
	public:
		int x;	 // the (x,y) positions of the node
		int y;

		MapSearchNode() { x = y = 0; }
		MapSearchNode( int px, int py ){ x=px; y=py; }

		float GoalDistanceEstimate( MapSearchNode &nodeGoal );
		bool IsGoal( MapSearchNode &nodeGoal );
		bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
		float GetCost( MapSearchNode &successor );
		bool IsSameState( MapSearchNode &rhs );

		void PrintNodeInfo();

};

#endif
