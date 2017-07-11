////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <ctime>

#include <ros/ros.h>

#include "stlastar.h" 
#include "map_search.h"
#include "map_proc.h"

#include <image_transport/image_transport.h>


#define DEBUG_LISTS 1
#define DEBUG_LIST_LENGTHS_ONLY 0

// Global data
const int MAP_WIDTH_MAX = 1000;
const int MAP_HEIGHT_MAX = 1000; // mean 50m*50m at resolution 0.05m/grid
int world_map[ MAP_WIDTH_MAX * MAP_HEIGHT_MAX ] = {};
int MAP_WIDTH = 1;
int MAP_HEIGHT = 1;

bool SearchNodeInit(map_proc & Obj, map_point &start, map_point & goal, MapSearchNode &nodeStart, MapSearchNode &nodeEnd);
bool SearchNodeInit(map_proc & Obj, pix_point &start, pix_point & goal, MapSearchNode &nodeStart, MapSearchNode &nodeEnd);
bool SearchAndObatinNodes(AStarSearch<MapSearchNode> &astarObj, map_proc & mapObj);


int main( int argc, char *argv[] )
{

	clock_t start,finish;
	double totaltime;

	ros::init(argc, argv, "find_route");
	map_proc mapObj;
	
	ros::Rate loop_rate(5);	

	mapObj.SearchMapPreProc();
	mapObj.ParseMapOrigin();
	
	AStarSearch<MapSearchNode> astarsearch;

	pix_point test_start = {390, 150};
	pix_point test_goal = {600, 97};

	MapSearchNode nodeStart;
	MapSearchNode nodeEnd;
	
	bool isOK = SearchNodeInit(mapObj, test_start, test_goal, nodeStart, nodeEnd);

	if(true == isOK)
	{
		astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );
		bool searchOk = SearchAndObatinNodes(astarsearch, mapObj);	
		astarsearch.EnsureMemoryFreed();
	}
	else
	{
		cout<<"The search point is not normal!"<< endl;
	}

	vector<smpix_point> tmp_smnodes; 

	tmp_smnodes	= Smooth5p3t(mapObj.nav_nodes);
	
	mapObj.PixNodes2NavPath(tmp_smnodes, mapObj.nav_path);

	while (mapObj.nh_img.ok()) 
	{
	  mapObj.PubNavPath(mapObj.nav_path);

	  ros::spinOnce();
		
	  loop_rate.sleep();
	}
	
	return 0;
}

bool SearchNodeInit(map_proc & Obj, map_point &start, map_point & goal, MapSearchNode &nodeStart, MapSearchNode &nodeEnd)
{
	// pos transfer to pix info
	pix_point start_node = {1, 1};
	pix_point goal_node = {1, 1};
	pix_point revised_node = {1, 1};
	bool trans_start_flag = false;
	bool trans_goal_flag = false;
	
	trans_start_flag = Obj.NavPos2ImgPix(start, start_node);
	trans_goal_flag = Obj.NavPos2ImgPix(goal, goal_node);

	if(trans_start_flag && trans_goal_flag)
	{
		
		nodeStart.x = start_node.x;
		nodeStart.y = start_node.y;

		bool isgoal = Obj.CalcGoalEdgePoint(goal_node, revised_node);
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

bool SearchNodeInit(map_proc & Obj, pix_point &start, pix_point & goal, MapSearchNode &nodeStart, MapSearchNode &nodeEnd)
{
	// pos transfer to pix info
	pix_point start_node = {1, 1};
	pix_point goal_node = {1, 1};
	pix_point revised_node = {1, 1};
	bool start_in_flag = false;
	bool goal_in_flag = false;
	
	start_in_flag = Obj.PixBoundCheck(start);
	goal_in_flag = Obj.PixBoundCheck(goal);

	if(start_in_flag && goal_in_flag)
	{
		
		nodeStart.x = start.x;
		nodeStart.y = start.y;

		bool isgoal = Obj.CalcGoalEdgePoint(goal, revised_node);
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

bool SearchAndObatinNodes(AStarSearch<MapSearchNode> &astarObj, map_proc & mapObj)
{
	unsigned int SearchState;
	unsigned int SearchSteps = 0;
	
	do
	{
		SearchState = astarObj.SearchStep();
		SearchSteps++;
	}
	while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );
	
	if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
	{
			cout << "Search found goal state\n";
	
			pix_point tmp_nav_node;
			mapObj.nav_nodes.clear();
	
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
				mapObj.nav_nodes.push_back(tmp_nav_node);
				
	
			};
	
			cout << "Solution steps " << steps << endl;
	
			// Once you're done with the solution you can free the nodes up
			astarObj.FreeSolutionNodes();

			
	
	}
	else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED )
	{
		return false;
		cout << "Search terminated. Did not find goal state\n";
	
	}
	
	// Display the number of loops the search went through
	cout << "SearchSteps : " << SearchSteps << "\n";
	return true;

}

