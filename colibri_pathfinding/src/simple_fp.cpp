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

// The world map

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
		nodeEnd.y = 80;
		// Set Start and goal states

		pix_point t_end;
		t_end.x = nodeEnd.x;
		t_end.y = nodeEnd.y;
		pix_point end;
		
		bool isgoal = mapObj.CalcGoalEdgePoint(t_end, end);
		if(true == isgoal)
		{
			nodeEnd.x = end.x;
			nodeEnd.y = end.y;
		}

		astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

		unsigned int SearchState;
		unsigned int SearchSteps = 0;

		start=clock();

		do
		{
			SearchState = astarsearch.SearchStep();
			SearchSteps++;
		}
		while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

		finish=clock();
		totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
		cout<<"searching running time:	"<<totaltime<<endl;

		if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
		{
			cout << "Search found goal state\n";

				pix_point tmp_nav_node;
				mapObj.nav_nodes.clear();

				MapSearchNode *node = astarsearch.GetSolutionStart();
				int steps = 0;

				node->PrintNodeInfo();
				for( ;; )
				{
					node = astarsearch.GetSolutionNext();

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

