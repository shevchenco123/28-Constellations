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

static const std::string OUTPUT = "Output";

// The world map

int main( int argc, char *argv[] )
{

	ros::init(argc, argv, "find_route");
	map_proc mapObj;
	
	ros::Rate loop_rate(5);	

	ofstream  path_node; 
	path_node.open ("/home/colibri/clbri_ws/src/colibri_pathfinding/path/nodes.txt");

	namedWindow(OUTPUT, CV_WINDOW_AUTOSIZE); 
	mapObj.SearchMapPreProc();
	mapObj.ParseMapOrigin();

	imwrite("/home/colibri/clbri_ws/src/colibri_pathfinding/maps/Dilate_Img.pgm", mapObj.gray_img);
	
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

		do
		{
			SearchState = astarsearch.SearchStep();
			SearchSteps++;
		}
		while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

		if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
		{
			cout << "Search found goal state\n";

				pix_point tmp_nav_node;
				mapObj.nav_nodes.clear();

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
					
					tmp_nav_node.x = node->x;
					tmp_nav_node.y = node->y;				
					mapObj.nav_nodes.push_back(tmp_nav_node);
					
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

	mapObj.PixNodes2NavPath(mapObj.nav_nodes, mapObj.nav_path);

	while (mapObj.nh_img.ok()) 
	{
	  mapObj.PubNavPath(mapObj.nav_path);

	  ros::spinOnce();
	
	  imshow(OUTPUT, mapObj.dilation_img);  
	  waitKey(1000);
	
	  loop_rate.sleep();
	}
	
	destroyWindow(OUTPUT);

	return 0;
}

