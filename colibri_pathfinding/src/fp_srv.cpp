
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <ctime>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "stlastar.h" 
#include "map_search.h"
#include "map_proc.h"

#define DEBUG_LISTS 1
#define DEBUG_LIST_LENGTHS_ONLY 0

// Global data
const int MAP_WIDTH_MAX = 1000;
const int MAP_HEIGHT_MAX = 1000; // mean 50m*50m at resolution 0.05m/grid
int world_map[ MAP_WIDTH_MAX * MAP_HEIGHT_MAX ] = {};
int MAP_WIDTH = 1;
int MAP_HEIGHT = 1;

AStarSearch<MapSearchNode> astarsearch;

int main( int argc, char *argv[] )
{

	ros::init(argc, argv, "path_plan_node");

	map_proc mapObj;
	pix_point test_start = {390, 150};
	pix_point test_goal = {600, 97};

	MapSearchNode nodeStart;
	MapSearchNode nodeEnd;

	ros::Rate loop_rate(1); 
	
	ROS_INFO("Ready to provide path_planning srv.");

	while (mapObj.nh_img.ok()) 
	{
	  mapObj.StdNavPath(mapObj.nav_path);
	  mapObj.pub4path.publish(mapObj.plan_path);

	  ros::spinOnce();
		
	  loop_rate.sleep();
	}
	
	return 0;
}



