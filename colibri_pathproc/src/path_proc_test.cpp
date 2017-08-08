#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "path_proc.h"
#include <ros/ros.h>


using namespace std;
string taskpath;

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "pathproc_test_node");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(10); 
	//taskpath.assign(argv[1]);

	PathProc pathProcObj;

	point2d_pix tmp_start;
	point2d_pix tmp_end;
	vector<point2d_pix> tmp_line_points;
	tmp_start.x = 30;
	tmp_start.y = 10;
	tmp_end.x = 40;
	tmp_end.y = 10;	
	
	CalcPixesInLine(tmp_start, tmp_end, tmp_line_points);  

	pathProcObj.CalcAllPointsInSegs();
	
	route_list route;
	route.target_id = 1;
	route.seg_list.push_back(0);
	route.seg_list.push_back(1);	

	pathProcObj.CatSeg2Route(route);

	
	while(ros::ok())
	{
		cout<<pathProcObj.map_resol_<<endl;
		ros::spinOnce();	  
		loop_rate.sleep();
	}


	return 0;
}

