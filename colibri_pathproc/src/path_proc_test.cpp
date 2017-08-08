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

	int sp_nodes[] = {1,2,5,6,7,8,9,10};
	int cnt_nodes = sizeof(sp_nodes) / sizeof(int);
	vector<int> check_nodes(sp_nodes, sp_nodes + cnt_nodes);
	int sub_route_num = 0;

	int seg_list[] = {0,1,2,3,4,5,6,7,8,9};
	int cnt_segs = sizeof(seg_list) / sizeof(int);
	vector<int> segs(seg_list, seg_list + cnt_segs);
	
	int path[] = {1,2,3,4,5,6,7,8,9};
	int cnt_path = sizeof(path) / sizeof(int);
	vector<int> path_segs(path, path + cnt_path);


	for(vector<int>::iterator it = segs.begin(); it != segs.end(); ++it)
	{
		pathProcObj.node_seg_map_.insert(pair<int, int>((*it)+1, (*it)));
		pathProcObj.seg_node_map_.insert(pair<int, int>((*it), (*it)+1));
	}

	
	CalcPixesInLine(tmp_start, tmp_end, tmp_line_points);  

	pathProcObj.CalcAllPointsInSegs();
	
	route_list route;
	route.target_id = 1;
	route.seg_list.push_back(0);
	route.seg_list.push_back(1);
	

	pathProcObj.CatSeg2Route(route);

	
	pathProcObj.DecomposeRoute(path_segs, check_nodes, sub_route_num);
	while(ros::ok())
	{
		cout<<pathProcObj.map_resol_<<endl;
		ros::spinOnce();	  
		loop_rate.sleep();
	}


	return 0;
}

