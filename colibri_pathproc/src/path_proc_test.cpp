#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "path_proc.h"


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

	int sp_nodes[] = {2, 4, 5, 6, 7};
	
	float heading[] = {0.0, 90.0, 0.0, 90.0, 90.0, -90.0, -90.0, 0.0, 0.0};



//	int sp_nodes[] = {0, 1, 3};
	int cnt_nodes = sizeof(sp_nodes) / sizeof(int);
	pathProcObj.InitKneeNodes(sp_nodes, cnt_nodes);
	
//	float heading[] = {90.0, 0.0, 90.0, 90.0, 90.0};
	pathProcObj.ConfigNodesHeading(heading, pathProcObj.segs_num_);

	int sub_route_num = 0;

	pathProcObj.MakeNodeSegMap(pathProcObj.nodes_heading_);

	pathProcObj.CalcAllPointsInSegs();
	
	route_list route;
	route.target_id = 9;
	route.target_heading = 0.0;
	for(int j= 0; j < 9; j++)
	{
		route.seg_list.push_back(j);
	}

	pathProcObj.AddTargetNode2KneeNodes(route.target_id);
	pathProcObj.DecomposeRoute(route.seg_list, pathProcObj.knee_nodes_, sub_route_num);
		
	pathProcObj.CatSeg2Route(route);	

	cout<<pathProcObj.map_name_<<endl;
	int i = 0; 
	while(ros::ok())
	{
		/*
		i++;
		if(i <= 10)
		{
			pathProcObj.CatSeg2Route(pathProcObj.sub_route_vec_[0]);
		}
		else if(i <= 20 && i > 10)
		{
			pathProcObj.CatSeg2Route(pathProcObj.sub_route_vec_[1]);		
		}
		else if(i <= 30 && i > 20)
		{
			pathProcObj.CatSeg2Route(pathProcObj.sub_route_vec_[2]);
		}
		else if(i <= 40 && i > 30)
		{
			pathProcObj.CatSeg2Route(route);
			
		}
		else
		{
			i = 0;
		}
		*/
		
		pathProcObj.StdNavPath(pathProcObj.route_map_);
	  	pathProcObj.pub_route_.publish(pathProcObj.plan_path_);
		ros::spinOnce();	  
		loop_rate.sleep();
	}


	return 0;
}

