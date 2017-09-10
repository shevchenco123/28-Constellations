#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>

#include "path_proc.h"

using namespace std;
string taskpath;
ofstream  file1; 

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "path_handle_node");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(10);
	
#ifdef MANUAL_PATH

#else
	taskpath.assign(argv[1]);
	cout<<"Load YAML Name: "<<taskpath<<endl;
#endif

	PathProc pathProcObj;

	int sp_nodes[] = {3, 4, 11, 12, 14, 15, 16, 17, 33, 20, 21, 22, 28, 30, 31, 23, 25, 26};
	point2d_map cur_robot = {0.0, 0.0};
	int cur_seg = 255;

	int cnt_nodes = sizeof(sp_nodes) / sizeof(int);
	pathProcObj.InitKneeNodes(sp_nodes, cnt_nodes);

	pathProcObj.MakeNodeSegMap();
	pathProcObj.CalcAllPointsInSegs();
	pathProcObj.Seg2LengthMap();
	pathProcObj.InitMarkers();

	while(ros::ok())
	{

		cur_robot.x = pathProcObj.robot_nav_state_.robot.x;
		cur_robot.y = pathProcObj.robot_nav_state_.robot.y;
		
		if(get_coordinator_flag == true)
		{
			get_coordinator_flag = false;

			pathProcObj.pub_route_.publish(pathProcObj.plan_path_);

			cur_seg = pathProcObj.CalcRobotOnCurSeg(cur_robot, pathProcObj.sub_route_vec_[pathProcObj.sub_seg_index_cache_], pathProcObj.route_map_);
			cout<<"sub_seg_index: "<<sub_seg_index<<endl;
			cout<< "pathProcObj.route_map_.size: "<<pathProcObj.route_map_.size()<<endl;
			pathProcObj.FillRobotCmd();
			
		}

		pathProcObj.pub_marker_.publish(pathProcObj.goalmark_list_);
		pathProcObj.pub_robot_cmd_.publish(pathProcObj.robot_cmd_);
		
		cout<<"robot cur_seg: "<<cur_seg<<endl;
		cout<<"robot_nav_state_.at_target_flag: "<<pathProcObj.robot_nav_state_.at_target_flag<<endl;
		cout<<"robot_nav_state_.achieve_flag: "<<pathProcObj.robot_nav_state_.achieve_flag<<endl;
		
		pathProcObj.ClearFlags4NextTask();
		
		ros::spinOnce();	  
		loop_rate.sleep();
	}



	return 0;
}

