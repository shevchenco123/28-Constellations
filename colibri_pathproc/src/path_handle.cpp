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
	
#ifndef MANUAL_PATH
	taskpath.assign(argv[1]);
	cout<<"Load YAML Name: "<<taskpath<<endl;
#endif

	PathProc pathProcObj;

	int sp_nodes[] = {3, 5, 7};

	int cnt_nodes = sizeof(sp_nodes) / sizeof(int);
	pathProcObj.InitKneeNodes(sp_nodes, cnt_nodes);

	pathProcObj.MakeNodeSegMap();
	pathProcObj.CalcAllPointsInSegs();
	pathProcObj.InitMarkers();

	while(ros::ok())
	{
		pathProcObj.FillRobotCmd();
		if(get_coordinator_flag == true)
		{
			pathProcObj.pub_route_.publish(pathProcObj.plan_path_);
			get_coordinator_flag = false;
		}

		pathProcObj.pub_marker_.publish(pathProcObj.goalmark_list_);
		pathProcObj.pub_robot_cmd_.publish(pathProcObj.robot_cmd_);
		
		cout<<"robot_nav_state_.at_target_flag: "<<pathProcObj.robot_nav_state_.at_target_flag<<endl;
		cout<<"robot_nav_state_.achieve_flag: "<<pathProcObj.robot_nav_state_.achieve_flag<<endl;
		cout<<"sub_seg_index: "<<sub_seg_index<<endl;
		ros::spinOnce();	  
		loop_rate.sleep();
	}



	return 0;
}

