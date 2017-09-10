#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>

#include "path_proc.h"

using namespace std;
string routes_path;
string sp_nodes_path;
ofstream  file1; 

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "Path_Handle_Node");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(10);
	
#ifdef MANUAL_PATH

#else
	routes_path.assign(argv[1]);
	sp_nodes_path.assign(argv[2]);
	cout<<"Load Routes YAML Name: "<<routes_path<<endl;
	cout<<"Load Special Nodes YAML Name: "<<sp_nodes_path<<endl;
#endif

	PathProc pathProcObj;

	point2d_map cur_robot = {0.0, 0.0};
	int cur_seg = 127;

	pathProcObj.InitKneeNodes();

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

