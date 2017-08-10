#ifndef _Node_PROC_H_
#define _Node_PROC_H_

#include <fstream>
#include "yaml-cpp/yaml.h"

#include <math.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <string.h>
#include <map>
#include <utility>

#include <ros/ros.h>

#include "nav_msgs/Path.h"
#include "std_msgs/Bool.h"
#include "colibri_msgs/Coordinator.h"
#include "colibri_msgs/NavState.h"
#include "geometry_msgs/PoseStamped.h"
#include "colibri_msgs/Node.h"

using namespace std;
extern string taskpath;

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

template <class Type>  
Type stringToNum(const string& str)  
{  
    istringstream iss(str);  
    Type num;  
    iss >> num;  
    return num;      
}  

typedef struct st_point2D_int{
	int x;
	int y;
}point2d_pix;

typedef struct st_point2D_float{
	float x;
	float y;
}point2d_map;

typedef struct st_pose{
	float x;
	float y;
	float yaw;
}pose;


typedef struct st_segment{
	int seg_id;
	int start_id;
	int end_id;
	vector<point2d_pix> points_pix;
	vector<point2d_map> points_map;
}segment;

typedef struct st_nav_state{
	int target_node;
	int target_heading;
	int cur_seg;
	std_msgs::Bool at_target_flag;
	std_msgs::Bool achieve_flag;
	pose target;
	pose robot;
	int err_code;
}nav_state;

typedef struct st_seg_prop{
	int seg_id;
	int start_id;
	int end_id;
	point2d_pix start;
	point2d_pix end;
}seg_property;

typedef struct st_route_list
{
	int target_id;
	float target_heading;
	vector<int> seg_list;
}route_list;


class NodeProc{

	public:

		ros::NodeHandle nh_route_;
		ros::Subscriber sub_coodinator_;
		ros::Publisher pub_nav_state_;	

		string map_name_;
		float map_origin_[3];
		int map_size_[2];
		float map_resol_;
		int segs_num_;
		vector<seg_property> vec_seg_property_;
		vector<segment> vec_seg_;
		vector<point2d_map> route_map_;
		vector<point2d_pix> route_pix_;
		route_list cur_route_;
		vector<route_list> sub_route_vec_;
		map<int, int> node_seg_map_;
		map<int, int> seg_node_map_;
		map<int, float> node_heading_map_;

		vector<int> knee_nodes_;
		vector<float> nodes_heading_;

		nav_state robot_nav_state;
		
		int basic_ctrl_;
		nav_msgs::Path plan_path_;

		PathProc();
		~PathProc();
		void ConfigNodesHeading(float *head_array, int &array_size);
		void MakeNodeSegMap(vector<float> &vec_heading);

	private:

		void CoordinatorCallBack(const colibri_msgs::Coordinator::ConstPtr& coordinator);

};

#endif
