#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <algorithm>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include "colibri_msgs/AngPotnEngy.h"
#include "colibri_aiv/Ultrasonic.h"
#include "colibri_msgs/EnvSecurity.h"
#include "colibri_msgs/Ultrasonic.h"
#include "colibri_msgs/NavState.h"
#include "colibri_msgs/Coordinator.h"
#include "colibri_msgs/RobotCmd.h"

#include "nav_node_proc.h"

nav_state cur_nav_state;
colibri_msgs::Coordinator coodinator;
colibri_msgs::RobotCmd robot_cmd;

void NavStateCallback(const colibri_msgs::NavState::ConstPtr & nav_state);
int main(int argc, char* argv[])
{
	ros::NodeHandle nh_test;
	ros::Subscriber sub4NavState;
	ros::Publisher pub4Coordinator;
	ros::Publisher pub4Robot_cmd;

	NavNodeProc nodeObj;

	nodeObj.InitNodeAndSegMap(nodeObj.segs_num_);

	sub4NavState = nh_test.subscribe<colibri_msgs::NavState>("/nav_state", 5, NavStateCallback);
	pub4Coordinator = nh_test.advertise<colibri_msgs::Coordinator>("/coordinator", 1);
	pub4Robot_cmd = nh_test.advertise<colibri_msgs::RobotCmd>("/robot_cmd", 1);

	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		// obatain init nav state 
		// cmd to n1
		// if reach n1 ok
		   // go to n5
		   //else

		
		ros::spinOnce();
		loop_rate.sleep();

	}

	
	
	
	return 0;
}

void Init(void)
{
	cur_nav_state.target_node = 0;
	cur_nav_state.target_heading = 0.0;
	cur_nav_state.cur_seg = 0;
	cur_nav_state.robot.x = 0.0;
	cur_nav_state.robot.y = 0.0;
	cur_nav_state.robot.yaw = 0.0;
	
	cur_nav_state.target.x = 10.0;
	cur_nav_state.target.y = 10.0;
	cur_nav_state.target.yaw = 0.0;
	cur_nav_state.err_code = 0;

	cur_nav_state.at_target_flag= false;
	cur_nav_state.achieve_flag = false;

}
void NavStateCallback(const colibri_msgs::NavState::ConstPtr & nav_state)
{
	cur_nav_state.target_node = nav_state->target_node;
	cur_nav_state.target_heading = nav_state->target_heading;
	
	cur_nav_state.cur_seg = nav_state->cur_seg;
	cur_nav_state.robot.x = nav_state->cur_x;
	cur_nav_state.robot.y = nav_state->cur_y;
	cur_nav_state.robot.yaw = nav_state->cur_yaw;
	
	cur_nav_state.target.x = nav_state->target_x;
	cur_nav_state.target.y = nav_state->target_y;
	cur_nav_state.target.yaw = nav_state->target_yaw; 
	
	cur_nav_state.err_code = nav_state->err_code;

	cur_nav_state.at_target_flag= nav_state->at_target_flag;
	cur_nav_state.achieve_flag = nav_state->achieve_flag;

}


void FillCoordinator(void)
{

}

void FillRobotCmd(void)
{

}

