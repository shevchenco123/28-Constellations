#include <ros/ros.h>

#include "aiv_monitor/Screen.h"
#include "aiv_monitor/Battery.h"

#include <iostream>

using namespace std;

void BatteryRecvCallBack(const aiv_monitor::Battery::ConstPtr& msg);


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "sub_battery_node");
	
	ros::NodeHandle nh;

	ros::Subscriber bat_sub = nh.subscribe<aiv_monitor::Battery>("bat_info" ,10, BatteryRecvCallBack);

	ros::spin();

	return 0;
}

void BatteryRecvCallBack(const aiv_monitor::Battery::ConstPtr& msg)
{
	aiv_monitor::Battery recv_batinfo;
	for(int i = 0; i < 10; i++)
	{
		recv_batinfo.batteryinfo[i] = msg->batteryinfo[i];
		
		ROS_INFO(" Recv battery info[%d]: %d", i, recv_batinfo.batteryinfo[i]);

	}

}


