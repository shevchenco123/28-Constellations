#include <ros/ros.h>

#include "aiv_monitor/Screen.h"
#include "aiv_monitor/Battery.h"

#include <iostream>

using namespace std;

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "pub_battery_node");
	
	ros::NodeHandle nh;
	ros::Publisher apf_pub = nh.advertise<aiv_monitor::Battery>("bat_info", 50);
	
	ros::Rate loop_rate(10);


	unsigned int  j = 0;
	aiv_monitor::Battery tmp_batinfo;

	tmp_batinfo.header.stamp = ros::Time::now();
	tmp_batinfo.header.frame_id = "battery";

	ROS_INFO("Start pub a battery info");
	 while (ros::ok())
	{
		
		tmp_batinfo.header.stamp = ros::Time::now();
	
		for(int i = 0; i < 10; i++)
		{
			tmp_batinfo.batteryinfo[i] = j++;
		}

		apf_pub.publish(tmp_batinfo);

		ros::spinOnce();

		loop_rate.sleep();
	}


	cout << "finish send battery" << endl;

	return 0;
}














