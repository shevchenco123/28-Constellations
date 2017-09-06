#include <ros/ros.h>
#include "colibri_msgs/NavState.h"

colibri_msgs::NavState nav_state;

void FillNavState(int &node_id)
{
	nav_state.header.stamp = ros::Time::now();
	nav_state.header.frame_id = "/robot";
	nav_state.target_node = node_id;
	nav_state.target_heading = 90.0;
	nav_state.achieve_flag = true;
	nav_state.at_target_flag = false;
	nav_state.cur_x = 12.30;
	nav_state.cur_y = 45.60;
	nav_state.cur_yaw = -30.0;
	nav_state.err_code = 0;
	nav_state.target_x = 1.11;
	nav_state.target_y = 3.33;
	nav_state.target_yaw = -18.0;


}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "pub_state");
	ros::NodeHandle nh;
	ros::Publisher pub_nav_state;
	pub_nav_state = nh.advertise<colibri_msgs::NavState>("/modbus_data_info", 1);
	
	ros::Rate loop(10);
	int i = 0;
	while(ros::ok())
	{			
	
		if(i > 60)
		{
			i = 0;
		}

		FillNavState(i);
	
		pub_nav_state.publish(nav_state);
		loop.sleep();
		ros::spinOnce();
		i++;

	}

	return 0;	
}
