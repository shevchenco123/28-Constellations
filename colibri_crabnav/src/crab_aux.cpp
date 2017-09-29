#include "colibri_ca.h"
#include "colibri_local_nav.h"
#include "colibri_action.h"
#include "PID_controller.h"
#include "global_planner.h"
#include "task_mgr.h"
#include "nav_node_proc.h"

#include "colibri_msgs/MusicMode.h"
#include "colibri_msgs/NavState.h"
#include "colibri_battery/Battery.h"
#include "colibri_msgs/SafeVel.h"

int obs_rec_coder = 0;
float bat_vol_ratio = 1.0;
void SafeVelCallBack(const colibri_msgs::SafeVel::ConstPtr& safe_vel)
{

};
void BatteryCallBack(const colibri_battery::Battery::ConstPtr& bat_info)
{

};
void NavStateCallBack(const colibri_msgs::NavState::ConstPtr& nav_sts)
{

};


int main(int argc, char* argv[])
{	

	// ROS nav node initial
	ros::init(argc, argv, "Nav_Aux_Node");
	ros::NodeHandle nh_aux;
	ros::Subscriber sub_safe_vel;
	ros::Subscriber sub_bat_info;
	ros::Subscriber sub_nav_state;
	ros::Publisher pub_music_mode;

	ros::Rate loop_rate(2);

	sub_safe_vel = nh_aux.subscribe<colibri_msgs::SafeVel>("/laser_safe_vel", 1, &SafeVelCallBack);
	sub_bat_info = nh_aux.subscribe<colibri_battery::Battery>("/battery_info", 1, &BatteryCallBack);
	sub_nav_state = nh_aux.subscribe<colibri_msgs::NavState>("/nav_state", 1, &NavStateCallBack);
	pub_music_mode = nh_aux.advertise<colibri_msgs::MusicMode>("/music", 1);

	while (ros::ok())
	{			
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;	
}

