#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "colibri_msgs/EnvSecurity.h"
#include "colibri_msgs/AuxInfo.h"
#include "cartodom/Carotodom.h"

geometry_msgs::Twist aux_twist;
colibri_msgs::EnvSecure aux_envsec;
cartodom::Cartodom aux_cartodom;

void SubTwistCallBack(const geometry_msgs::Twist::ConstPtr & cmd_vel);
void SubEnvSecurityCallBack(const colibri_msgs::EnvSecure::ConstPtr & env_sec);
void StateFbCallback(const cartodom::Cartodom::ConstPtr & carto_odom);


int main(int argc, char* argv[])
{	

	// ROS nav node initial
	ros::init(argc, argv, "Auxiliary Info Pub");
	ROS_INFO("Start to Pub the Auxinfo ... ");
	
	ros::NodeHandle nh_aux;
	ros::Subscriber twist_sub;
	ros::Subscriber env_sub;
	ros::Subscriber state_fb_sub;
	ros::Publisher aux_info_pub;

	ros::Rate loop_rate(10);	
	
	colibri_msgs::AuxInfo aux_info;
	twist_sub = nh_aux.subscribe<geometry_msgs::Twist>("/t_cmd_vel", 1, &SubTwistCallBack, this);
	env_sub = nh_aux.subscribe<colibri_msgs::EnvSecure>("/env_secure", 1, &SubEnvSecurityCallBack, this);
	state_fb_sub = nh_aux.subscribe<cartodom::Cartodom>("/cartodom", 1, &StateFbCallback, this);

	while(ros::ok())
	{
		aux_info.header.stamp = ros::Time::now();
		aux_info.header.frame_id = "robot";
		
		aux_info.lf_light = 0;
		aux_info.lr_light = 0;
		aux_info.rf_light = 0;
		aux_info.rr_light = 0;
		aux_info.horn = 0;
		aux_info.laser_mindis = 0;
		aux_info.laser_mindir = 0;
		aux_info.ultra_switch = 0;
		aux_info.ros_fault = 0;

		aux_info_pub.publish(aux_info);
		
		ros::spinOnce();
		loop_rate.sleep();


	}

	return 0;
}

void SubTwistCallBack(const geometry_msgs::Twist::ConstPtr & cmd_vel)
{
	aux_twist.linear.x = cmd_vel->linear.x;
	aux_twist.angular.z = cmd_vel->angular.z;
}

void SubEnvSecurityCallBack(const colibri_msgs::EnvSecure::ConstPtr & env_sec)
{
	aux_envsec.laser_min_dis = env_sec->laser_min_dis;
	aux_envsec.laser_min_angle = env_sec->laser_min_angle;
}

void StateFbCallback(const cartodom::Cartodom::ConstPtr & carto_odom)
{
	aux_cartodom.x = carto_odom->x;
	aux_cartodom.y = carto_odom->y;
	aux_cartodom.yaw = carto_odom->yaw;
	
	aux_cartodom.vx = carto_odom->vx;
	aux_cartodom.vth = carto_odom->vth;
}



