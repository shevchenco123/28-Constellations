#ifndef _PROTECT_H_
#define _PROTECT_H_

#include <iostream>
#include <cmath>
#include <string>
#include <vector>

#include <fstream>
#include <iomanip>
#include <ctime>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include "colibri_ca.h"

#include <colibri_aiv/ultrasonic.h>
#include <colibri_aiv/collision.h>

// 3 layers for running safty : laser / ultrosonic  /bumper


#define SCAN4SAFE_NUM 211	//obtain the  210 degree laser scan for resolution at 1 degree 
#define ULTRA_NUM	  4		//front 4 ultrasonic, ignore the back 4 ultrasonic

#define LASER_SAFE_MIN	0.1	//static safe limit
#define LASER_SAFE_MAX	0.5

#define ULTRA_SAFE_MIN	0.3

#define UNSAFE_PROB		0.9
#define LEVEL_1_PROB 	0.85
#define LEVEL_2_PROB 	0.35
#define LEVEL_3_PROB 	0.15

#define V_EXCPT_VAL   1.5
#define VTH_EXCPT_VAL 0.6

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))


using namespace std;

typedef enum
{
	HOLD_STATE,
	DEC_VEL,
	TURN_LEFT,
	TURN_RIGHT,
	STILL_ROT,
	STOP,

}enum_act4safe;

typedef enum
{
	OMNI,	// omni-dir means no dir
	LEFT,
	RIGHT,
	FRONT,
	BACK,
	
}enum_bearing;

class protector
{
	public:
		
		float scan4safty[SCAN4SAFE_NUM];
		float ultra_vec[ULTRA_NUM];

		float min_scan;
		float min_ultra;

		unsigned int min_index_scan;
		unsigned int min_index_ultra;
		
		bool bumper_signal;

		float v;
		float vth;

		enum_act4safe advise_action;

		bool collision_flag;
		float colision_prob;

		float laser_unsafe_prob;
		float ultra_unsafe_prob;
		
		ros::NodeHandle nh_safety;
		
		ros::Subscriber scan_sub4safe;
		ros::Subscriber	ultra_sub4safe;
		ros::Subscriber	bumper_sub4safe;
		ros::Subscriber	Odom_sub4safe;

		protector();
		
		~protector();

		float CalcMinDis4LaserScan(float* laser_vec);
	
		float CalcMinDis4Ultrosonic(float* ultra_vec);
		float IntegrateMultiInfo4Safety(enum_act4safe* advise_action);
		bool StopMovingInForce(bool collision_flag, float colision_prob);

		bool Detect4ExceptHighVel(float* v, float* vth);
		
	private:
		
		void ScanSafeCallBack(const sensor_msgs::LaserScan::ConstPtr& scan4safe);
		void UltraSafeCallBack(const colibri_aiv::ultrasonic::ConstPtr& ultra4safe);
		void BumperSafeCallBack(const colibri_aiv::collision::ConstPtr& bumper4safe);
		void OdomSafeCallBack(const nav_msgs::Odometry::ConstPtr& odom4safe);



};

#endif

