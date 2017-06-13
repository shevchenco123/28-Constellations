#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <ctime>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include "colibri_msgs/AngPotnEngy.h"

#ifndef _COLIBRI_CA_H_
#define _COLIBRI_CA_H_

using namespace std;

#define NUM_RAY4CA 				181		//every degree for front semi-cirle
#define ANGLE4CA_START 			0
#define ANGLE4CA_START_INDEX 	61		// 61=(0-(-30))/0.5+1
#define ANGLE4CA_END 			180
#define ANGLE4CA_END_INDEX 		421		// 421=(180-(-30))/0.5+1
#define LASER_EDGE_MIN 			0.06
#define RAY_RESOL4CA			1.0		//laser ray resolution for colision avoidence

#define K_SF 	1.5		//adj factor for latitude dir in polar frame
#define WIDTH 	0.56
#define D_SF	0.336	//0.5*K_SF*WIDTH  lateral safe dis

#define K_SR 	1.1		//adj factor for longitude dir in polar frame
#define ACC_DEC -2.0		//accerlaration for decrease vel
#define D_M 	5.0		// local ca distance
#define KP_PHI_INF 10000.0

#define V_MAX 		0.35
#define V_MIN 		0.02
#define THETA_V_MAX 0.35

#define KRF_CORRECTOR 0.1

#define PASSFCN_THD_RATIO 	0.2
#define MAX_PASSFCN_SCOPE	0.1
#define PASSVAL_TOLLERENCE  0.1

#define RAD2DEG 	57.2958
#define DEG2RAD 	0.01745
#define PI 			3.1415926

#define	DEG2RAD_PARAM(deg)	PI * deg / 180.0
#define	RAD2DEG_PARAM(rad)	180.0 * rad / PI

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))


#define DELAY_CNT_MAX 	10

class scan_ca
{
	public:
		
		float *ptrScan4ca;
		float scan4ca[NUM_RAY4CA];
		
		float delta_phi_vec[NUM_RAY4CA];
		float kp_phi_vec[NUM_RAY4CA];

		float krf_vec[NUM_RAY4CA];
		float kaf_vec[NUM_RAY4CA];
		float passfcn_vec[NUM_RAY4CA];
		float max_passfcn_val;

		int fwd_maxpass_cnt ;
		int bwd_maxpass_cnt ;

		int maxfcn_fwdbnd;
		int maxfcn_bwdbnd;

		int phi_start_vec[NUM_RAY4CA];
		int phi_end_vec[NUM_RAY4CA];

		float goal_dir;	//goal direction in degree
		float v;
		float vth;
		
		float angle_adj;
		float vel_adj;
		unsigned int apf_alarm;
		unsigned int wander;
		
		ros::NodeHandle nh_ca;
		ros::Subscriber scan_sub4ca;
		ros::Publisher apf_pub4mntr;
		ros::Publisher rf_pub4mntr;
		
		scan_ca();
		~scan_ca();
		
		float CalcKpPhi(float vel_center, float d_phi);
		void CalcKrfTheta(float* ptrKp_phi_vector, int* ptrPhi_range_start, int* ptrPhi_range_end);	
		void CalcPhiRange(int i, int range_num, int* ptrPhi_start, int* ptrPhi_end);
		void CalcPassFcnAndFwdBnd(unsigned int flag, float* max_passfcn_val, float* ptrK_pg);
		void CalcPassFcnAndBwdBnd(unsigned int flag, float* max_passfcn_val, float* ptrK_pg);
		float CalcAdjDir(float* ptrPass_fcn_vector, float max_passfcn_val, int* fwd_bound, int* bwd_bound);
		void CalcCorrectedKrf(void);
		void CalcAlarmInAPF(void);
		void CalcPhiParam(float vel_center, float& dir_goal_inlaser);
		void ResetMaxPassValCnt(void);
		void LimitAngle(float& delta_ang);
		
	private:

		void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_ca);

		float CalcDsrVc(float vel_center);
		float CalcKrfCorrectFactor(int index);

};


#endif

