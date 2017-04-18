#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <pthread.h>
#include <string>  
#include <vector>  
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "colibri_aiv/ultrasonic.h"
#include "colibri_aiv/collision.h"
#include "cartodom/Cartodom.h"


#ifndef _AIV_DRIVER_H_
#define _AIV_DRIVER_H_

using namespace std;
using namespace boost::asio;

//The definition of the protocol
#define CMD_MAX_LEN                    32
#define START_BYTE1                     0
#define START_BYTE2                     1
#define SERIAL_NUM_BYTE              2
#define REP_REQ_BYTE                   3
#define DATA_COUNT_BYTE             4
#define CMD_BYTE                           5
#define CMD_CONTROL                   6
#define DATA_BYTE                          7
#define END_BYTE1                          30
#define END_BYTE2                          31

//The definition of the cmd byte
#define ENABLE_DISABLE_MOTOR         0x00
#define SEND_TWIST                                    0x01
#define REQUEST_ENCODER                     0x02
#define REQUEST_IMU                                  0x03
#define REQUEST_ULTRASONIC                0x04
#define REQUEST_COLLISION                    0x05
#define REQUEST_VELOCITY                     0x06


//The definition of the response type
#define CMD_START1                              0xfa
#define CMD_START2                              0x55
#define CMD_SERIAL_NUM                       0x00
#define CMD_REQUEST                            0x05
#define CMD_RESPONSE_SUCCESS           0x06
#define CMD_RESPONSE_FAIL                  0x15
#define CMD_START                                0x00
#define CMD_STOP                                  0xff
#define CMD_END1                                   0x55
#define CMD_END2                                   0xfa


//The definition of the timeout--3 seconds
#define TIMEOUT                                      1
#define PI                                                    3.14159
#define WHEEL_TRACK                          0.46
#define WHEEL_RADIUS                          0.1
#define WHEEL_GEAR                              25


//The definiotion of the offset by ww
#define OFFSET_LASER_X		0.352
#define DEADZONE_X			0.005
#define DEADZONE_Y			0.005
#define DEADZONE_YAW_DEGREE			0.05
#define RAD_TO_DEGREE_COEFF		57.296
#define DEGREE_TO_RAD_COEFF		0.01745

void *read_data_thread(void *args);


class colibri_serial
{
	private:
		void produce_cmd(unsigned char *cmd_name,unsigned char cmd,unsigned char data_count,unsigned char control,unsigned char *data);
		void twist_callback(const geometry_msgs::Twist::ConstPtr & twist);
		void print_cmd_list(unsigned char *cmd_list);
	public:
		unsigned int serial_send_times;
		unsigned int serial_recv_times;

		//cmd list
		unsigned char       enable_motor[CMD_MAX_LEN];
		unsigned char       disable_motor[CMD_MAX_LEN];
		unsigned char            send_twist[CMD_MAX_LEN];
		
		unsigned char req_encoder_start[CMD_MAX_LEN];
		unsigned char req_imu_start[CMD_MAX_LEN];
		unsigned char req_ultrasonic_start[CMD_MAX_LEN];
		unsigned char req_collision_start[CMD_MAX_LEN];
		unsigned char req_velocity_start[CMD_MAX_LEN];
		
		unsigned char req_encoder_stop[CMD_MAX_LEN];
		unsigned char req_imu_stop[CMD_MAX_LEN];
		unsigned char req_ultrasonic_stop[CMD_MAX_LEN];
		unsigned char req_collision_stop[CMD_MAX_LEN];
		unsigned char req_velocity_stop[CMD_MAX_LEN];
		
		static volatile bool enable_motor_finish;
		static volatile bool disable_motor_finish;
		static volatile bool send_twist_finish;
		static volatile bool req_encoder_start_finish;
		static volatile bool req_imu_start_finish;
		static volatile bool req_ultrasonic_start_finish;
		static volatile bool req_collision_start_finish;
		static volatile bool req_velocity_start_finish;
		
		static volatile bool req_encoder_stop_finish;
		static volatile bool req_imu_stop_finish;
		static volatile bool req_ultrasonic_stop_finish;
		static volatile bool req_collision_stop_finish;
		static volatile bool req_velocity_stop_finish;		

		static unsigned char *send_cache;

		double cartodom_x;
		double cartodom_y;
		float cartodom_yaw;

		float cartodom_qx;
		float cartodom_qy;
		float cartodom_qz;
		float cartodom_qw;

		float cartodom_vx;
		float cartodom_vth;
		float cartodom_interval;
		
		//Constructor
		colibri_serial();
		//Destructor
		~colibri_serial();

		bool init_port(const string port_name);

		bool init_sub_and_pub();

		//write data to serial port
		void write_to_serial(const unsigned char * data);
		//read data from serial port
		void read_from_serial(void *args);
		
		//the asyanc callback function of asyanc_read
		void read_callback(unsigned char buf[],boost::system::error_code ec,std::size_t bytes_transferred);
		//to call io_service::run function to call asyanc callback function
		void call_handle();

		void creat_thread(void *(*start_routine) (void *));

		void send_cmd(const unsigned char *cmd ,volatile bool &send_flag);
		
		void cartodom_callback(const cartodom::Cartodom::ConstPtr & carto_odom);


	private:

		pthread_t thread_id;

		double time_period;

		ros::Time last_time;
		
		ros::Time current_time;

		float left_rotation_rate;

		float right_rotation_rate;

		float left_last_velocity;

		float left_current_velocity;

		float right_last_velocity;

		float right_current_velocity;

		float left_average_velocity;
		
		float right_average_velocity;
		
		float left_average_distance;
		
		float right_average_distance;

		float aiv_dx;

		float aiv_vx;

		float aiv_dth;

		float aiv_vth;

		float aiv_position_x;

		float aiv_position_y;

		float aiv_angle;
		
		// the vars below are extended for trio calc
		float aiv_pose_trans_x;
		float aiv_pose_trans_y;
		float aiv_pose_rotate_yaw;

		ros::Subscriber twist_sub;

		ros::Publisher odom_pub;

		ros::Publisher ultrasonic_pub;

		ros::Publisher collision_pub;

		tf::TransformBroadcaster odom_broadcaster;

		//extended by ww

		ros::Subscriber cartodom_sub;
		cartodom::Cartodom cartodom;
		
		

		
	
};

#endif

