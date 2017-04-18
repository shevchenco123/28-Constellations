#include "colibri_aiv.h" 

int main(int argc, char* argv[])
{	
	ros::init(argc, argv, "twist_odom_node");
	colibri_serial my_port;
	my_port.init_port("/dev/ttyUSB0");
	my_port.creat_thread(read_data_thread);
	//my_port.send_cmd(my_port.enable_motor,colibri_serial::enable_motor_finish);
	//my_port.send_cmd(my_port.req_velocity_start,colibri_serial::req_velocity_start_finish);

	//my_port.send_cmd(my_port.req_encoder_start,colibri_serial::req_encoder_start_finish);

	//my_port.send_cmd(my_port.req_ultrasonic_start,colibri_serial::req_ultrasonic_start_finish);

	//my_port.send_cmd(my_port.req_collision_start,colibri_serial::req_collision_start_finish);

	ros::spin();

	return 0;	
}

