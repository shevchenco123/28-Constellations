#include "aiv_driver.h" 

static unsigned char recv_cache[CMD_MAX_LEN];
static io_service colibri_ios;
static serial_port pserialport(colibri_ios);
static boost::system::error_code ec;

volatile bool colibri_serial::enable_motor_finish = false;
volatile bool colibri_serial::disable_motor_finish = false;
volatile bool colibri_serial::send_twist_finish = false;
volatile bool colibri_serial::req_encoder_start_finish = false;
volatile bool colibri_serial::req_imu_start_finish = false;
volatile bool colibri_serial::req_ultrasonic_start_finish = false;
volatile bool colibri_serial::req_collision_start_finish = false;
volatile bool colibri_serial::req_velocity_start_finish = false;
volatile bool colibri_serial::req_encoder_stop_finish = false;
volatile bool colibri_serial::req_imu_stop_finish = false;
volatile bool colibri_serial::req_ultrasonic_stop_finish = false;
volatile bool colibri_serial::req_collision_stop_finish = false;
volatile bool colibri_serial::req_velocity_stop_finish = false;

unsigned char *colibri_serial::send_cache = NULL;

void *read_data_thread(void *args)
{
	colibri_serial example_port;
	example_port.init_sub_and_pub();
	example_port.read_from_serial(NULL);
}

colibri_serial::colibri_serial()
{
	serial_send_times = 0;
	serial_recv_times = 0;

	last_time = ros::Time::now();
	current_time = ros::Time::now();
	
	left_rotation_rate = 0.0;
	right_rotation_rate = 0.0;
	left_last_velocity = 0.0;
	left_current_velocity = 0.0;
	right_last_velocity = 0.0;
	right_current_velocity = 0.0;	
	left_average_velocity = 0.0;
	right_average_velocity = 0.0;
	left_average_distance = 0.0;
	right_average_distance = 0.0;

	aiv_dx = 0.0;	
	aiv_dth = 0.0;	
	aiv_vx = 0.0;	
	aiv_vth = 0.0;
	aiv_position_x = 0.0;	
	aiv_position_y = 0.0;
	aiv_angle= 0.0;

	aiv_pose_trans_x = 0.0;
	aiv_pose_trans_y = 0.0;
	aiv_pose_rotate_yaw = 0.0;

	

	unsigned char cmd_data[21];
	memset(cmd_data,0,21);
	
	cmd_data[0] = 0xff;
	produce_cmd(enable_motor,0x00,0x01,0x00,cmd_data);
	cmd_data[0] = 0x55;
	produce_cmd(disable_motor,0x00,0x01,0x00,cmd_data);
	cmd_data[0] = 0x00;
	produce_cmd(send_twist,0x01,0x04,0x00,cmd_data);
	cmd_data[0] = 0x00;
	produce_cmd(req_encoder_start,0x02,0x00,CMD_START,cmd_data);
	produce_cmd(req_imu_start,0x03,0x00,CMD_START,cmd_data);
	produce_cmd(req_ultrasonic_start,0x04,0x00,CMD_START,cmd_data);
	produce_cmd(req_collision_start,0x05,0x00,CMD_START,cmd_data);
	produce_cmd(req_velocity_start,0x06,0x00,CMD_START,cmd_data);
	
	produce_cmd(req_encoder_stop,0x02,0x00,CMD_STOP,cmd_data);
	produce_cmd(req_imu_stop,0x03,0x00,CMD_STOP,cmd_data);
	produce_cmd(req_ultrasonic_stop,0x04,0x00,CMD_STOP,cmd_data);
	produce_cmd(req_collision_stop,0x05,0x00,CMD_STOP,cmd_data);
	produce_cmd(req_velocity_stop,0x06,0x00,CMD_STOP,cmd_data);	
/*	
	cout<<"enable_motor:"<<endl;
	print_cmd_list(enable_motor);
	
	cout<<"disable_motor:"<<endl;
	print_cmd_list(disable_motor);
	
	cout<<"send_twist:"<<endl;
	print_cmd_list(send_twist);
	
	cout<<"request_encoder:"<<endl;
	print_cmd_list(request_encoder);
	
	cout<<"request_imu:"<<endl;
	print_cmd_list(request_imu);	
	
	cout<<"request_ultrasonic:"<<endl;
	print_cmd_list(request_ultrasonic);
	
	cout<<"request_collision:"<<endl;
	print_cmd_list(request_collision);

	cout<<"request_velocity:"<<endl;
	print_cmd_list(request_velocity);
*/

	cartodom_x = -OFFSET_LASER_X;
	cartodom_y = 0.0;
	cartodom_yaw = 0.0;
	
	cartodom_qx = 0.0;
	cartodom_qy = 0.0;
	cartodom_qz = 0.0;
	cartodom_qw = 1.0;
	
	cartodom_vx = 0.0;
	cartodom_vth = 0.0;
	cartodom_interval = 0.02;
	


}

colibri_serial::~colibri_serial()
{


}

bool colibri_serial::init_port(const string port_name)
{
/*
	pserialport = serial_port(colibri_ios);
	if(!pserialport){
		cout << "pserialport is NULL !"
		return false;
	}
*/
	//todo:check if it is open successfully
	pserialport.open(port_name,ec);

       pserialport.set_option( serial_port::baud_rate( 115200 ), ec );  
       pserialport.set_option( serial_port::flow_control( serial_port::flow_control::none ), ec );  
       pserialport.set_option( serial_port::parity( serial_port::parity::none ), ec );  
       pserialport.set_option( serial_port::stop_bits( serial_port::stop_bits::one ), ec);  
       pserialport.set_option( serial_port::character_size( 8 ), ec);  

	 cout<<port_name<<" serial port init complete"<<endl;

	return true;

	
}

void colibri_serial::produce_cmd(unsigned char *cmd_name,unsigned char cmd,unsigned char data_count,unsigned char control,unsigned char *data)
{
	memset(cmd_name,0,CMD_MAX_LEN);

	*(cmd_name + 0) = 0xfa;
	*(cmd_name + 1) = 0x55;	
	*(cmd_name + 3) = 0x05;
	*(cmd_name + 28) = 0xcc;
	*(cmd_name + 29) = 0xcc;
	*(cmd_name + 30) = 0x55;
	*(cmd_name + 31) = 0xfa;

	*(cmd_name + CMD_BYTE) = cmd;
	*(cmd_name + DATA_COUNT_BYTE) = data_count;
	*(cmd_name + CMD_CONTROL) = control;

	if(data_count != 0)
	{
		for(int i = 0;i < data_count;i++)
		{
			*(cmd_name + DATA_BYTE) = *(data + i);

		}
	}

	return;
}


void colibri_serial::write_to_serial(const unsigned char * data)
{
	size_t len = write(pserialport, buffer(data,CMD_MAX_LEN),ec);
	cout <<"send "<<len<<" Bytes:";

	char i;
	for(i = 0;i < CMD_MAX_LEN; i ++)
	{
		printf(" %x",data[i]);	

	}
	cout << endl;
}


void colibri_serial::read_callback(unsigned char buf[],boost::system::error_code ec,std::size_t bytes_transferred)
{	
	//cout<<"callback read "<<bytes_transferred<<" bytes:";

	unsigned char recv_data[CMD_MAX_LEN];
        memset(recv_data,0,CMD_MAX_LEN);
        if((recv_cache[31] == 0xfa) && (recv_cache[0] == 0x55)){
            memcpy(recv_data,&recv_cache[31],1);
            memcpy(&recv_data[1],recv_cache,31);
        }else{   
            for(int i = 0;i < bytes_transferred - 1; i ++) 
            {   
                if((recv_cache[i] == 0xfa) && (recv_cache[i + 1] == 0x55)){
                    memcpy(recv_data,&recv_cache[i],bytes_transferred - i); 
                    memcpy(&recv_data[bytes_transferred - i],recv_cache,i);
                }
            }   
	}	
	
	for(int i = 0;i < bytes_transferred; i ++)
	{
		//printf(" %x",(unsigned char)recv_data[i]);	

	}
	//cout << "  ---finished"<<endl;
	
	if((recv_data[START_BYTE1] != CMD_START1) || (recv_data[START_BYTE2] != CMD_START2) || (recv_data[END_BYTE1] != CMD_END1) || (recv_data[END_BYTE2] != CMD_END2))
	{
		cout<<"The format of received data is not correct !"<<endl;
		for(int i = 0;i < bytes_transferred; i ++)
		{
			printf(" %x",(unsigned char)recv_cache[i]);	

		}		
		cout << endl;
		
		for(int i = 0;i < bytes_transferred; i ++)
		{
			printf(" %x",(unsigned char)recv_data[i]);	

		}
		cout << endl;
		
		return;
	}

	//TODO:CRC verify

	if(recv_data[REP_REQ_BYTE] != CMD_RESPONSE_SUCCESS)
	{
		if(recv_data[REP_REQ_BYTE] == CMD_RESPONSE_FAIL)
		{
			cout<<"TRIO respones failed !"<<endl;
			return;
		}else{
			cout<<"TRIO returned a unknown byte in 3rd Byte:"<<recv_data[REP_REQ_BYTE]<<endl;
			return;
		}
	}

	switch(recv_data[CMD_BYTE])
	{
		case ENABLE_DISABLE_MOTOR:
			if((colibri_serial::enable_motor_finish == true) ||(colibri_serial::disable_motor_finish == true)){
				if(recv_data[DATA_COUNT_BYTE] != 0x01){
					cout<<"The data count byte of the respones of the enable_disable_motor cmd shoud be 0x01,but returned is: "<<recv_data[DATA_COUNT_BYTE]<<endl;
					return;
				}
				
				if((colibri_serial::enable_motor_finish == true) && (recv_data[DATA_BYTE + 0] == 0xff)){
					enable_motor_finish = false;
					cout<<"enable_motor cmd is executed successfully !"<<endl;
					return;
					
				}else if((colibri_serial::disable_motor_finish == true) && (recv_data[DATA_BYTE + 0] == 0x55)){
					colibri_serial::disable_motor_finish = false;
					cout<<"disable_motor cmd is executed successfully !"<<endl;
					return;
				}
				
			}else{
				cout<<"ROS does not send enable or disable motor cmd,but recv a response cmd !"<<endl;
				return;
			}

			break;
			
		case SEND_TWIST:
			if(colibri_serial::send_twist_finish == true){
				if(recv_data[DATA_COUNT_BYTE] != 0x04)
				{
					cout<<"The data count byte of the respones of the send_twist cmd shoud be 0x04,but returned is: "<<recv_data[DATA_COUNT_BYTE]<<endl;
					return;
				}

				if( recv_data[DATA_BYTE + 0] != colibri_serial::send_cache[DATA_BYTE + 0] || recv_data[DATA_BYTE + 1] != colibri_serial::send_cache[DATA_BYTE + 1] ||
				    recv_data[DATA_BYTE + 2] != colibri_serial::send_cache[DATA_BYTE + 2] || recv_data[DATA_BYTE + 3] != colibri_serial::send_cache[DATA_BYTE + 3]){
					cout<<"The received data bytes is not same with the send data byte"<<endl;
					return;
				}else{
					colibri_serial::send_twist_finish = false;

					cout<<"send_twist cmd is executed successfully !"<<endl;
				}
			
			}else{
				cout<<"ROS does not send send_twist cmd,but recv a response cmd !"<<endl;
				return;
			}
			break;
		case REQUEST_ENCODER:
//			if((colibri_serial::req_encoder_start_finish == true) || (colibri_serial::req_encoder_stop_finish == true)){
				if(recv_data[DATA_COUNT_BYTE] != 0x06){
					cout<<"The data count byte of the respones of the request_encoder cmd shoud be 0x06,but returned is: "<<recv_data[DATA_COUNT_BYTE]<<endl;
					return;
				}else{
					if(recv_data[CMD_CONTROL] == CMD_START){
						colibri_serial::req_encoder_start_finish = false;
					}else if(recv_data[CMD_CONTROL] == CMD_STOP){
						colibri_serial::req_encoder_stop_finish = false;
					}
				}
//			}else{		
//				cout<<"ROS does not send request_encoder cmd,but recv a response cmd !"<<endl;
//				return;
//			}
			
			break;
		case REQUEST_IMU:
//			if((colibri_serial::req_imu_start_finish == true) || (colibri_serial::req_imu_stop_finish == true)){			
				if(recv_data[DATA_COUNT_BYTE] != 0x0c)
				{
					cout<<"The data count byte of the respones of the request_imu cmd shoud be 0x0c,but returned is: "<<recv_data[DATA_COUNT_BYTE]<<endl;
					return;
				}else{
					cout<<"request_imu cmd is executed successfully !"<<endl;
					
					if(recv_data[CMD_CONTROL] == CMD_START){
						colibri_serial::req_imu_start_finish = false;
					}else if(recv_data[CMD_CONTROL] == CMD_STOP){
						colibri_serial::req_imu_stop_finish = false;
					}					

				}
//			}else{
//				cout<<"ROS does not send request_IMU cmd,but recv a response cmd !"<<endl;
//				return;
//			}
			break;
		case REQUEST_ULTRASONIC:
//			if((colibri_serial::req_ultrasonic_start_finish == true) || (colibri_serial::req_ultrasonic_stop_finish == true)){	
				if(recv_data[DATA_COUNT_BYTE] != 0x0c)
				{
					cout<<"The data count byte of the respones of the request_ultrasonic cmd shoud be 0x0c,but returned is: "<<recv_data[DATA_COUNT_BYTE]<<endl;
					return;
				}else{
					if(recv_data[CMD_CONTROL] == CMD_START){
						colibri_aiv::ultrasonic ultra;
						ultra.header.stamp = ros::Time::now();
						ultra.header.frame_id = "ultrasonic";
						ultra.ultrasonic1 = (recv_data[DATA_BYTE + 0] * 256 + recv_data[DATA_BYTE + 1]) & 0xffff;
						ultra.ultrasonic2 = (recv_data[DATA_BYTE + 2] * 256 + recv_data[DATA_BYTE + 3]) & 0xffff;
						ultra.ultrasonic3 = (recv_data[DATA_BYTE + 4] * 256 + recv_data[DATA_BYTE + 5]) & 0xffff;
						ultra.ultrasonic4 = (recv_data[DATA_BYTE + 6] * 256 + recv_data[DATA_BYTE + 7]) & 0xffff;
						ultra.ultrasonic5 = (recv_data[DATA_BYTE + 8] * 256 + recv_data[DATA_BYTE + 9]) & 0xffff;
						ultra.ultrasonic6 = (recv_data[DATA_BYTE + 10] * 256 + recv_data[DATA_BYTE + 11]) & 0xffff;
						ultrasonic_pub.publish(ultra);
						colibri_serial::req_ultrasonic_start_finish = false;
						cout<<"request_ultrasonic cmd is executed successfully !"<<endl;
					}else if(recv_data[CMD_CONTROL] == CMD_STOP){
						colibri_serial::req_ultrasonic_stop_finish = false;
					}
				}
//			}
			break;
			
		case REQUEST_COLLISION:
			if(recv_data[DATA_COUNT_BYTE] != 0x01)
			{
				cout<<"The data count byte of the respones of the request_collision cmd shoud be 0x01,but returned is: "<<recv_data[DATA_COUNT_BYTE]<<endl;
				return;
			}else{
				if(recv_data[CMD_CONTROL] == CMD_START){
					colibri_aiv::collision colli;
					colli.header.stamp = ros::Time::now();
					colli.header.frame_id = "collision";
					if(recv_data[DATA_BYTE + 0] == 0x55){
						colli.collision.data = false;
					}else if (recv_data[DATA_BYTE + 0] == 0xff){
						colli.collision.data = true;
					}else{
						cout<<"request_collision cmd recv a illegal data !"<<endl;
						return;
					}
					collision_pub.publish(colli);
					colibri_serial::req_collision_start_finish = false;
					cout<<"request_collision cmd is executed successfully !"<<endl;
				}else if(recv_data[CMD_CONTROL] == CMD_STOP){
					colibri_serial::req_collision_stop_finish = false;
				}
			}

			break;

		case REQUEST_VELOCITY:
//			if(colibri_serial::request_velocity_finished == true){
				if(recv_data[DATA_COUNT_BYTE] != 0x06)
				{
					cout<<"The data count byte of the respones of the request_velocity cmd shoud be 0x06,but returned is: "<<recv_data[DATA_COUNT_BYTE]<<endl;
					return;
				}
				if(recv_data[CMD_CONTROL] == CMD_START){
					if(recv_data[DATA_BYTE + 0] == 0x00){
						left_rotation_rate = (float)( (recv_data[DATA_BYTE + 1] << 8) |recv_data[DATA_BYTE + 2]);
					}else if(recv_data[DATA_BYTE + 0] == 0xff){
						left_rotation_rate =(-1)*(float) ( (recv_data[DATA_BYTE + 1] << 8) |recv_data[DATA_BYTE + 2]);
					}else{
						cout<<"The [DATA_BYTE + 0] of the request_velocity cmd is illegal !"<<endl;
						return;
					}

					if(recv_data[DATA_BYTE + 3] == 0x00){
						right_rotation_rate = (-1)*(float)( (recv_data[DATA_BYTE + 4] << 8) |recv_data[DATA_BYTE + 5]);	
					}else if(recv_data[DATA_BYTE + 3] == 0xff){
						right_rotation_rate = (float)( (recv_data[DATA_BYTE + 4] << 8) |recv_data[DATA_BYTE + 5]);	
					}else{
						cout<<"The [DATA_BYTE + 0] of the request_velocity cmd is illegal !"<<endl;
						return;
					}

					//cout<<"left_rotation_rate:"<<left_rotation_rate<<"   right_rotation_rate:"<<right_rotation_rate<<endl;

					current_time = ros::Time::now();
					time_period = (current_time - last_time).toSec();
					last_time = current_time;

					

					left_current_velocity = (2 * PI * WHEEL_RADIUS * left_rotation_rate)/(60 * WHEEL_GEAR);  //left speed:m/s
					right_current_velocity = (2 * PI * WHEEL_RADIUS * right_rotation_rate)/(60 * WHEEL_GEAR); //right speed:m/s
					//cout<<"left_current_velocity:"<<left_current_velocity<<"   right_current_velocity:"<<right_current_velocity<<endl;
					//cout<<"left_last_velocity:"<<left_last_velocity<<"   right_last_velocity:"<<right_last_velocity<<endl;
					
					left_average_velocity = (left_current_velocity + left_last_velocity) / 2.0;
					right_average_velocity = (right_current_velocity + right_last_velocity) / 2.0;
					left_last_velocity = left_average_velocity;
					right_last_velocity = right_average_velocity;

					left_average_distance = left_average_velocity * time_period;
					right_average_distance = right_average_velocity * time_period;

					aiv_dx = (left_average_distance + right_average_distance) / 2.0;
					aiv_dth = (right_average_distance - left_average_distance) / WHEEL_TRACK;
					aiv_vx = (left_average_velocity + right_average_velocity) / 2.0;
					//aiv_vth = aiv_dth / time_period;

					geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(cartodom_yaw);
					
					geometry_msgs::TransformStamped odom_trans;
					odom_trans.header.stamp = current_time;
					odom_trans.header.frame_id = "odom";
					//odom_trans.child_frame_id = "base_link";
					odom_trans.child_frame_id = "base_footprint";					

					odom_trans.transform.translation.x = cartodom_x;
					odom_trans.transform.translation.y = cartodom_y;
					odom_trans.transform.translation.z = 0.0;
					odom_trans.transform.rotation = odom_quat;

					//send the transform
					odom_broadcaster.sendTransform(odom_trans);
					
					//publish the odometry message over ROS
					nav_msgs::Odometry odom;
					odom.header.stamp = current_time;
					odom.header.frame_id = "odom";
					
					//set the position
					odom.pose.pose.position.x = cartodom_x;
					odom.pose.pose.position.y = cartodom_y;
					odom.pose.pose.position.z = 0.0;
					odom.pose.pose.orientation = odom_quat;
					
					//set the velocity
					odom.child_frame_id = "base_footprint";
					odom.twist.twist.linear.x = aiv_vx;
					odom.twist.twist.linear.y = 0;
					odom.twist.twist.angular.z = cartodom_vth;
					
					//publish the message
					odom_pub.publish(odom);
					
					//ROS_INFO("virtual x/y/yaw: %0.3lfm %0.3lfm %0.2lfdeg", cartodom_x, cartodom_y, cartodom_yaw*RAD_TO_DEGREE_COEFF);

					//ROS_INFO("virtual vx/vth: %0.3lfm/s  %0.2lfrad/s",cartodom_vx,cartodom_vth);

					colibri_serial::req_velocity_start_finish = false;
					//cout<<"request_velocity cmd is executed successfully !"<<endl;
				}else if(recv_data[CMD_CONTROL] == CMD_STOP){
					colibri_serial::req_velocity_stop_finish = false;
				}
		
//			}else{			
//				cout<<"ROS does not send request_velocity cmd,but recv a response cmd !"<<endl;
//				return;
//			}
			
			break;
			
		default :

			cout<<"received unknown cmd_byte:"<<recv_data[CMD_BYTE]<<endl;
			break;
			
	}
		
	return;
}


void colibri_serial::read_from_serial(void *args)
{
	while(ros::ok())
	{
		async_read(pserialport,buffer(recv_cache,CMD_MAX_LEN),boost::bind(&colibri_serial::read_callback,this,recv_cache,_1,_2));
		call_handle();
		serial_recv_times ++;
		//cout <<"recv times: "<<serial_recv_times<<endl;
	}
}


void colibri_serial::call_handle()
{
//	colibri_ios.poll();
	colibri_ios.run();
	colibri_ios.reset();
}

void colibri_serial::creat_thread(void *(*start_routine) (void *))
{	
	int ret = pthread_create(&thread_id,NULL,start_routine,NULL);
	if( ret ){
	   cout << "pthread_create error,error_code: "<< ret <<endl;
	   return;
	}
}

bool colibri_serial::init_sub_and_pub()
{
	//ros::NodeHandle global_nh("/move_base");
	ros::NodeHandle global_nh;	
	twist_sub= global_nh.subscribe<geometry_msgs::Twist>("t_cmd_vel", 1, boost::bind(&colibri_serial::twist_callback, this, _1));

	ros::NodeHandle nh_cartodom;
	cartodom_sub= global_nh.subscribe<cartodom::Cartodom>("cartodom", 1, boost::bind(&colibri_serial::cartodom_callback, this, _1));
	
	ros::NodeHandle nh_odom;
	odom_pub = nh_odom.advertise<nav_msgs::Odometry>("odom", 50);

	ros::NodeHandle nh_ultrasonic;
	ultrasonic_pub = nh_ultrasonic.advertise<colibri_aiv::ultrasonic>("ultrasonic", 50);
	
	ros::NodeHandle nh_collision;
	collision_pub = nh_collision.advertise<colibri_aiv::collision>("collision", 50);
	
}
void colibri_serial::twist_callback(const geometry_msgs::Twist::ConstPtr & twist)
{
	//rostopic pub -1 /move_base/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
	//use {rostopic pub /move_base/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]} for test.
	
//	cout << "Twist_callback executed:"<<endl<<*twist<<endl;
	if(twist->linear.x < 0 )
	{
		send_twist[DATA_BYTE + 0] = 0xff;
		send_twist[DATA_BYTE + 1] = abs(100*twist->linear.x);
	}else{
		send_twist[DATA_BYTE + 0] = 0x00;
		send_twist[DATA_BYTE + 1] = 100*twist->linear.x;
	}

	if(twist->angular.z < 0 )
	{
		send_twist[DATA_BYTE + 2] = 0xff;
		send_twist[DATA_BYTE + 3] = abs(100*twist->angular.z);
	}else{
		send_twist[DATA_BYTE + 2] = 0x00;
		send_twist[DATA_BYTE + 3] = 100*twist->angular.z;
	}

	colibri_serial::send_cache = send_twist;
	send_cmd(send_twist,colibri_serial::send_twist_finish);
	
	serial_send_times ++;
	//cout <<"send times: "<<serial_send_times<<endl;
}

void colibri_serial::cartodom_callback(const cartodom::Cartodom::ConstPtr & carto_odom)
{
	cartodom_x = carto_odom->x;
	cartodom_y = carto_odom->y;
	cartodom_yaw = carto_odom->yaw;
	
	cartodom_qx = carto_odom->qx;
	cartodom_qy = carto_odom->qy;
	cartodom_qz = carto_odom->qz;
	cartodom_qw = carto_odom->qw;
	
	cartodom_vx = carto_odom->vx;
	cartodom_vth = carto_odom->vth;
	cartodom_interval = carto_odom->interval;
}


void colibri_serial::send_cmd(const unsigned char *cmd ,volatile bool &send_flag)
{
	write_to_serial(cmd);
	send_flag = true;
	ros::Time start_time = ros::Time::now();

	while(send_flag == true)
	{
		if(ros::Time::now() - start_time > ros::Duration(TIMEOUT, 0)){
			switch(cmd[CMD_BYTE])
			{
				case ENABLE_DISABLE_MOTOR:
					if(cmd[DATA_BYTE + 0] == 0xff){
						cout<<"TRIO respones the enable_motor cmd timeout !"<<endl;
					}else if(cmd[DATA_BYTE + 0] == 0x55){
						cout<<"TRIO respones the disable_motor cmd timeout !"<<endl;	
					}
					break;
					
				case SEND_TWIST:
					cout<<"TRIO respones the send_twist cmd timeout !"<<endl;
					break;
					
				case REQUEST_ENCODER:
					cout<<"TRIO respones the request_encoder cmd timeout !"<<endl;
					break;

				case REQUEST_IMU:
					cout<<"TRIO respones the request_imu cmd timeout !"<<endl;
					break;

				case REQUEST_ULTRASONIC:
					cout<<"TRIO respones the request_ultrasonic cmd timeout !"<<endl;
					break;

				case REQUEST_COLLISION:
					cout<<"TRIO respones the request_collision cmd timeout !"<<endl;
					break;

				case REQUEST_VELOCITY:
					cout<<"TRIO respones the request_velocity cmd timeout !"<<endl;
					break;

				default :
					break;
			}
			
			break;
		}
	 }

}

void colibri_serial::print_cmd_list(unsigned char *cmd_list)
{
	char i;
	
	for(i = 0;i < CMD_MAX_LEN; i ++)
	{
		printf(" %x",cmd_list[i]);	
	
	}
	
	cout <<endl;
}


