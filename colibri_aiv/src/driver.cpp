#include "aiv_driver.h" 

static unsigned char recv_cache[CONST_PROTOCOL_LEN];
static io_service AIV_ios;
static serial_port pserialport(AIV_ios);
static boost::system::error_code ec;

volatile bool AIV_Driver::enable_motor_finish = false;
volatile bool AIV_Driver::disable_motor_finish = false;
volatile bool AIV_Driver::send_twist_finish = false;
volatile bool AIV_Driver::req_encoder_start_finish = false;
volatile bool AIV_Driver::req_imu_start_finish = false;
volatile bool AIV_Driver::req_ultra_start_finish = false;
volatile bool AIV_Driver::req_bumper_start_finish = false;
volatile bool AIV_Driver::req_vel_start_finish = false;
volatile bool AIV_Driver::req_encoder_stop_finish = false;
volatile bool AIV_Driver::req_imu_stop_finish = false;
volatile bool AIV_Driver::req_ultra_stop_finish = false;
volatile bool AIV_Driver::req_bumper_stop_finish = false;
volatile bool AIV_Driver::req_vel_stop_finish = false;

unsigned char *AIV_Driver::send_cache = NULL;

void *ReadDataThread(void *args)
{
	AIV_Driver obj_driver;
	obj_driver.InitSubandPub();
	obj_driver.ReadFromCom(NULL);
}

AIV_Driver::AIV_Driver()
{
	send_cnt = 0;
	recv_cnt = 0;

	last_time = ros::Time::now();
	current_time = ros::Time::now();
	
	left_rot_rate = 0.0;
	right_rot_rate = 0.0;
	
	left_last_vel = 0.0;
	left_cur_vel = 0.0;
	right_last_vel = 0.0;
	right_cur_vel = 0.0;
	
	left_avg_vel = 0.0;
	right_avg_vel = 0.0;
	
	left_avg_distance = 0.0;
	right_avg_distance = 0.0;

	aiv_dx = 0.0;	
	aiv_dth = 0.0;	
	aiv_vx = 0.0;	
	aiv_vth = 0.0;

	unsigned char cmd_data[21];	//used to store for valid data in defined protocol
	memset(cmd_data,0,21);
	
	cmd_data[0] = ENABLE_MOTOR;
	GenerateCmd(enable_motor, ENABLE_DISABLE_MOTOR, 0x01, RSVD_VAL, cmd_data);
	
	cmd_data[0] = DISABLE_MOTOR;
	GenerateCmd(disable_motor, ENABLE_DISABLE_MOTOR, 0x01, RSVD_VAL, cmd_data);
	
	GenerateCmd(send_twist, SEND_TWIST, 0x04,RSVD_VAL, cmd_data);

	GenerateCmd(req_encoder_start, REQ_ENCODER, RSVD_VAL, FRAME_CMD_START, cmd_data);
	GenerateCmd(req_imu_start, REQ_IMU, RSVD_VAL, FRAME_CMD_START, cmd_data);
	GenerateCmd(req_ultra_start, REQ_ULTRASONIC, RSVD_VAL, FRAME_CMD_START, cmd_data);
	GenerateCmd(req_bumper_start, REQ_BUMPER, RSVD_VAL, FRAME_CMD_START, cmd_data);
	GenerateCmd(req_vel_start, REQ_VELOCITY, RSVD_VAL, FRAME_CMD_START, cmd_data);
	
	GenerateCmd(req_encoder_stop, REQ_ENCODER, RSVD_VAL, FRAME_CMD_STOP, cmd_data);
	GenerateCmd(req_imu_stop, REQ_IMU, RSVD_VAL, FRAME_CMD_STOP, cmd_data);
	GenerateCmd(req_ultra_stop, REQ_ULTRASONIC, RSVD_VAL, FRAME_CMD_STOP, cmd_data);
	GenerateCmd(req_bumper_stop, REQ_BUMPER, RSVD_VAL, FRAME_CMD_STOP, cmd_data);
	GenerateCmd(req_vel_stop, REQ_VELOCITY, RSVD_VAL, FRAME_CMD_STOP, cmd_data);
		
	//cout<<"enable_motor:"<<endl;
	DisplayFrame(req_ultra_start);
	
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

AIV_Driver::~AIV_Driver()
{

}

bool AIV_Driver::InitCom(const string port_name)
{
	/*
	pserialport = serial_port(AIV_ios);
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
	pserialport.close();
	sleep(0.2);
	
	pserialport.open(port_name,ec);

	pserialport.set_option( serial_port::baud_rate( 115200 ), ec );  
	pserialport.set_option( serial_port::flow_control( serial_port::flow_control::none ), ec );  
	pserialport.set_option( serial_port::parity( serial_port::parity::none ), ec );  
	pserialport.set_option( serial_port::stop_bits( serial_port::stop_bits::one ), ec);  
	pserialport.set_option( serial_port::character_size( 8 ), ec);  

	cout<<port_name<<" serial port init complete"<<endl;

	return true;
}

void AIV_Driver::GenerateCmd(unsigned char *cmd_name,unsigned char cmd,unsigned char valid_data_len,unsigned char control,unsigned char *data)
{
	memset(cmd_name,0,CONST_PROTOCOL_LEN);

	*(cmd_name + START_INDX_1) = FRAME_START_1;
	*(cmd_name + START_INDX_2) = FRAME_START_2;	
	*(cmd_name + RSVD_BYTE_INDX) = FRAME_RSVD;
	*(cmd_name + REQ_RES_INDX) = FRAME_REQ_RES;
	*(cmd_name + VALID_DATA_LEN_INDX) = valid_data_len;
	*(cmd_name + CMD_BYTE_INDX) = cmd;
	*(cmd_name + CMD_CTRL_INDX) = control;	

	if(valid_data_len != 0)
	{
		for(unsigned char i = 0; i < valid_data_len; i++)
		{
			*(cmd_name + VALID_DATA_START_INDX + i) = *(data + i);
		}
	}

	*(cmd_name + CRC_INDX_1) = RSVD_VAL;		// TODO : CRC SUM
	*(cmd_name + CRC_INDX_2) = RSVD_VAL;
	*(cmd_name + END_INDX_1) = FRAME_END_1;
	*(cmd_name + END_INDX_2) = FRAME_END_2;

}


void AIV_Driver::WriteToCom(const unsigned char * data)
{
	size_t len = write(pserialport, buffer(data, CONST_PROTOCOL_LEN), ec);
	
	cout <<"send "<<len<<" Bytes:";
	int i;
	for(i = 0; i < CONST_PROTOCOL_LEN; i ++)
	{
		//printf(" %x", data[i]);	
	}
	cout << endl;
}


void AIV_Driver::ReadInfoProc(unsigned char buf[], boost::system::error_code ec, std::size_t bytes_transferred)
{	
	//cout<<"callback read "<<bytes_transferred<<" bytes:";
	unsigned char recv_data[CONST_PROTOCOL_LEN];
    memset(recv_data,0,CONST_PROTOCOL_LEN);
		
    if((recv_cache[31] == FRAME_START_1) && (recv_cache[0] == FRAME_START_2) && (recv_cache[1] == FRAME_RSVD))
	{
        memcpy(recv_data, &recv_cache[31], 1);
        memcpy(&recv_data[1], recv_cache, 31);
    }
	else
	{   
		for(int i = 0;i < bytes_transferred - 1; i ++) 
		{   
			if((recv_cache[i] == FRAME_START_1) && (recv_cache[i + 1] == FRAME_START_2) && (recv_cache[i + 2] == FRAME_RSVD))
			{
			    memcpy(recv_data, &recv_cache[i], bytes_transferred - i); 
			    memcpy(&recv_data[bytes_transferred - i], recv_cache, i);
			}
		}   
	}	
	
	if((recv_data[START_INDX_1] != FRAME_START_1)
		|| (recv_data[START_INDX_2] != FRAME_START_2)
		|| (recv_data[END_INDX_1] != FRAME_END_1)
		|| (recv_data[END_INDX_2] != FRAME_END_2))
	{
		cout<<"The head and tail in received frame is not correct !"<<endl;
		for(int j = 0; j < bytes_transferred - 1; j++)
		{
			printf(" %x", recv_cache[j]);
		}
		cout << endl;

		for(int k = 0; k < bytes_transferred - 1; k++)
		{
			printf(" %x", recv_data[k]);
		}
		cout << endl;
		
		return;
	}

	//TODO:CRC verify

	if(recv_data[REQ_RES_INDX] != FRAME_RES_SUCC)
	{
		if(recv_data[REQ_RES_INDX] == FRAME_RES_FAIL)
		{
			cout<<"TRIO respones failed !"<<endl;
			return;
		}
		else
		{
			cout<<"TRIO returned a unknown byte in 4rd Byte:"<<recv_data[REQ_RES_INDX]<<endl;
			return;
		}
	}

	switch(recv_data[CMD_BYTE_INDX])
	{
		case ENABLE_DISABLE_MOTOR:
			if((AIV_Driver::enable_motor_finish == true) ||(AIV_Driver::disable_motor_finish == true))
			{
				if(recv_data[VALID_DATA_LEN_INDX] != 0x01)
				{
					cout<<"The valid data lenth of the respones for enable_disable_motor shoud be 0x01,but returned is: "<<recv_data[VALID_DATA_LEN_INDX]<<endl;
					return;
				}
				
				if((AIV_Driver::enable_motor_finish == true) && (recv_data[VALID_DATA_START_INDX + 0] == 0xff))
				{
					enable_motor_finish = false;
					cout<<"enable_motor cmd is executed successfully !"<<endl;
					return;
					
				}
				else if((AIV_Driver::disable_motor_finish == true) && (recv_data[VALID_DATA_START_INDX + 0] == 0x55))
				{
					AIV_Driver::disable_motor_finish = false;
					cout<<"disable_motor cmd is executed successfully !"<<endl;
					return;
				}
				
			}
			else
			{
				cout<<"ROS does not send enable or disable motor cmd,but recv a response cmd !"<<endl;
				return;
			}

			break;
			
		case SEND_TWIST:
			if(AIV_Driver::send_twist_finish == true)
			{
				if(recv_data[VALID_DATA_LEN_INDX] != 0x04)
				{
					cout<<"The data count byte of the respones of the send_twist cmd shoud be 0x04,but returned is: "<<recv_data[VALID_DATA_LEN_INDX]<<endl;
					return;
				}

				if( recv_data[VALID_DATA_START_INDX + 0] != AIV_Driver::send_cache[VALID_DATA_START_INDX + 0]
					|| recv_data[VALID_DATA_START_INDX + 1] != AIV_Driver::send_cache[VALID_DATA_START_INDX + 1]
					|| recv_data[VALID_DATA_START_INDX + 2] != AIV_Driver::send_cache[VALID_DATA_START_INDX + 2]
					|| recv_data[VALID_DATA_START_INDX + 3] != AIV_Driver::send_cache[VALID_DATA_START_INDX + 3] )
				{
					cout<<"The received data bytes is not same with the send data byte"<<endl;
					return;
				}
				else
				{
					AIV_Driver::send_twist_finish = false;

					//cout<<"send_twist cmd is executed successfully !"<<endl;
				}
			
			}
			else
			{
				cout<<"ROS does not send send_twist cmd,but recv a response cmd !"<<endl;
				return;
			}
			
			break;
			
		case REQ_ENCODER:
			if((AIV_Driver::req_encoder_start_finish == true) || (AIV_Driver::req_encoder_stop_finish == true))
			{
				if(recv_data[VALID_DATA_LEN_INDX] != 0x06)
				{
					cout<<"The data count byte of the respones of the request_encoder cmd shoud be 0x06,but returned is: "<<recv_data[VALID_DATA_LEN_INDX]<<endl;
					return;
				}
				else
				{
					if(recv_data[CMD_CTRL_INDX] == FRAME_CMD_START)
					{
						AIV_Driver::req_encoder_start_finish = false;
					}
					else if(recv_data[CMD_CTRL_INDX] == FRAME_CMD_STOP)
					{
						AIV_Driver::req_encoder_stop_finish = false;
					}
				}
			}
			else
			{		
				cout<<"ROS does not send request_encoder cmd,but recv a response cmd !"<<endl;
				return;
			}
			
			break;
			
		case REQ_IMU:
			if((AIV_Driver::req_imu_start_finish == true) || (AIV_Driver::req_imu_stop_finish == true))
			{			
				if(recv_data[VALID_DATA_LEN_INDX] != 0x0c)
				{
					cout<<"The data count byte of the respones of the request_imu cmd shoud be 0x0c,but returned is: "<<recv_data[VALID_DATA_LEN_INDX]<<endl;
					return;
				}else
				{
					cout<<"request_imu cmd is executed successfully !"<<endl;
					
					if(recv_data[CMD_CTRL_INDX] == FRAME_CMD_START)
					{
						AIV_Driver::req_imu_start_finish = false;
					}
					else if(recv_data[CMD_CTRL_INDX] == FRAME_CMD_STOP)
					{
						AIV_Driver::req_imu_stop_finish = false;
					}					

				}
			}
			else
			{
				cout<<"ROS does not send request_IMU cmd,but recv a response cmd !"<<endl;
				return;
			}
			
			break;
			
		case REQ_ULTRASONIC:
			//if((AIV_Driver::req_ultra_start_finish == true) || (AIV_Driver::req_ultra_stop_finish == true))
			//{	
				if(recv_data[VALID_DATA_LEN_INDX] != 0x10)
				{
					cout<<"The data count byte of the respones of the request_ultrasonic cmd shoud be 0x0c,but returned is: "<<recv_data[VALID_DATA_LEN_INDX]<<endl;
					return;
				}
				else
				{
					if(recv_data[CMD_CTRL_INDX] == FRAME_CMD_START)
					{
						colibri_aiv::Ultrasonic ultra;
						ultra.header.stamp = ros::Time::now();
						ultra.header.frame_id = "ultrasonic";
						ultra.ultrasonic1 = (recv_data[VALID_DATA_START_INDX + 0] * 256 + recv_data[VALID_DATA_START_INDX + 1]) & 0xffff;
						ultra.ultrasonic2 = (recv_data[VALID_DATA_START_INDX + 2] * 256 + recv_data[VALID_DATA_START_INDX + 3]) & 0xffff;
						ultra.ultrasonic3 = (recv_data[VALID_DATA_START_INDX + 4] * 256 + recv_data[VALID_DATA_START_INDX + 5]) & 0xffff;
						ultra.ultrasonic4 = (recv_data[VALID_DATA_START_INDX + 6] * 256 + recv_data[VALID_DATA_START_INDX + 7]) & 0xffff;
						ultra.ultrasonic5 = (recv_data[VALID_DATA_START_INDX + 8] * 256 + recv_data[VALID_DATA_START_INDX + 9]) & 0xffff;
						ultra.ultrasonic6 = (recv_data[VALID_DATA_START_INDX + 10] * 256 + recv_data[VALID_DATA_START_INDX + 11]) & 0xffff;
						ultra.ultrasonic7 = (recv_data[VALID_DATA_START_INDX + 12] * 256 + recv_data[VALID_DATA_START_INDX + 13]) & 0xffff;
						ultra.ultrasonic8 = (recv_data[VALID_DATA_START_INDX + 14] * 256 + recv_data[VALID_DATA_START_INDX + 15]) & 0xffff;						
						ultrasonic_pub.publish(ultra);
						AIV_Driver::req_ultra_start_finish = false;
						//cout<<"request_ultrasonic cmd is executed successfully !"<<endl;
					}
					else if(recv_data[CMD_CTRL_INDX] == FRAME_CMD_STOP)
					{
						AIV_Driver::req_ultra_stop_finish = false;
					}
				}
	//		}
	//		else
	//		{
	//			cout<<"ROS does not send request_ultra but recv a response cmd"<<endl;
	//			return;
	//		}

			break;
			
		case REQ_BUMPER:
			if((AIV_Driver::req_ultra_start_finish == true) || (AIV_Driver::req_ultra_stop_finish == true))
			{	
				if(recv_data[VALID_DATA_LEN_INDX] != 0x01)
				{
					cout<<"The data count byte of the respones of the request_collision cmd shoud be 0x01,but returned is: "<<recv_data[VALID_DATA_LEN_INDX]<<endl;
					return;
				}
				else
				{
					if(recv_data[CMD_CTRL_INDX] == FRAME_CMD_START)
					{
						colibri_aiv::Bumper bumper;
						bumper.header.stamp = ros::Time::now();
						bumper.header.frame_id = "bumper";
						if(recv_data[VALID_DATA_START_INDX + 0] == 0x55)
						{
							bumper.bumper.data = false;
						}
						else if (recv_data[VALID_DATA_START_INDX + 0] == 0xff)
						{
							bumper.bumper.data = true;
						}
						else
						{
							cout<<"request_bumper cmd recv a illegal data !"<<endl;
							return;
						}
						bumper_pub.publish(bumper);
						AIV_Driver::req_bumper_start_finish = false;
						cout<<"request_bumper cmd is executed successfully !"<<endl;
					}
					else if(recv_data[CMD_CTRL_INDX] == FRAME_CMD_STOP)
					{
						AIV_Driver::req_bumper_stop_finish = false;
					}
				}
			}
			else
			{
				cout<<"ROS does not send request_bumper but recv a response cmd"<<endl;
				return;
			}

			break;

		case REQ_VELOCITY:
			
				if(recv_data[VALID_DATA_LEN_INDX] != 0x06)
				{
					cout<<"The recved valid data lenth of respones for the request_vel shoud be 0x06,but returned is: "<<recv_data[VALID_DATA_LEN_INDX]<<endl;
					return;
				}
				
				if(recv_data[CMD_CTRL_INDX] == FRAME_CMD_START)
				{

					ParseWheelRpm(&recv_data[VALID_DATA_START_INDX]);

					//cout<<"left_rot_rate:"<<left_rot_rate<<"   right_rot_rate:"<<right_rot_rate<<endl;

					current_time = ros::Time::now();
					time_period = (current_time - last_time).toSec();
					last_time = current_time;

					left_cur_vel = (2 * PI * WHEEL_RADIUS * left_rot_rate)/(60 * WHEEL_GEAR);  //left speed:m/s
					right_cur_vel = (2 * PI * WHEEL_RADIUS * right_rot_rate)/(60 * WHEEL_GEAR); //right speed:m/s
					
					left_avg_vel = (left_cur_vel + left_last_vel) / 2.0;
					right_avg_vel = (right_cur_vel + right_last_vel) / 2.0;
					left_last_vel = left_avg_vel;
					right_last_vel = right_avg_vel;

					left_avg_distance = left_avg_vel * time_period;
					right_avg_distance = right_avg_vel * time_period;

					aiv_dx = (left_avg_distance + right_avg_distance) / 2.0;
					aiv_dth = (right_avg_distance - left_avg_distance) / WHEEL_TRACK;
					aiv_vx = (left_avg_vel + right_avg_vel) / 2.0;
					//aiv_vth = aiv_dth / time_period;

					geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(cartodom_yaw);
					
					geometry_msgs::TransformStamped odom_trans;
					odom_trans.header.stamp = current_time;
					odom_trans.header.frame_id = "odom";
					odom_trans.child_frame_id = "base_footprint";					

					odom_trans.transform.translation.x = cartodom_x;	// this tf value from the cartodom
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
					
					//set the vel
					odom.child_frame_id = "base_footprint";
					odom.twist.twist.linear.x = aiv_vx;		//in topic /odom ,the aiv total v from encoder
					odom.twist.twist.linear.y = 0;
					odom.twist.twist.angular.z = cartodom_vth;
					
					//publish the message
					odom_pub.publish(odom);
					
					ROS_INFO("virtual x/y/yaw: %0.3lfm %0.3lfm %0.2lfdeg", cartodom_x, cartodom_y, cartodom_yaw * RAD2DEG);
					ROS_INFO("virtual vx/vth: %0.3lfm/s  %0.2lfrad/s",aiv_vx,cartodom_vth);

					AIV_Driver::req_vel_start_finish = false; 
				}
				else if(recv_data[CMD_CTRL_INDX] == FRAME_CMD_STOP)
				{
					AIV_Driver::req_vel_stop_finish = false;
				} 
			
			break;
			
		default :

			cout<<"received unknown cmd_byte:"<<recv_data[CMD_BYTE_INDX]<<endl;
			break;
			
	}
		
	return;
}

void AIV_Driver::ReadFromCom(void *args)
{
	while(ros::ok())
	{
		async_read(pserialport,buffer(recv_cache,CONST_PROTOCOL_LEN),boost::bind(&AIV_Driver::ReadInfoProc,this,recv_cache,_1,_2));

		ComCallHandle();	
		recv_cnt ++;
		//cout <<"recv times: "<<recv_cnt<<endl;
	}

}

void AIV_Driver::ComCallHandle()
{
//	AIV_ios.poll();
	AIV_ios.run();
	AIV_ios.reset();
}

void AIV_Driver::CreateThread(void *(*start_routine) (void *))
{	
	int ret = pthread_create(&thread_id,NULL,start_routine,NULL);
	if( ret )
	{
	   cout << "pthread_create error,error_code: "<< ret <<endl;
	   return;
	}
}

bool AIV_Driver::InitSubandPub()
{

	ros::NodeHandle global_nh;	
	twist_sub= global_nh.subscribe<geometry_msgs::Twist>("t_cmd_vel", 1, boost::bind(&AIV_Driver::TwistCallback, this, _1));

	ros::NodeHandle nh_cartodom;
	cartodom_sub= global_nh.subscribe<cartodom::Cartodom>("cartodom", 1, boost::bind(&AIV_Driver::CartodomCallback, this, _1));
	
	ros::NodeHandle nh_odom;
	odom_pub = nh_odom.advertise<nav_msgs::Odometry>("odom", 50);

	ros::NodeHandle nh_ultrasonic;
	ultrasonic_pub = nh_ultrasonic.advertise<colibri_aiv::Ultrasonic>("ultrasonic", 50);
	
	ros::NodeHandle nh_bumper;
	bumper_pub = nh_bumper.advertise<colibri_aiv::Bumper>("bumper", 50);
	
}

void AIV_Driver::TwistCallback(const geometry_msgs::Twist::ConstPtr & twist)
{

	if(twist->linear.x < 0 )
	{
		send_twist[VALID_DATA_START_INDX + 0] = NEG_SIGN;
		send_twist[VALID_DATA_START_INDX + 1] = abs(100*twist->linear.x);
	}
	else
	{
		send_twist[VALID_DATA_START_INDX + 0] = POS_SIGN;
		send_twist[VALID_DATA_START_INDX + 1] = 100*twist->linear.x;
	}

	if(twist->angular.z < 0 )
	{
		send_twist[VALID_DATA_START_INDX + 2] = NEG_SIGN;
		send_twist[VALID_DATA_START_INDX + 3] = abs(100*twist->angular.z);
	}
	else
	{
		send_twist[VALID_DATA_START_INDX + 2] = POS_SIGN;
		send_twist[VALID_DATA_START_INDX + 3] = 100*twist->angular.z;
	}

	send_cache = send_twist;
	SendCmd(send_twist, send_twist_finish);
	
	send_cnt++;
	cout <<"send times: "<<send_cnt<<endl;
}

void AIV_Driver::CartodomCallback(const cartodom::Cartodom::ConstPtr & carto_odom)
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

void AIV_Driver::SendCmd(const unsigned char *cmd ,volatile bool &send_flag)
{
	WriteToCom(cmd);
	send_flag = true;
	ros::Time start_time = ros::Time::now();

	while(send_flag == true)
	{
		if(ros::Time::now() - start_time > ros::Duration(TIMEOUT, 0))
		{
			switch(cmd[CMD_BYTE_INDX])
			{
				case ENABLE_DISABLE_MOTOR:
					if(cmd[VALID_DATA_START_INDX + 0] == 0xff)
					{
						cout<<"TRIO respones the enable_motor cmd timeout !"<<endl;
					}
					else if(cmd[VALID_DATA_START_INDX + 0] == 0x55)
					{
						cout<<"TRIO respones the disable_motor cmd timeout !"<<endl;	
					}
					break;
					
				case SEND_TWIST:
					cout<<"TRIO respones the send_twist cmd timeout !"<<endl;
					break;
					
				case REQ_ENCODER:
					cout<<"TRIO respones the request_encoder cmd timeout !"<<endl;
					break;

				case REQ_IMU:
					cout<<"TRIO respones the request_imu cmd timeout !"<<endl;
					break;

				case REQ_ULTRASONIC:
					cout<<"TRIO respones the request_ultrasonic cmd timeout !"<<endl;
					break;

				case REQ_BUMPER:
					cout<<"TRIO respones the request_bumper cmd timeout !"<<endl;
					break;

				case REQ_VELOCITY:
					cout<<"TRIO respones the request_vel cmd timeout !"<<endl;
					break;

				default :
					break;
			}
			
			break;
		}
	 }

}

void AIV_Driver::DisplayFrame(unsigned char *cmd)
{
	int i;
	
	for(i = 0;i < CONST_PROTOCOL_LEN; i ++)
	{
		printf(" %x",cmd[i]);		
	}
	
	cout <<endl;
}


void AIV_Driver::ParseWheelRpm(const unsigned char *valid_data)
{
	if(*(valid_data  + 0)== 0x00)
	{
		left_rot_rate = (float)((*(valid_data + 1) << 8) | (*(valid_data + 2)));
	}
	else if(*(valid_data  + 0)== 0xff)
	{
		left_rot_rate =(-1.0)*(float)((*(valid_data + 1) << 8) | (*(valid_data + 2)));
	}
	else
	{
		cout<<"The [VALID_DATA_START_INDX + 0] of the request_vel cmd is illegal: data sign is not correct!"<<endl;
		return;
	}

	
	
	if(*(valid_data + 3)== 0x00)
	{
		right_rot_rate = (float)((*(valid_data + 4) << 8) | (*(valid_data + 5)));

	}
	else if(*(valid_data + 3)== 0xff)
	{
		right_rot_rate =(-1.0)*(float)((*(valid_data + 4) << 8) | (*(valid_data + 5)));

	}
	else
	{
		cout<<"The [VALID_DATA_START_INDX + 3] of the request_vel cmd is illegal: data sign is not correct!"<<endl;
		return;
	}

}


