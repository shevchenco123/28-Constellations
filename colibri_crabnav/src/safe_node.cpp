#include "protect.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "SafeNode");
		
	int delay_cnt = 0;
	int CNT = 5;

	float laser_linear_vel_safe = 0.0;
	float laser_angular_vel_safe = 0.0;
	int laser_steer = 0;
	int laser_rect_encoder = 0;
	int laser_area_sts = 0;
	
	int ultra_area_sts = 0;
	float ultra_linear_vel_safe = 0.0;
	float ultra_angular_vel_safe = 0.0;
	int ultra_steer = 0;
	float coli_prob = 0.0;

	//ofstream  file1; 
	//file1.open ("ultra.txt");
	//int rec_max = 1800;
	//int cnt = 0;
	protector protectObj;		// init laser ultra and bumper anti collision obj
	protectObj.InitRectPolarVec();	// init the 3 rectangle area for laser obstacle encoder

	ros::Rate loop_rate(10);
	ROS_INFO("Start to detect the safety...");

	range_finder laser_property, ultra_property;
	safe_state laser_safe, ultra_safe;

	while (ros::ok())
	{	

		if(delay_cnt < CNT)
		{
			delay_cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		if(delay_cnt >= CNT)
		{
			protectObj.CalcMinDis4LaserScan();	
			cout<<"protectObj.min_scan: "<< protectObj.min_scan << endl;
			cout<<"protectObj.min_scan_angle: "<< protectObj.min_scan_angle<< endl;
			cout<<"protectObj.laser_prob: "<< protectObj.laser_unsafe_prob<< endl;
 			laser_property.min_dis =  protectObj.min_scan;
			laser_property.min_index =  protectObj.min_scan_angle;

			
			protectObj.CalcMinDis4Ultrosonic(protectObj.ultra_vec);
			cout<<"protectObj.min_ultra: "<< protectObj.min_ultra<< endl;
			cout<<"protectObj.min_index_ultra: "<< protectObj.min_index_ultra<< endl;
			cout<<"protectObj.ultra_unsafe_prob: "<< protectObj.ultra_unsafe_prob<< endl;
 			ultra_property.min_dis =  protectObj.min_ultra;
			ultra_property.min_index =  protectObj.min_index_ultra;
			
			coli_prob = protectObj.IntegrateMultiInfo4Safety(&protectObj.advise_action);
			cout<<"Intg coli_prob: "<< coli_prob<< endl;

			protectObj.Intg4EnvSecure();	
			protectObj.security_pub4env.publish(protectObj.env_secure);

			laser_rect_encoder = protectObj.LaserRectEncoder();
			cout<<"laser_rect_encoder: "<<laser_rect_encoder<<endl;

			protectObj.CalcCrabLaserCA(laser_rect_encoder,laser_property, laser_safe);
			//protectObj.CalcLaserCA(protectObj.min_scan , protectObj.min_scan_angle, laser_steer, &laser_linear_vel_safe, &laser_angular_vel_safe,laser_area_sts);
			cout<<"laser_linear_vel_safe_thd: "<< laser_safe.linear_up_vel<< endl;
			cout<<"laser_angular_vel_safe_thd: "<< laser_safe.angular_up_vel<< endl;
			cout<<"laser_steer_thd: "<< laser_safe.steer<< endl;
			cout<<"laser_area_sts: "<< laser_safe.area_state<< endl;


			protectObj.CalcCrabUltraCA(ultra_property, ultra_safe);
			//protectObj.CalcUltraCA(protectObj.min_ultra, protectObj.min_index_ultra, ultra_steer, &ultra_linear_vel_safe, &ultra_angular_vel_safe,ultra_area_sts);
			cout<<"ultra_linear_vel_safe_thd: "<< ultra_safe.linear_up_vel<< endl;
			cout<<"ultra_angular_vel_safe_thd: "<< ultra_safe.angular_up_vel<< endl;
			cout<<"ultra_steer_thd: "<< ultra_safe.steer<< endl;
			cout<<"ultra_area_sts: "<< ultra_safe.area_state<< endl;
			


/*
			file1 << fixed << setprecision(4) << protectObj.ultra_vec[0];
			file1 << '\t';
			file1 << fixed << setprecision(4) << protectObj.ultra_vec[1];
			file1 << '\t';	
			file1 << fixed << setprecision(4) << protectObj.ultra_vec[2];
			file1 << '\t';
			file1 << fixed << setprecision(4) << protectObj.ultra_vec[3];
			file1 << '\n';

			cnt ++;
			if(cnt == rec_max)
			{
				file1.close();	//record laser dis completed
			}
*/
			// publish the laser module safe vel
			protectObj.laser_safe_vel.header.stamp = ros::Time::now();
			protectObj.laser_safe_vel.header.frame_id = "laser";

			if((laser_linear_vel_safe == LINEAR_STOP) && (laser_angular_vel_safe == ANGULAR_STOP) && (laser_steer == 0))
			{
				protectObj.laser_safe_vel.stop.data = true;
			}
			else
			{
				protectObj.laser_safe_vel.stop.data = false;
			}				
			protectObj.laser_safe_vel.linear_safe_thd = laser_linear_vel_safe;
			protectObj.laser_safe_vel.area_status = laser_area_sts;
			protectObj.laser_safe_vel.steer = 0;
			protectObj.laser_safe_vel.angular_safe_thd = laser_angular_vel_safe;
			protectObj.laser_safe_vel.rsvd = laser_rect_encoder;
			
			protectObj.security_pub4laser.publish(protectObj.laser_safe_vel);

			// publish the ultra module safe vel
			protectObj.ultra_safe_vel.header.stamp = ros::Time::now();
			protectObj.ultra_safe_vel.header.frame_id = "ultra";

			if((ultra_linear_vel_safe == LINEAR_STOP) && (ultra_angular_vel_safe == ANGULAR_STOP) && (ultra_steer == 0))
			{
				protectObj.ultra_safe_vel.stop.data = true;
			}
			else
			{
				protectObj.ultra_safe_vel.stop.data = false;
			}				
			protectObj.ultra_safe_vel.linear_safe_thd = ultra_linear_vel_safe;
			protectObj.ultra_safe_vel.area_status = ultra_area_sts;
			protectObj.ultra_safe_vel.steer = ultra_steer;
			protectObj.ultra_safe_vel.angular_safe_thd = ultra_angular_vel_safe;
			protectObj.ultra_safe_vel.rsvd = 0.0;
			
			protectObj.security_pub4ultra.publish(protectObj.ultra_safe_vel);

			ros::spinOnce();
			loop_rate.sleep();
			ROS_INFO("end ...");
		}
	

	}

	return 0;
}

