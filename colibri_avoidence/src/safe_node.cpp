#include "protect.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Anti_collision_node");
	
	protector protectObj;
	
	ros::Rate loop_rate(10);
	
	int delay_cnt = 0;
	float coli_prob = 0.0;
	
	ofstream  file1; 
	file1.open ("laser.txt"); 

	ROS_INFO("Start to detect the safety...");
	
	while (ros::ok())
	{	
		if(delay_cnt < DELAY_CNT_MAX)
		{
			delay_cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		if(delay_cnt >= DELAY_CNT_MAX)
		{
			protectObj.CalcMinDis4LaserScan(protectObj.scan4safty);
			protectObj.CalcMinDis4Ultrosonic(protectObj.ultra_vec);
			
			cout<<"protectObj.min_scan: "<< protectObj.min_scan << endl;
			cout<<"protectObj.min_index_scan: "<< protectObj.min_index_scan<< endl;
			cout<<"protectObj.min_scan_angle: "<< protectObj.min_scan_angle<< endl;
			cout<<"protectObj.laser_prob: "<< protectObj.laser_unsafe_prob<< endl;
			
			for(int j = 1; j <= SCAN4SAFE_NUM; j++)
			{
				file1<< fixed << setprecision(4) << protectObj.scan4safty[j];
				file1 << '\t';
			}

			file1.close();
			coli_prob = protectObj.IntegrateMultiInfo4Safety(&protectObj.advise_action);
			cout<<"coli_prob: "<< coli_prob<< endl;

			float linear_vel_safe = 0.0;
			float angular_vel_safe = 0.0;
			int steer = 0;
			protectObj.CalcLaserSafeVelThd(protectObj.min_scan , protectObj.min_scan_angle, steer, &linear_vel_safe, &angular_vel_safe);
			cout<<"linear_vel_safe: "<< linear_vel_safe<< endl;
			cout<<"angular_vel_safe: "<< angular_vel_safe<< endl;


			protectObj.Intg4EnvSecure();	
			protectObj.security_pub4env.publish(protectObj.env_secure);

			protectObj.safe_vel.header.stamp = ros::Time::now();
			protectObj.safe_vel.header.frame_id = "robot";
			
			protectObj.safe_vel.stop.data = true;
			protectObj.safe_vel.linear_safe_thd = linear_vel_safe;
			protectObj.safe_vel.linear_safe_vel = 0.0;
			protectObj.safe_vel.steer = steer;
			protectObj.safe_vel.angular_safe_thd = angular_vel_safe;
			protectObj.safe_vel.angular_safe_vel = 0.0;
			protectObj.security_pub4nav.publish(protectObj.safe_vel);


			ros::spinOnce();
			loop_rate.sleep();
			ROS_INFO("end ...");
		}
	

	}

	return 0;
}

