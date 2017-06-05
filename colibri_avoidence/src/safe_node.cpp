#include "protect.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Anti_collision_node");
	
	protector protectObj;
	
	ros::Rate loop_rate(15);
	
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
			
			cout<<"protectObj.min_scan: "<< protectObj.min_scan << endl;
			cout<<"protectObj.min_index_scan: "<< protectObj.min_index_scan << endl;
			cout<<"protectObj.laser_prob: "<< protectObj.laser_unsafe_prob<< endl;
			
			for(int j = 1; j <= SCAN4SAFE_NUM; j++)
			{
				file1<< fixed << setprecision(4) << protectObj.scan4safty[j];
				file1 << '\t';
			}

			file1.close();
			coli_prob = protectObj.IntegrateMultiInfo4Safety(&protectObj.advise_action);
			cout<<"coli_prob: "<< coli_prob<< endl;

			protectObj.Intg4EnvSecure();	
			protectObj.security_pub4env.publish(protectObj.env_secure);

			ros::spinOnce();
			loop_rate.sleep();
			ROS_INFO("end ...");
		}
	

	}

	return 0;
}

