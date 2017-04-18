#include "colibri_ca.h"

int main(int argc, char* argv[])
{	
	ros::init(argc, argv, "colibri_ca_node");
	scan_ca scan4caObj;

	ros::Rate loop_rate(10);

	ROS_INFO("Start to avoid colision for local scope ... ");

	float v_0 = 0.0;
	int range_num = 0;

	float tmp_theta_obs = 0.0;	//obstacle 's theta angle
	float tmp_passfcn_value = 0.0;

	ofstream  file1,file2,file3; 
	file1.open ("cppscan4ca_vector.txt"); 
	file2.open ("cppkrf_vector.txt"); 
	file3.open ("cpppassfcn_vector.txt"); 
	int delay_cnt = 0;
	
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
			ROS_INFO("start ...");
			for(int i = 1; i <= NUM_RAY4CA; i++)
			{
				scan4caObj.delta_phi_vec[i-1] = asin(D_SF / (*(scan4caObj.ptrScan4ca + i - 1))) * RAD2DEG;
				scan4caObj.kp_phi_vec[i-1] = scan4caObj.CalcKpPhi(v_0, *(scan4caObj.ptrScan4ca + i - 1));
				range_num = floor(scan4caObj.delta_phi_vec[i-1] / RAY_RESOL4CA);
				
				file1 << fixed << setprecision(4) << *(scan4caObj.ptrScan4ca + i - 1);
				file1 << '\t';	
				
				scan4caObj.CalcPhiRange(i,range_num,&scan4caObj.phi_start_vec[i-1],&scan4caObj.phi_end_vec[i-1]);

				tmp_theta_obs = i - 1.0; // unit in degree
				scan4caObj.kaf_vec[i-1] = cos((tmp_theta_obs - scan4caObj.goal_dir) * DEG2RAD);
				scan4caObj.krf_vec[i-1] = scan4caObj.kp_phi_vec[i-1];

			}


			file1.close();
		
			scan4caObj.CalcKrfTheta(scan4caObj.kp_phi_vec, scan4caObj.phi_start_vec, scan4caObj.phi_end_vec);
	
			scan4caObj.CalcPassFcnAndFwdBnd(scan4caObj.wander, &scan4caObj.max_passfcn_val , scan4caObj.passfcn_vec);
			scan4caObj.CalcPassFcnAndBwdBnd(scan4caObj.wander, &scan4caObj.max_passfcn_val , scan4caObj.passfcn_vec);


			for(int j = 1; j <= NUM_RAY4CA; j++)
			{
				file2<< fixed << setprecision(4) << scan4caObj.krf_vec[j-1];
				file2 << '\t';
				file3<< fixed << setprecision(4) << scan4caObj.passfcn_vec[j-1];
				file3 << '\t';
			}


			scan4caObj.angle_adj = scan4caObj.CalcAdjDir(scan4caObj.passfcn_vec,scan4caObj.max_passfcn_val, &scan4caObj.maxfcn_fwdbnd,&scan4caObj.maxfcn_bwdbnd);
				
			file2.close();
			file3.close();

			if(scan4caObj.max_passfcn_val < PASSFCN_THD_RATIO * D_M)
			{
				scan4caObj.colision_alarm = 1;
			}
			else
			{
				scan4caObj.colision_alarm = 0;

			}
			cout<<"fwd_maxpass_num: "<<scan4caObj.fwd_maxpass_num<<endl;
			cout<<"bwd_maxpass_num: "<<scan4caObj.bwd_maxpass_num<<endl;
			
			cout<<"passfcn_max_fwdbound: "<< scan4caObj.maxfcn_fwdbnd <<endl;
			cout<<"passfcn_max_bwdbound: "<< scan4caObj.maxfcn_bwdbnd <<endl;
			cout<<"adj_angle: "<< scan4caObj.angle_adj <<endl;
			cout<<"passfcn_max_value: "<< scan4caObj.max_passfcn_val <<endl;
			cout<<"colision_alarm: "<< scan4caObj.colision_alarm <<endl;

			scan4caObj.fwd_maxpass_num = 0;
			scan4caObj.bwd_maxpass_num = 0;
			
			ros::spinOnce();
			loop_rate.sleep();
			ROS_INFO("end ...");
		}
	}

	return 0;	
}



