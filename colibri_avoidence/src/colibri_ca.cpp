#include "colibri_ca.h"

scan_ca::scan_ca()
{
	goal_dir = 60;	//goal direction
	memset(scan4ca, 0.2, NUM_RAY4CA);
	ptrScan4ca = scan4ca;

	memset(ultra4ca, 20.0, NUM_RAY4CA);
	memset(ultra_dis, 20.0, ULTRA4CA_NUM);
	
	memset(delta_phi_vec, 0, NUM_RAY4CA);
	memset(kp_phi_vec, 0, NUM_RAY4CA);

	memset(phi_start_vec, 1, NUM_RAY4CA);
	memset(phi_end_vec, 181, NUM_RAY4CA);

	memset(krf_vec, 1/D_M, NUM_RAY4CA);
	memset(kaf_vec, 0.0, NUM_RAY4CA);

	memset(passfcn_vec, 0, NUM_RAY4CA);
	max_passfcn_val = 0.0;

	fwd_maxpass_cnt = 0;
	bwd_maxpass_cnt = 0;

	maxfcn_fwdbnd = 0;
	maxfcn_bwdbnd = 0;

	v = 0.0;
	vth = 0.0;
	
	angle_adj = 0.0;
	vel_adj = 0.0;
	apf_alarm = 0;

	wander = 0;
	
	scan_sub4ca = nh_ca.subscribe<sensor_msgs::LaserScan>("/scan", 1, &scan_ca::ScanCallBack, this);
	apf_pub4mntr = nh_ca.advertise<colibri_msgs::AngPotnEngy>("/apf", 5);
	rf_pub4mntr = nh_ca.advertise<colibri_msgs::AngPotnEngy>("/rf", 5);
}

scan_ca::~scan_ca()
{
	
}

void scan_ca::ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_ca)
{
	int j = 61;
	for(int i = 0; i < NUM_RAY4CA; i++)
	{
		scan4ca[i] = scan_ca->ranges[j];
/*		if(scan4ca[i] < 0.1)
		{
			scan4ca[i] = (scan_ca->ranges[j-2]+scan_ca->ranges[j+2]+scan_ca->ranges[j-5]+scan_ca->ranges[j+5]) / 4 + LASER_EDGE_MIN;
		}
*/
		if(scan4ca[i] < 0.05)
		{
			scan4ca[i] = scan4ca[i-1];
		}
		j = j + 2; 
	
	}
	
}

void scan_ca::UltraSonicCallBack(const colibri_aiv::Ultrasonic::ConstPtr& ultra_ca)
{
		float tmp_min = 6.5;
		ultra_dis[0] = (ultra_ca->ultrasonic1) / 100.0; //for front ultra unit cm to m
		ultra_dis[1] = (ultra_ca->ultrasonic2) / 100.0;
		ultra_dis[2] = (ultra_ca->ultrasonic3) / 100.0;
		ultra_dis[3] = (ultra_ca->ultrasonic4) / 100.0;

		if(ultra_dis[0] < D_M)
		{
			memset(ultra4ca+ULTRA1_RIGHT_BND, ultra_dis[0], (ULTRA1_LEFT_BND-ULTRA1_RIGHT_BND));
		}
		else
		{
			memset(ultra4ca+ULTRA1_RIGHT_BND, 20.0, (ULTRA1_LEFT_BND-ULTRA1_RIGHT_BND));
		}

		if(ultra_dis[1] < D_M)
		{
			memset(ultra4ca+ULTRA2_RIGHT_BND, ultra_dis[0], (ULTRA2_LEFT_BND-ULTRA2_RIGHT_BND));
		}
		else
		{
			memset(ultra4ca+ULTRA2_RIGHT_BND, 20.0, (ULTRA2_LEFT_BND-ULTRA2_RIGHT_BND));
		}

		if(ultra_dis[1] < D_M && ultra_dis[2] < D_M && abs(ultra_dis[1] - ultra_dis[2]) < 0.5)
		{
			tmp_min = MIN(ultra_dis[1], ultra_dis[2]);
			memset(ultra4ca+ULTRA_RIGHT_BND, tmp_min, (ULTRA_LEFT_BND-ULTRA_RIGHT_BND));
		}
		else
		{
			memset(ultra4ca+ULTRA_RIGHT_BND, 20.0, (ULTRA_LEFT_BND-ULTRA_RIGHT_BND));
		}

		if(ultra_dis[2] < D_M)
		{
			memset(ultra4ca+ULTRA3_RIGHT_BND, ultra_dis[2],(ULTRA3_LEFT_BND-ULTRA3_RIGHT_BND));
		}
		else
		{
			memset(ultra4ca+ULTRA3_RIGHT_BND, 20.0, (ULTRA3_LEFT_BND-ULTRA3_RIGHT_BND));
		}

		if(ultra_dis[3] < D_M)
		{
			memset(ultra4ca+ULTRA4_RIGHT_BND, ultra_dis[3], (ULTRA4_LEFT_BND-ULTRA4_RIGHT_BND));
		}
		else
		{
			memset(ultra4ca+ULTRA4_RIGHT_BND, 20.0, (ULTRA4_LEFT_BND-ULTRA4_RIGHT_BND));
		}
		

}

void scan_ca::CalcKrfTheta(float* ptrKp_phi_vector, int* ptrPhi_range_start, int* ptrPhi_range_end)
{

	for(int theta_index = 0; theta_index < NUM_RAY4CA; theta_index++)
	{
		for(int phi_index = 0; phi_index < NUM_RAY4CA; phi_index++)
		{
			if((theta_index >= *(ptrPhi_range_start+phi_index)) && (theta_index <= *(ptrPhi_range_end+phi_index)))
			{
				if(krf_vec[theta_index] <= *(ptrKp_phi_vector + phi_index)) // to obtain the max [Krf(phi,theta)] as Krf(theta)
				{	
					krf_vec[theta_index] = *(ptrKp_phi_vector + phi_index);				
				}

			}

			
		}
	}
	
}

void scan_ca::CalcPhiRange(int i, int range_num, int* ptrPhi_start, int* ptrPhi_end)
{
	int index_start = 0;
	int index_end = 0;

	index_start = i - range_num;
	index_end = i + range_num;

	if(index_start < 1)
	{
		*ptrPhi_start = 1;
		*ptrPhi_end = index_end;
	}else if(index_end > NUM_RAY4CA)
	{
		*ptrPhi_start = index_start;
		*ptrPhi_end = NUM_RAY4CA;				
	}else
	{
		*ptrPhi_start = index_start;
		*ptrPhi_end = index_end;
	}

}

void scan_ca::CalcPassFcnAndFwdBnd(unsigned int flag, float* max_passfcn_val, float* ptrK_pg)
{
	float tmp_passfcn_value = 0.0;
	int tmp_bound_forward = 0;
		
		for(int j = 0; j < NUM_RAY4CA; j++) //from right to left search the max passfcn val  and the max passfcn's bound index should locate at left terminal
		{
			if(flag == 1)
			{
				*(ptrK_pg + j) = 1 / (krf_vec[j]);
			}
			else
			{
				*(ptrK_pg + j) = kaf_vec[j] / (krf_vec[j]);
			}
		
			if(tmp_passfcn_value <= *(ptrK_pg + j))
			{
				tmp_bound_forward = j;
				tmp_passfcn_value = *(ptrK_pg + j);
			}	
		}
		
		maxfcn_fwdbnd = tmp_bound_forward;
		*max_passfcn_val = tmp_passfcn_value;

}

void scan_ca::CalcPassFcnAndBwdBnd(unsigned int flag, float* max_passfcn_val, float* ptrK_pg)
{
	float tmp_passfcn_value = 0.0;
	int tmp_bound_backward = 0;

	for(int k = NUM_RAY4CA - 1; k >= 0; k--) //from left to right search the max passfcn val  and the max passfcn's bound index should locate at right terminal
	{
		if(flag == 1)
		{
			*(ptrK_pg + k) = 1 / (krf_vec[k]);
		}
		else
		{
			*(ptrK_pg + k) = kaf_vec[k] / (krf_vec[k]);
		}		
		if(tmp_passfcn_value <= *(ptrK_pg + k))
		{
			tmp_bound_backward = k;
			tmp_passfcn_value = *(ptrK_pg + k);
		}	
	}
		
	maxfcn_bwdbnd = tmp_bound_backward;
	*max_passfcn_val = tmp_passfcn_value;

}

void scan_ca::CalcAlarmInAPF(void)
{
	if(max_passfcn_val <= PASSFCN_THD_RATIO * D_M)
	{
		apf_alarm = 1;
	}
	else
	{
		apf_alarm = 0;
	}	
}

float scan_ca::CalcAdjDir(float* ptrPassfcn_vector, float max_passfcn_val, int* fwd_bound, int* bwd_bound)
{
	int mid_num = 90;
	float adj_dir = 0.0;

	float tmp_scope = MIN(PASSVAL_TOLLERENCE * max_passfcn_val, MAX_PASSFCN_SCOPE);
	
	for(int m = 0; m <= mid_num; m++)  //backward directrion(CW 180->0) search using forward bound
	{
		if(abs(*(ptrPassfcn_vector + *(fwd_bound) - m) - max_passfcn_val) < tmp_scope)
		{
			fwd_maxpass_cnt++;
		}else
		{
			break;
		}
	}
	for(int n = 0; n <= mid_num; n++)	// forward direction(CCW 0->180) search using backward bound
	{
		if(abs(*(ptrPassfcn_vector + *(bwd_bound) + n) - max_passfcn_val) < tmp_scope)
		{
			bwd_maxpass_cnt++;
		}else
		{
			break;
		}
	}

	
	if (fwd_maxpass_cnt <= bwd_maxpass_cnt)
	{
		adj_dir = *bwd_bound + bwd_maxpass_cnt / 2.0 - mid_num;
	}else
	{
		adj_dir = *fwd_bound - fwd_maxpass_cnt / 2.0 - mid_num;
	}

	return adj_dir;

}
		
float scan_ca::CalcDsrVc(float vel_center)
{
	float d_sr_v = 0.0;
	
	d_sr_v = -0.5 * K_SR * vel_center * vel_center / ACC_DEC;

	return d_sr_v;
}

float scan_ca::CalcKpPhi(float vel_center, float d_phi)
{
	float tmpDsr = 0.0;
	float kp_phi = 0.0;
	tmpDsr = CalcDsrVc(vel_center);

	if(d_phi <= tmpDsr)
	{
		kp_phi = KP_PHI_INF;
	}
	else if((d_phi <= D_M)&&(tmpDsr < d_phi))
	{
		kp_phi = 1 / (d_phi - tmpDsr);
	}
	else if(d_phi > D_M)
	{
		kp_phi = 1 / (D_M - tmpDsr);
	}
	else
	{
		cout<<"Calc Kp_phi exception in CalcKpPhi Function ! : " <<endl;
		cout<< "tmpDsr : "<< tmpDsr <<endl;
		cout<< "d_phi : "<< d_phi <<endl;
	}
	
	return kp_phi;
}

void scan_ca::ResetMaxPassValCnt(void)
{
	fwd_maxpass_cnt = 0;
	bwd_maxpass_cnt = 0;
}

void scan_ca::LimitAngle(float & delta_ang)
{
	if(delta_ang >= 90) // for calc attract field , the delta angle should be -90~90 deg to ignore neg val
	{
		delta_ang = 90;
	}
	else if (delta_ang <= -90)
	{
		delta_ang = -90;
	}
	else
	{

	}
}

void scan_ca::CalcCorrectedKrf(void)
{
	float corrector;
	
	for(int i = 0; i < NUM_RAY4CA; i++)
	{
		corrector = CalcKrfCorrectFactor(i);
		krf_vec[i] = corrector * krf_vec[i];
	}	
}

float scan_ca::CalcKrfCorrectFactor(int index)
{
	//to make the scan front circle krf_vec factor as : y = 1+k * pow(x,2)
	float factor = 1.0;
	float tmp_puv = (90 - index) / 90.0; 	// index : 0~180
	
	factor = 1.0 + KRF_CORRECTOR * pow(tmp_puv, 2);

	return factor;
}

void scan_ca::CalcPhiParam(float vel_center, float& dir_goal_inlaser)
{

	int range_num = 0;
	//float tmp_theta_laseray = 0.0;	//obstacle 's theta angle
	float tmp_delta = 0.0;

	for(int i = 0; i < NUM_RAY4CA; i++)
	{
		delta_phi_vec[i] = asin(D_SF / (*(ptrScan4ca + i))) * RAD2DEG;
		range_num = floor(delta_phi_vec[i] / RAY_RESOL4CA);
		CalcPhiRange(i, range_num, &phi_start_vec[i], &phi_end_vec[i]);	

		kp_phi_vec[i] = CalcKpPhi(vel_center, *(ptrScan4ca + i));		
		krf_vec[i] = kp_phi_vec[i];
	
		tmp_delta = i - dir_goal_inlaser;			// unit in degree
		LimitAngle(tmp_delta);
		kaf_vec[i] = cos(tmp_delta * DEG2RAD);  //dir_goal_in_laser is 0~180 degree from right side
		
	}

}




