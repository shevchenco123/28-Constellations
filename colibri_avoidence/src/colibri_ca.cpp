#include "colibri_ca.h"

scan_ca::scan_ca()
{
	goal_dir = 60;	//goal direction
	memset(scan4ca, 0.2, NUM_RAY4CA);
	ptrScan4ca = scan4ca;
	
	memset(delta_phi_vec, 0, NUM_RAY4CA);
	memset(kp_phi_vec, 0, NUM_RAY4CA);

	memset(phi_start_vec, 1, NUM_RAY4CA);
	memset(phi_end_vec, 181, NUM_RAY4CA);

	memset(krf_vec, 0.1, NUM_RAY4CA);
	memset(kaf_vec, 0, NUM_RAY4CA);

	memset(passfcn_vec, 0, NUM_RAY4CA);
	max_passfcn_val = 0.0;

	fwd_maxpass_num = 0;
	bwd_maxpass_num = 0;

	maxfcn_fwdbnd = 0;
	maxfcn_bwdbnd = 0;

	v = 0.0;
	vth = 0.0;
	
	angle_adj = 0.0;
	vel_adj = 0.0;
	colision_alarm = 0;

	wander = 0;
	
	scan_sub4ca = nh_ca.subscribe<sensor_msgs::LaserScan>("/scan", 1, &scan_ca::ScanCallBack, this);
	
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
		if(scan4ca[i] < 0.1)
		{
			scan4ca[i] = (scan_ca->ranges[j-2]+scan_ca->ranges[j+2]+scan_ca->ranges[j-5]+scan_ca->ranges[j+5]) / 4 + LASER_EDGE_MIN;
		}
		
		j = j + 2; 
	
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
				if(krf_vec[theta_index] <= *(ptrKp_phi_vector + phi_index))
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
		
		for(int j = 0; j < NUM_RAY4CA; j++)
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

	for(int k = NUM_RAY4CA - 1; k >= 0; k--)
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

void scan_ca::CalcCollisionInAPF(void)
{
	if(max_passfcn_val <= PASSFCN_THD_RATIO * D_M)
	{
		colision_alarm = 1;
	}
	else
	{
		colision_alarm = 0;
	}	
}

float scan_ca::CalcAdjDir(float* ptrPassfcn_vector, float max_passfcn_val, int* fwd_bound, int* bwd_bound)
{
	int mid_num = 90;
	float adj_dir = 0.0;
	
	for(int m = 0; m <= mid_num; m++)  //backward directrion search using forward bound
	{
		if(abs(*(ptrPassfcn_vector + *(fwd_bound) - m) - max_passfcn_val) < MAX_PASSFCN_SCOPE)
		{
			fwd_maxpass_num++;
		}else
		{
			break;
		}
	}
	for(int n = 0; n <= mid_num; n++)	// forward direction search using backward bound
	{
		if(abs(*(ptrPassfcn_vector + *(bwd_bound) + n) - max_passfcn_val) < MAX_PASSFCN_SCOPE)
		{
			bwd_maxpass_num++;
		}else
		{
			break;
		}
	}

	
	if (fwd_maxpass_num <= bwd_maxpass_num)
	{
		adj_dir = *bwd_bound + bwd_maxpass_num / 2.0 - mid_num;
	}else
	{
		adj_dir = *fwd_bound - fwd_maxpass_num / 2.0 - mid_num;
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
		kp_phi = 10000;
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
	float tmp_theta_laseray = 0.0;	//obstacle 's theta angle

	for(int i = 0; i < NUM_RAY4CA; i++)
	{
		delta_phi_vec[i] = asin(D_SF / (*(ptrScan4ca + i))) * RAD2DEG;
		range_num = floor(delta_phi_vec[i] / RAY_RESOL4CA);
		CalcPhiRange(i, range_num, &phi_start_vec[i], &phi_end_vec[i]);	

		kp_phi_vec[i] = CalcKpPhi(vel_center, *(ptrScan4ca + i));		
		krf_vec[i] = kp_phi_vec[i];
	
		tmp_theta_laseray = i;			// unit in degree
		kaf_vec[i] = cos((tmp_theta_laseray - dir_goal_inlaser) * DEG2RAD);  //dir_goal_in_laser is 0~180 degree from right side
		
	}

}



