#include "protect.h"

protector::protector()
{
	memset(scan4safty, 1.0, SCAN4SAFE_NUM);
	memset(ultra_vec, 1.0, ULTRA_NUM);
	bumper_signal = false;
	
	v = 0.0;
	vth = 0.0;
	advise_action = HOLD_STATE;

	collision_flag = false;
	colision_prob = 0.0;
	
	laser_unsafe_prob = 0.0;
	ultra_unsafe_prob = 0.0;
	
	min_scan = 1.0;
	min_ultra = 1.0;

	min_index_scan = 0;
	min_index_ultra = 0;	
	
	scan_sub4safe = nh_safety.subscribe<sensor_msgs::LaserScan>("/scan", 1, &protector::ScanSafeCallBack, this);
	ultra_sub4safe = nh_safety.subscribe<colibri_aiv::ultrasonic>("/ultrasonic", 1, &protector::UltraSafeCallBack, this);
	bumper_sub4safe = nh_safety.subscribe<colibri_aiv::collision>("/collision", 1, &protector::BumperSafeCallBack, this);
	Odom_sub4safe = nh_safety.subscribe<nav_msgs::Odometry>("/odom", 1, &protector::OdomSafeCallBack, this);

}
protector::~protector()
{

}

float protector::CalcMinDis4LaserScan(float* laser_vec)
{
	float tmp_range = *laser_vec;
	unsigned int index_mindis = 0;
	
	for(int i = 0; i < SCAN4SAFE_NUM; i++)
	{
		if(*(laser_vec + i) <= tmp_range)
		{
			tmp_range = *(laser_vec + i);
			index_mindis = i;
		}
	}
	
	min_scan = tmp_range;
	min_index_scan = index_mindis;

	if(tmp_range <= LASER_SAFE_MIN)
	{
		laser_unsafe_prob = 1.0;
	}
	else if(tmp_range <= LASER_SAFE_MAX)
	{
		laser_unsafe_prob = (LASER_SAFE_MAX - tmp_range) / (LASER_SAFE_MAX - LASER_SAFE_MIN);
	}
	else
	{
		laser_unsafe_prob = 0.0;
	}

	return laser_unsafe_prob;
}

float protector::CalcMinDis4Ultrosonic(float* ultra_vec)
{
	float tmp_range = *ultra_vec;
	unsigned int index_mindis = 0;
	
	for(int i = 0; i < ULTRA_NUM; i++)
	{
		if(*(ultra_vec + i) <= tmp_range)
		{
			tmp_range = *(ultra_vec + i);
			index_mindis = i;
		}
	}
	
	min_ultra = tmp_range;
	min_index_ultra = index_mindis;

	if(tmp_range <= ULTRA_SAFE_MIN)
	{
		laser_unsafe_prob = (ULTRA_SAFE_MIN - tmp_range) / ULTRA_SAFE_MIN;
	}
	else
	{
		ultra_unsafe_prob = 0.0;
	}

	return ultra_unsafe_prob;

}

float protector::IntegrateMultiInfo4Safety(enum_act4safe* advise_action)
{

	enum_bearing obs_dir = OMNI;
	
	if(bumper_signal == true || ultra_unsafe_prob > LEVEL_1_PROB || laser_unsafe_prob > LEVEL_1_PROB)
	{
		collision_flag = 1;
		colision_prob = 1.0;
		*advise_action = STOP;
	}
	else
	{
		if(bumper_signal == false)
		{
			colision_prob = MAX(laser_unsafe_prob, ultra_unsafe_prob);
			if(colision_prob == laser_unsafe_prob)
			{
				if(min_index_scan <= (SCAN4SAFE_NUM - 1)/2)
				{
					obs_dir = RIGHT;
				}
				else
				{
					obs_dir = LEFT;
				}			
			}
			else
			{
				if(min_index_scan <= ULTRA_NUM / 2)
				{
					obs_dir = RIGHT;
				}
				else
				{
					obs_dir = LEFT;
				}	
			}
		}
		
		if(colision_prob <= LEVEL_3_PROB)
		{
			*advise_action = HOLD_STATE;
		}
		else if(colision_prob <= LEVEL_2_PROB)
		{
			*advise_action = DEC_VEL;
		}
		else
		{
			if(obs_dir == RIGHT)
			{
				*advise_action = TURN_LEFT;
			}
			else if(obs_dir == LEFT)
			{
				*advise_action = TURN_RIGHT;
			}
			else
			{
				*advise_action = STOP;
			}			
		}

	}

	return colision_prob;
}

bool protector::StopMovingInForce(bool collision_flag, float colision_prob)
{
	if(collision_flag == true || colision_prob >= UNSAFE_PROB)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}

bool protector::Detect4ExceptHighVel(float* v, float* vth)
{
	if(*v >= V_EXCPT_VAL || *vth >= VTH_EXCPT_VAL)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void protector::ScanSafeCallBack(const sensor_msgs::LaserScan::ConstPtr& scan4safe)
{
	int j = 31;	//from laser ray at  -15deg starts : 31 = 15/0.5+1;
	for(int i = 0; i < SCAN4SAFE_NUM; i++)
	{
		scan4safty[i] = scan4safe->ranges[j];
		
		if(scan4safty[i] <= 0.08)
		{
			scan4safty[i] = (0.2*scan4safe->ranges[j-4] + 0.3*scan4safe->ranges[j-1] + 0.3*scan4safe->ranges[j+1] + 0.2*scan4safe->ranges[j+4]) ;
		}
		
		j = j + 2; 
	}

}

void protector::UltraSafeCallBack(const colibri_aiv::ultrasonic::ConstPtr& ultra4safe)
{
	ultra_vec[0] = ultra4safe->ultrasonic1;
	ultra_vec[1] = ultra4safe->ultrasonic2;
	ultra_vec[2] = ultra4safe->ultrasonic3;
	ultra_vec[3] = ultra4safe->ultrasonic4;
}

void protector::BumperSafeCallBack(const colibri_aiv::collision::ConstPtr& bumper4safe)
{	
	bumper_signal = bumper4safe->collision.data;	
}

void protector::OdomSafeCallBack(const nav_msgs::Odometry::ConstPtr& odom4safe)
{
	v = odom4safe->twist.twist.linear.x;
	vth = odom4safe->twist.twist.angular.z;	
}



