#include "colibri_local_nav.h"

local_nav::local_nav()
{	
		//memset(goal_state, 0, POS_DIM);
		goal_state[0] = -2.55;
		goal_state[1] = -2.45;
		goal_state[2] = 90.0;
			
		memset(robot_init_state, 0, POS_DIM);
		memset(cur_robot_state, 0, POS_DIM);
		memset(last_robot_state, 0, POS_DIM);

		memset(amcl_cur_state, 0, POS_DIM);//store the linear vel and angular vel

		memset(cur_robot_vel, 0, VEL_DIM);//store the linear vel and angular vel
		memset(last_robot_vel, 0, VEL_DIM);


		
		memset(apf_ctrl_output, 0, VEL_DIM);

		apf_cmd_vel.linear.x = 0.0;
		apf_cmd_vel.angular.z = 0.0;

		linear_filter_aplha = 0.08;		// filter factor
		angular_filter_aplha = 0.08;
		
		position_OK_flag = false;
		orintation_OK_flag = false;
		approaching_flag = false;

		sub_carto_odom = nh_nav.subscribe<nav_msgs::Odometry>("/odom", 1, &local_nav::CartoOdomCallBack, this);
		pub_apf_twist = nh_nav.advertise<geometry_msgs::Twist>("/t_cmd_vel", 1);

		sub_amcl_pose = nh_nav.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &local_nav::AmclPoseCallBack, this);

		safe_vel_sub = nh_nav.subscribe<colibri_msgs::SafeVel>("/safe_vel", 1, &local_nav::SafeVelCallBack, this);
}

local_nav::~local_nav()
{
	
}

void local_nav::CartoOdomCallBack(const nav_msgs::Odometry::ConstPtr& carto_odom)
{
	float tmp_yaw_rad = 0.0;
	
	cur_robot_state[0] = carto_odom->pose.pose.position.x;
	cur_robot_state[1] = carto_odom->pose.pose.position.y;
	
	cur_carto_quat = carto_odom->pose.pose.orientation;
	tmp_yaw_rad = tf::getYaw(cur_carto_quat);
	
	cur_robot_state[2] = tmp_yaw_rad * RAD2DEG;

	cur_robot_vel[0] = carto_odom->twist.twist.linear.x;
	cur_robot_vel[1] = carto_odom->twist.twist.angular.z;
	
}

void local_nav::AmclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose)
{
	float tmp_yaw_rad = 0.0;
	
	amcl_cur_state[0] = amcl_pose->pose.pose.position.x;
	amcl_cur_state[1] = amcl_pose->pose.pose.position.y;
	
	amcl_cur_quat = amcl_pose->pose.pose.orientation;
	tmp_yaw_rad = tf::getYaw(amcl_cur_quat);
	
	amcl_cur_state[2] = tmp_yaw_rad * RAD2DEG;
	
}


float local_nav::LinearVelFilter(float* ctrl_linear_vel, float* last_linear_vel)
{
	float filter_linear_vel = *ctrl_linear_vel;

	filter_linear_vel = linear_filter_aplha * (*last_linear_vel) + (1 - linear_filter_aplha) * (*ctrl_linear_vel);
	
	return filter_linear_vel;
	
}

float local_nav::AngularVelFilter(float* ctrl_anglar_vel, float* last_angular_vel)
{
	float filter_angular_vel = *ctrl_anglar_vel;

	filter_angular_vel = angular_filter_aplha * (*last_angular_vel) + (1 - angular_filter_aplha) * (*ctrl_anglar_vel);
	
	return filter_angular_vel;

}

void local_nav::CalcOffsetOfGoalAndRobot(float* cur_robot_state, float* goal_state, float* delta_dis, float* delta_robot2goal_yaw,float* delta_laser2goal_yaw)
{
	float tmp_delta_x = 0.0;
	float tmp_delta_y = 0.0;

	float yaw_offset = 0.0;

	float tmp_laser_x = 0.0;
	float tmp_laser_y = 0.0;
	
	tmp_delta_x =  *(goal_state) - *(cur_robot_state);	
	tmp_delta_y =  *(goal_state + 1) - *(cur_robot_state + 1);


	*delta_dis = sqrt(pow(tmp_delta_x, 2) + pow(tmp_delta_y, 2));
	*delta_robot2goal_yaw =  atan2(tmp_delta_y, tmp_delta_x) * RAD2DEG;

	RobotPos2LaserPos(cur_robot_state, &tmp_laser_x, &tmp_laser_y);	// fixed rotate transform;

	tmp_delta_x = *(goal_state) - tmp_laser_x;
	tmp_delta_y = *(goal_state + 1) - tmp_laser_y;

	cout << "tmp_delta_x:" << tmp_delta_x << endl;
	cout << "tmp_delta_y:" << tmp_delta_y << endl;
	*delta_laser2goal_yaw = atan2(tmp_delta_y, tmp_delta_x) * RAD2DEG;	//corrected by goal in laser frame x dir in degree

	// vector: robot->goal  in the original map frame  bias angle in x dir : -pi ~ + pi
    /*
 								goal   *
                                                                                 /
                                                                               /
				      		y ^    		     /
						   |		   /
						   | 	          /
					 	   | robot ./ 
						   |       current robot center
	robot center orignal point :  o  ----->x

    	*/
		
}

void local_nav::RobotPos2LaserPos(float* robot_pos, float* laser_x, float* laser_y)
{
	float tmp_yaw_rad = 0.0;
	float* ptrLaser_pos = NULL;
	
	tmp_yaw_rad = *(robot_pos + 2) * DEG2RAD;
	
	*laser_x = *(robot_pos) + OFFSET_LASER2ROBOT * cos(tmp_yaw_rad);
	*laser_y = *(robot_pos + 1) + OFFSET_LASER2ROBOT * sin(tmp_yaw_rad);

}

bool local_nav::CalcGoalDirOfLaserView(float* dir_laser2goal, float* dir_laser, float* dir_goal_in_laser, float* self_rot_angle)
{
	float tmp_goal_in_laser = 0.0;
	float tmp_laser_dir = 0.0;
	
	if(*dir_laser < -0.0001)		// make the yaw scope output in 0~360 degree
	{
		tmp_laser_dir = 360 + *dir_laser;
	}
	else
	{
		tmp_laser_dir = *dir_laser;
	}
	
	tmp_goal_in_laser = (*dir_laser2goal) - tmp_laser_dir + 90.0;	// goal in laser using  the scan 0 degree at robot right side and 180 degree as the robot left side

	if(tmp_goal_in_laser >= 180.0)
	{
		*dir_goal_in_laser = tmp_goal_in_laser - 360.0;           	
	}
	else if(tmp_goal_in_laser <= -180.0)
	{
		*dir_goal_in_laser = 360.0 + tmp_goal_in_laser;
	}
	else
	{
		*dir_goal_in_laser = tmp_goal_in_laser;

	}
	
	if((*dir_goal_in_laser >= 180.0)||(*dir_goal_in_laser <= 0.0))
	{
		*self_rot_angle = *dir_goal_in_laser - 90.0;		// ideally using dir_robot2goal - dir_laser may better
		return false;
	}
	else
	{
		*self_rot_angle = 0.0;
		return true;
	}
	
}

void local_nav::CalcEuclidDistance(float * pos_start, float * pos_end, float &dis)
{

	float tmp_delta_x = 0.0;
	float tmp_delta_y = 0.0;	

	tmp_delta_x = *pos_start - *pos_end;
	tmp_delta_y = *(pos_start + 1) - *(pos_end + 1);

	dis = sqrt(pow(tmp_delta_x, 2) + pow(tmp_delta_y, 2));

}

bool local_nav::ReachGoalPositionOK(float* delta_dis)
{
	if(*(delta_dis) < TRANSLATION_TOLERANCE)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}

bool local_nav::ReachApprochingAreaOK(float* delta_dis)
{
	if(*(delta_dis) < GOAL_NGHBORHD)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}

bool local_nav::ReachGoalOrientationOK(float* delta_yaw)
{
	if(abs(*(delta_yaw)) < ROTATION_TOLERANCE)
	{
		return true;
	}
	else
	{
		return false;
	}

}

void local_nav::SatuateCmdVel(float* cmd_linear_vel, float* cmd_angular_vel)
{
	int tmp_sgn = 0;
	
	if(abs(*cmd_linear_vel) > V_MAX)
	{
		tmp_sgn = SgnOfData(cmd_linear_vel);
		*cmd_linear_vel = tmp_sgn * V_MAX;	
	}

	if(abs(*cmd_angular_vel) > THETA_V_MAX)
	{
		tmp_sgn = SgnOfData(cmd_angular_vel);
		*cmd_angular_vel = tmp_sgn * THETA_V_MAX;	
	}
}

bool local_nav::EmergencyStop(float* cmd_vel)
{
	*cmd_vel = 0.0;
	*(cmd_vel + 1) = 0.0;

	return true;
}
		
int local_nav::SgnOfData(float* input)
{
	if( *input > 0.0001)
	{
		return 1;
	}
	else if(*input < -0.0001)
	{
		return -1;
	}
	else
	{
		return 0;
	}
		
}

void local_nav::SafeVelCallBack(const colibri_msgs::SafeVel::ConstPtr& safe_vel)
{
	safe_velocity.header.stamp = safe_vel->header.stamp;
	safe_velocity.header.frame_id = safe_vel->header.frame_id;
	safe_velocity.header.seq = safe_vel->header.seq;

	safe_velocity.stop.data = safe_vel->stop.data;		
	safe_velocity.linear_safe_thd = safe_vel->linear_safe_thd;
	safe_velocity.linear_safe_vel = safe_vel->linear_safe_vel;
	safe_velocity.steer = safe_vel->steer;
	safe_velocity.angular_safe_thd = safe_vel->angular_safe_thd;
	safe_velocity.angular_safe_vel = safe_vel->angular_safe_vel;

}



