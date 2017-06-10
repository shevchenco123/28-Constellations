#include "colibri_ca.h"
#include "colibri_local_nav.h"
#include "colibri_action.h"
#include "PID_controller.h"
#include "global_planner.h"
#include "task_mgr.h"

#include <boost/bind.hpp>

#include "geometry_msgs/PoseStamped.h"

#include<signal.h>

bool node_shutdown  = false;
int cnt_null_cmdvel = 0;

void PlannerCallback(planner *plannerObj, float* start_pos, float* goal_pos, bool *finish_flag);
void MySigintHandler(int sig);
int main(int argc, char* argv[])
{	

	// ROS nav node initial
	ros::init(argc, argv, "Nav_Mult_RvizGoal_Node");
	ROS_INFO("Start to Go to Mult Goals in Rviz ... ");

	// Auto nav obj initial
	scan_ca scan4caObj;	
	local_nav local4navObj;
	nav_action actionObj;
	planner plannerObj;
	task_mgr taskObj;

	// Local key points relation param initial
	float tmp_delta_dis = 0.0;
	float tmp_robot2goal_yaw = 0.0;
	float tmp_laser2goal_yaw = 0.0;
	float dir_goal_in_laser = 0.0;
	float self_rotation_angle = 0.0;
	bool goal_inlaser_flag = true;

	unsigned int turn_adj_flag = 1;

	float tmp_action_cmd_t[2] = {0.0, 0.0};
	float* ptr_action_cmd_t = tmp_action_cmd_t;
	unsigned int micro_adj_flag = 0;

	unsigned int adj_flag = 0;
	
	bool obtain_flag = false;
	unsigned int search_start = 0;
	
	ros::NodeHandle nh_pp;
	ros::Timer planner_timer;
	bool finish_plan = false;

	bool tmp_timer_finish =false;
	bool *timer_finish = &tmp_timer_finish;

	static unsigned int index4gravaton = 0; 

	float delta_robot2gravaton = 0.0;
	bool at_gravaton_flag = false;
	bool exist_gravaton_flag = false;
	bool replan_flag = false;

	float rt_r2g_dis = 100.0;

	ros::Rate loop_rate(10);		// Set control  freq at 10 hz
	unsigned int delay_cnt = 0;		// Init delay conter for scanObj 

	signal(SIGINT, MySigintHandler);

	while(taskObj.obtain_goal_flag == false)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}	
	local4navObj.goal_state[0] = taskObj.cur_goal[0];	// Extract the Rviz goal for nav
	local4navObj.goal_state[1] = taskObj.cur_goal[1];
	local4navObj.goal_state[2] = taskObj.cur_goal[2];	
	
	while (!ros::service::waitForService(plannerObj.srv4make_plan, ros::Duration(0.2)))		// Waiting for path paln srv
	{
		ROS_INFO("Waiting for srv move_base/make_plan to become available");
	}	
	ROS_INFO("Srv move_base/make_plan prepared OK...");

	// Inital path plan calling  and gravaton calc
	finish_plan = plannerObj.ExecMonoPlanAndGravaton(plannerObj,&local4navObj.cur_robot_state[0],&local4navObj.goal_state[0], search_start,index4gravaton);

	// Set path plan timer
	planner_timer = nh_pp.createTimer(ros::Duration(PLAN_INTERVAL), boost::bind(&PlannerCallback, &plannerObj, &local4navObj.amcl_cur_state[0],&taskObj.cur_goal[0], timer_finish));

	while (ros::ok())
	{	
			
		if(delay_cnt < DELAY_CNT_MAX)	// Waiting for scan data
		{
			delay_cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		if(delay_cnt >= DELAY_CNT_MAX)
		{
			ROS_INFO("------ start ------");

			//--------------------------  Calc gravaton for nav ------------------------------
			if(*timer_finish == true)	// Path plan timer OK
			{
				*timer_finish = false;
				index4gravaton = 0;
				replan_flag = true;
			}
			else
			{						
				// If path replan or robot at gravaton but not in approaching target goal, calc a new gravaton in the existed planned path
				if((at_gravaton_flag == true && local4navObj.approaching_flag == false)||(replan_flag == true))
				{
					plannerObj.CalcPath2RobotDeltaDis(plannerObj.path_array, local4navObj.amcl_cur_state);
					index4gravaton = plannerObj.CalcGravatonFromPath(plannerObj.path_array, plannerObj.path2robot_array, index4gravaton, plannerObj.gravaton, exist_gravaton_flag);
					at_gravaton_flag = false;
					replan_flag = false;
				}

				//judge the amcl pos and gravaton distance relation			
				at_gravaton_flag = actionObj.ReachGravatonOK(&local4navObj.amcl_cur_state[0],&plannerObj.gravaton.x, delta_robot2gravaton);

				// if robot goes to the setting target goal approching radius, set gravaton same as target goal 
				if(local4navObj.approaching_flag == true)
				{
					plannerObj.gravaton.x = taskObj.cur_goal[0];
					plannerObj.gravaton.y = taskObj.cur_goal[1];
					plannerObj.gravaton.yaw = taskObj.cur_goal[2];
				}
			}

			//---------------------------------------------------------------------
			
			local4navObj.CalcOffsetOfGoalAndRobot(local4navObj.amcl_cur_state, &plannerObj.gravaton.x, &tmp_delta_dis, &tmp_robot2goal_yaw, &tmp_laser2goal_yaw);

			goal_inlaser_flag = local4navObj.CalcGoalDirOfLaserViewNew(&tmp_laser2goal_yaw, &local4navObj.amcl_cur_state[2], &dir_goal_in_laser, &self_rotation_angle);

			scan4caObj.CalcPhiParam(local4navObj.cur_robot_vel[0], dir_goal_in_laser);

			scan4caObj.CalcKrfTheta(scan4caObj.kp_phi_vec, scan4caObj.phi_start_vec, scan4caObj.phi_end_vec);
			scan4caObj.CalcCorrectedKrf();
			scan4caObj.CalcPassFcnAndFwdBnd(scan4caObj.wander, &scan4caObj.max_passfcn_val, scan4caObj.passfcn_vec);
			scan4caObj.CalcPassFcnAndBwdBnd(scan4caObj.wander, &scan4caObj.max_passfcn_val, scan4caObj.passfcn_vec);

			scan4caObj.angle_adj = scan4caObj.CalcAdjDir(scan4caObj.passfcn_vec,scan4caObj.max_passfcn_val, &scan4caObj.maxfcn_fwdbnd,&scan4caObj.maxfcn_bwdbnd);
				
			scan4caObj.CalcCollisionInAPF();
			
			if(goal_inlaser_flag == true)
			{
				local4navObj.apf_ctrl_output[0] = (V_MAX - V_MIN) * (scan4caObj.max_passfcn_val / D_M) + V_MIN;
				local4navObj.apf_ctrl_output[1] = scan4caObj.angle_adj / 200.0;	
			}
			else	//if gravaton is not in front of  laser , should exec the still rot 
			{
				local4navObj.apf_ctrl_output[0] = 0.0;
				local4navObj.apf_ctrl_output[1] = 0.0;	
			}
							
			local4navObj.SatuateCmdVel(local4navObj.apf_ctrl_output, local4navObj.apf_ctrl_output+1);

			local4navObj.CalcEuclidDistance(local4navObj.amcl_cur_state, taskObj.cur_goal, rt_r2g_dis);	// rt_r2g_dis(robot2goal) is different from tmp_delta_dis(robot2gravaton)	
			local4navObj.approaching_flag = local4navObj.ReachApprochingAreaOK(&rt_r2g_dis);
			
			if(tmp_delta_dis >= GOAL_NGHBORHD)
			{
				ptr_action_cmd_t = actionObj.AdjustMovingDirAction(&local4navObj.amcl_cur_state[2], &dir_goal_in_laser, &tmp_robot2goal_yaw, &turn_adj_flag);
			}
			else
			{

			}
			
			if(turn_adj_flag == 1)
			{
				if(local4navObj.approaching_flag == false)
				{
					*ptr_action_cmd_t = local4navObj.apf_ctrl_output[0];
					*(ptr_action_cmd_t + 1) = local4navObj.apf_ctrl_output[1];
				}
				else
				{
					ptr_action_cmd_t = actionObj.ApproachingGoalAction(&local4navObj.amcl_cur_state[0], &local4navObj.goal_state[0], &dir_goal_in_laser,&micro_adj_flag);
				}

			}
						
			//local4navObj.position_OK_flag = local4navObj.ReachGoalPositionOK(&tmp_delta_dis);

			local4navObj.SatuateCmdVel(ptr_action_cmd_t,ptr_action_cmd_t + 1);

			local4navObj.apf_cmd_vel.linear.x = *ptr_action_cmd_t;
			local4navObj.apf_cmd_vel.angular.z = *(ptr_action_cmd_t + 1);

			cout<<"tmp_delta_dis: " << tmp_delta_dis <<endl;
			
			if(local4navObj.position_OK_flag == true)
			{
				local4navObj.apf_cmd_vel.linear.x = 0.0;
				local4navObj.apf_cmd_vel.angular.z = 0.0;
				cout<<"tmp_delta_dis: " << tmp_delta_dis <<endl;
			}

			if(node_shutdown == true)
			{
				local4navObj.apf_cmd_vel.linear.x = 0.0;
				local4navObj.apf_cmd_vel.angular.z = 0.0;
				++cnt_null_cmdvel;
				if(cnt_null_cmdvel > 5)
				{
					ros::shutdown();
				}
			}

			local4navObj.pub_apf_twist.publish(local4navObj.apf_cmd_vel);
			
			cout<<"pub_linear_x: " << local4navObj.apf_cmd_vel.linear.x <<endl;
			cout<<"pub_angular_z: " << local4navObj.apf_cmd_vel.angular.z <<endl;

			scan4caObj.ResetMaxPassValCnt();


			ros::spinOnce();
			loop_rate.sleep();
			ROS_INFO("------- end --------");
		}


		
	}

	return 0;	
}

void PlannerCallback(planner *plannerObj, float* start_pos, float* goal_pos, bool *finish_flag)
{		
	plannerObj->ObtainPathArray(plannerObj->serviceClient, plannerObj->path_srv, start_pos, goal_pos, finish_flag);
}

void MySigintHandler(int sig)
{
	node_shutdown = true;
}
