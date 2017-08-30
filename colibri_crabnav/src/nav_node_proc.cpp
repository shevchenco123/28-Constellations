#include "nav_node_proc.h"

NavNodeProc::NavNodeProc()
{

	string path_name;
	path_name.assign("/home/aiv-1/colibri_ws/src/colibri_crabnav/path/path6.yaml");

	ifstream fin_path(path_name.c_str());
	if(fin_path.fail())
	{
		cout<<"yaml file can not open in parse the yaml argv in proc"<<endl;
		exit(-1);
	}

	YAML::Node doc_path = YAML::Load(fin_path);
	try 
	{ 

		doc_path["path"]["image"] >> map_name_;
		doc_path["path"]["resolution"] >> map_resol_;
		doc_path["path"]["origin"][0] >> map_origin_[0];
		doc_path["path"]["origin"][1] >> map_origin_[1];
		doc_path["path"]["origin"][2] >> map_origin_[2];
		doc_path["path"]["map_size"][0] >> map_size_[0];
		doc_path["path"]["map_size"][1] >> map_size_[1];
		doc_path["path"]["seg_num"] >> segs_num_;
		
		string seg_prop_name;
		string seg_terminal_name;
		stringstream sstr_num; 
		string num2str;
		seg_property tmp_seg_prop;
		for(int seg_index = 0; seg_index < segs_num_; seg_index++)
		{
			sstr_num << seg_index;
		    num2str = sstr_num.str();
			seg_prop_name = "seg" + num2str+ "_property";
			seg_terminal_name = "seg" + num2str+ "_vector";
		
			doc_path["path"][seg_prop_name][0] >> tmp_seg_prop.seg_id;
			doc_path["path"][seg_prop_name][1] >> tmp_seg_prop.start_id;
			doc_path["path"][seg_prop_name][2] >> tmp_seg_prop.end_id;
			
			doc_path["path"][seg_terminal_name][0] >> tmp_seg_prop.start.x;		
			doc_path["path"][seg_terminal_name][1] >> tmp_seg_prop.start.y;
			doc_path["path"][seg_terminal_name][2] >> tmp_seg_prop.end.x;
			doc_path["path"][seg_terminal_name][3] >> tmp_seg_prop.end.y;

			vec_seg_property_.push_back(tmp_seg_prop);
			sstr_num.str("");
		}
	
	}
	catch (YAML::InvalidScalar) 
	{ 
		cout<<"The yaml does not contain an origin tag or it is invalid."<<endl;
		exit(-1);
	}

	test_var = 0;


	cur_nav_node_ = 255;	//if node_id ==255 means no nav node to go shoule stop immediately
	obtain_goal_flag = false;

	robot_nav_state_.header.stamp = ros::Time::now();
	robot_nav_state_.header.frame_id = "robot";
	robot_nav_state_.target_node = 1;
	robot_nav_state_.target_heading = 0.0;
	robot_nav_state_.cur_seg = 0;
	robot_nav_state_.at_target_flag = false;
	robot_nav_state_.achieve_flag = false;
	robot_nav_state_.target_x = 0.0;
	robot_nav_state_.target_y = 0.0;
	robot_nav_state_.target_yaw = 0.0;
	robot_nav_state_.cur_x = 0.0;
	robot_nav_state_.cur_y = 0.0;
	robot_nav_state_.cur_yaw = 0.0;
	robot_nav_state_.err_code = 0;

	basic_ctrl_ = 0;
	clr_at_target_ = 0;
	clr_achieve_target_ = 0;
	target_node_ = 0;

	pub_nav_state_ = nh_nav_node_.advertise<colibri_msgs::NavState>("/nav_state", 1);
	sub_robot_cmd_ = nh_nav_node_.subscribe<colibri_msgs::RobotCmd>("/robot_cmd", 1, &NavNodeProc::RobotCmdCallBack, this);
	sub_node_id_ = nh_nav_node_.subscribe<colibri_msgs::NavNode>("/nav_node", 1, &NavNodeProc::NavNodeCallBack, this);

}

NavNodeProc::~NavNodeProc()
{

}


void NavNodeProc::NavNodeCallBack(const colibri_msgs::NavNode::ConstPtr& node)
{
	cur_nav_node_ = int (node->node_id);
	NavNode2NavPose();

}


void NavNodeProc::InitNodeAndSegMap(float *head_array, int &array_size)
{

	int j = 0;
	for(vector<seg_property>::iterator it = vec_seg_property_.begin(); it != vec_seg_property_.end(); ++it)
	{
		node_seg_map_.insert(make_pair((*it).end_id, (*it).seg_id));
		node_head_map_.insert(pair<int, float>((*it).end_id, *(head_array + j)));
		seg_node_map_.insert(make_pair((*it).seg_id, (*it).end_id));
		j++;
		if(j > array_size)
		{
			cout<<"Horriable, the pointer go out of the range..."<<endl;
			break;
		}
		
	}

}

bool NavNodeProc::PubNavState(void)
{

	
	colibri_msgs::NavState nav_sta;
	nav_sta.header.stamp = ros::Time::now();
	nav_sta.header.frame_id = "robot";
	nav_sta.target_node = robot_nav_state_.target_node;
	nav_sta.target_heading = robot_nav_state_.target_heading;
	nav_sta.cur_seg = test_var;
	nav_sta.at_target_flag = robot_nav_state_.at_target_flag;
	nav_sta.achieve_flag = robot_nav_state_.achieve_flag;
	nav_sta.target_x = robot_nav_state_.target_x;
	nav_sta.target_y = robot_nav_state_.target_y;
	nav_sta.target_yaw = robot_nav_state_.target_yaw;
	nav_sta.cur_x = robot_nav_state_.cur_x;
	nav_sta.cur_y = robot_nav_state_.cur_y;
	nav_sta.cur_yaw = robot_nav_state_.cur_yaw;
	nav_sta.err_code = robot_nav_state_.err_code;
	test_var++;
	
	pub_nav_state_.publish(nav_sta);

}



bool NavNodeProc::NavNode2NavPose()
{	
	if(cur_nav_node_ != 255)
	{
		point2d_pix tmp_end_pix;
		vector<seg_property>::iterator it = find_if(vec_seg_property_.begin(), vec_seg_property_.end(),FindX<int,seg_property>(cur_nav_node_));
		if(it != vec_seg_property_.end())
		{
			tmp_end_pix = (*it).end;
			int tmp_x = tmp_end_pix.x;
			int tmp_y = map_size_[1] - tmp_end_pix.y;
			cur_goal[0] = (float) (map_origin_[0] + tmp_x * map_resol_);
			cur_goal[1] = (float) (map_origin_[1] + tmp_y * map_resol_);
			cur_goal[2] = node_head_map_[(*it).end_id];
			obtain_goal_flag = true;
		}
		else
		{
			cout<<"the input nav node not in the path nodes"<<endl;
			obtain_goal_flag = false;
			return false;
		}
	
	}
	else
	{
		cout<<"the input nav node is 255 for stop"<<endl;
		obtain_goal_flag = false;
		return false;
	}

}


void NavNodeProc::RobotCmdCallBack(const colibri_msgs::RobotCmd::ConstPtr& cmd)
{

	basic_ctrl_ = int (cmd->basic_ctrl);
	clr_at_target_ = int (cmd->clr_at_target);
	clr_achieve_target_ = int (cmd->clr_achieve_target);
	target_node_ = int (cmd->target_node);
}

void NavNodeProc::Quaternion2Yaw(const geometry_msgs::PoseStamped &pose, float &yaw)
{
	float x = 0.0;
	float y = 0.0;
	
	y = 2.0 * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y);
	x = 1.0 - 2 * (pow(pose.pose.orientation.y, 2) + pow(pose.pose.orientation.z, 2));
	yaw = atan2(y,x) * RAD2DEG;

}


