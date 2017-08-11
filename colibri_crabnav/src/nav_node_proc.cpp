#include "nav_node_proc.h"

NavNodeProc::NavNodeProc()
{

	string path_name;
	path_name.assign("/home/colibri/colibri_ws/src/colibri_crabnav/path/path4.yaml");

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

	cur_nav_node = 255;	//if node_id ==255 means no nav node to go shoule stop immediately
	obtain_goal_flag = false;

	pub_nav_state_ = nh_nav_node_.advertise<colibri_msgs::NavState>("/nav_state", 1);
	sub_coodinator_ = nh_nav_node_.subscribe<colibri_msgs::Coordinator>("/Coordinator", 1, &NavNodeProc::CoordinatorCallBack, this);
	sub_node_id_ = nh_nav_node_.subscribe<colibri_msgs::NavNodeId>("/NavNodeId", 1, &NavNodeProc::NavNodeCallBack, this);

}

NavNodeProc::~NavNodeProc()
{

}


void NavNodeProc::NavNodeCallBack(const colibri_msgs::NavNodeId::ConstPtr& node_id)
{

	cur_nav_node = node_id->node_id;
	obtain_goal_flag = true;

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


void NavNodeProc::CoordinatorCallBack(const colibri_msgs::Coordinator::ConstPtr& coordinator)
{

	basic_ctrl_ = coordinator->basic_ctrl;
	
}



