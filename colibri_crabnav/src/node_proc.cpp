#include "node_proc.h"

NodeProc::NodeProc()
{
/*
	//string path_name(taskpath);
	string path_name;
	path_name.assign("/home/colibri/colibri_ws/src/colibri_pathproc/path/path4.yaml");

	ifstream fin_path(path_name.c_str());
	if(fin_path.fail())
	{
		cout<<"yaml file can not open in parse the yaml argv in proc"<<endl;
		exit(-1);
	}

	YAML::Node doc_path = YAML::Load(fin_path);
	try 
	{ 

		doc_path["path"]["map_name"] >> map_name_;
		doc_path["path"]["resolution"] >> map_resol_;
		doc_path["path"]["map_origin"][0] >> map_origin_[0];
		doc_path["path"]["map_origin"][1] >> map_origin_[1];
		doc_path["path"]["map_origin"][2] >> map_origin_[2];
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

	pub_route_ = nh_route_.advertise<nav_msgs::Path>("/nav_path", 1);
	sub_coodinator_ = nh_route_.subscribe<colibri_msgs::Coordinator>("/Coordinator", 1, &PathProc::CoordinatorCallBack, this);
	sub_nav_state_ = nh_route_.subscribe<colibri_msgs::NavState>("/Nav_State", 1, &PathProc::NavStateCallBack, this);
*/
}

NodeProc::~NodeProc()
{

}


void NodeProc::MakeNodeSegMap(vector<float> &vec_heading)
{
/*
	int i = 0;
	for(vector<seg_property>::iterator it = vec_seg_property_.begin(); it != vec_seg_property_.end(); ++it)
	{
		node_seg_map_.insert(pair<int, int>((*it).end_id, (*it).seg_id));
		seg_node_map_.insert(pair<int, int>((*it).seg_id, (*it).end_id));
		node_heading_map_.insert(pair<int, float>((*it).end_id, vec_heading[i]));
		i++;
	}
*/
}

void NodeProc::ConfigNodesHeading(float *head_array, int &array_size)
{
/*
	nodes_heading_.clear();
	vector<float> ().swap(nodes_heading_);
	for(int i = 0; i < array_size; i++)
	{
		nodes_heading_.push_back(*(head_array + i));
	}
*/
}


void NodeProc::CoordinatorCallBack(const colibri_msgs::Coordinator::ConstPtr& coordinator)
{
/*
	int seg_num = 0;
	cur_route_.seg_list.clear();
	vector<int> ().swap(cur_route_.seg_list);
	basic_ctrl_ = coordinator->basic_ctrl;
	cur_route_.target_id = coordinator->target_node;
	cur_route_.target_heading = coordinator->target_heading;
	seg_num = coordinator->route_segs_num;
	for(int i = 0; i < seg_num; i++)
	{
		cur_route_.seg_list.push_back(coordinator->segs_vector[i]);
	}
*/
}



