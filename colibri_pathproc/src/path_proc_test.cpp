#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "path_proc.h"
#include <ros/ros.h>

#include <fstream>
#include "yaml-cpp/yaml.h"

using namespace std;

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "pathproc_test_node");
	ros::NodeHandle nh;

	string path_name(argv[1]);
	//string path_name;
	//path_name.assign("/home/colibri/colibri_ws/src/colibri_pathproc/path/path.yaml");
	
	ros::Rate loop_rate(10); 

	PathProc pathProcObj;
	
	ifstream fin_path(path_name.c_str());
	if(fin_path.fail())
	{
		ROS_ERROR("yaml file can not open in parse the yaml argv in proc");
		exit(-1);
	}

#ifdef HAVE_NEW_YAMLCPP
		YAML::Node doc_path = YAML::Load(fin_path);
#else
		YAML::Parser parser(fin_path);
		YAML::Node  doc_path;		
		parser.GetNextDocument(doc_path);
#endif

	try 
	{ 

		doc_path["path"]["map_name"] >> pathProcObj.map_name_;
		doc_path["path"]["resolution"] >> pathProcObj.map_resol_;
		doc_path["path"]["map_origin"][0] >> pathProcObj.map_origin_[0];
		doc_path["path"]["map_origin"][1] >> pathProcObj.map_origin_[1];
		doc_path["path"]["map_origin"][2] >> pathProcObj.map_origin_[2];

		doc_path["path"]["seg_num"] >> pathProcObj.segs_num_;
		string seg_prop_name;
		string seg_terminal_name;
		stringstream sstr_num; 
		string num2str;
		seg_property tmp_seg_prop;
		for(int seg_index = 0; seg_index < pathProcObj.segs_num_; seg_index++)
		{
			sstr_num << seg_index;
		    num2str = sstr_num.str();
			seg_prop_name = "seg" + num2str+ "_property";
			seg_terminal_name = "seg" + num2str+ "_vector";
		
			doc_path["path"][seg_prop_name][0] >> tmp_seg_prop.seg_id;
			doc_path["path"][seg_prop_name][1] >> tmp_seg_prop.start_id;
			doc_path["path"][seg_prop_name][2] >> tmp_seg_prop.end_id;
			
			doc_path["path"][seg_terminal_name][0] >> tmp_seg_prop.start_pix_x;		
			doc_path["path"][seg_terminal_name][1] >> tmp_seg_prop.start_pix_y;
			doc_path["path"][seg_terminal_name][2] >> tmp_seg_prop.end_pix_x;
			doc_path["path"][seg_terminal_name][3] >> tmp_seg_prop.end_pix_y;

			pathProcObj.vec_seg_property_.push_back(tmp_seg_prop);
			sstr_num.str("");
		}
	
	}
	catch (YAML::InvalidScalar) 
	{ 
		ROS_ERROR("The yaml does not contain an origin tag or it is invalid.");
		exit(-1);
	}

	while(ros::ok())
	{
		cout<<pathProcObj.map_resol_<<endl;
		ros::spinOnce();	  
		loop_rate.sleep();
	}


	return 0;
}

