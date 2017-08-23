#include "path_proc.h"

PathProc::PathProc()
{

	//string path_name(taskpath);
	string path_name;
	path_name.assign("/home/colibri/colibri_ws/src/colibri_pathproc/path/path5.yaml");

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

	parsed_node_ = 255;
	req4path_flag = false;


	pub_route_ = nh_route_.advertise<nav_msgs::Path>("/nav_path", 1);
	sub_coodinator_ = nh_route_.subscribe<colibri_msgs::Coordinator>("/coordinator", 1, &PathProc::CoordinatorCallBack, this);
	sub_nav_state_ = nh_route_.subscribe<colibri_msgs::NavState>("/nav_state", 1, &PathProc::NavStateCallBack, this);
	pub_marker_ = nh_route_.advertise<visualization_msgs::Marker>("waypoint_markers", 10);
 	srv4getpath_ = nh_route_.advertiseService("/move_base/make_plan", &PathProc::ExecGetPathSrv, this);

}

PathProc::~PathProc()
{

}

void PathProc::InitMarkers(void)
{
	goalmark_list_.ns       = "waypoints";
	goalmark_list_.id       = 0;
	goalmark_list_.type     = visualization_msgs::Marker::CUBE_LIST;
	goalmark_list_.action   = visualization_msgs::Marker::ADD;
	goalmark_list_.lifetime = ros::Duration();//0 is forever
	goalmark_list_.scale.x  = 0.2;
	goalmark_list_.scale.y  = 0.2;
	goalmark_list_.color.r  = 1.0;
	goalmark_list_.color.g  = 0.0;
	goalmark_list_.color.b  = 0.0;
	goalmark_list_.color.a  = 1.0;

	goalmark_list_.header.frame_id = "map";
	goalmark_list_.header.stamp = ros::Time::now();

}

int PathProc::FillMarkerPose(route_list & route)
{

	goalmark_list_.points.clear();
	geometry_msgs::Pose subgoal_list;

	for(vector<int>::iterator it = route.seg_list.begin(); it != route.seg_list.end(); ++it)
	{
		vector<segment>::iterator tmp_it = find_if(vec_seg_.begin(), vec_seg_.end(),FindX<int,segment>(*it));
		if(tmp_it != vec_seg_.end())
		{
			subgoal_list.position.x = (*tmp_it).points_map.back().x;
			subgoal_list.position.y = (*tmp_it).points_map.back().y;
			subgoal_list.orientation.w = 1.0;
			goalmark_list_.points.push_back(subgoal_list.position);
		}
	}
	return goalmark_list_.points.size();
}


/* Calc the bresham line pix coordinate from all existed segs to fill in segment struct vec_seg_*/
void PathProc::CalcAllPointsInSegs(void) 
{
	vec_seg_.clear();
	vector<segment> ().swap(vec_seg_);
	segment tmp;
	
	for (vector<seg_property>::iterator it = vec_seg_property_.begin(); it!=vec_seg_property_.end(); ++it)
	{

		segment tmp;
		tmp.seg_id = (*it).seg_id;
		tmp.start_id = (*it).start_id;
		tmp.end_id = (*it).end_id;
		CalcPixesInLine((*it).start, (*it).end, tmp.points_pix);
		Pix2Map(tmp.points_pix, tmp.points_map);
		vec_seg_.push_back(tmp);

	}
	
}

void PathProc::Pix2Map(vector<point2d_pix> &points_pix, vector<point2d_map> &points_map)
{
	point2d_map tmp;
	points_map.clear();
	vector<point2d_map> ().swap(points_map);
	for (vector<point2d_pix>::iterator it = points_pix.begin(); it!=points_pix.end(); ++it)
	{
		int tmp_x = (*it).x;
		int tmp_y = map_size_[1] - (*it).y;
		tmp.x = (float) (map_origin_[0] + tmp_x * map_resol_);
		tmp.y = (float) (map_origin_[1] + tmp_y * map_resol_);
		points_map.push_back(tmp);
		
	}
}

/* Calc the whole  map and pix route removing the seg start from the route_list struct */
void PathProc::CatSeg2Route(route_list &route)
{
	route_pix_.clear();
	vector<point2d_pix> ().swap(route_pix_);
	route_map_.clear();
	vector<point2d_map> ().swap(route_map_);
	for (vector<int>::iterator it = route.seg_list.begin(); it!=route.seg_list.end(); ++it)
	{
		vector<segment>::iterator tmp = find_if(vec_seg_.begin(), vec_seg_.end(),FindX<int,segment>(*it));
		segment tmp_rm_start(*tmp);
		tmp_rm_start.points_pix.erase(tmp_rm_start.points_pix.begin());	// remove the start point
		tmp_rm_start.points_map.erase(tmp_rm_start.points_map.begin());
		route_pix_.insert(route_pix_.end(), tmp_rm_start.points_pix.begin(), tmp_rm_start.points_pix.end());
		route_map_.insert(route_map_.end(), tmp_rm_start.points_map.begin(), tmp_rm_start.points_map.end());
	}

}

/* Calc the sub segs from a known seg_list which is the whole route from hostpc and decompose it into sub_route_vec_*/
bool PathProc::DecomposeRoute(vector<int> &seg_list, vector<int> &check_nodes, int &sub_route_num)
{
	vector<int> remain_segs(seg_list);
	route_list tmp_sub_route;
	sub_route_num = 0;
	for(vector<int>::iterator it = remain_segs.begin(); it != remain_segs.end(); ++it) //get every segs in the whole route
	{
		int tmp_node = seg_node_map_[*it];
		vector<int>::iterator iElement = find(check_nodes.begin(), check_nodes.end(), tmp_node);// check the node in route is in check_nodes
		if(iElement != check_nodes.end())	//node in the check_nodes
		{
			for(vector<int>::iterator sub_it = remain_segs.begin(); sub_it <= it; ++sub_it)
			{
				tmp_sub_route.seg_list.push_back(*sub_it);
			}
			tmp_sub_route.target_id = *iElement;
			tmp_sub_route.target_heading = node_heading_map_[*iElement];
			sub_route_vec_.push_back(tmp_sub_route);
			sub_route_num++;
		}
		tmp_sub_route.seg_list.clear();
		vector<int> ().swap(tmp_sub_route.seg_list);		
	}

	sub_route_vec_.clear();
	vector<route_list> ().swap(sub_route_vec_);

	int sub_num = sub_route_vec_.size(); 
	for(int i = 1; i < sub_num; i++)
	{
		for(vector<int>::iterator index = sub_route_vec_[sub_num-1-i].seg_list.begin(); index < sub_route_vec_[sub_num-1-i].seg_list.end(); ++index)
		{
			vector<int>::iterator iElem = find(sub_route_vec_[sub_num-i].seg_list.begin(), sub_route_vec_[sub_num-i].seg_list.end(), (*index));
			sub_route_vec_[sub_num-i].seg_list.erase(iElem);
		}
	}


	return true;
	
}

bool PathProc::ExecGetPathSrv(nav_msgs::GetPlan::Request & req, nav_msgs::GetPlan::Response & res)
{

	point2d_map tmp_start,tmp_goal;
	bool parse_node_flag = false;
	
	tmp_start.x = req.start.pose.position.x;
	tmp_start.y = req.start.pose.position.y;
	tmp_goal.x = req.goal.pose.position.x;
	tmp_goal.y = req.goal.pose.position.y;
	cout<<"req start.x: "<<tmp_start.x<<endl;
	cout<<"req start.y: "<<tmp_start.y<<endl;
	cout<<"req goal.x: "<<tmp_goal.x<<endl;
	cout<<"req goal.y: "<<tmp_goal.y<<endl;	
	
	parse_node_flag = MapPose2NavNode(tmp_goal, parsed_node_);
	
	if(parse_node_flag == true)
	{
		route_list route;
		route.target_id = parsed_node_;
		route.target_heading = 0.0;
		int micro_seg_num;
		for(int j= 0; j < parsed_node_; j++)
		{
			route.seg_list.push_back(j);
		}
		
		AddTargetNode2KneeNodes(route.target_id);
		DecomposeRoute(route.seg_list, knee_nodes_, micro_seg_num);	
		CatSeg2Route(route);
		FillMarkerPose(route);
		StdNavPath(route_map_);
		res.plan = this->plan_path_;
		req4path_flag = true;

	}
	else
	{
		return false;
	}

	return true;
	
}


bool PathProc::StdNavPath(vector<point2d_map> &nav_path)
{
	if(nav_path.empty())
	{
		return false;
	}
	else
	{
		plan_path_.poses.clear();
		geometry_msgs::PoseStamped tmp_pose_stamped;
		
		plan_path_.header.stamp = ros::Time::now();
		plan_path_.header.frame_id = "map";
		tmp_pose_stamped.header.stamp = ros::Time::now();
		tmp_pose_stamped.header.frame_id = "map";

		size_t len = nav_path.size();
		for (size_t i = 0; i < len; i++)
		{
			tmp_pose_stamped.pose.position.x = nav_path[i].x;			
			tmp_pose_stamped.pose.position.y = nav_path[i].y;
			tmp_pose_stamped.pose.position.z = 0.0;
			tmp_pose_stamped.pose.orientation.x = 0.0;
			tmp_pose_stamped.pose.orientation.y = 0.0;
			tmp_pose_stamped.pose.orientation.z = 0.0;
			tmp_pose_stamped.pose.orientation.w = 1.0;
			plan_path_.poses.push_back(tmp_pose_stamped);
		}
		
		return true;
	}
	
}

bool PathProc::MapPose2NavNode(point2d_map & pose, int & rev_node_id)
{

	point2d_pix tmp_uv;
	vector<seg_property> tt(vec_seg_property_);

	tmp_uv.x =  (pose.x - map_origin_[0]) / map_resol_;
	tmp_uv.y =  map_size_[1] - (pose.y - map_origin_[1]) / map_resol_;

	if(NavPixValid(tmp_uv))
	{
		
		for(vector<seg_property>::iterator it = tt.begin(); it != tt.end(); it++)
		{

			int delta_u = abs((*it).end.x - tmp_uv.x);
			int delta_v = abs((*it).end.y - tmp_uv.y);
			if( (delta_u < 3) && (delta_v < 3))
			{
				rev_node_id = (*it).end_id;
				return true;
			}

		}

		cout <<" Can not find the nav node from the request info"<<endl;
		rev_node_id = 255;
		return false;
		
	}
	else
	{
		rev_node_id = 255;
		return false;
	}

	
}

bool PathProc::NavPixValid(point2d_pix &pix_uv)
{
	if(pix_uv.x >= map_size_[0] || pix_uv.x <= 0 || pix_uv.y >= map_size_[1] || pix_uv.y <= 0 )
	{
		return false;
	}
	else
	{
		return true;
	}
}


void PathProc::MakeNodeSegMap(vector<float> &vec_heading)
{
	int i = 0;
	for(vector<seg_property>::iterator it = vec_seg_property_.begin(); it != vec_seg_property_.end(); ++it)
	{
		node_seg_map_.insert(pair<int, int>((*it).end_id, (*it).seg_id));
		seg_node_map_.insert(pair<int, int>((*it).seg_id, (*it).end_id));
		node_heading_map_.insert(pair<int, float>((*it).end_id, vec_heading[i]));
		i++;
	}
}

void PathProc::ConfigNodesHeading(float *head_array, int &array_size)
{
	nodes_heading_.clear();
	vector<float> ().swap(nodes_heading_);
	for(int i = 0; i < array_size; i++)
	{
		nodes_heading_.push_back(*(head_array + i));
	}

}

void PathProc::InitKneeNodes(int *node_array, int &array_size)
{
	knee_nodes_.clear();
	vector<int> ().swap(knee_nodes_);
	for(int i = 0; i < array_size; i++)
	{
		knee_nodes_.push_back(*(node_array + i));
	}
}


bool PathProc::AddTargetNode2KneeNodes(int &target_node)
{
	vector<int>::iterator iElement = find(knee_nodes_.begin(), knee_nodes_.end(), target_node);	
	if(iElement == knee_nodes_.end())
	{
		knee_nodes_.push_back(target_node);	
		return true;
	}
	else
	{
		return false;
	}

}

void PathProc::NavStateCallBack(const colibri_msgs::NavState::ConstPtr& nav_state)
{
	robot_nav_state.target_node = nav_state->target_node;
	robot_nav_state.target_heading = nav_state->cur_seg;
	robot_nav_state.cur_seg = nav_state->cur_seg;
	robot_nav_state.at_target_flag = nav_state->at_target_flag;
	robot_nav_state.achieve_flag = nav_state->achieve_flag;
	robot_nav_state.target.x = nav_state->target_x;
	robot_nav_state.target.y = nav_state->target_y;
	robot_nav_state.target.yaw = nav_state->target_yaw;
	robot_nav_state.robot.x = nav_state->cur_x;
	robot_nav_state.robot.y = nav_state->cur_y;
	robot_nav_state.robot.yaw = nav_state->cur_yaw;
	robot_nav_state.err_code = nav_state->err_code;

}

void PathProc::CoordinatorCallBack(const colibri_msgs::Coordinator::ConstPtr& coordinator)
{
	int seg_num = 0;
	cur_route_.seg_list.clear();
	vector<int> ().swap(cur_route_.seg_list);
	basic_ctrl_ = coordinator->basic_ctrl;
	cout<<"basic_ctrl_: "<<basic_ctrl_<<endl;
	cur_route_.target_id = coordinator->target_node;
	cur_route_.target_heading = coordinator->target_heading;
	seg_num = coordinator->route_segs_num;
	for(int i = 0; i < seg_num; i++)
	{
		cur_route_.seg_list.push_back(coordinator->segs_vector[i]);
	}
	cout<<"seg_num: "<<seg_num<<endl;
}

bool VerticalLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &ver_line)
{
	
	point2d_pix tmp_point;
	
	int index = start.y;
	if(start.y < end.y)
	{
		do
		{
			tmp_point.x = start.x;
			tmp_point.y = index;
			ver_line.push_back(tmp_point);
			index++;
		}while(index <= end.y); 	
	}
	else
	{
		do
		{
			tmp_point.x = start.x;
			tmp_point.y = index;
			ver_line.push_back(tmp_point);
			index--;
		}while(index >= end.y); 	
	}

	return true;

}

bool BresenhamBasic(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &point_at_line) 
{  
	// Calc the slop [0, 1] bresenham
	int dx = fabs(end.x - start.x);
	int dy = fabs(end.y - start.y);  
    int p = 2 * dy - dx;  
    int twoDy = 2 * dy;
	int twoDyMinusDx = 2 * (dy - dx);
    int x,y;
	int x_limit = end.x;
	
	point2d_pix tmp_point;

	bool reverse_flag = false;
	if(start.x > end.x)  
	{  
	  x = end.x;  
	  y = end.y;  
	  x_limit = start.x;
	  reverse_flag = true;
	}  
	else	
	{  
	  x = start.x;	
	  y = start.y;	
	}  
	tmp_point.x = x;
	tmp_point.y = y;
	point_at_line.push_back(tmp_point);

	while(x < x_limit)	
	{  
	  x++;	
	  if(p<0)
	  {
		  p+=twoDy; 
	  }
	  else	
	  {  
		  y++;	
		  p+=twoDyMinusDx;	
	  }  
	  tmp_point.x = x;
	  tmp_point.y = y;
	  point_at_line.push_back(tmp_point); 
	} 

	if(reverse_flag == true)
	{
	  reverse(point_at_line.begin(), point_at_line.end()); 
	}
  
} 

bool CalcPixesInLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &point_at_line)
{

	point_at_line.clear();
	vector<point2d_pix> ().swap(point_at_line);

	if (start.x==end.x && start.y==end.y)
		return false;
	
	if (start.x == end.x)
	{
		VerticalLine(start, end, point_at_line);

		return true;
	}

	float k = (float)(end.y-start.y)/(end.x-start.x);

	if (k >= 0 && k <= 1)
	{

		BresenhamBasic(start, end, point_at_line);

	}
	else if (k > 1)
	{
		int tmp = start.x;
		start.x = start.y;
		start.y = tmp;
		tmp = end.x;
		end.x = end.y;
		end.y = tmp;
		BresenhamBasic(start, end, point_at_line);
		for (vector<point2d_pix>::iterator it = point_at_line.begin(); it!=point_at_line.end(); ++it)
		{
			int tmp = (*it).x;
			(*it).x = (*it).y;
			(*it).y = tmp;
		}

	}
	else if (k >= -1 && k < 0)
	{
		start.y = -1 * start.y;
		end.y = -1 * end.y;

		BresenhamBasic(start, end, point_at_line);
		for (vector<point2d_pix>::iterator it = point_at_line.begin(); it!=point_at_line.end(); ++it)
		{
			(*it).y = -(*it).y;
		}

	}
	else if (k < -1)
	{

		int tmp = start.x;
		start.x = -1 * start.y;
		start.y = tmp;
		tmp = end.x;
		end.x = -1 * end.y;
		end.y = tmp;

		BresenhamBasic(start, end, point_at_line);
		for (vector<point2d_pix>::iterator it = point_at_line.begin(); it!=point_at_line.end(); ++it)
		{
			int tmp = (*it).x;
			(*it).x = (*it).y;
			(*it).y = tmp;
			(*it).y = -(*it).y;
		}

	}

	return true;

}

void int2str(int & i_val, string & str)
{
	stringstream stream;
	stream << i_val;
	str = stream.str();
}



