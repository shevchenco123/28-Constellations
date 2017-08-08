#include "path_proc.h"

PathProc::PathProc()
{

	//string path_name(taskpath);
	string path_name;
	path_name.assign("/home/colibri/colibri_ws/src/colibri_pathproc/path/path.yaml");

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

}

PathProc::~PathProc()
{

}

void PathProc::CalcAllPointsInSegs()
{
	vec_seg_.clear();
	vector<segment> ().swap(vec_seg_);
	segment tmp;
	
	for (vector<seg_property>::iterator it = vec_seg_property_.begin(); it!=vec_seg_property_.end(); ++it)
	{

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
		tmp.x = (float) map_origin_[0] + tmp_x * map_resol_;
		tmp.y = (float) map_origin_[1] + tmp_y * map_resol_;
		points_map.push_back(tmp);
		
	}
}

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

bool PathProc::DecomposeRoute(int &check_node, int &sub_route_num)
{
	//vector<int>::iterator iElement = find(cur_route_.seg_list.begin(), cur_route_.seg_list.end(), check_node); 
}

void PathProc::MakeNodeSegMap()
{
	for(vector<seg_property>::iterator it = vec_seg_property_.begin(); it != vec_seg_property_.end(); ++it)
	{
		seg_point_map_.insert(pair<int, int>((*it).end_id, (*it).seg_id));
	}
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
	
	point2d_pix tmp_point;

	bool reverse_flag = false;
	if(start.x > end.x)  
	{  
	  x = end.x;  
	  y = end.y;  
	  end.x = start.x;
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

	while(x < end.x)	
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


