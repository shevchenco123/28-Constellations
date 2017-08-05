#ifndef _PATH_PROC_H_
#define _PATH_PROC_H_

#include <fstream>
#include "yaml-cpp/yaml.h"

#include <math.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <string.h>

using namespace std;

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif


template <class Type>  
Type stringToNum(const string& str)  
{  
    istringstream iss(str);  
    Type num;  
    iss >> num;  
    return num;      
}  

typedef struct st_point2D_int{
	int x;
	int y;
}point2d_pix;

typedef struct st_point2D_float{
	float x;
	float y;
}point2d_map;


typedef struct st_segment{
	int seg_id;
	int start_id;
	int end_id;
	vector<point2d_pix> points_pix;
	vector<point2d_map> points_map;
}segment;

typedef struct st_seg_prop{
	int seg_id;
	int start_id;
	int end_id;
	int start_pix_x;
	int start_pix_y;
	int end_pix_x;
	int end_pix_y;	
}seg_property;

bool HorizontalLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &hor_line);
bool VerticalLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &ver_line);
bool BresenhamLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &point_at_line);  


class PathProc{

	public:
		string map_name_;
		float map_origin_[3];
		float map_resol_;
		int segs_num_;
		vector<seg_property> vec_seg_property_;
		vector<segment> vec_seg_;

		PathProc();
		~PathProc();
		void ObatainSegProperty();
		void InterpolatingLine();
		void CatLine2Route();

};


#endif
