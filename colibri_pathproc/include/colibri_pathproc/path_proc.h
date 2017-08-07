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
extern string taskpath;

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
	point2d_pix start;
	point2d_pix end;
}seg_property;

typedef struct st_route_list
{
	int target_id;
	vector<int> seg_list;
}route_list;


bool VerticalLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &ver_line);
bool BresenhamBasic(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &point_at_line);
bool CalcPixesInLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &point_at_line);

class PathProc{

	public:
		string map_name_;
		float map_origin_[3];
		int map_size_[2];
		float map_resol_;
		int segs_num_;
		vector<seg_property> vec_seg_property_;
		vector<segment> vec_seg_;
		vector<point2d_map> route_map_;
		vector<point2d_pix> route_pix_;
		route_list cur_route_;
		route_list last_route_;
		route_list next_route_;

		PathProc();
		~PathProc();
		void CalcAllPointsInSegs();
		void CatSeg2Route(route_list &route);

	private:
		void Pix2Map(vector<point2d_pix> &points_pix, vector<point2d_map> &points_map);


};

template <class T>  
class FindX
{
	public:
         FindX(const T ref){ x_ = ref;}
         T GetX() {return x_;}

         bool operator()(segment &seg)
		 {
	         if( abs(seg.seg_id - x_) < 0.0001)

	              return true;
	         else
	              return false;
          }

	private: 
		 T x_;

};


#endif
