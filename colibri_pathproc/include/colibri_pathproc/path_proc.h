#ifndef _PATH_PROC_H_
#define _PATH_PROC_H_

#include <fstream>
#include "yaml-cpp/yaml.h"
#include <math.h>
#include <vector>

template <class T>
typedef struct st_point2D{
	T x;
	T y;
}point2d;


template <class T>
typedef struct st_segment{
	int seg_id;
	int start_id;
	int end_id;
	vector<point2d<T> > point_stack;
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

vector HorizontalLine(point2d<int> &start, point2d<int> &end);
vector VerticalLine(point2d<int> &start, point2d<int> &end);
vector BresenhamLine(point2d<int> start, point2d<int>end);  

class PathProc{

	public:
		
		int map_origin_[3];
		int map_resol_;
		int segs_num_;
		vector<seg_property> vec_seg_property_;
		vector<segment<int> > vec_path_inpix_;
		vector<segment<float> > vec_path_inmap_;

		PathProc();
		~PathProc();
		ObatainSegProperty();
		InterpolatingLine();
		CatLine2Route();

};


#endif
