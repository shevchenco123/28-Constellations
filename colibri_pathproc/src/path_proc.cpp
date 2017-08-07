#include "path_proc.h"

PathProc::PathProc()
{
	memset(map_origin_, 0.0, 3);

	map_resol_ = 0.05;
	segs_num_ = 0;

}

PathProc::~PathProc()
{

}

void PathProc::ObatainSegProperty()
{

}

void PathProc::InterpolatingLine()
{

}

void PathProc::CatLine2Route()
{

}

bool VerticalLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &ver_line)
{
	
	point2d_pix tmp_point;
	ver_line.clear();

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

bool CalcPointsInLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &point_at_line)
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


