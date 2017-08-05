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


bool  HorizontalLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &hor_line)
{
	point2d_pix tmp_point;
	hor_line.clear();
	int index = start.x;
	if(start.x < end.x)
	{
		do
		{
			tmp_point.x = index;
			tmp_point.y = start.y;
			hor_line.push_back(tmp_point);
			index++;
		}while(index <= end.x); 	
	}
	else
	{
		do
		{
			tmp_point.x = index;
			tmp_point.y = start.y;
			hor_line.push_back(tmp_point);
			index--;
		}while(index >= end.x); 	
	}
	return true;


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

bool BresenhamLine(point2d_pix &start, point2d_pix &end, vector<point2d_pix> &point_at_line) 
{  
    int dx = fabs(end.x - start.x);
	int dy = fabs(end.y - start.y);  
    int p = 2 * dy - dx;  
    int twoDy = 2 * dy;
	int twoDyMinusDx = 2 * (dy - dx);
    int x,y;
	
	point2d_pix tmp_point;
	point_at_line.clear();

	if(start.x == end.x)
	{
		HorizontalLine(start, end, point_at_line);		
	}
	else if(start.y == end.y)
	{
		VerticalLine(start, end, point_at_line);
	}
	else	//non vertical and horizontal line
	{
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

		  while(x <= end.x)	
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

	return true;	
  
} 


