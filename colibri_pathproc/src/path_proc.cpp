#include "path_proc.h"

void PathProc::PathProc()
{

}

void PathProc::~PathProc()
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


vector HorizontalLine(point2d<int> &start, point2d<int> &end)
{
	vector<point2d<int> > hor_line;
	point2d<int> tmp_point;

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
	return hor_line;


}

vector VerticalLine(point2d<int> &start, point2d<int> &end)
{
	vector<point2d<int> > ver_line;
	point2d<int> tmp_point;

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

	return ver_line;

}

vector BresenhamLine(point2d<int> & start, point2d<int> & end);  
{  
    int dx = fabs(end.x - start.x);
	int dy = fabs(end.y - start.y);  
    int p = 2 * dy - dx;  
    int twoDy = 2 * dy;
	int twoDyMinusDx = 2 * (dy - dx);
    int x,y;
	vector<point2d<int> > point_at_line;
	point2d<int> tmp_point;

	if(start.x == end.x)
	{
	 	point_at_line = HorizontalLine(start, end);		
	}
	else if(start.y == end.y)
	{
		point_at_line = VerticalLine(start, end);
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

	return point_at_line;	
  
} 


