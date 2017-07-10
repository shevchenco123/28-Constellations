#include <string>
#include <vector>
#include <cmath>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h" 

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

#ifndef _MAP_PROC_H_
#define _MAP_PROC_H_

extern int world_map[];
extern int MAP_WIDTH;
extern int MAP_HEIGHT;

#define DILATION_TYPE MORPH_ELLIPSE
#define DILATION_SIZE 7
#define GOAL_EDGE_MAX 15

#define RAD2DEG 57.296

typedef struct st_pix_point
{
	int x;
	int y;

}pix_point;

typedef struct st_smpix_point
{
	float x;
	float y;
	
}smpix_point;

typedef struct st_map_point
{
	float x;
	float y;
	float yaw;
	
}map_point;

template<typename T1, typename T2>
vector<T1> Smooth5p3t(const vector<T2> input, const int iter)
{
	vector<T1> output;
	vector<T1> mid_val(input);
	T1 tmp_val;
	size_t len = input.size();
	if(len < 5)
	{
		cout<<"can not exec the 5 points 3 time smooth filter!"<<endl;
		output.swap(input);
		return output;
	}
	
	for(int i = 0; i < iter; i++)
	{
		tmp_val = (69 * mid_val[0] + 4 * (mid_val[1] + mid_val[3]) - 6 * mid_val[2] - mid_val[4]) / 70.0;
		output.push_back(tmp_val);
		tmp_val = (2 * (mid_val[0] + mid_val[4]) + 27 * mid_val[1] + 12 * mid_val[2] - 8 * mid_val[3] ) / 35.0;
		output.push_back(tmp_val);

		for(size_t j = 2; j < (len - 2); j++)
		{
			tmp_val = (-3 * (mid_val[j-2] + mid_val[j + 2]) + 12 * (mid_val[j -1] + mid_val[j + 1]) + 17 * mid_val[j]) / 35.0;
			output.push_back(tmp_val);
		}
		
		tmp_val = (2 * (mid_val[len - 1] + mid_val[len - 5]) + 27 * mid_val[len - 2] + 12 * mid_val[len - 3] - 8 * mid_val[len - 4]) / 35.0;
		output.push_back(tmp_val);
		tmp_val = (69 * mid_val[len -1] + 4 * (mid_val[len - 2] + mid_val[len - 4]) - 6 * mid_val[len - 3] - mid_val[len - 4]) / 70.0;
		output.push_back(tmp_val);

		mid_val.swap(output);

	}

	return output;
}



class map_proc
{
	public :
		ros::NodeHandle nh_img;
		ros::Publisher pub4path;

		Mat map_image;
		Mat dilation_img;  
		Mat structe_element;
		Mat gray_img;
		
		sensor_msgs::ImagePtr msg;

		float cur_goal[3];		
		bool obtain_goal_flag;
		
		pix_point start;
		pix_point terminal;
		pix_point revised_terminal;

		string str_origin;
		float map_origin[3];
		float map_resol;

		vector<pix_point> nav_nodes;
		vector<map_point> nav_path;
		nav_msgs::Path plan_path;

		map_proc();
		~map_proc();
		
		bool CalcGoalEdgePoint(pix_point & end, pix_point & revised_end);
		bool SearchMapPreProc(void);
		
		bool ImgPix2NavPos(pix_point & pix, map_point & position);
		bool NavPos2ImgPix(map_point & position, pix_point & pix);
		
		bool PixNodes2NavPath(vector<pix_point> & nav_nodes, vector<map_point> &nav_path);

		void NavPath2PixNodes();
		bool PubNavPath(vector<map_point> &nav_path);

		bool LocalMapUpdate(void); //TODO
		bool LoadGoalFromTask(void); //TODO

		bool ParseMapOrigin(void);


	private:
		
		void ObtainRvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& rviz_goal);
		void Quaternion2Yaw(const geometry_msgs::PoseStamped &pose, float &yaw);

		bool PixBoundCheck(pix_point & pix);


};


#endif
