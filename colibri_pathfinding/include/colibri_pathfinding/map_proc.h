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

vector<smpix_point> Smooth5p3t(vector<pix_point> &input);

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
		bool ImgPix2NavPos(smpix_point & pix, map_point & position);
		bool NavPos2ImgPix(map_point & position, pix_point & pix);
		
		bool PixNodes2NavPath(vector<pix_point> & nav_nodes, vector<map_point> &nav_path);
		bool PixNodes2NavPath(vector<smpix_point> & smnav_nodes, vector<map_point> &nav_path);

		void NavPath2PixNodes();
		bool PubNavPath(vector<map_point> &nav_path);

		bool LocalMapUpdate(void); //TODO
		bool LoadGoalFromTask(void); //TODO

		void ParseMapOrigin(void);

		bool PixBoundCheck(pix_point & pix);

	private:
		
		void ObtainRvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& rviz_goal);
		void Quaternion2Yaw(const geometry_msgs::PoseStamped &pose, float &yaw);


};


#endif
