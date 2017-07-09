#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h" 

#include <vector>
#include <math>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

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

typedef struct st_pix_point
{
	int x;
	int y;

}pix_point;

typedef struct st_map_point
{
	float x;
	float y;
	float yaw;
	
}map_point;

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

		float map_origin[3];
		float map_resol;

		vector<map_point> nav_path;

		map_proc();
		~map_proc();
		
		bool CalcGoalEdgePoint(pix_point & end, pix_point & revised_end);
		bool SearchMapPreProc(void);
		
		bool ImgPix2NavPos(pix_point & pix, map_point & position);
		bool NavPos2ImgPix(map_point & position, pix_point & pix);
		
		void PixNodes2NavPath();
		void NavPath2PixNodes();
		void PubNavPath();

		bool LocalMapUpdate(); //TODO
		bool LoadGoalFromTask(); //TODO


	private:
		
		void ObtainRvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& rviz_goal);
		void Quaternion2Yaw(const geometry_msgs::PoseStamped &pose, float &yaw);

		bool PixBoundCheck(pix_point & pix);


};


#endif
