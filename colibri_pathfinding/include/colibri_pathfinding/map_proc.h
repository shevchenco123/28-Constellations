#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h" 

#include <vector>

#ifndef _MAP_PROC_H_
#define _MAP_PROC_H_

extern int world_map[];
extern int MAP_WIDTH;
extern int MAP_HEIGHT;

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

		pix_point start;
		pix_point terminal;
		pix_point revised_terminal;

		vector

		map_proc();
		~map_proc();
		
		bool CalcGoalEdgePoint();
		bool DilatatingMapImg();
		
		void ImgPix2NavPos();
		void NavPos2ImgPix();
		
		void PixNodes2NavPath();
		void NavPath2PixNodes();

		bool LocalMapUpdate();

	private:
		
		void ObtainRvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& rviz_goal);
		void Quaternion2Yaw(const geometry_msgs::PoseStamped &pose, float &yaw);


};



// subscribe the goal info
// pub the topic path
// read the task yaml
// map inflaion for path planning
// point pix <  --- > phiscal pos  
// path pix (smooth or not)  < ---> phisical path
// goal edge adjustment

//local  map data update for new plan

#endif
