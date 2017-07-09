#include "map_proc.h"

map_proc::map_proc()
{
	cur_goal[3] = {0.0, 0.0, 0.0};		
	obtain_goal_flag = false;
	
	start.x = 1;
	start.y = 1;
	terminal.x = 1;
	terminal.y = 1;
	revised_terminal.x = terminal.x;
	revised_terminal.y = terminal.y;
	
	nh_img.param("origin", map_origin, [-1.0, -1.0, 0]);
	nh_img.param("resolution", map_resol, 0.05);

	pub4path = nh_img.advertise<nav_msgs::Path>("/nav_path", 1);

	map_image = imread("/home/colibri/clbri_ws/src/colibri_pathfinding/maps/626_mdf.pgm", CV_LOAD_IMAGE_COLOR);
	if(map_image.empty())
	{
	 	cout<<"open existed map error!"<<endl;
	}

	structe_element = cv::getStructuringElement( DILATION_TYPE, Size( 2*DILATION_SIZE + 1, 2*DILATION_SIZE+1 ),
										 		Point( DILATION_SIZE, DILATION_SIZE ) );
	
}

map_proc::~map_proc()
{

}

bool map_proc::CalcGoalEdgePoint(pix_point & end, pix_point & revised_end)
{

	if(world_map[end.y * MAP_WIDTH + end.x] < 255) // if the set goal is not in black area
	{
		revised_end.x = end.x;
		revised_end.y = end.y;	
	}
	else
	{
		for(int i = 1; i <= GOAL_EDGE_MAX; i++)	// should conce
		{
			if(world_map[end.y * MAP_WIDTH + end.x + i] < 255)
			{
				revised_end.x = end.x + i;
				revised_end.y = end.y;	

				break;
			}
			if(world_map[(end.y - i) * MAP_WIDTH + end.x] < 255)
			{
				revised_end.x = end.x ;
				revised_end.y = end.y - i;	

				break;
			}

			if(world_map[end.y * MAP_WIDTH + end.x - i] < 255)
			{
				revised_end.x = end.x - i;
				revised_end.y = end.y;	

				break;
			}

			if(world_map[(end.y + i) *MAP_WIDTH + end.x] < 255)
			{
				revised_end.x = end.x;
				revised_end.y = end.y + i;

				break;
			}
			
			if( GOAL_EDGE_MAX == i)
			{
				cout <<"No nearest edge point to approx the goal!"<<endl;
				revised_end.x = end.x;
				revised_end.y = end.y;

				return false;
			}

		}
	}

	return true;

}


bool map_proc::SearchMapPreProc(void)
{
	erode( map_image, dilation_img, structe_element );		
	cvtColor(dilation_img, gray_img, CV_BGR2GRAY);
	msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", gray_img).toImageMsg();

	MAP_WIDTH = msg->width;
	MAP_HEIGHT = msg->height;

	for (int pix_index = 0; pix_index < MAP_WIDTH*MAP_HEIGHT; pix_index++)
	{
		world_map[pix_index] = 255 - msg->data[pix_index];

	}

}

bool map_proc::ImgPix2NavPos(pix_point & pix, map_point & position)
{
	bool transfer_flag = false;
	transfer_flag = PixBoundCheck(pix);
	if(false == transfer_flag)
	{
		return false;
	}
	else
	{
		position.x = map_origin[0] + map_resol * pix.x;
		position.y = map_origin[1] + (MAP_HEIGHT - pix.y) * map_resol;
		position.yaw = 0.0;
		return true;
	}
	
}

bool map_proc::NavPos2ImgPix(map_point & position, pix_point & pix)
{
	bool transfer_flag = false;

	pix.x = floor((position.x - map_origin[0]) / map_resol);
	pix.y = floor(MAP_HEIGHT - ((position.x - map_origin[1]) / map_resol));

	transfer_flag = PixBoundCheck(pix);

	return transfer_flag;
}

bool map_proc::PixNodes2NavPath(vector<pix_point> & nav_nodes, vector<map_point> &nav_path)
{
	map_point tmp_pos;
    for (auto iter = nav_nodes.begin(); iter != nav_nodes.end(); iter++)
    {

		if(ImgPix2NavPos(*iter, tmp_pos))
		{
			nav_path.push_back(tmp_pos);
		}
		else
		{
			return false;
		}
		
    }
	return true;
}

void map_proc::NavPath2PixNodes()
{

}

bool map_proc::LocalMapUpdate()
{

}

bool map_proc::LoadGoalFromTask()
{

}

void map_proc::PubNavPath(vector<map_point> &nav_path)
{
	
}

void map_proc::ObtainRvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& rviz_goal)
{

	float yaw = 0.0;
	
	cur_goal[0] = rviz_goal->pose.position.x;
	cur_goal[1] = rviz_goal->pose.position.y;
	
	Quaternion2Yaw(*rviz_goal, yaw);
	cur_goal[2] = yaw;

	obtain_goal_flag = true;

}

void map_proc::Quaternion2Yaw(const geometry_msgs::PoseStamped &pose, float &yaw)
{
	float x = 0.0;
	float y = 0.0;
	
	y = 2.0 * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y);
	x = 1.0 - 2 * (pow(pose.pose.orientation.y, 2) + pow(pose.pose.orientation.z, 2));
	yaw = atan2(y,x) * RAD2DEG;

}

bool map_proc::PixBoundCheck(pix_point & pix)
{

	if(pix.x < 1 || pix.x > MAP_WIDTH || pix.y < 1 || pix.y > MAP_HEIGHT)
	{
		cout<<"The pix u/v in Img is out of the img itself"<<endl;
		return false;
	}

}



