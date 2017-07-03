#include <ros/ros.h>
#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

static const std::string INPUT = "Input";
static const std::string OUTPUT = "Output";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  Mat image = imread("/home/colibri/626_mdf.pgm", CV_LOAD_IMAGE_COLOR);
  namedWindow(INPUT, CV_WINDOW_AUTOSIZE); 
  imshow(INPUT, image); 

  namedWindow(OUTPUT, CV_WINDOW_AUTOSIZE); 
  Mat dilation_dst;  


  int dilation_type = MORPH_ELLIPSE;
  int dilation_size = 7; 
  Mat element = cv::getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  erode( image, dilation_dst, element );
  //cv::cvtColor(image, img_out, CV_RGB2GRAY);  //转换成灰度图象  
  imshow(OUTPUT, dilation_dst); 


  imwrite( "/home/colibri/Gray_Image.pgm", dilation_dst);

  if(image.empty()){
   printf("open error\n");
   }
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dilation_dst).toImageMsg();

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
 
    imshow(INPUT, image);
    imshow(OUTPUT, dilation_dst);  
    waitKey(1000);

    loop_rate.sleep();
  }

  destroyWindow(INPUT);  
  destroyWindow(OUTPUT);  

  return 0;
}
