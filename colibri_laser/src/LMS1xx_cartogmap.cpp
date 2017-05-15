/*
 * LMS1xx.cpp
 *
 *  Created on: 09-08-2010
 *  Author: Konrad Banachowicz
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include <csignal>
#include <cstdio>
#include <colibri_laser/LMS1xx.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define DEG2RAD M_PI/180.0

int main(int argc, char **argv)
{
  // laser data
  LMS1xx laser;
  scanCfg cfg;
  scanOutputRange outputRange;
  scanDataCfg dataCfg;
  sensor_msgs::LaserScan scan_msg;

  sensor_msgs::LaserScan gmapscan_msg;

  // switch frame flag
  int cartoFlag = 1;

  // parameters
  std::string host;
  std::string frame_id;
  
  std::string gmapframe_id;

  ros::init(argc, argv, "lms1xx");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("cartoscan", 1);

  ros::Publisher gmapscan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);

  n.param<std::string>("host", host, "192.168.1.100");
  n.param<std::string>("frame_id", frame_id, "laser_frame");  //cartographer use "laser_frame",

  n.param<std::string>("gmapframe_id", gmapframe_id, "gmaplaser_frame");  //gmap use "gmaplaser_frame",

  ROS_INFO_STREAM(" Testing output... " );

  while (ros::ok())
  {
    ROS_INFO_STREAM("Connecting to laser at " << host);
    laser.connect(host);
    if (!laser.isConnected())
    {
      ROS_WARN("Unable to connect, retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    ROS_INFO_STREAM("Logging in to laser.");
    laser.login();
    cfg = laser.getScanCfg();
	
   ROS_INFO_STREAM("cfg.scaningFrequency =" << cfg.scaningFrequency );
   ROS_INFO_STREAM("cfg.angleResolution=" << cfg.angleResolution);
   ROS_INFO_STREAM("cfg.startAngle  =" << cfg.startAngle);
   ROS_INFO_STREAM("cfg.stopAngle  =" << cfg.stopAngle);
	  
    ROS_INFO_STREAM("Connecting to laser at " << host);	
    outputRange = laser.getScanOutputRange();

   ROS_INFO_STREAM("outputRange.angleResoluton=" << outputRange.angleResolution);
   ROS_INFO_STREAM("outputRange.startAngle  =" << outputRange.startAngle);
   ROS_INFO_STREAM("outputRange.stopAngle	=" << outputRange.stopAngle);


    if (cfg.scaningFrequency != 2500)
    {	  
      laser.disconnect();
      ROS_WARN("Unable to get laser output range. Retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    ROS_INFO("Connected to laser.");

    ROS_DEBUG("Laser configuration: scaningFrequency %d, angleResolution %d, startAngle %d, stopAngle %d",
              cfg.scaningFrequency, cfg.angleResolution, cfg.startAngle, cfg.stopAngle);
    ROS_DEBUG("Laser output range:angleResolution %d, startAngle %d, stopAngle %d",
              outputRange.angleResolution, outputRange.startAngle, outputRange.stopAngle);

    scan_msg.header.frame_id = frame_id;
    scan_msg.range_min = 0.01;
    scan_msg.range_max = 20.0;
    scan_msg.scan_time = 100.0 / cfg.scaningFrequency;
    scan_msg.angle_increment = (double)cfg.angleResolution / 10000.0 * DEG2RAD;
    scan_msg.angle_min = (double)outputRange.startAngle / 10000.0 * DEG2RAD - M_PI / 2;
    scan_msg.angle_max = (double)outputRange.stopAngle / 10000.0 * DEG2RAD - M_PI / 2;

    gmapscan_msg.header.frame_id = gmapframe_id;
    gmapscan_msg.range_min = 0.01;
    gmapscan_msg.range_max = 20.0;
    gmapscan_msg.scan_time = 100.0 / cfg.scaningFrequency;
    gmapscan_msg.angle_increment = (double)cfg.angleResolution / 10000.0 * DEG2RAD;
    gmapscan_msg.angle_min = (double)outputRange.startAngle / 10000.0 * DEG2RAD - M_PI / 2;
    gmapscan_msg.angle_max = (double)outputRange.stopAngle / 10000.0 * DEG2RAD - M_PI / 2;

    ROS_INFO_STREAM("Device resolution is " << (double)outputRange.angleResolution / 10000.0 << " degrees.");
    ROS_INFO_STREAM("Device frequency is " << (double)cfg.scaningFrequency / 100.0 << " Hz");

    int angle_range = outputRange.stopAngle - outputRange.startAngle;
    int num_values = angle_range / cfg.angleResolution ;


	
    if (angle_range % cfg.angleResolution == 0)
    {
      // Include endpoint
      ++num_values;
    }

    ROS_INFO_STREAM("Device num_values is " << num_values);	
    scan_msg.ranges.resize(num_values);
    scan_msg.intensities.resize(num_values);

	gmapscan_msg.ranges.resize(num_values);
    gmapscan_msg.intensities.resize(num_values);

    scan_msg.time_increment =
      (cfg.angleResolution / 10000.0)
      / 360.0
      / (cfg.scaningFrequency / 100.0);

    ROS_INFO_STREAM("Time increment is " << static_cast<int>(scan_msg.time_increment * 1000000) << " microseconds");

    dataCfg.outputChannel = 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.deviceName = false;
    dataCfg.outputInterval = 1;

    //ROS_DEBUG("Setting scan data configuration.");
    //laser.setScanDataCfg(dataCfg);

    ROS_INFO_STREAM("Starting measurements.");
    laser.startMeas();

    ROS_INFO_STREAM("Waiting for ready status.");
    ros::Time ready_status_timeout = ros::Time::now() + ros::Duration(5);

    //while(1)
    //{
    status_t stat = laser.queryStatus();
    ros::Duration(1.0).sleep();
    if (stat != ready_for_measurement)
    {
      ROS_WARN("Laser not ready. Retrying initialization.");
      laser.disconnect();
      ros::Duration(1).sleep();
      continue;
    }
    /*if (stat == ready_for_measurement)
    {
      ROS_DEBUG("Ready status achieved.");
      break;
    }

      if (ros::Time::now() > ready_status_timeout)
      {
        ROS_WARN("Timed out waiting for ready status. Trying again.");
        laser.disconnect();
        continue;
      }

      if (!ros::ok())
      {
        laser.disconnect();
        return 1;
      }
    }*/

    ROS_DEBUG("Starting device.");
    laser.startDevice(); // Log out to properly re-enable system after config

    ROS_INFO_STREAM("Commanding continuous measurements.");
    laser.scanContinous(1);

    while (ros::ok())
    {

      ros::Time start = ros::Time::now();

      scan_msg.header.stamp = start;
      ++scan_msg.header.seq;

      gmapscan_msg.header.stamp = start;
      ++gmapscan_msg.header.seq;	

      scanData data;
      ROS_DEBUG("Reading scan data.");
      if (laser.getScanData(&data))
      {
        for (int i = 0; i < data.dist_len1; i++)
        {
		  scan_msg.ranges[data.dist_len1-1-i] = data.dist1[i] * 0.001;  //built for lms1xxinv_node for cartographer 
		  gmapscan_msg.ranges[data.dist_len1-1-i] = data.dist1[i] * 0.001;  //built for lms1xxinv_node for cartographer 		  
        }

        for (int i = 0; i < data.rssi_len1; i++)
        {
          scan_msg.intensities[i] = data.rssi1[i];
		  gmapscan_msg.intensities[i] = data.rssi1[i];
        }

        ROS_DEBUG("Publishing scan data.");
        scan_pub.publish(scan_msg);
		gmapscan_pub.publish(gmapscan_msg);
      }
      else
      {
        ROS_ERROR("Laser timed out on delivering scan, attempting to reinitialize.");
        break;
      }

      ros::spinOnce();
    }

    laser.scanContinous(0);
    laser.stopMeas();
    laser.disconnect();
  }

  return 0;
}