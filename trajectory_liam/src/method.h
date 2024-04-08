/**
 @file sample.h
 @brief Header file for the Sample class.
*/

#ifndef SAMPLE_H
#define SAMPLE_H

#include "ros/ros.h"
#include <atomic>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include "readMap.h"
#include <ros/package.h>
#include "opencv2/opencv.hpp"
#include "prm.h"
#include <visualization_msgs/Marker.h>


#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include <geometry_msgs/Point.h>




class Method
{
public:

  Method(ros::NodeHandle nh);

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void mapMetadataCallback(const nav_msgs::MapMetaData::ConstPtr& msg);

  
  void testPathPlanningControl();  


  void seperateThread();


  void publishMarkers(const std::vector<geometry_msgs::Point>& nodes, ros::Publisher& marker_pub);

  // Prameters for ROS
  ros::NodeHandle nh_;
  ros::Subscriber map_sub;
  ros::Subscriber mapSub;
  ros::Subscriber mapMetadataSub;
  ros::Publisher marker_pub;
  
  readMap mapReader;

  PRM prmMap; 

  nav_msgs::OccupancyGrid latestMapData_;
  nav_msgs::MapMetaData latestMapMetaData_;



  std::mutex MapData_Lock;
  std::mutex MapMetaData_Lock;

};

  



#endif // SAMPLE_H