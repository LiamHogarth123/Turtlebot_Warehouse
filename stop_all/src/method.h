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
#include <ros/package.h>


#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include <geometry_msgs/Point.h>




class Method
{
public:

  Method(ros::NodeHandle nh);

  // Prameters for ROS
  ros::NodeHandle nh_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher cmd_vel_pub_1;
  ros::Publisher cmd_vel_pub_2;
  void separateThread();

};

  



#endif // SAMPLE_H