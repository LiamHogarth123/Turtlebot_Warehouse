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

//Keep only the headers needed



#include <ros/package.h>





class Method
{
public:

  Method(ros::NodeHandle nh);

  void seperateThread();

  



  // Prameters for ROS
  ros::NodeHandle nh_;
  



};

  



#endif // SAMPLE_H