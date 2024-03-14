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


std::vector<std::pair<geometry_msgs::Point, std::pair<double, double>>>euclideanDistance(const std::vector<geometry_msgs::Point>& itemLocations, const geometry_msgs::Pose& robot1Location, const geometry_msgs::Pose& robot2Location);


  // Parameters for ROS
  ros::NodeHandle nh_;
  
private:

  nav_msgs::Odometry robot1Location;
  nav_msgs::Odometry robot2Location;


};

#endif // SAMPLE_H