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


class TaskAlloction
{
public:

  TaskAlloction();

  std::vector<std::vector<geometry_msgs::Point>> taskAllocation();

  double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
  
  // Parameters for ROS
 

  //double greedySort();

  
private:
  // assign x and y values for the item locations
  std::vector<geometry_msgs::Point> itemLocations;




};

#endif // SAMPLE_H
