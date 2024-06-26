
#include "method.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <nav_msgs/Odometry.h>

#include <visualization_msgs/MarkerArray.h>

#include "ros/ros.h"
#include <fstream>
#include <chrono>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"






Method::Method(ros::NodeHandle nh) :
  nh_(nh)

{
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  cmd_vel_pub_1 = nh_.advertise<geometry_msgs::Twist>("tb3_0/cmd_vel", 10);
  cmd_vel_pub_2 = nh_.advertise<geometry_msgs::Twist>("tb3_1/cmd_vel", 10);  
  
}

 void Method::separateThread(){
  while (true){
   geometry_msgs::Twist stop_cmd;
  stop_cmd.linear.x = 0.0;
  stop_cmd.linear.y = 0.0;
  stop_cmd.linear.z = 0.0;
  stop_cmd.angular.x = 0.0;
  stop_cmd.angular.y = 0.0;
  stop_cmd.angular.z = 0.0;

  cmd_vel_pub_.publish(stop_cmd);
  cmd_vel_pub_1.publish(stop_cmd);
  cmd_vel_pub_2.publish(stop_cmd);
  }
}

