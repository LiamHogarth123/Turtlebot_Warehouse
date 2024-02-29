#include "method.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <nav_msgs/Odometry.h>

// #include <kobuki_msgs/DigitalOutput.h>
#include "ros/ros.h"

#include <fstream>



using std::cout;
using std::endl;

Method::Method(ros::NodeHandle nh) :
  nh_(nh)

{

  
}

void Method::seperateThread() {
  //User input
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
  std::cout << "Hellow World" << std::endl;
  geometry_msgs::Twist speed_Command;
  speed_Command.linear.x = 0.5;
  speed_Command.angular.x = 0.1;

}



