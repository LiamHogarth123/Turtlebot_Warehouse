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
  // laserProcessingPtr_(nullptr)
{
//Ros construction
//Subscribing to TurtleBot3 ROS features



// Robot 1 -----------------------------------------------------
  sub1_ = nh_.subscribe("tb3/odom", 1000, &Method::odomCallback,this);

  sub2_ = nh_.subscribe("tb3/scan", 10, &Method::LidaCallback,this);

  sub3_ = nh_.subscribe("tb3/camera/rgb/image_raw", 1000, &Method::RGBCallback, this);

  sub4_ = nh_.subscribe("tb3/camera/depth/image_raw", 1000, &Method::ImageDepthCallback, this);

  cmd_velocity_tb1 = nh_.advertise<geometry_msgs::Twist>("tb3/cmd_vel",10);

  // Robot 2 guider ---------------------

  sub5_ = nh_.subscribe("tb3_1/odom", 1000, &Method::guiderOdomCallback,this);

  cmd_velocity_tb2 = nh.advertise<geometry_msgs::Twist>("tb3_1/cmd_vel",10);


  
}

/**
 * @brief Control the robots based on user input and threading preferences.
 *
 * This function allows the user to specify the control method for the robots and whether multi-threading
 * should be used. It provides options for reading goals, providing a single goal, or manually controlling
 * the guider robot. Based on the user's input, it configures the control method and threading accordingly.

 * The function starts by presenting the user with a menu to select the desired control method. The available
 * options are:
 * 1. Use provided goals (readGoal function).
 * 2. Provide a single goal.
 * 3. Control the guider robot with user input (teleoperation mode).

 * Depending on the selected option, the function configures the control method accordingly. If multi-threading
 * is chosen, it runs 'multiThread' for concurrent control. In teleoperation mode, it spawns a thread to run
 * 'followingRobotThread' and a separate thread for teleoperation. In other cases, it runs 'singleThread' to
 * control the robots sequentially. User preferences for multi-threading are considered when setting 'Threading_switch'.

 * @note This function serves as the entry point for controlling the robots, allowing users to specify the
 * control method and threading settings.
 */
void Method::seperateThread() {
  //User input
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
  std::cout << "Hellow World" << std::endl;
  geometry_msgs::Twist speed_Command;
  speed_Command.linear.x = 0.5;
  speed_Command.angular.x = 0.1;

}


// Publishing functions 
///////////////////////////////////////////////////////////////////////////////////////////
void Method::Send_cmd_tb1(geometry_msgs::Twist intructions){
  cmd_velocity_tb1.publish(intructions);
}

void Method::Send_cmd_tb2(geometry_msgs::Twist intructions){
  cmd_velocity_tb2.publish(intructions);
}



//callbacks
///////////////////////////////////////////////////////////////////////////////////////////////

void Method::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
  std::unique_lock<std::mutex> lck3 (odom_locker);
  Current_Odom = *odomMsg;
}

void  Method::RGBCallback(const sensor_msgs::Image::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (RGB_locker);
  updated_RGB = *Msg;

}

void Method::LidaCallback(const sensor_msgs::LaserScan::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (Lida_locker);
  updated_Lida = *Msg;
}

void Method::ImageDepthCallback(const sensor_msgs::Image::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (ImageDepth_locker);
  updated_imageDepth = *Msg;
}

void Method::guiderOdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
  std::unique_lock<std::mutex> lck3 (odom_locker2);
  guider_Odom = *odomMsg;
}




