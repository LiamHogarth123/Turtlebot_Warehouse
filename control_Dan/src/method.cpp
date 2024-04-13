#include "method.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include <fstream>


Method::Method(ros::NodeHandle nh) :
  nh_(nh)

{

 teleop_mode = false;

 goal_index = 0;

 missionComplete = false;

// Robot 1 -----------------------------------------------------
  sub1_ = nh_.subscribe("/odom", 1000, &Method::odomCallback,this);

  sub2_ = nh_.subscribe("/scan", 10, &Method::LidaCallback,this);

  sub3_ = nh_.subscribe("/camera/rgb/image_raw", 1000, &Method::RGBCallback, this);

  sub4_ = nh_.subscribe("/camera/depth/image_raw", 1000, &Method::ImageDepthCallback, this);

  cmd_velocity_tb1 = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",10);

  // Robot 2 guider ---------------------

  sub5_ = nh_.subscribe("tb3_1/odom", 1000, &Method::guiderOdomCallback,this);

  cmd_velocity_tb2 = nh.advertise<geometry_msgs::Twist>("tb3_1/cmd_vel",10);


  
}

void Method::separateThread() {
  //User input
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::string userInput;
  std::cout << "Please enter control method"<< std::endl;
  std::cout << "1. use provide goals"<< std::endl;
  std::cout << "2. provide a goal"<< std::endl;
  std::cout << "3. control the robot with user input"<< std::endl;
  std::cin >> userInput;
  int input_int = std::stoi(userInput);

  switch (input_int) {
    case 1: {
      geometry_msgs::Point point1;
      point1.x = 1.0;
      point1.y = -0.5;
      Leader_goals.push_back(point1);

      geometry_msgs::Point point2;
      point2.x = 3.0;
      point2.y = 0.0;
      Leader_goals.push_back(point2);
      break;
      }
    case 2:{
      Leader_goals.clear();
      geometry_msgs::Point temp;
      std::string userInput; // Assuming userInput is declared as string
      std::cout << "enter x"<< std::endl;
      std::cin >> userInput;
      temp.x = std::stod(userInput); // Convert string to double
      std::cout << "enter y"<< std::endl;
      std::cin >> userInput;
      temp.y = std::stod(userInput); // Convert string to double
      Leader_goals.push_back(temp);
      break;
      }
    case 3:{
      teleop_mode = true;
      break;
      }
    default:{
    std::cout << "invalid input - therefore using default goals"<< std::endl;
    }
  }

  
  //Code start
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  

  if(teleop_mode){
    while (true){
      
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

  }
  else{
    while (!missionComplete){
      turtleMovement();

      
    }
  }
}




void Method::turtleMovement(){

    geometry_msgs::Point targetGoal;
    
    // std::cout << Leader_goals.size() << std::endl;
    // std::cout << goal_index << std::endl;
    targetGoal = Leader_goals.at(goal_index);
    

    TurtleGPS.updateGoal(targetGoal, Current_Odom);
    geometry_msgs::Twist botTraj = TurtleGPS.reachGoal();

    // Lidar.Newdata(updated_Lida);
    // double x = Lidar.findTurtlebot();
    
    if (TurtleGPS.goal_hit(targetGoal, Current_Odom)){
        std::cout << "goal hit" << std::endl;
      if (goal_index != Leader_goals.size() - 1){
         
         goal_index++;
         
      }
      else{
        missionComplete = true;
      }
    }
    

    Send_cmd_tb1(botTraj);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
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




