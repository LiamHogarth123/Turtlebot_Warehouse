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

  pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",3,false);

// Robot 1 ---------------------------- tb3_0
  sub1_ = nh_.subscribe("tb3_0/odom", 1000, &Method::odomCallback,this);

  sub2_ = nh_.subscribe("tb3_0/scan", 10, &Method::LidaCallback,this);

  // sub3_ = nh_.subscribe("tb3_0/camera/rgb/image_raw", 1000, &Method::RGBCallback, this);

  // sub4_ = nh_.subscribe("tb3_0/camera/depth/image_raw", 1000, &Method::ImageDepthCallback, this);

  cmd_velocity_tb1 = nh_.advertise<geometry_msgs::Twist>("tb3_0/cmd_vel",10);

// Robot 2 guider --------------------- tb3_1

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
      point1.y = 0;
      Leader_goals.push_back(point1);

      geometry_msgs::Point point2;
      point2.x = 3.0;
      point2.y = 0.0;
      Leader_goals.push_back(point2);

      geometry_msgs::Point point3;
      point3.x = 5.0;
      point3.y = 0.0;
      Leader_goals.push_back(point3);

      geometry_msgs::Point point4;
      point4.x = 7.0;
      point4.y = 0.0;
      Leader_goals.push_back(point4);
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
      Lidar.Newdata(updated_Lidar);
      // Lidar.scanningRange(20);
      Lidar.findObstacle();
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
    visualization_msgs::MarkerArray markers;
    visualiseCones(Leader_goals, markers);
    pub_.publish(markers);
    //std::cout << "Size of markers vector: " << markers.markers.size() << std::endl;

    while (!missionComplete){
      turtleMovement();
    }



    // std::vector<std::vector<double>> plots = TurtleGPS.getPlots();
    
    // // Print xPlot
    // std::cout << "xPlot:" << std::endl;
    // for (double value : plots[0]) {
    //     std::cout << value << std::endl;
    // }

    // // Print zPlot
    // std::cout << "zPlot:" << std::endl;
    // for (double value : plots[1]) {
    //     std::cout << value << std::endl;
    // }

    // // Print velPlot
    // std::cout << "velPlot:" << std::endl;
    // for (double value : plots[2]) {
    //     std::cout << value << std::endl;
    // }

  }
}


void Method::turtleMovement(){
    
    // std::cout << Leader_goals.size() << std::endl;
    // std::cout << goal_index << std::endl;
    geometry_msgs::Point targetGoal = Leader_goals.at(goal_index);
    

    TurtleGPS.updateControlParam(targetGoal, Current_Odom, updated_Lidar);
    geometry_msgs::Twist botTraj = TurtleGPS.reachGoal();
    
    if (TurtleGPS.goal_hit(targetGoal, Current_Odom) || goalInObstacleCheck()){ 
        std::cout << "goal hit" << std::endl;
      if (goal_index != Leader_goals.size() - 1){
         
         goal_index++;
         
      } else {
        missionComplete = true;
        botTraj.linear.x = 0;
        botTraj.linear.z = 0;
      }
    } 

    Send_cmd_tb1(botTraj);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

bool Method::goalInObstacleCheck() { // doesnt work if there is only 1 goal and it is inside an object

  geometry_msgs::Point currentGoal = Leader_goals.at(goal_index);
  geometry_msgs::Point currentPos = Current_Odom.pose.pose.position;
  geometry_msgs::Point referenceGoal;

  if (goal_index != Leader_goals.size() - 1) {                              // for cases when there is a next goal
    geometry_msgs::Point referenceGoal = Leader_goals.at(goal_index + 1);
  } else if (Leader_goals.size() != 1) {                                    // for cases when it is the last goal and there is a previous goal
    geometry_msgs::Point referenceGoal = Leader_goals.at(goal_index - 1);
  }

  // cosine rule
  double a = hypot(referenceGoal.x - currentPos.x, referenceGoal.y - currentPos.y); // distance between next goal and bot
  double b = hypot(currentGoal.x - currentPos.x, currentGoal.y - currentPos.y); // distance between current goal and bot
  double c = hypot(referenceGoal.x - currentGoal.x, referenceGoal.y - currentGoal.y); // distance between goals

  double A = (acos((b*b + c*c - a*a) / (2 * b * c))) * 180.0 / M_PI; // cosine rule for angle in degrees

  if (A < 95 && A > 85) {   // checks if the bot is directly next to the current goal with reference to the next goal
    return true;
  }

  return false;
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
  std::unique_lock<std::mutex> lck3 (Lidar_locker);
  updated_Lidar = *Msg;
}

void Method::ImageDepthCallback(const sensor_msgs::Image::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (ImageDepth_locker);
  updated_imageDepth = *Msg;
}

void Method::guiderOdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
  std::unique_lock<std::mutex> lck3 (odom_locker2);
  guider_Odom = *odomMsg;
}


visualization_msgs::MarkerArray Method::visualiseCones(std::vector<geometry_msgs::Point> cones, visualization_msgs::MarkerArray& markerArray) {

    unsigned int ct=0;

    for (auto pt:cones){
        visualization_msgs::Marker marker;

        //We need to set the frame
        // Set the frame ID and time stamp.
        marker.header.frame_id = "odom";
        //single_marker_person.header.stamp = ros::Time();
        marker.header.stamp = ros::Time();
        //We set lifetime (it will dissapear in this many seconds)
        marker.lifetime = ros::Duration(10000); //zero is forever

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "cones"; //This is namespace, markers can be in diofferent namespace  --------- cones , road
        marker.id = ct++; // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

        // The marker type
        marker.type = visualization_msgs::Marker::CYLINDER;

        // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pt.x;
        marker.pose.position.y = pt.y;
        marker.pose.position.z = pt.z + 0.1; //0.1 z-offset given 0.2 z-scale


        //Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;


        // Set the scale of the marker -- 1m side
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.2;

        //Let's send a marker with color (green for reachable, red for now)
        std_msgs::ColorRGBA color;
        color.a=0.7;//a is alpha - transparency 0.5 is 50%;
        color.r=1.0;
        color.g=0;
        color.b=0;

        marker.color = color;

        markerArray.markers.push_back(marker);
    }
    return markerArray;
}

