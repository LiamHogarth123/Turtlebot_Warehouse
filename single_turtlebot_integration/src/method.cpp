#include "method.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include <fstream>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"



Method::Method(ros::NodeHandle nh) :
  nh_(nh)

{

  teleop_mode = false;

  goal_index = 0;

  missionComplete = false;

  pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",3,false);

// Robot 1 -----------------------------------------------------
  sub1_ = nh_.subscribe("/odom", 1000, &Method::odomCallback,this);

  sub2_ = nh_.subscribe("/scan", 10, &Method::LidaCallback,this);

  sub3_ = nh_.subscribe("/camera/rgb/image_raw", 1000, &Method::RGBCallback, this);

  sub4_ = nh_.subscribe("/camera/depth/image_raw", 1000, &Method::ImageDepthCallback, this);

  cmd_velocity_tb1 = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",10);

  // Robot 2 guider ---------------------

  sub5_ = nh_.subscribe("tb3_1/odom", 1000, &Method::guiderOdomCallback,this);

  cmd_velocity_tb2 = nh.advertise<geometry_msgs::Twist>("tb3_1/cmd_vel",10);


  // Map ros topics
  mapSub = nh_.subscribe("/map", 1000, &Method::mapCallback, this);
  
  mapMetadataSub = nh_.subscribe("/map_metadata", 1000, &Method::mapMetadataCallback, this);

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);

  
  
}

void Method::separateThread() {
    while (latestMapData_.data.empty()){
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  
  //Brendan task allocation
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  geometry_msgs::Point goal;

  geometry_msgs::Point start;

  //Liam Map generation
  /////////////////////////////////////////////////////////////////////////
  prmMap.GeneratePRM(latestMapData_, latestMapMetaData_);
  std::vector<geometry_msgs::Point> trajectory;


  while (true){

  std::cout << "Enter x-coordinate: ";
  std::cin >> goal.x;

  std::cout << "Enter y-coordinate: ";
  std::cin >> goal.y;
  start.x = Current_Odom.pose.pose.position.x;
  start.y = Current_Odom.pose.pose.position.y; 

  
  trajectory = prmMap.DijkstraToGoal(start, goal);
  publishMarkers(trajectory, marker_pub);
  Leader_goals = trajectory;


  //Dan control start
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  


    while (!missionComplete){
      turtleMovement();
    }
  missionComplete = false;
  goal_index = 0;
  // GET TO FINAL GOAL
  //////////////////////////////////////////////////////////
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

void Method::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  ROS_INFO("Received map with resolution %f meters/pixel", msg->info.resolution);
  std::unique_lock<std::mutex> lck3 (MapData_Lock);
  latestMapData_ = *msg;
}

void Method::mapMetadataCallback(const nav_msgs::MapMetaData::ConstPtr& msg) {
  ROS_INFO("Received map metadata. Width: %d, Height: %d", msg->width, msg->height);
  std::unique_lock<std::mutex> lck3 (MapMetaData_Lock);
  latestMapMetaData_ = *msg;  
}



visualization_msgs::MarkerArray Method::visualiseCones(std::vector<geometry_msgs::Point> cones, visualization_msgs::MarkerArray& markerArray) {

  return markerArray;
}

void Method::publishMarkers(const std::vector<geometry_msgs::Point>& nodes, ros::Publisher& marker_pub) {
  visualization_msgs::MarkerArray markerArray;
  int id = 0; // Unique ID for each marker

  for (const auto& node : nodes) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "odom"; // or your relevant frame
      marker.header.stamp = ros::Time::now();
      marker.ns = "nodes";
      marker.id = id++; // Assign and increment the unique ID
      marker.type = visualization_msgs::Marker::SPHERE; // Use SPHERE, CUBE, etc., as preferred
      marker.action = visualization_msgs::Marker::ADD;
      
      marker.pose.position.x = node.x;
      marker.pose.position.y = node.y;
      marker.pose.position.z = 0; // Assuming a flat map, set z to 0
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.2; // Specify the size of the individual markers
      marker.scale.y = 0.2;
      marker.scale.z = 0.2; // Add z dimension for SPHERE, CUBE, etc.

      marker.color.r = 1.0; // Color: Red
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0; // Alpha (transparency)

      markerArray.markers.push_back(marker); // Add the marker to the array
      std::cout << "marker created" << std::endl;
      std::cout << "tesrt" << std::endl;
      std::cout << "tesrt" << std::endl;  
  }
  
  marker_pub.publish(markerArray); // Publish the entire array
}

// #include "method.h"
// #include <iostream>
// #include <cmath>
// #include <thread>
// #include <chrono>
// #include <time.h>
// #include <nav_msgs/Odometry.h>

// #include <visualization_msgs/MarkerArray.h>

// #include "ros/ros.h"
// #include <fstream>
// #include <chrono>

// #include "nav_msgs/OccupancyGrid.h"
// #include "nav_msgs/MapMetaData.h"






// Method::Method(ros::NodeHandle nh) :
//   nh_(nh)

// {
  
//   mapSub = nh_.subscribe("/map", 1000, &Method::mapCallback, this);
  
//   mapMetadataSub = nh_.subscribe("/map_metadata", 1000, &Method::mapMetadataCallback, this);

//   marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);

  
  
// }

// void Method::testPathPlanningControl() {
  

//   taskAllo

//   prmMap.GeneratePRM(latestMapData_, latestMapMetaData_);
  
//   std::vector<geometry_msgs::Point> trajectory;
//   geometry_msgs::Point start;
//   geometry_msgs::Point goal;

//   start.x = 1;
//   start.y = 1;
//   goal.x = 5;
//   goal.y = 5;

//   trajectory = prmMap.DijkstraToGoal(start, goal);

//   //SEND TO CONTROL

// }




// void Method::seperateThread() {
//   //User input
//   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// // Load the PGM map

//   //FIX calback
//   while (latestMapData_.data.empty()){
//     std::this_thread::sleep_for(std::chrono::milliseconds(1));
//   }


//   std::cout << "openning" << std::endl;
 
//   prmMap.UpdateMapData(latestMapData_, latestMapMetaData_);
//   std::vector<geometry_msgs::Point> trajectory;
//   trajectory = prmMap.test();
//   std::cout << "PRM FINISH" << std::endl;
//   geometry_msgs::Point temp;
//   temp.x = 0;z
//   temp.y = 0;
//   trajectory.push_back(temp);
//   publishMarkers(trajectory, marker_pub);
//   std::cout << "visualisation DONE" << std::endl;

// }







// void Method::publishMarkers(const std::vector<geometry_msgs::Point>& nodes, ros::Publisher& marker_pub) {
//     visualization_msgs::MarkerArray markerArray;
//     int id = 0; // Unique ID for each marker

//     for (const auto& node : nodes) {
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = "map"; // or your relevant frame
//         marker.header.stamp = ros::Time::now();
//         marker.ns = "nodes";
//         marker.id = id++; // Assign and increment the unique ID
//         marker.type = visualization_msgs::Marker::SPHERE; // Use SPHERE, CUBE, etc., as preferred
//         marker.action = visualization_msgs::Marker::ADD;
        
//         marker.pose.position.x = node.x;
//         marker.pose.position.y = node.y;
//         marker.pose.position.z = 0; // Assuming a flat map, set z to 0
//         marker.pose.orientation.w = 1.0;

//         marker.scale.x = 0.2; // Specify the size of the individual markers
//         marker.scale.y = 0.2;
//         marker.scale.z = 0.2; // Add z dimension for SPHERE, CUBE, etc.

//         marker.color.r = 1.0; // Color: Red
//         marker.color.g = 0.0;
//         marker.color.b = 0.0;
//         marker.color.a = 1.0; // Alpha (transparency)

//         markerArray.markers.push_back(marker); // Add the marker to the array
//     }

//     marker_pub.publish(markerArray); // Publish the entire array
// }