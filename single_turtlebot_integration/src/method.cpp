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

  single_marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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
  prmMap.show_Prm();
  std::vector<geometry_msgs::Point> trajectory;


  while (true){

    std::cout << "Enter x-coordinate: ";
    std::cin >> goal.x;

    std::cout << "Enter y-coordinate: ";
    std::cin >> goal.y;
    start.x = Current_Odom.pose.pose.position.x;
    start.y = Current_Odom.pose.pose.position.y; 

    
    trajectory = prmMap.A_star_To_Goal(start, goal);
    publishMarkers(trajectory, marker_pub);
    Leader_goals = trajectory;


    //Dan control start
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    

    int loop_interation = 0;
    while (!missionComplete){
      geometry_msgs::Point targetGoal;
      geometry_msgs::Twist botTraj;
      

      if (loop_interation < 1) {
        targetGoal = Leader_goals.at(loop_interation+1);
        TurtleGPS.updateGoal(targetGoal, Current_Odom);
        botTraj = TurtleGPS.reachGoal();

        if (TurtleGPS.goal_hit(targetGoal, Current_Odom)){
          loop_interation++;  
        }
      }
      else {
        targetGoal = findLookAheadPoint(Leader_goals, Current_Odom.pose.pose.position, 0.25);

        TurtleGPS.updateGoal(targetGoal, Current_Odom);
        botTraj = TurtleGPS.reachGoal();
      
        if (TurtleGPS.goal_hit(Leader_goals.back(), Current_Odom)){
          missionComplete = true;
        }
      
      }
      Send_cmd_tb1(botTraj);
    
      std::cout << "Look-Ahead Point: (" << targetGoal.x << ", " << targetGoal.y << ")" << std::endl;
      publishLookAheadMarker(targetGoal);
      
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
      
    missionComplete = false;
    goal_index = 0;

    
  }
}


geometry_msgs::Point Method::findLookAheadPoint(const std::vector<geometry_msgs::Point>& path, const geometry_msgs::Point& current_position, double look_ahead_distance) {
    double cumulative_distance = 0.0;
    double closest_goal = 9999999;
    size_t current_gaol_id;
    geometry_msgs::Point look_ahead_point = path[0];

    for (size_t j = 0; j< path.size(); j++){

      double temp = sqrt(pow(current_position.x - path[j].x, 2) + pow(current_position.y - path[j].y, 2));
      if (temp < closest_goal){
        look_ahead_point = path[j];
        current_gaol_id = j;
        closest_goal = temp;
      }
    }



  for (size_t i = current_gaol_id; i < path.size() - 1; ++i) {
    double segment_length = sqrt(pow(path[i+1].x - path[i].x, 2) + pow(path[i+1].y - path[i].y, 2));
    cumulative_distance += segment_length;

    if (cumulative_distance >= look_ahead_distance) {
      double overshoot = cumulative_distance - look_ahead_distance;
      double ratio = (segment_length - overshoot) / segment_length;
      look_ahead_point.x = path[i].x + ratio * (path[i+1].x - path[i].x);
      look_ahead_point.y = path[i].y + ratio * (path[i+1].y - path[i].y);
      return look_ahead_point;
    }
  }

  return path.back();

}



void Method::turtleMovement(){

  
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


void Method::publishLookAheadMarker(const geometry_msgs::Point& look_ahead_point) {
  visualization_msgs::Marker marker;

  marker.header.frame_id = "odom"; // or "map" or the appropriate frame
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = look_ahead_point.x;
  marker.pose.position.y = look_ahead_point.y;
  marker.pose.position.z = 0; // Set z-coordinate if necessary
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.lifetime = ros::Duration();

  single_marker_pub_.publish(marker);
}