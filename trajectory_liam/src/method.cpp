
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
  
  mapSub = nh_.subscribe("/map", 1000, &Method::mapCallback, this);
  
  mapMetadataSub = nh_.subscribe("/map_metadata", 1000, &Method::mapMetadataCallback, this);

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);

  
  
}

void Method::testPathPlanningControl() {
  prmMap.GeneratePRM(latestMapData_, latestMapMetaData_, false);
  
  std::vector<geometry_msgs::Point> trajectory;
  geometry_msgs::Point start;
  geometry_msgs::Point goal;

  start.x = 1;
  start.y = 1;
  goal.x = 5;
  goal.y = 5;

  trajectory = prmMap.DijkstraToGoal(start, goal);

  //SEND TO CONTROL



}


// // Ideal system

// generate PRM
// initilise brendan system 

// Using function setgoal()
// using funfction djsktra to goal




void Method::seperateThread() {
  //User input
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Load the PGM map

  //FIX calback
  while (latestMapData_.data.empty()){
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }


  std::cout << "openning" << std::endl;
 
  prmMap.GeneratePRM(latestMapData_, latestMapMetaData_, true);
  


  prmMap.show_Prm();
  
  prmMap.User_remove_Nodes();


  ros::shutdown();


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
      marker.header.frame_id = "map"; // or your relevant frame
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
  }

  marker_pub.publish(markerArray); // Publish the entire array
}
