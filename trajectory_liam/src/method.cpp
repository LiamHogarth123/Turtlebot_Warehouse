
#include "method.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <nav_msgs/Odometry.h>


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

  
  
}

void Method::seperateThread() {
  //User input
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Load the PGM map

  //FIX calback
  while (latestMapData_.data.empty()){
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }


  std::cout << "openning" << std::endl;
 
  prmMap.UpdateMapData(latestMapData_, latestMapMetaData_);

  prmMap.test();
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


