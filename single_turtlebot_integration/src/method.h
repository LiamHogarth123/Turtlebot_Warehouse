#ifndef SAMPLE_H
#define SAMPLE_H

#include "ros/ros.h"
#include <atomic>
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

#include "control.h"
#include "sensorprocessing.h"
#include "prm.h"

#include <ros/package.h>

#include "visualization_msgs/MarkerArray.h"

#include "turtlebot.h"
#include "taskAlloction.h"





/**
 @class Sample
 @brief A class representing a sample.
*/
class Method
{
public:
  /*
  @brief Default constructor.
  @param nh The ROS node handle.
  */
  Method(ros::NodeHandle nh);

  void Send_cmd_tb12(geometry_msgs::Twist intructions);

  ros::Publisher cmd_velocity_tb1;


  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  void mapMetadataCallback(const nav_msgs::MapMetaData::ConstPtr& msg);




 

  void separateThread();

  void turtleMovement();

  geometry_msgs::Point findLookAheadPoint(const std::vector<geometry_msgs::Point>& path, const geometry_msgs::Point& current_position, double look_ahead_distance);

  void publishMarkers(const std::vector<geometry_msgs::Point>& nodes, ros::Publisher& marker_pub);

  void publishLookAheadMarker(const geometry_msgs::Point& look_ahead_point);

  // ros::Publisher marker_pub;


  // CONTROL FUNCTIONS
  // bool goalInObstacleCheck();
  bool tagAlignment(std::pair<int, geometry_msgs::Point> temp_tag, double temp_angleToGoal);
  double calcDistance(geometry_msgs::Point temp_point1, geometry_msgs::Point temp_point2);



  bool running;

// Prameters for ROS
  ros::NodeHandle nh_;
  ros::Publisher cmd_velocity_tb12;
  ros::Publisher cmd_velocity_tb2;
  ros::Publisher pub_;
  ros::Publisher single_marker_pub_;

  
// Subscribers for turtlebot 1
  ros::Subscriber sub1_;
  ros::Subscriber sub2_;
  ros::Subscriber sub3_;
  ros::Subscriber sub4_;

// Subscribers for turtlebot 2 (tb3_1)
  ros::Subscriber sub5_;

//mutexs
  std::mutex odom_locker;
  std::mutex odom_locker2;
  std::mutex RGB_locker;
  std::mutex Lida_locker;
  std::mutex ImageDepth_locker;
  std::mutex goal_lock;
  

//variables for callbacks
  DefaultTurtleBot* tb1; 

//Geometry variable to do with movenment;
  geometry_msgs::Point goal;
  geometry_msgs::Point goal_gobal_frame;
  geometry_msgs::Twist traj;
  geometry_msgs::Point guiderGoal;

  double goal_index;

//Declaration of class objects
  Control TurtleGPS;
  Sensorprocessing Lidar;

  std::vector<geometry_msgs::Point> Leader_goals;

  bool teleop_mode;
  bool missionComplete;



// Map requreiments
  // ros::NodeHandle nh_;
  ros::Subscriber map_sub;
  ros::Subscriber mapSub;
  ros::Subscriber mapMetadataSub;
  ros::Publisher marker_pub;
  
  // readMap mapReader;

  PRM prmMap; 
  TaskAlloction TA;

  nav_msgs::OccupancyGrid latestMapData_;
  nav_msgs::MapMetaData latestMapMetaData_;



  std::mutex MapData_Lock;
  std::mutex MapMetaData_Lock;

};




  
#endif // SAMPLE_H
