#ifndef SAMPLE_H
#define SAMPLE_H

#include "ros/ros.h"
#include <atomic>
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

#include "control.h"
#include "prm.h"
#include "taskAlloction.h"
#include "turtlebot.h"
// #include "taskAlloction.h"

#include <ros/package.h>

#include "visualization_msgs/MarkerArray.h"


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


  void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
  void LidaCallback(const sensor_msgs::LaserScan::ConstPtr& Msg);
  void RGBCallback(const sensor_msgs::Image::ConstPtr& Msg);
  void ImageDepthCallback(const sensor_msgs::Image::ConstPtr& Msg);
  void guiderOdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void mapMetadataCallback(const nav_msgs::MapMetaData::ConstPtr& msg);


  void Send_cmd_tb1(geometry_msgs::Twist intructions);

  void Send_cmd_tb2(geometry_msgs::Twist intructions);

  void separateThread();

  void turtleMovement();

  visualization_msgs::MarkerArray visualiseCones(std::vector<geometry_msgs::Point> cones, visualization_msgs::MarkerArray& markerArray);
  void publishMarkers(const std::vector<geometry_msgs::Point>& nodes, ros::Publisher& marker_pub);

  // ros::Publisher marker_pub;


// Prameters for ROS
  ros::NodeHandle nh_;
  ros::Publisher cmd_velocity_tb1;
  ros::Publisher cmd_velocity_tb2;
  ros::Publisher pub_;

  
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
  nav_msgs::Odometry Current_Odom;
  nav_msgs::Odometry guider_Odom;
  sensor_msgs::Image updated_RGB;
  sensor_msgs::LaserScan updated_Lida;
  sensor_msgs::Image updated_imageDepth;


//Geometry variable to do with movenment;
  geometry_msgs::Point goal;
  geometry_msgs::Point goal_gobal_frame;
  geometry_msgs::Twist traj;
  geometry_msgs::Point guiderGoal;

  double goal_index;


 

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
  Control TurtleGPS;

  nav_msgs::OccupancyGrid latestMapData_;
  nav_msgs::MapMetaData latestMapMetaData_;



  std::mutex MapData_Lock;
  std::mutex MapMetaData_Lock;

};




  
#endif // SAMPLE_H