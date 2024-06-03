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
#include "path_avoidance.h"

#include <ros/package.h>

#include "visualization_msgs/MarkerArray.h"


#include "std_msgs/Int16.h"
#include "marker_msgs/marker.h"



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


  
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void mapMetadataCallback(const nav_msgs::MapMetaData::ConstPtr& msg);


  void Send_cmd_tb1(geometry_msgs::Twist intructions);

  void Send_cmd_tb2(geometry_msgs::Twist intructions);


  void separateThread();

  void turtleMovement();


  void Function(std::vector<geometry_msgs::Point> trajectory , DefaultTurtleBot* turtleboi, Control* Turtle_GPS);

  void Function2(PRM* Trajectory_Planner, DefaultTurtleBot* turtleboi, Control* Turtle_GPS, std::vector<std::pair<int, geometry_msgs::Point>> Goals);

  
  visualization_msgs::MarkerArray publishMarkers(const std::vector<geometry_msgs::Point>& nodes, ros::Publisher& marker_pub);
  void publishCollisons(const std::vector<geometry_msgs::Point>& nodes, ros::Publisher& marker_pub, visualization_msgs::MarkerArray trajectory_makers);


  // ros::Publisher marker_pub;

  geometry_msgs::Point findLookAheadPoint(const std::vector<geometry_msgs::Point>& path, const geometry_msgs::Point& current_position, double look_ahead_distance);



  bool tagAlignment(std::pair<int, geometry_msgs::Point> temp_tag, double temp_angleToGoal, DefaultTurtleBot* tb1);
  double calcDistance(geometry_msgs::Point temp_point1, geometry_msgs::Point temp_point2);


  //visualisation variable
  int id;



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

  std::vector<bool> Finished;


 

  std::vector<geometry_msgs::Point> Leader_goals;

  bool teleop_mode;
  bool missionComplete;

  marker_msgs::marker arTag;
  std_msgs::Int16 boundaryStatus;


   geometry_msgs::Twist zero_vel;




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