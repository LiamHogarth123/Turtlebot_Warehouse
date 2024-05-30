
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "turtlebot.h"


DefaultTurtleBot::DefaultTurtleBot(const std::string& name, const ros::NodeHandle& nh) : namespace_(name) {
    // Create publishers and subscribers with namespaced topics/services
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(namespace_ + "/cmd_vel", 10);
    // some_subscriber_ = nh_.subscribe(namespace_ + "/some_topic", 10, &TurtleBot::callback, this);

    sub1_ = nh_.subscribe(namespace_ + "/odom", 1000, &DefaultTurtleBot::odomCallback,this);

    sub2_ = nh_.subscribe(namespace_ + "/scan", 10, &DefaultTurtleBot::LidaCallback,this);

    sub3_ = nh_.subscribe(namespace_ + "/camera/rgb/image_raw", 1000, &DefaultTurtleBot::RGBCallback, this);

    sub4_ = nh_.subscribe(namespace_ + "/camera/depth/image_raw", 1000, &DefaultTurtleBot::ImageDepthCallback, this);

}

void DefaultTurtleBot::Send_cmd_tb1(geometry_msgs::Twist intructions){
  cmd_vel_pub_.publish(intructions);
}

// DefaultTurtleBot DefaultTurtleBot::cloneWithNamespace(const std::string& name) const {
//     return DefaultTurtleBot(name, nh_); // Use the same node handle as the original
// }
nav_msgs::Odometry DefaultTurtleBot::GetCurrent_Odom(){
    return Current_Odom;
}

sensor_msgs::Image DefaultTurtleBot::GetCurrentupdated_RGB(){
    return updated_RGB;
}
sensor_msgs::LaserScan DefaultTurtleBot::Getupdated_Lida(){
    return updated_Lida;
}

sensor_msgs::Image DefaultTurtleBot::Getupdated_imageDepth(){
    return updated_imageDepth;
}

nav_msgs::Odometry DefaultTurtleBot::odomToMap(nav_msgs::Odometry odom, geometry_msgs::Transform map)
{
  nav_msgs::Odometry updatedOdom;
  updatedOdom.pose.pose.position.x = odom.pose.pose.position.x + map.translation.x;
  updatedOdom.pose.pose.position.y = odom.pose.pose.position.y + map.translation.y;
  updatedOdom.pose.pose.position.z = odom.pose.pose.position.x;
  updatedOdom.pose.pose.orientation.x = odom.pose.pose.orientation.x + map.rotation.x;
  updatedOdom.pose.pose.orientation.y = odom.pose.pose.orientation.y + map.rotation.y;
  updatedOdom.pose.pose.orientation.z = odom.pose.pose.orientation.z + map.rotation.z;
  updatedOdom.pose.pose.orientation.w = odom.pose.pose.orientation.w + map.rotation.w;
  return updatedOdom;
}


//callbacks
///////////////////////////////////////////////////////////////////////////////////////////////

void DefaultTurtleBot::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
  std::unique_lock<std::mutex> lck3 (odom_locker);
  Current_Odom = *odomMsg;
}

void  DefaultTurtleBot::RGBCallback(const sensor_msgs::Image::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (RGB_locker);
  updated_RGB = *Msg;

}

void DefaultTurtleBot::LidaCallback(const sensor_msgs::LaserScan::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (Lida_locker);
  updated_Lida = *Msg;
}

void DefaultTurtleBot::ImageDepthCallback(const sensor_msgs::Image::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (ImageDepth_locker);
  updated_imageDepth = *Msg;
}

void DefaultTurtleBot::guiderOdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
  std::unique_lock<std::mutex> lck3 (odom_locker2);
  guider_Odom = *odomMsg;
}
