
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "turtlebot.h"


DefaultTurtleBot::DefaultTurtleBot(const std::string& name, const ros::NodeHandle& nh, geometry_msgs::Transform map_Offset)  : namespace_(name),  map(map_Offset) {  // Create publishers and subscribers with namespaced topics/services
    

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
  updateOdomWithTransform();
  // return transformed_Odom; .
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

void DefaultTurtleBot::updateOdomWithTransform() {
  tf::StampedTransform transform;
  try {
    listener_.lookupTransform("/map", namespace_ + "/odom", ros::Time(0), transform);
    transformed_Odom = Current_Odom;

    // Apply the transformation to the odometry
    tf::Vector3 new_origin = transform.getOrigin();
    tf::Quaternion new_rotation = transform.getRotation();

    transformed_Odom.pose.pose.position.x += new_origin.x();
    transformed_Odom.pose.pose.position.y += new_origin.y();
    transformed_Odom.pose.pose.position.z += new_origin.z();
    transformed_Odom.pose.pose.orientation.x += new_rotation.x();
    transformed_Odom.pose.pose.orientation.y += new_rotation.y();
    transformed_Odom.pose.pose.orientation.z += new_rotation.z();
    transformed_Odom.pose.pose.orientation.w += new_rotation.w();
  } catch (tf::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
}

double DefaultTurtleBot::GetCurrentSpeed(){
  return current_speed_;
}

std_msgs::Int16 DefaultTurtleBot::getBoundaryStatus(){
  return boundaryStatus;
}

marker_msgs::marker DefaultTurtleBot::getARtag(){
  return arTag;
}  


//callbacks
///////////////////////////////////////////////////////////////////////////////////////////////

void DefaultTurtleBot::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
  std::unique_lock<std::mutex> lck3 (odom_locker);
  Current_Odom = *odomMsg;
  current_speed_ = std::sqrt(std::pow(odomMsg->twist.twist.linear.x, 2) + std::pow(odomMsg->twist.twist.linear.y, 2) + std::pow(odomMsg->twist.twist.linear.z, 2));
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

void DefaultTurtleBot::boundaryCallback(const std_msgs::Int16::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (boundary_locker);
  boundaryStatus = *Msg;
}

void DefaultTurtleBot::tagCallback(const marker_msgs::marker::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (marker_locker);
  arTag = *Msg;
}