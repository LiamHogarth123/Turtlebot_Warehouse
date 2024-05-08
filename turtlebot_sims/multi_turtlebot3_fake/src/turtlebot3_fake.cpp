// /*******************************************************************************
// * Copyright 2016 ROBOTIS CO., LTD.
// *
// * Licensed under the Apache License, Version 2.0 (the "License");
// * you may not use this file except in compliance with the License.
// * You may obtain a copy of the License at
// *
// *     http://www.apache.org/licenses/LICENSE-2.0
// *
// * Unless required by applicable law or agreed to in writing, software
// * distributed under the License is distributed on an "AS IS" BASIS,
// * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// * See the License for the specific language governing permissions and
// * limitations under the License.
// *******************************************************************************/

// /* Authors: Yoonseok Pyo */

// #include <turtlebot3_fake/turtlebot3_fake.h>


// Turtlebot3Fake::Turtlebot3Fake()
// : nh_priv_("~")
// {
//   // ns = nh_.getNamespace();
//   // ns = (ns != "/" ? ns + "/" : ""); // Ensure namespace format

//   // // Initialize frame names with namespace
//   // base_frame_id_ = ns + "base_footprint";
//   // odom_frame_id_ = ns + "odom";
//   // left_wheel_joint_name_ = ns + "wheel_left_joint";
//   // right_wheel_joint_name_ = ns + "wheel_right_joint";
//   // left_wheel_joint_name_ = ns + "wheel_left_link";
//   // right_wheel_joint_name_ = ns + "wheel_right_link";
//   // base_link_frame_id_ = ns + "base_link";
//   // base_scan_frame_id_ = ns + "base_scan";

//   //Init fake turtlebot node
//   bool init_result = init();
//   ROS_ASSERT(init_result);
// }

// Turtlebot3Fake::~Turtlebot3Fake()
// {
// }

// /*******************************************************************************
// * Init function
// *******************************************************************************/
// bool Turtlebot3Fake::init()
// {
//   // initialize ROS parameter

  

//   std::string robot_model = nh_.param<std::string>("tb3_model", "");


//   if (!robot_model.compare("burger"))
//   {
//     wheel_seperation_ = 0.160;
//     turning_radius_   = 0.080;
//     robot_radius_     = 0.105;
//   }
//   else if (!robot_model.compare("waffle") || !robot_model.compare("waffle_pi"))
//   {
//     wheel_seperation_ = 0.287;
//     turning_radius_   = 0.1435;
//     robot_radius_     = 0.220;
//   }

//   nh_.param("wheel_left_joint_name", joint_states_name_[LEFT],  std::string("wheel_left_joint"));
//   nh_.param("wheel_right_joint_name", joint_states_name_[RIGHT],  std::string("wheel_right_joint"));
//   nh_.param("joint_states_frame", joint_states_.header.frame_id, std::string("base_footprint"));
//   nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
//   nh_.param("base_frame", odom_.child_frame_id, std::string("base_footprint"));

//   // initialize variables
//   wheel_speed_cmd_[LEFT]  = 0.0;
//   wheel_speed_cmd_[RIGHT] = 0.0;
//   goal_linear_velocity_   = 0.0;
//   goal_angular_velocity_  = 0.0;
//   cmd_vel_timeout_        = 1.0;
//   last_position_[LEFT]    = 0.0;
//   last_position_[RIGHT]   = 0.0;
//   last_velocity_[LEFT]    = 0.0;
//   last_velocity_[RIGHT]   = 0.0;

//   double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
//                         0, 0.1,   0,   0,   0, 0,
//                         0,   0, 1e6,   0,   0, 0,
//                         0,   0,   0, 1e6,   0, 0,
//                         0,   0,   0,   0, 1e6, 0,
//                         0,   0,   0,   0,   0, 0.2};
//   memcpy(&(odom_.pose.covariance),pcov,sizeof(double)*36);
//   memcpy(&(odom_.twist.covariance),pcov,sizeof(double)*36);

//    double init_x, init_y, init_yaw;
//   nh_priv_.param("initial_x", init_x, 0.0);
//   nh_priv_.param("initial_y", init_y, 0.0);
//   nh_priv_.param("initial_yaw", init_yaw, 0.0);


//   odom_pose_[0] = init_x;
//   odom_pose_[1] = init_y;
//   odom_pose_[2] = init_yaw;

//   odom_vel_[0] = 0.0;
//   odom_vel_[1] = 0.0;
//   odom_vel_[2] = 0.0;

//   joint_states_.name.push_back(joint_states_name_[LEFT]);
//   joint_states_.name.push_back(joint_states_name_[RIGHT]);
//   joint_states_.position.resize(2,0.0);
//   joint_states_.velocity.resize(2,0.0);
//   joint_states_.effort.resize(2,0.0);

//   // initialize publishers
//   joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
//   odom_pub_         = nh_.advertise<nav_msgs::Odometry>("odom", 100);

//   // initialize subscribers
//   cmd_vel_sub_  = nh_.subscribe("cmd_vel", 100,  &Turtlebot3Fake::commandVelocityCallback, this);

//   prev_update_time_ = ros::Time::now();
//   publishStaticTransforms();
  
//   return true;
// }


// void Turtlebot3Fake::publishStaticTransforms()
// {
//     static tf::TransformBroadcaster br;
//     tf::Transform transform;

//     // Ensure the namespace is properly prefixed
//     std::string ns = nh_.getNamespace();
//     if (ns != "/" && !ns.empty()) ns += "/";

//     // Example transform from `base_footprint` to `base_link`
//     transform.setOrigin(tf::Vector3(0.0, 0.0, 0.1)); // Small offset in z-axis
//     transform.setRotation(tf::Quaternion(0, 0, 0, 1)); // No rotation

//     std::string base_footprint_frame = ns + "base_footprint";
//     std::string base_link_frame = ns + "base_link";

//     // This transform is published once since it is static
//     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_footprint_frame, base_link_frame));

//     // Continue for other static transforms, e.g., from `base_link` to `camera_link`
//     transform.setOrigin(tf::Vector3(0.064, -0.065, 0.094)); // Position of the camera on the robot
//     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_link_frame, ns + "camera_link"));

//     // Add other necessary static transforms
// }

// /*******************************************************************************
// * Callback function for cmd_vel msg
// *******************************************************************************/
// void Turtlebot3Fake::commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg)
// {
//   last_cmd_vel_time_ = ros::Time::now();

//   goal_linear_velocity_  = cmd_vel_msg->linear.x;
//   goal_angular_velocity_ = cmd_vel_msg->angular.z;

//   wheel_speed_cmd_[LEFT]  = goal_linear_velocity_ - (goal_angular_velocity_ * wheel_seperation_ / 2);
//   wheel_speed_cmd_[RIGHT] = goal_linear_velocity_ + (goal_angular_velocity_ * wheel_seperation_ / 2);
// }

// /*******************************************************************************
// * Calculate the odometry
// *******************************************************************************/
// bool Turtlebot3Fake::updateOdometry(ros::Duration diff_time)
// {
//   double wheel_l, wheel_r; // rotation value of wheel [rad]
//   double delta_s, delta_theta;
//   double v[2], w[2];

//   wheel_l = wheel_r     = 0.0;
//   delta_s = delta_theta = 0.0;

//   v[LEFT]  = wheel_speed_cmd_[LEFT];
//   w[LEFT]  = v[LEFT] / WHEEL_RADIUS;  // w = v / r
//   v[RIGHT] = wheel_speed_cmd_[RIGHT];
//   w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;

//   last_velocity_[LEFT]  = w[LEFT];
//   last_velocity_[RIGHT] = w[RIGHT];

//   wheel_l = w[LEFT]  * diff_time.toSec();
//   wheel_r = w[RIGHT] * diff_time.toSec();

//   if(isnan(wheel_l))
//   {
//     wheel_l = 0.0;
//   }

//   if(isnan(wheel_r))
//   {
//     wheel_r = 0.0;
//   }

//   last_position_[LEFT]  += wheel_l;
//   last_position_[RIGHT] += wheel_r;

//   delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
//   delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / wheel_seperation_;

//   // compute odometric pose
//   odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
//   odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
//   odom_pose_[2] += delta_theta;

//   // compute odometric instantaneouse velocity
//   odom_vel_[0] = delta_s / diff_time.toSec();     // v
//   odom_vel_[1] = 0.0;
//   odom_vel_[2] = delta_theta / diff_time.toSec(); // w

//   odom_.pose.pose.position.x = odom_pose_[0];
//   odom_.pose.pose.position.y = odom_pose_[1];
//   odom_.pose.pose.position.z = 0;
//   odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose_[2]);

//   // We should update the twist of the odometry
//   odom_.twist.twist.linear.x  = odom_vel_[0];
//   odom_.twist.twist.angular.z = odom_vel_[2];

//   return true;
// }

// /*******************************************************************************
// * Calculate the joint states
// *******************************************************************************/
// void Turtlebot3Fake::updateJoint(void)
// {
//   joint_states_.position[LEFT]  = last_position_[LEFT];
//   joint_states_.position[RIGHT] = last_position_[RIGHT];
//   joint_states_.velocity[LEFT]  = last_velocity_[LEFT];
//   joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];
// }

// /*******************************************************************************
// * Calculate the TF
// *******************************************************************************/
// void Turtlebot3Fake::updateTF(geometry_msgs::TransformStamped& odom_tf)
// {
//     ros::Time current_time = ros::Time::now();

//     // Namespace prepending logic (make sure this is in your constructor or initializer)
//     std::string ns = ros::this_node::getNamespace();
//     ns = (!ns.empty() && ns != "/" ? ns + "/" : "");

//     // Adjust the frame names to include the namespace
//     std::string ns_base_link = ns + "base_link";
//     std::string ns_wheel_left_link = ns + "wheel_left_link";
//     std::string ns_wheel_right_link = ns + "wheel_right_link";

//     // Odom to Base Footprint Transform
//     odom_tf.header.stamp = current_time;
//     odom_tf.header.frame_id = ns + "odom";
//     odom_tf.child_frame_id = ns + "base_footprint";
//     odom_tf.transform.translation.x = odom_.pose.pose.position.x;
//     odom_tf.transform.translation.y = odom_.pose.pose.position.y;
//     odom_tf.transform.translation.z = odom_.pose.pose.position.z;
//     odom_tf.transform.rotation = odom_.pose.pose.orientation;
//     tf_broadcaster_.sendTransform(odom_tf);

//     // Base Footprint to Base Link Transform
//     geometry_msgs::TransformStamped base_link_tf;
//     base_link_tf.header.stamp = current_time;
//     base_link_tf.header.frame_id = ns + "base_footprint";
//     base_link_tf.child_frame_id = ns_base_link;
//     base_link_tf.transform.translation.x = 0.0; // Adjust according to your model specifics
//     base_link_tf.transform.translation.y = 0.0;
//     base_link_tf.transform.translation.z = 0.1; // Example: height of base_link from base_footprint
//     base_link_tf.transform.rotation = tf::createQuaternionMsgFromYaw(0);
//     tf_broadcaster_.sendTransform(base_link_tf);

//     // Wheel Transforms
//     // Left Wheel
//     geometry_msgs::TransformStamped left_wheel_tf;
//     left_wheel_tf.header.stamp = current_time;
//     left_wheel_tf.header.frame_id = ns_base_link;
//     left_wheel_tf.child_frame_id = ns_wheel_left_link;
//     left_wheel_tf.transform.translation.x = 0.0;
//     left_wheel_tf.transform.translation.y = 0.144;
//     left_wheel_tf.transform.translation.z = 0.023;
//     left_wheel_tf.transform.rotation = tf::createQuaternionMsgFromYaw(0);
//     tf_broadcaster_.sendTransform(left_wheel_tf);

//     // Right Wheel
//     geometry_msgs::TransformStamped right_wheel_tf;
//     right_wheel_tf.header.stamp = current_time;
//     right_wheel_tf.header.frame_id = ns_base_link;
//     right_wheel_tf.child_frame_id = ns_wheel_right_link;
//     right_wheel_tf.transform.translation.x = 0.0;
//     right_wheel_tf.transform.translation.y = -0.144;
//     right_wheel_tf.transform.translation.z = 0.023;
//     right_wheel_tf.transform.rotation = tf::createQuaternionMsgFromYaw(0);
//     tf_broadcaster_.sendTransform(right_wheel_tf);
// }

// /*******************************************************************************
// * Update function
// *******************************************************************************/
// bool Turtlebot3Fake::update()
// {
//   ros::Time time_now = ros::Time::now();
//   ros::Duration step_time = time_now - prev_update_time_;
//   prev_update_time_ = time_now;

//   // zero-ing after timeout
//   if((time_now - last_cmd_vel_time_).toSec() > cmd_vel_timeout_)
//   {
//     wheel_speed_cmd_[LEFT]  = 0.0;
//     wheel_speed_cmd_[RIGHT] = 0.0;
//   }

//   // odom
//   updateOdometry(step_time);
//   odom_.header.stamp = time_now;
//   odom_pub_.publish(odom_);

//   // joint_states
//   updateJoint();
//   joint_states_.header.stamp = time_now;
//   joint_states_pub_.publish(joint_states_);

//   // tf
//   geometry_msgs::TransformStamped odom_tf;
//   updateTF(odom_tf);
//   tf_broadcaster_.sendTransform(odom_tf);

//   return true;
// }

// /*******************************************************************************
// * Main function
// *******************************************************************************/
// int main(int argc, char* argv[])
// {
//   ros::init(argc, argv, "turtlebot3_fake_node");
//   Turtlebot3Fake tb3fake;

//   ros::Rate loop_rate(30);

//   while (ros::ok())
//   {
//     tb3fake.update();
//     tb3fake.publishStaticTransforms();
//     ros::spinOnce();
//     loop_rate.sleep();
//   }

//   return 0;
// }


/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo */

#include <turtlebot3_fake/turtlebot3_fake.h>

Turtlebot3Fake::Turtlebot3Fake()
: nh_priv_("~")
{
  //Init fake turtlebot node
  bool init_result = init();
  ROS_ASSERT(init_result);
  ns = nh_.getNamespace();
  ns = (ns != "/" ? ns + "/" : ""); // Ensure namespace format
}

Turtlebot3Fake::~Turtlebot3Fake()
{
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Fake::init()
{
  // initialize ROS parameter

  std::string robot_model = nh_.param<std::string>("tb3_model", "");

  if (!robot_model.compare("burger"))
  {
    wheel_seperation_ = 0.160;
    turning_radius_   = 0.080;
    robot_radius_     = 0.105;
  }
  else if (!robot_model.compare("waffle") || !robot_model.compare("waffle_pi"))
  {
    wheel_seperation_ = 0.287;
    turning_radius_   = 0.1435;
    robot_radius_     = 0.220;
  }

  nh_.param("wheel_left_joint_name", joint_states_name_[LEFT],  std::string("wheel_left_joint"));
  nh_.param("wheel_right_joint_name", joint_states_name_[RIGHT],  std::string("wheel_right_joint"));
  nh_.param("joint_states_frame", joint_states_.header.frame_id, std::string("base_footprint"));
  nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
  nh_.param("base_frame", odom_.child_frame_id, std::string("base_footprint"));

  // initialize variables
  wheel_speed_cmd_[LEFT]  = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
  goal_linear_velocity_   = 0.0;
  goal_angular_velocity_  = 0.0;
  cmd_vel_timeout_        = 1.0;
  last_position_[LEFT]    = 0.0;
  last_position_[RIGHT]   = 0.0;
  last_velocity_[LEFT]    = 0.0;
  last_velocity_[RIGHT]   = 0.0;

  double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
                        0, 0.1,   0,   0,   0, 0,
                        0,   0, 1e6,   0,   0, 0,
                        0,   0,   0, 1e6,   0, 0,
                        0,   0,   0,   0, 1e6, 0,
                        0,   0,   0,   0,   0, 0.2};
  memcpy(&(odom_.pose.covariance),pcov,sizeof(double)*36);
  memcpy(&(odom_.twist.covariance),pcov,sizeof(double)*36);


   // Read initial pose parameters
  double init_x, init_y, init_yaw;
  nh_priv_.param("initial_x", init_x, 0.0);
  nh_priv_.param("initial_y", init_y, 0.0);
  nh_priv_.param("initial_yaw", init_yaw, 0.0);

  // Set initial pose
  odom_pose_[0] = init_x;
  odom_pose_[1] = init_y;
  odom_pose_[2] = init_yaw;

  odom_vel_[0] = 0.0;
  odom_vel_[1] = 0.0;
  odom_vel_[2] = 0.0;

  joint_states_.name.push_back(joint_states_name_[LEFT]);
  joint_states_.name.push_back(joint_states_name_[RIGHT]);
  joint_states_.position.resize(2,0.0);
  joint_states_.velocity.resize(2,0.0);
  joint_states_.effort.resize(2,0.0);

  // initialize publishers
  joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
  odom_pub_         = nh_.advertise<nav_msgs::Odometry>("odom", 100);

  // initialize subscribers
  cmd_vel_sub_  = nh_.subscribe("cmd_vel", 100,  &Turtlebot3Fake::commandVelocityCallback, this);

  prev_update_time_ = ros::Time::now();

  return true;
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void Turtlebot3Fake::commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg)
{
  last_cmd_vel_time_ = ros::Time::now();

  goal_linear_velocity_  = cmd_vel_msg->linear.x;
  goal_angular_velocity_ = cmd_vel_msg->angular.z;

  wheel_speed_cmd_[LEFT]  = goal_linear_velocity_ - (goal_angular_velocity_ * wheel_seperation_ / 2);
  wheel_speed_cmd_[RIGHT] = goal_linear_velocity_ + (goal_angular_velocity_ * wheel_seperation_ / 2);
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool Turtlebot3Fake::updateOdometry(ros::Duration diff_time)
{
  double wheel_l, wheel_r; // rotation value of wheel [rad]
  double delta_s, delta_theta;
  double v[2], w[2];

  wheel_l = wheel_r     = 0.0;
  delta_s = delta_theta = 0.0;

  v[LEFT]  = wheel_speed_cmd_[LEFT];
  w[LEFT]  = v[LEFT] / WHEEL_RADIUS;  // w = v / r
  v[RIGHT] = wheel_speed_cmd_[RIGHT];
  w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;

  last_velocity_[LEFT]  = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];

  wheel_l = w[LEFT]  * diff_time.toSec();
  wheel_r = w[RIGHT] * diff_time.toSec();

  if(isnan(wheel_l))
  {
    wheel_l = 0.0;
  }

  if(isnan(wheel_r))
  {
    wheel_r = 0.0;
  }

  last_position_[LEFT]  += wheel_l;
  last_position_[RIGHT] += wheel_r;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / wheel_seperation_;

  // compute odometric pose
  odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel_[0] = delta_s / diff_time.toSec();     // v
  odom_vel_[1] = 0.0;
  odom_vel_[2] = delta_theta / diff_time.toSec(); // w

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose_[2]);

  // We should update the twist of the odometry
  odom_.twist.twist.linear.x  = odom_vel_[0];
  odom_.twist.twist.angular.z = odom_vel_[2];

  return true;
}

/*******************************************************************************
* Calculate the joint states
*******************************************************************************/
void Turtlebot3Fake::updateJoint(void)
{
  joint_states_.position[LEFT]  = last_position_[LEFT];
  joint_states_.position[RIGHT] = last_position_[RIGHT];
  joint_states_.velocity[LEFT]  = last_velocity_[LEFT];
  joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];
}

/*******************************************************************************
* Calculate the TF
*******************************************************************************/
void Turtlebot3Fake::updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = ns +  odom_.child_frame_id;

  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;
}

/*******************************************************************************
* Update function
*******************************************************************************/
bool Turtlebot3Fake::update()
{
  ros::Time time_now = ros::Time::now();
  ros::Duration step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  // zero-ing after timeout
  if((time_now - last_cmd_vel_time_).toSec() > cmd_vel_timeout_)
  {
    wheel_speed_cmd_[LEFT]  = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }

  // odom
  updateOdometry(step_time);
  odom_.header.stamp = time_now;
  odom_pub_.publish(odom_);

  // joint_states
  updateJoint();
  joint_states_.header.stamp = time_now;
  joint_states_pub_.publish(joint_states_);

  // tf
  geometry_msgs::TransformStamped odom_tf;
  updateTF(odom_tf);
  tf_broadcaster_.sendTransform(odom_tf);

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_fake_node");
  Turtlebot3Fake tb3fake;

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    tb3fake.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
