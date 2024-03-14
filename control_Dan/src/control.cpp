#include "control.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <vector>
#include <algorithm>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
/**
 @file dataprocessing.h
 @brief This file contains the implementation of the DataProcessing class.
*/

Control::Control(){
    
    distance_from_goal = 0.5;

    toleranceDistance = 0.3;
    targetAngle = 0.1;
    prev_error_ = 0;
    Kp_ = 1;
    Ki_ = 1;
    Kd_ = 1;

}


void Control::updateGoal(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_odom){
    goal = temp_goal;
    odom = temp_odom;
}

/**
 * @brief Calculate the required linear and angular velocities to reach a specified goal point.
 *
 * This function performs the necessary calculations to determine the linear and angular velocities
 * required for a robot to reach a given goal point. The function takes into account the distance to
 * the goal and the angle between the robot's current orientation and the direction to the goal.

 * @return A geometry_msgs::Twist containing the calculated linear and angular velocities.

 * The function calculates the distance (DirectDistance) to the goal point, the angle (Angle) between the
 * robot's current orientation and the direction to the goal, the turning radius (radius), and the default
 * linear velocity (default_velocity). Based on these calculations, it determines the linear and angular
 * velocities required to reach the goal point.

 * Additionally, it includes conditions to handle specific situations:
 * - If the robot is close to the goal (within a specified distance), it applies braking.
 * - If the goal point is exactly at the current robot position, it applies rotation to search for the goal.
 * - If the goal is to the right or left of the robot, it adjusts the angular velocity for turning.
 * - If the goal is directly in front of the robot, turning is stopped.

 * @note This function is a critical component of robot motion planning and control, ensuring that the
 * robot moves towards its goal with the appropriate velocities and behaviors.
 */
geometry_msgs::Twist Control::reachGoal(){

    ///////// Forward control /////////
    double current_distance = distanceToGoal(goal, odom);

    // Calculate error
    double error = toleranceDistance - current_distance;

    // Update integral and derivative terms
    integral_ += error;
    double derivative = error - prev_error_;

    // Calculate control command
    double control_command = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;

    ///////// Angular control /////////
    
    double current_heading = 10;
    double heading_error = targetAngle - current_heading;

    // Update integral and derivative terms for heading error
    heading_integral_ += heading_error;
    double heading_derivative = heading_error - prev_heading_error_;

    // Calculate control command for angular velocity
    double angular_command = Kp_ * heading_error + Ki_ * heading_integral_ + Kd_ * heading_derivative;
    

    //// Create and publish Twist message for velocity control
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = control_command;
    cmd_vel.angular.z = angular_command;

    // cmd_vel_pub_.publish(cmd_vel);

    // Update previous error
    prev_error_ = error;
    
    return cmd_vel;
    
}


bool Control::collisionDetection() {



    return true;
}

/**
 * @brief Check if a goal has been reached based on current robot pose.
 *
 * This function determines whether a specified goal has been reached by comparing the distance between
 * the goal point and the current robot pose with a predefined threshold (distance_from_goal).

 * @param temp_goal The geometry_msgs::Point representing the goal to be checked.
 * @param temp_Current_Pose The nav_msgs::Odometry containing the current robot pose.

 * @return True if the goal has been reached, false otherwise.

 * The function calculates the difference in x (delta_x) and y (delta_y) coordinates between the goal and
 * the current robot pose. It then computes the direct distance (DirectDistance) to the goal point using
 * these differences.

 * If the DirectDistance is less than or equal to the predefined threshold (distance_from_goal), the function
 * returns true, indicating that the goal has been reached. Otherwise, it returns false.

 * @note This function is useful for checking whether the robot has reached its intended goal, allowing
 * for decision-making in navigation and control systems.
 */
bool Control::goal_hit(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_odom){
    double delta_x = temp_goal.x - temp_odom.pose.pose.position.x;
    double delta_y = temp_goal.y - temp_odom.pose.pose.position.y;
    DirectDistance = sqrt(std::pow(delta_x,2) + std::pow(delta_y,2));
    if (DirectDistance <= distance_from_goal) {
        return true;
    }
    else{
        return false;
    }

    integral_ = 0; //reset PID integral 

} 

double Control::distanceToGoal(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_odom){

    double delta_x = temp_goal.x - temp_odom.pose.pose.position.x;
    double delta_y = temp_goal.y - temp_odom.pose.pose.position.y;
    double distance = sqrt(std::pow(delta_x,2) + std::pow(delta_y,2));

    return distance;
}