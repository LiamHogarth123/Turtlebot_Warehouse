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
    prev_heading_error_ = 0;
    prev_integral_ = 0;
    prev_heading_integral_ = 0;
    Kp_ = 0.2;
    Ki_ = 0.5;
    Kd_ = 0.0;

    Kp_h = 0.01;
    Ki_h = 0.01;
    Kd_h = 0.1;

    iteration_time = 500;

}


void Control::updateGoal(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_odom){
    goal = temp_goal;
    odom = temp_odom;
}


geometry_msgs::Twist Control::reachGoal(){

    ///////// Forward control /////////
    double current_distance = distanceToGoal();

    // Calculate error
    double error = toleranceDistance - current_distance;

    // Update integral and derivative terms
    integral_ = prev_integral_ + error*iteration_time;
    double derivative = (error - prev_error_)/iteration_time;

    // Calculate control command
    double control_command = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
    control_command *= 0.5; //scaling down

    ///////// Angular control /////////
    double current_heading = angleToGoal();
    double heading_error = targetAngle - current_heading;

    // Update integral and derivative terms for heading error
    heading_integral_ = prev_heading_integral_ + heading_error*iteration_time;
    double heading_derivative = (heading_error - prev_heading_error_)/iteration_time;

    // Calculate control command for angular velocity
    double angular_command = Kp_h * heading_error + Ki_h * heading_integral_ + Kd_h * heading_derivative;
    
    // Ensure the angular command corresponds to the sign of the heading error
    if (heading_error > 0) {
        angular_command = fabs(angular_command);  // Turn right
    } else {
        angular_command = -fabs(angular_command); // Turn left
    }

    //// Create and publish Twist message for velocity control
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = control_command;
    cmd_vel.angular.z = angular_command;

    // Update previous error and integral
    prev_error_ = error;
    prev_heading_error_ = heading_error;
    prev_integral_ = integral_;
    prev_heading_integral_ = heading_integral_;
    
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "distanceToGoal: " << distanceToGoal() << std::endl;
    std::cout << "angleToGoal: " << angleToGoal() << std::endl;
    std::cout << "headingError: " << heading_error << std::endl;
    std::cout << "angularCommand: " << angular_command << std::endl;

    return cmd_vel;
    
}


bool Control::collisionDetection() {



    return true;
}


bool Control::goal_hit(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_odom){
    
    if (distanceToGoal() <= distance_from_goal) {
        
        prev_integral_ = 0; //reset PID integral 
        prev_heading_integral_ = 0;
        return true;
    }
    else{
        return false;
    }

    

} 

double Control::distanceToGoal(){

    double delta_x = goal.x - odom.pose.pose.position.x;
    double delta_y = goal.y - odom.pose.pose.position.y;
    double distance = sqrt(std::pow(delta_x,2) + std::pow(delta_y,2));

    return distance;
}

double Control::angleToGoal(){

    tf::Quaternion current_orientation;

    tf::quaternionMsgToTF(odom.pose.pose.orientation, current_orientation);

    // Normalize the quaternion
    current_orientation.normalize();

    // Calculate the heading direction vector
    tf::Vector3 heading_vector(1, 0, 0);  // Assumes the robot's heading direction is along the x-axis

    // Rotate the heading vector to the current orientation
    heading_vector = tf::quatRotate(current_orientation, heading_vector);

    // Calculate the vector to the goal
    tf::Vector3 goal_vector(goal.x - odom.pose.pose.position.x, goal.y - odom.pose.pose.position.y, 0);

    // Calculate the angle between the heading direction and the goal vector
    double angle = atan2(goal_vector.y(), goal_vector.x()) - atan2(heading_vector.y(), heading_vector.x());

    return angle;
}

long Control::getIterationTime(){
   
    return iteration_time;
}