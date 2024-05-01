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
    
 
    toleranceDistance = 0.2;
    toleranceAngle = 0.05;
    prev_error_ = 0;
    prev_heading_error_ = 0;
    
    Kp_ = 0.4;
    Ki_ = 0.1;
    Kd_ = 1.5;

    Kp_h = 0.5;
    Ki_h = 0.1;
    Kd_h = 2;

    maxVelx = 0.26; // m/s
    maxVelz = 1.82; // rad/s

    maxIntegral = 10;
    maxHeadingIntegral = 15;

    integral_ = 0;
    heading_integral_ = 0;

    prevOdom.pose.pose.position.x = 0;
    prevOdom.pose.pose.position.y = 0;
    

}


void Control::updateGoal(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_odom, sensor_msgs::LaserScan temp_lidar){
    goal = temp_goal;
    odom = temp_odom;
    lidar = temp_lidar;
}


geometry_msgs::Twist Control::reachGoal(){
  

    //// Create and publish Twist message for velocity control
    geometry_msgs::Twist cmd_vel;

    // calculating velocity commands to reach goal
    double velocityX = velocityPID();
    double velocityZ = steeringPID();

    // Object avoidence
    double obstacleMidpoint = collisionDetection();
    double avoidanceFactor = -0.1; // the value determining the rate of avoidance (lower is faster rate of change)
    if (obstacleMidpoint > 0) {
        std::cout << "avoiding object on left" << std::endl;
        velocityZ = avoidanceFactor/obstacleMidpoint; 
        if (velocityZ < -maxVelz) {
            velocityZ = -maxVelz;
        }
    } else if (obstacleMidpoint < 0) {
        std::cout << "avoiding object on right" << std::endl;
        velocityZ = avoidanceFactor/obstacleMidpoint;
        if (velocityZ > maxVelz) {
            velocityZ = maxVelz;
        }
    }

    // setting final velocity commands
    cmd_vel.linear.x = velocityX;
    cmd_vel.angular.z = velocityZ;

    // std::cout << "---------------------------------------------" << std::endl;
    // std::cout << "distanceToGoal: " << distanceToGoal() << std::endl;
    // std::cout << "forward velocity command: " << velocityX << std::endl;
    // std::cout << "angleToGoal: " << angleToGoal() << std::endl;
    // std::cout << "angular velocity command: " << velocityZ << std::endl;

    // xPlot.push_back(velocityX);
    // zPlot.push_back(velocityZ);
    // fillVelPlot();

    return cmd_vel;
    
}

double Control::velocityPID(){
    ///////// Forward control /////////
    double current_distance = distanceToGoal();

    // Calculate error
    double error = -(toleranceDistance - current_distance);

    // Update integral and derivative terms
    integral_ += error;

    if (integral_ > maxIntegral) {
        integral_ = maxIntegral;
    }
    else if (integral_ < -maxIntegral){
        integral_ = -maxIntegral;
    }

    double derivative = error - prev_error_;

    // std::cout << "integral: " << integral_ << std::endl;

    // Calculate control command
    double control_command = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
    control_command *= 0.15; //scaling down

    // Update previous error and integral
    prev_error_ = error;  

    if (control_command > maxVelx){
        control_command = maxVelx;
    }

    // Goal hit reset
    if (fabs(distanceToGoal()) < toleranceDistance){
        // control_command = 0;
        integral_ = 0;
    }

    if (control_command > maxVelx){
        control_command = maxVelx;
    }

    return control_command;
}

double Control::steeringPID(){
///////// Angular control /////////
    double current_heading = angleToGoal();
    double heading_error = -(toleranceAngle - current_heading);

    // Update integral and derivative terms for heading error
    heading_integral_ += heading_error;

    if (heading_integral_ > maxHeadingIntegral) {
            heading_integral_ = maxHeadingIntegral;
        }
        else if (heading_integral_ < -maxHeadingIntegral){
            heading_integral_ = -maxHeadingIntegral;
        }

    // std::cout << "heading integral: " << heading_integral_ << std::endl;

    double heading_derivative = heading_error - prev_heading_error_;

    // Calculate control command for angular velocity
    double angular_command = Kp_h * heading_error + Ki_h * heading_integral_ + Kd_h * heading_derivative;
    angular_command *= 0.6; //scaling down

    prev_heading_error_ = heading_error;

    if (fabs(angular_command) > maxVelz){
            angular_command = maxVelz;
        }

    // Steer smoothing
    if (fabs(angleToGoal()) < toleranceAngle){
        angular_command = 0;
        heading_integral_ = 0;
    }


    // Ensure the angular command corresponds to the sign of the heading error and is within range
    if (heading_error > 0) {
        angular_command = fabs(angular_command);  // Turn right
        if (angular_command > maxVelz){
            angular_command = maxVelz;
        }
    } else {
        angular_command = -fabs(angular_command); // Turn left
        if (angular_command < -maxVelz){
            angular_command = -maxVelz;
        }
    }

    return angular_command;
}



double Control::collisionDetection() {

    ObjectDetection.Newdata(lidar);
    double obstacleMidpoint = ObjectDetection.findObstacle();

    // if obstacle == 0 then does nothing

    if (obstacleMidpoint != 0) {
        integral_ = 0;
        heading_integral_ = 0;
    }


    return obstacleMidpoint;
}


bool Control::goal_hit(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_odom){
    
    if (distanceToGoal() <= toleranceDistance) {
        
        integral_ = 0; //reset PID integral 
        heading_integral_ = 0;
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

double Control::angleToGoal() {
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

    // Normalize the vectors
    heading_vector.normalize();
    goal_vector.normalize();

    // Calculate the angle between the heading direction and the goal vector using dot and cross products
    double angle = atan2(heading_vector.cross(goal_vector).z(), heading_vector.dot(goal_vector));

    return angle;
}

void Control::fillVelPlot(){
    double dx = odom.pose.pose.position.x - prevOdom.pose.pose.position.x;
    double dy = odom.pose.pose.position.y - prevOdom.pose.pose.position.y;
    double linear_velocity = sqrt(dx * dx + dy * dy);
    
    velPlot.push_back(linear_velocity);

    prevOdom = odom;
}

std::vector<std::vector<double>> Control::getPlots(){

    std::vector<std::vector<double>> temp = {xPlot, zPlot, velPlot};
    
    return temp;
}