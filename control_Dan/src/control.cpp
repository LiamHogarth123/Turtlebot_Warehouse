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
    prev_error_ = 0;
    Kp_ = 0;
    Ki_ = 0;
    Kd_ = 0;

}

/**
 * @brief Set a new movement goal and update the current pose.
 *
 * This function updates the movement goal and the current pose with the provided values.
 * It is typically used to set a new goal for a navigation system and update the robot's current pose
 * accordingly.

 * @param temp_goal The geometry_msgs::Point representing the new movement goal.
 * @param temp_Current_Pose The nav_msgs::Odometry representing the robot's current pose.

 * The function sets the new movement goal (Goal) and updates the robot's current pose (Current_Pose)
 * with the provided values.

 * @note This function is crucial for redefining the robot's navigation goal and ensuring that the
 * current pose is in sync with the robot's position and orientation.
 */
void Control::newGoal(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_Current_Pose){
    Goal = temp_goal;
    Current_Pose = temp_Current_Pose;
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
    double current_distance = 0.1;

    // Calculate error
    double error = toleranceDistance - current_distance;

    // Update integral and derivative terms
    integral_ += error;
    double derivative = error - prev_error_;

    // Calculate control command
    double control_command = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;

    ///////// Angular control /////////
    
    double current_heading = 0.1;
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
bool Control::goal_hit(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_Current_Pose){
    double delta_x = Goal.x - Current_Pose.pose.pose.position.x;
    double delta_y = Goal.y - Current_Pose.pose.pose.position.y;
    DirectDistance = sqrt(std::pow(delta_x,2) + std::pow(delta_y,2));
    if (DirectDistance <= distance_from_goal) {
        return true;
    }
    else{
        return false;
    }

    integral_ = 0; //reset PID integral 

} 


/**
 * @brief Calculate the angular velocity required to align with the goal direction.
 *
 * This function calculates the angular velocity required to align the robot's orientation with the
 * direction of the goal point. It is essential for controlling the robot's rotation to face the goal.

 * @return The calculated angular velocity (positive for clockwise, negative for counterclockwise).

 * The function starts by defining constants for ANGULAR_SPEED (maximum angular velocity) and ANGULAR_DEAD_ZONE
 * (a threshold to prevent wobbling due to small angle changes).

 * It extracts the current orientation of the robot from the provided nav_msgs::Odometry, normalizes the quaternion,
 * and calculates the heading direction vector (assumes the robot's heading is along the x-axis).

 * The function then calculates the vector from the robot's current position to the goal and determines the angle
 * between the heading direction and the goal vector.

 * Based on the calculated angle, the function adjusts the angular velocity:
 * - If the angle is within the ANGULAR_DEAD_ZONE, it returns a zero angular velocity to prevent excessive rotation.
 * - If the angle is positive (indicating a clockwise rotation is needed), it returns a positive ANGULAR_SPEED.
 * - If the angle is negative (indicating a counterclockwise rotation is needed), it returns a negative ANGULAR_SPEED.

 * @note This function plays a crucial role in achieving the correct orientation to face the goal during navigation
 * and motion control.
 */
double Control::calculateAngularVelocity() {

}
