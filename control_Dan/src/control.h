#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H


#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class Control
{
    public:
    
    /**
    @brief Constructor for the DataProcessing class.
    */
    Control();

    void updateGoal(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_odom );

    geometry_msgs::Twist reachGoal();
    bool collisionDetection();

    bool goal_hit(geometry_msgs::Point temp_goal, nav_msgs::Odometry temp_odom);

    double distanceToGoal();

    double angleToGoal();

    //parameters
    private:
    geometry_msgs::Point goal;
    nav_msgs::Odometry odom;

    //Calculation variable declaration
    double DirectDistance;
    double Angle;
    double theta;
    double radius;
    double default_velocity;
    double distance_from_goal;

    
    double Kp_;
    double Ki_;
    double Kd_;
    double toleranceDistance;
    double integral_;
    double prev_error_;

    double Kp_h;
    double Ki_h;
    double Kd_h;
    double targetAngle;
    double prev_heading_error_;
    double heading_integral_;

};

#endif // DETECTCABINET_H


