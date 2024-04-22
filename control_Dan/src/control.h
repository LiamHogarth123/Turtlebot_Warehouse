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

    double steeringPID();

    double velocityPID();

    std::vector<std::vector<double>> getPlots();
    

    //parameters
    private:
    geometry_msgs::Point goal;
    nav_msgs::Odometry odom;

    //Calculation variable declaration
    double Kp_;
    double Ki_;
    double Kd_;
    double toleranceDistance;
    double integral_;
    double prev_error_;

    double Kp_h;
    double Ki_h;
    double Kd_h;
    double toleranceAngle;
    double heading_integral_;
    double prev_heading_error_;

    double maxVelx;
    double maxVelz;

    double maxIntegral;
    double maxHeadingIntegral;

    std::vector<double> xPlot;
    std::vector<double> zPlot;

 
};

#endif // DETECTCABINET_H


