#include <cmath>
#include "ros/ros.h"
#include <iostream>

struct Quaternion
{
    double x, y, z, w;
};

/** Function to convert a quaternion to a yaw value*/
double quaternionToYaw(double x, double y, double z, double w)
{
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
}

/** Function to convert a yaw value to a quaternion*/
Quaternion yawToQuaternion(double yaw)
{
    Quaternion q;

    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);

    return q;
}

int main(int argc, char **argv)
{
    /** Initialise node*/
    ros::init(argc, argv, "coord_converter");

    /** Create node handle*/
    ros::NodeHandle nh;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;

    double yaw = quaternionToYaw(x, y, z, w);
    std::cout << "Yaw: " << yaw << std::endl;

    /** */
    
    /** Store value if argument passed*/
    if (argc > 1)
    {
        tb = argv[1];
        ROS_INFO("The 'tb' argument is %s", tb.c_str());
    }
    /** Otherwise continue with no namespace*/
    else
    {
        std::cout << "No 'tb' argument provided!" << std::endl;
    }

    double yaw = ...

        Quaternion q = yawToQuaternion(yaw);
    std::cout << "Quaternion: (" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << ")" << std::endl;

    return 0;
}