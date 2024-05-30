#include <cmath>
#include "ros/ros.h"
#include <iostream>

/** Function to convert a quaternion to a yaw value*/
double quaternionToYaw(double x, double y, double z, double w)
{
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
}

/** Function to convert a yaw value to a quaternion*/
std::tuple<double, double, double, double> yawToQuaternion(double yaw)
{
    double x = 0.0;
    double y = 0.0;
    double z = std::sin(yaw / 2.0);
    double w = std::cos(yaw / 2.0);
    return std::make_tuple(x, y, z, w);
}

int main(int argc, char **argv)
{
    /** Initialise node*/
    ros::init(argc, argv, "coord_converter");

    /** Create node handle*/
    ros::NodeHandle nh;

    // /** Initialise process variable*/
    // int process;

    // /** Nominate process*/
    // if (argc > 1)
    // {
    //     process = std::stod(argv[1]);
    //     ROS_INFO("The 'process' argument is %d", process);
    // }
    // /** Otherwise warn*/
    // else
    // {
    //     ROS_WARN("No 'process' argument provided!");
    // }

    /** Initialise process*/
    double process;
    nh.getParam("process",process);

    std::cout << "process = " << process << std::endl;

    /** Initialise variables*/
    double yaw;
    double x, y, z, w;

    /** Process 0, convert Yaw to Quaternion only*/
    if (process > 0 - 1e-2 && process < 0 + 1e-2)
    {
        nh.getParam("yaw", yaw);
        /** Convert yaw to quaternion*/
        std::tuple<double, double, double, double> q = yawToQuaternion(yaw);
        ROS_INFO("Converted Yaw to Quaternion: [x: %f, y: %f, z: %f, w: %f]",
                 std::get<0>(q), std::get<1>(q),
                 std::get<2>(q), std::get<3>(q));
        
        /** Set params*/
        nh.setParam("converted_x",std::get<0>(q));
        nh.setParam("converted_y",std::get<1>(q));
        nh.setParam("converted_z",std::get<2>(q));
        nh.setParam("converted_w",std::get<3>(q));
    }
    /** Process 1, convert Quaternion to Yaw only*/
    else if (process > 1 - 1e-2 && process < 1 + 1e-2)
    {
        nh.getParam("quaternion_x", x);
        nh.getParam("quaternion_y", y);
        nh.getParam("quaternion_z", z);
        nh.getParam("quaternion_w", w);
        /** Convert quaternion to yaw*/
        double yaw_q = quaternionToYaw(x, y, z, w);
        ROS_INFO("Converted Quaternion to Yaw: %f", yaw_q);

        /** Set param*/
        nh.setParam("converted_yaw",yaw_q);
    }
    /** Process 2, convert both*/
    else if (process > 2 - 1e-2 && process < 2 + 1e-2)
    {
        nh.getParam("yaw", yaw);
        /** Convert yaw to quaternion*/
        std::tuple<double, double, double, double> q = yawToQuaternion(yaw);
        ROS_INFO("Converted Yaw to Quaternion: [x: %f, y: %f, z: %f, w: %f]",
                 std::get<0>(q), std::get<1>(q),
                 std::get<2>(q), std::get<3>(q));
        
        nh.getParam("quaternion_x", x);
        nh.getParam("quaternion_y", y);
        nh.getParam("quaternion_z", z);
        nh.getParam("quaternion_w", w);
        /** Convert quaternion to yaw*/
        double yaw_q = quaternionToYaw(x, y, z, w);
        ROS_INFO("Converted Quaternion to Yaw: %f", yaw_q);

        /** Set params*/
        nh.setParam("converted_x",std::get<0>(q));
        nh.setParam("converted_y",std::get<1>(q));
        nh.setParam("converted_z",std::get<2>(q));
        nh.setParam("converted_w",std::get<3>(q));
        nh.setParam("converted_yaw",yaw_q);
        
    }

    ros::spin();

    return 0;
}