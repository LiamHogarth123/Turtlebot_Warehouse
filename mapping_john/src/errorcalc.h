#ifndef ERRORCALC_H
#define ERRORCALC_H
#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <vector>

class ErrorCalc
{
public:
    /**
     * @brief Constructor
     */
    ErrorCalc();

    /**
     * @brief Destructor
     */
    ~ErrorCalc();

    /**
     * @brief Calculate position error from robot to AR tag
     * @param[in] roboPos Robot position
     * @param[in] tagPose AR tag pose
     * @return Vector of doubles identifying 
     */
    std::pair<double,double> positionError(nav_msgs::Odometry roboPos, geometry_msgs::Pose tagPose);
};

#endif