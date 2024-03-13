#ifndef OBSTACLERECOG_H
#define OBSTACLERECOG_H
// #include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

struct polar2cart{
    float angle;
    float range;
};

class ObstacleRecog
{
public:
    /**
     * @brief Constructor
     */
    ObstacleRecog();

    /**
     * @brief Destructor
     */
    ~ObstacleRecog();

    /**
     * @brief Detect Static obstacle
     * @param[in] scan Scan from LiDAR
     * @return True if static obstacle detected, else false
     */
    bool detectStatic(sensor_msgs::LaserScan scan);
};

#endif