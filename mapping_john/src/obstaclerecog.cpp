#include "obstaclerecog.h"
#include <math.h>
#include <vector>
#include <iostream>

ObstacleRecog::ObstacleRecog()
{

}

ObstacleRecog::~ObstacleRecog()
{

}

bool ObstacleRecog::detectStatic(sensor_msgs::LaserScan scan)
{
    // Boolean for if a static obstacle is detected
    bool obstacle = false;

    // Minimum angle of scans to process
    float angle_min = 0;

    // Maximum angle of scans to process (30 degree arc)
    float angle_max = M_PI/6;

    // Increment of angle to iterate through (10 times)
    float angle_increment = M_PI/60;

    // Check range for each scan value

    // Calculate reading location in local coordinates
    

    // Return status
    return obstacle;
}