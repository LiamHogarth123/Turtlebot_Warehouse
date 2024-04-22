#include "obstaclerecog.h"
#include <math.h>
#include <vector>
#include <iostream>

ObstacleRecog::ObstacleRecog(ros::NodeHandle nh) : THRESHOLD_RANGE(0.3), THRESHOLD_WIDTH(0.01)
{
    subScan_ = nh_.subscribe("/scan", 1000, &ObstacleRecog::scanCallback, this);
}

ObstacleRecog::ObstacleRecog() : THRESHOLD_RANGE(0.3), THRESHOLD_WIDTH(0.01)
{
}

ObstacleRecog::~ObstacleRecog()
{
}

void ObstacleRecog::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
}

bool ObstacleRecog::detectStatic(sensor_msgs::LaserScan scan)
{
    // Boolean for if a static obstacle is detected
    bool obstacle = false;

    // Lower boundary of angle of scans to process
    float angle_lb = 0;

    // Upper boundary of angle of scans to process (30 degree arc)
    float angle_ub = M_PI / 6;

    // Store index of scans with satisfactory range
    for (unsigned int i = 0; i < scan.ranges.size(); i++)
    {
        if (i < angle_ub && (isfinite(scan.ranges.at(i))) && i < THRESHOLD_RANGE)
        {
            obstacle_indexes_.push_back(i);
        }
    }

    // Calculate reading location in local Cartesian coordinates
    for (unsigned int i = 0; i < obstacle_indexes_.size(); i++)
    {
        float range = scan.ranges.at(i);
        unsigned int index = obstacle_indexes_[i];
        float angle = scan.angle_min + scan.angle_increment * index;

        std::pair<float,float> carts = ObstacleRecog::polar2Cart(range, angle);
        obstacle_carts_.push_back(carts);
    }

    // Evaluate distances between obstacle points
    std::vector<int> segments;
    for (unsigned int i = 1; i < obstacle_carts_.size(); i++)
    {
        std::pair<float,float> prev = obstacle_carts_[i-1];
        std::pair<float,float> curr = obstacle_carts_[i];

        double distance = 0;

        distance = std::sqrt(std::pow(curr.first - prev.first, 2) + std::pow(curr.second - prev.second, 2));

        if (distance < THRESHOLD_WIDTH)
        {
            segments.push_back(1);
        }
    }

    // Threshold for an obstacle is 5 * THRESHOLD_WIDTH = (5cm / 0.05m)
    if (segments.size() > 5 * THRESHOLD_WIDTH)
    {
        obstacle = true;
    }
    else
    {
        obstacle = false;
    }

    // Return status
    return obstacle;
}

std::pair<float, float> ObstacleRecog::polar2Cart(float range, float angle)
{
    float x = range * cos(angle);
    float y = range * sin(angle);

    return std::make_pair(x,y);
}