#ifndef OBSTACLERECOG_H
#define OBSTACLERECOG_H
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

class ObstacleRecog
{
public:
    /**
     * @brief Constructor with nodehandle
     */
    ObstacleRecog(ros::NodeHandle nh);
    
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

private:
    /**
     * @brief Convert polar coordinates to Cartesian coordinates
     * @param[in] range Range value from scan
     * @param[in] index Index value of scan
     * @return [x,y] Coordinates in Cartesian plane
    */
   std::pair<float,float> polar2Cart(float range, float angle);

    /** Threshold value for obstacle point to be of concern*/
    const double THRESHOLD_RANGE;

    /** Threshold value for obstacle points to be considered an obstacle*/
    const double THRESHOLD_WIDTH;

    /** Scans with obstacle points under threshold*/
    std::vector<int> obstacle_indexes_;

    /** Cartesian coordinates of obstacle points*/
    std::vector<std::pair<double,double>> obstacle_carts_;

    /**
     * @brief Scan Callback
     * @param sensor_msgs::LaserScan::ConstPtr - The scan message
     * @note This function and the declaration are ROS specific
     * @return void
     */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

protected:
    /** Nodehandle for this node. Note, only 1 nodehandle is required (there is only 1 node).*/
    ros::NodeHandle nh_;

    /**
     * Subscriber to image topic to get image from RGB-D sensor
     * @typedef sensor_msgs/LaserScan
     * @topic /scan
     */
    ros::Subscriber subScan_;
};

#endif