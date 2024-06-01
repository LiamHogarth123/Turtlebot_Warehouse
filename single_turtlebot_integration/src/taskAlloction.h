#include "ros/ros.h"
#include <nav_msgs/Odometry.h> // Include only necessary headers


class TaskAlloction {
  
public:
    TaskAlloction();

    // Function declarations
    std::vector<std::vector<std::pair<int, geometry_msgs::Point>>> taskAllocation();
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    void SetTurtlebotPositions(const std::vector<geometry_msgs::Point>& Turtlebot_Positions);
    void SetGoals(const std::vector<std::pair<int, geometry_msgs::Point>> goals);
    void SetGoals();
  
private:
    std::vector<std::pair<int, geometry_msgs::Point>> itemLocations;
    std::vector<geometry_msgs::Point> robotPositions;
};

