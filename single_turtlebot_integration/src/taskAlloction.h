#include "ros/ros.h"
#include <nav_msgs/Odometry.h> // Include only necessary headers


class TaskAlloction {
  
public:
    TaskAlloction();

    // Function declarations
    std::vector<std::vector<std::pair<int, geometry_msgs::Point>>> taskAllocation();
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    void SetTurtlebotPositions(const std::vector<geometry_msgs::Point>& Turtlebot_Positions);
    void SetGoals(std::vector<int> temp_goalIds);
    void SetGoals();
  
private:
    std::vector<std::pair<int, geometry_msgs::Point>> itemLocations;
    std::vector<geometry_msgs::Point> robotPositions;

    std::pair<int, geometry_msgs::Point> location1;
    std::pair<int, geometry_msgs::Point> location2;
    std::pair<int, geometry_msgs::Point> location3;
    std::pair<int, geometry_msgs::Point> location4;
    std::pair<int, geometry_msgs::Point> location5;
    std::pair<int, geometry_msgs::Point> location6;
    std::pair<int, geometry_msgs::Point> location7;
    std::pair<int, geometry_msgs::Point> location8;
    std::pair<int, geometry_msgs::Point> location9;
    std::pair<int, geometry_msgs::Point> location10;
    std::pair<int, geometry_msgs::Point> location11;
    std::pair<int, geometry_msgs::Point> location12;
    std::pair<int, geometry_msgs::Point> location13;
    std::pair<int, geometry_msgs::Point> location14;
    std::pair<int, geometry_msgs::Point> location15;
    std::pair<int, geometry_msgs::Point> location16;
    std::pair<int, geometry_msgs::Point> location17;
    std::pair<int, geometry_msgs::Point> location18;
    std::pair<int, geometry_msgs::Point> location19;
    std::pair<int, geometry_msgs::Point> location20;

    bool randomiseGoals;

};

