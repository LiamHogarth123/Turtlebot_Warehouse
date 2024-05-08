#include "taskAlloction.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include <fstream>

TaskAlloction::TaskAlloction()
{
    robotPositions.clear();
  
}

void TaskAlloction::SetTurtlebotPositions(const std::vector<geometry_msgs::Point>& Turtlebot_Positions){
    std::vector<geometry_msgs::Point> robotPositions;
    geometry_msgs::Point robot1Pos;
    robot1Pos.x = 0.0;
    robot1Pos.y = 0.0;
    robot1Pos.z = 0.0;
    robotPositions.push_back(robot1Pos);

    geometry_msgs::Point robot2Pos;
    robot2Pos.x = 0.5;
    robot2Pos.y = 1.25;
    robot2Pos.z = 0.0;
    robotPositions.push_back(robot2Pos);
}

void TaskAlloction::SetGoals(const std::vector<geometry_msgs::Point> goals){
    itemLocations = goals;
}

void TaskAlloction::SetGoals(){
    // item location variables
    geometry_msgs::Point item1;
    geometry_msgs::Point item2;
    geometry_msgs::Point item3;
    geometry_msgs::Point item4;
    geometry_msgs::Point item5;
    geometry_msgs::Point item6;
    // Define item locations
    item1.x = 2.75;
    item1.y = 5.2;
    item1.z = 0.0;

    item2.x = 12.4;
    item2.y = 8.0;
    item2.z = 0.0;

    item3.x = -4.0;
    item3.y = -8.0;
    item3.z = 0.0;

    item4.x = 7.1;
    item4.y = 5.0;
    item4.z = 0.0;

    item5.x = 13.1;
    item5.y = -10.0;
    item5.z = 0.0;

    item6.x = 8.1;
    item6.y = -3.0;
    item6.z = 0.0;    
   
    itemLocations.push_back(item1);
    itemLocations.push_back(item2);
    itemLocations.push_back(item3);
    itemLocations.push_back(item4);
    itemLocations.push_back(item5);
    itemLocations.push_back(item6);

}


std::vector<std::vector<geometry_msgs::Point>> TaskAlloction::taskAllocation(){
   

    geometry_msgs::Point deliveryLocation;

    
    // Hypothetical drop off location
    deliveryLocation.x = 20.0;
    deliveryLocation.y = 20.0;
    deliveryLocation.z = 0.0;

    // Store item locations in the vector
    
   



    // initial robot positions
    std::vector<geometry_msgs::Point> robotPositions;
    geometry_msgs::Point robot1Pos;
    robot1Pos.x = 0.0;
    robot1Pos.y = 0.0;
    robot1Pos.z = 0.0;
    robotPositions.push_back(robot1Pos);

    geometry_msgs::Point robot2Pos;
    robot2Pos.x = 0.5;
    robot2Pos.y = 1.25;
    robot2Pos.z = 0.0;
    robotPositions.push_back(robot2Pos);

    // Set number of robots
    const int numRobots = robotPositions.size();

    // Vector to store points allocated to each robot
    std::vector<std::vector<geometry_msgs::Point>> allocatedPoints(numRobots);

    while (!itemLocations.empty()) {
        // Allocate one point to each robot based on distance
        for (int i = 0; i < numRobots; ++i) {
            if (itemLocations.empty()) break; // Break if there are no more items to allocate
            double minDistance = std::numeric_limits<double>::max();
            int closestItemIndex = -1;

            // Find the closest item to the current robot position
            for (size_t j = 0; j < itemLocations.size(); ++j) {
                double distance = calculateDistance(itemLocations[j], robotPositions[i]);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestItemIndex = j;
                }
            }

            // Assign the closest item to the robot
            if (closestItemIndex != -1) {
                allocatedPoints[i].push_back(itemLocations[closestItemIndex]);
                itemLocations.erase(itemLocations.begin() + closestItemIndex); // Remove allocated item
            }
        }

        // Print allocated points for each robot
        for (int i = 0; i < numRobots; ++i) {
            std::cout << "Robot " << i + 1 << " allocated points:" << std::endl;
            for (const auto& point : allocatedPoints[i]) {
                std::cout << "Point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
            }
        }

        // Update robot positions to the first allocated points
        for (int i = 0; i < numRobots; ++i) {
            if (!allocatedPoints[i].empty()) {
                robotPositions[i] = allocatedPoints[i][0];
                // Clear the allocated points for the robot
                allocatedPoints[i].clear();
                std::cout << "Robot " << i + 1 << " position after goal: (" << robotPositions[i].x << ", " << robotPositions[i].y << ", " << robotPositions[i].z << ")" << std::endl;
            }
        }
    }

    // Visit drop-off location after completing item collections
    for (int i = 0; i < numRobots; ++i) {
        allocatedPoints[i].push_back(deliveryLocation);
        std::cout << "Robot " << i + 1 << " visited drop-off location: (" << deliveryLocation.x << ", " << deliveryLocation.y << ", " << deliveryLocation.z << ")" << std::endl;
    }

    return allocatedPoints;
}


double TaskAlloction::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}
