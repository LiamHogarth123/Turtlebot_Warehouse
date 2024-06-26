#include "taskAlloction.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include <fstream>
#include <random> // for std::random_device, std::mt19937
#include <algorithm> // for std::shuffle


TaskAlloction::TaskAlloction()
{
    robotPositions.clear();
    itemLocations.clear();

  
}

void TaskAlloction::SetTurtlebotPositions(const std::vector<geometry_msgs::Point>& Turtlebot_Positions){
    // std::vector<geometry_msgs::Point> robotPositions;
    robotPositions = Turtlebot_Positions;
    
    // geometry_msgs::Point robot1Pos;
    // robot1Pos.x = 0.0;
    // robot1Pos.y = 0.0;
    // robot1Pos.z = 0.0;
    // robotPositions.push_back(robot1Pos);

    // geometry_msgs::Point robot2Pos;
    // robot2Pos.x = 0.5;
    // robot2Pos.y = 1.25;
    // robot2Pos.z = 0.0;
    // robotPositions.push_back(robot2Pos);
}

void TaskAlloction::SetGoals(std::vector<int> temp_goalIds){
    SetGoals(); // loads all goal locations
    std::vector<int> goalIds = temp_goalIds;
    std::vector<std::pair<int, geometry_msgs::Point>> filteredLocations;

    // Iterate through itemLocations
    for (const auto& location : itemLocations) {
        // Check if the index is in goalIds
        if (std::find(goalIds.begin(), goalIds.end(), location.first) != goalIds.end()) {
            // If it is, add it to the filteredLocations vector
            filteredLocations.push_back(location);
        }
    }
    itemLocations = filteredLocations;

    // std::cout << "Item Location IDs:" << std::endl;
    // for (const auto& location : itemLocations) {
    //     std::cout << "ID: " << location.first << std::endl;
    // }

    randomiseGoals = false;
}

void TaskAlloction::SetGoals(){
     // item location variables
    geometry_msgs::Point item1;
    geometry_msgs::Point item2;
    geometry_msgs::Point item3;
    geometry_msgs::Point item4;
    geometry_msgs::Point item5;
    geometry_msgs::Point item6;
    geometry_msgs::Point item7;
    geometry_msgs::Point item8;
    geometry_msgs::Point item9;
    geometry_msgs::Point item10;
    geometry_msgs::Point item11;
    geometry_msgs::Point item12;
    geometry_msgs::Point item13;
    geometry_msgs::Point item14;
    geometry_msgs::Point item15;
    geometry_msgs::Point item16;
    geometry_msgs::Point item17;
    geometry_msgs::Point item18;
    geometry_msgs::Point item19;
    geometry_msgs::Point item20;


    geometry_msgs::Point deliveryLocation;

    // Define item locations
    item1.x = -3.2544;
    item1.y = 3.5730;
    item1.z = 0.0;
    
    item2.x = -2.7472;
    item2.y = 3.5726;
    item2.z = 0.0;

    item3.x = -1.2976;
    item3.y = 3.6621;
    item3.z = 0.0;

    item4.x = -0.7855;
    item4.y = 3.6251;
    item4.z = 0.0;
    
    item5.x = 0.7801;
    item5.y = 3.7118;
    item5.z = 0.0;

    item6.x = 1.2212;
    item6.y = 3.7347;
    item6.z = 0.0; 

    item7.x = 1.2171;
    item7.y = 3.1830;
    item7.z = 0.0;

    item8.x = 0.7765;
    item8.y = 3.1796;
    item8.z = 0.0;

    item9.x = -0.7314;
    item9.y = 3.0769;
    item9.z = 0.0;

    item10.x = -1.2151;
    item10.y = 3.0938;
    item10.z = 0.0;

    item11.x = -2.6439;
    item11.y = 3.0570;
    item11.z = 0.0;

    item12.x = -3.1716;
    item12.y = 3.0556;
    item12.z = 0.0;

    item13.x = 1.2809;
    item13.y = 1.6715;
    item13.z = 0.0;

    item14.x = 0.7911;
    item14.y = 1.6353;
    item14.z = 0.0;

    item15.x = -0.7227;
    item15.y = 1.6829;
    item15.z = 0.0;

    item16.x = -1.1151;
    item16.y = 1.6685;
    item16.z = 0.0;

    item17.x = -2.6844;
    item17.y = 1.6303;
    item17.z = 0.0;

    item18.x = -3.1243;
    item18.y = 1.6487;
    item18.z = 0.0;

    item19.x = 0.8086;
    item19.y = 1.0466;
    item19.z = 0.0;

    item20.x = 1.2534;
    item20.y = 1.0532;
    item20.z = 0.0;   

  
    itemLocations.push_back(location2 = {2, item2});
    itemLocations.push_back(location4 = {4, item4});
    itemLocations.push_back(location5 = {5, item5});
    itemLocations.push_back(location6 = {6, item6});
    itemLocations.push_back(location7 = {7, item7});
    itemLocations.push_back(location8 = {8, item8});
    itemLocations.push_back(location9 = {9, item9});
    itemLocations.push_back(location10 = {10, item10});
    itemLocations.push_back(location11 = {11, item11});
    itemLocations.push_back(location12 = {12, item12});
    itemLocations.push_back(location14 = {14, item14});
    itemLocations.push_back(location16 = {16, item16});
    itemLocations.push_back(location17 = {17, item17});
    itemLocations.push_back(location19 = {19, item19});
    itemLocations.push_back(location20 = {20, item20});

    itemLocations.clear();
    itemLocations.push_back(location9 = {9, item9});
    itemLocations.push_back(location1 = {1, item1});
    itemLocations.push_back(location13 = {13, item13});
    itemLocations.push_back(location15 = {15, item15});
    itemLocations.push_back(location18 = {18, item18});
    itemLocations.push_back(location3 = {3, item3});




    randomiseGoals = true;
}






std::vector<std::vector<std::pair<int, geometry_msgs::Point>>> TaskAlloction::taskAllocation(){
   


    if (randomiseGoals){
        // randomly select the goals
        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(itemLocations.begin(), itemLocations.end(), gen);
        // Select the first 6 locations
        itemLocations.resize(6); 
    } 

    // Set number of robots
    const int numRobots = robotPositions.size();

    std::vector<geometry_msgs::Point> deliveryLocation = Set_Delievery_goals(numRobots);

    // Vector to store points allocated to each robot
    std::vector<std::vector<std::pair<int, geometry_msgs::Point>>> allocatedPoints(numRobots);

    while (!itemLocations.empty()) {
        // Allocate one point to each robot based on distance
        for (int i = 0; i < numRobots; ++i) {
            if (itemLocations.empty()) break; // Break if there are no more items to allocate
            double minDistance = std::numeric_limits<double>::max();
            int closestItemIndex = -1;

            // Find the closest item to the current robot position
            for (size_t j = 0; j < itemLocations.size(); ++j) {
                double distance = calculateDistance(itemLocations[j].second, robotPositions[i]);
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
            // std::cout << "Robot " << i + 1 << " allocated points:" << std::endl;
            for (const auto& point : allocatedPoints[i]) {
                // std::cout << "Point: (" << point.second.x << ", " << point.second.y << ", " << point.second.z << ")" << std::endl;
            }
        }

        // Update robot positions to the first allocated points
        for (int i = 0; i < numRobots; ++i) {
            if (!allocatedPoints[i].empty()) {
                robotPositions[i] = allocatedPoints[i][0].second;
                // Clear the allocated points for the robot
                // allocatedPoints[i].clear();
                // std::cout << "Robot " << i + 1 << " position after goal: (" << robotPositions[i].x << ", " << robotPositions[i].y << ", " << robotPositions[i].z << ")" << std::endl;
            }
        }
    }

    // Visit drop-off location after completing item collections
    for (int i = 0; i < numRobots; ++i) {
        allocatedPoints[i].push_back({-1, deliveryLocation.at(i)}); // Use the ID:'-1' to indicate the drop-off location
        // std::cout << "Robot " << i + 1 << " visited drop-off location: (" << deliveryLocation.at(i).x << ", " << deliveryLocation.at(i).y << ", " << deliveryLocation.at(i).z << ")" << std::endl;
    }
  


    return allocatedPoints;
}


double TaskAlloction::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}




std::vector<geometry_msgs::Point> TaskAlloction::Set_Delievery_goals(int num_robots) {
    std::vector<geometry_msgs::Point> deliveryLocations;

    // Define the default delivery location
    geometry_msgs::Point defaultDeliveryLocation;
    defaultDeliveryLocation.x = 0;
    defaultDeliveryLocation.y = 0;

    // Define offsets
    double lateralOffset = 0.6; // Offset to the left and right
    double forwardOffset = 1.0; // Forward offset for the next set of robots

    for (int i = 0; i < num_robots; ++i) {
        geometry_msgs::Point location;

        if (i == 0) {
            // First robot's delivery location is at the default location
            location = defaultDeliveryLocation;
        } else {
            int set = i / 3; // Determine which set of 3 the robot belongs to
            int position = i % 3; // Determine the position within the set

            // Set the y coordinate based on the set number and forwardOffset
            location.x = set * forwardOffset;

            // Set the x coordinate based on the position within the set
            if (position == 0) {
                location.y = 0.0; // Center
            } else if (position == 1) {
                location.y = -lateralOffset; // Left
            } else if (position == 2) {
                location.y = lateralOffset; // Right
            }
        }

        deliveryLocations.push_back(location);
    }

    return deliveryLocations;
}