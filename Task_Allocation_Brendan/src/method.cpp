#include "method.h"
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


Method::Method(ros::NodeHandle nh) :
  nh_(nh)

{

  
}

void Method::seperateThread() {
  //User input
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Call the pickupPointAllocation function to initialize itemLocations
    taskAllocation();
  
}

std::vector<std::vector<geometry_msgs::Point>> Method::taskAllocation(){
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
    item12.y = -3.0556;
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

    // Hypothetical drop off location
    deliveryLocation.x = 0;
    deliveryLocation.y = 0.1;
    deliveryLocation.z = 0.0;

 // Store item index and locations in the vector of pairs
std::vector<std::pair<int, geometry_msgs::Point>> itemLocations;
    itemLocations.push_back({1, item1});
    itemLocations.push_back({2, item2});
    itemLocations.push_back({3, item3});
    itemLocations.push_back({4, item4});
    itemLocations.push_back({5, item5});
    itemLocations.push_back({6, item6});
    itemLocations.push_back({7, item7});
    itemLocations.push_back({8, item8});
    itemLocations.push_back({9, item9});
    itemLocations.push_back({10, item10});
    itemLocations.push_back({11, item11});
    itemLocations.push_back({12, item12});
    itemLocations.push_back({13, item13});
    itemLocations.push_back({14, item14});
    itemLocations.push_back({15, item15});
    itemLocations.push_back({16, item16});
    itemLocations.push_back({17, item17});
    itemLocations.push_back({18, item18});
    itemLocations.push_back({19, item19});
    itemLocations.push_back({20, item20});


    // randomly select the goals
        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(itemLocations.begin(), itemLocations.end(), gen);
    // Select the first 6 locations
        itemLocations.resize(6); 


    // Set number of robots
    const int numRobots = 2;

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

    // Vector to store points allocated to each robot
    std::vector<std::vector<geometry_msgs::Point>> allocatedPoints(numRobots);

while (!itemLocations.empty()) {
    // Print remaining goals
    std::cout << "Remaining goals: ";
    for (size_t i = 0; i < itemLocations.size(); ++i) {
        std::cout << "Item " << itemLocations[i].first << ": (" << itemLocations[i].second.x << ", " << itemLocations[i].second.y << ", " << itemLocations[i].second.z << ") ";
    }
    std::cout << std::endl;

    // Print visited goals for each robot
    for (int i = 0; i < numRobots; ++i) {
        std::cout << "Robot " << i + 1 << " visited their allocated goals... ";
        for (const auto& point : allocatedPoints[i]) {
            std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ") ";
        }
        std::cout << std::endl;
    }

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
            std::cout << "Robot " << i + 1 << " selected item " << itemLocations[closestItemIndex].first << " at (" << itemLocations[closestItemIndex].second.x << ", " << itemLocations[closestItemIndex].second.y << ", " << itemLocations[closestItemIndex].second.z << ")" << std::endl;
            allocatedPoints[i].push_back(itemLocations[closestItemIndex].second);
            itemLocations.erase(itemLocations.begin() + closestItemIndex); // Remove allocated item
        }
    }

    // Update robot positions to the first allocated points
    for (int i = 0; i < numRobots; ++i) {
        if (!allocatedPoints[i].empty()) {
            robotPositions[i] = allocatedPoints[i][0];
        }
    }
}

// Visit drop-off location after completing item collections
for (int i = 0; i < numRobots; ++i) {
    double distanceToGoal = calculateDistance(robotPositions[i], deliveryLocation);
    std::cout << "Robot " << i + 1 << " reached the drop-off location at (" << deliveryLocation.x << ", " << deliveryLocation.y << ", " << deliveryLocation.z << ")" << std::endl;
    allocatedPoints[i].push_back(deliveryLocation);
}

return allocatedPoints;
}
double Method::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}
