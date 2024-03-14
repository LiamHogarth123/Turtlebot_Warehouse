#include "method.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <nav_msgs/Odometry.h>

#include "ros/ros.h"

#include <fstream>



using std::cout;
using std::endl;

Method::Method(ros::NodeHandle nh) :
  nh_(nh)

{

  
}

void Method::seperateThread() {
  //User input
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
  std::cout << "Hello World" << std::endl;

}

void pickupPointAllocation(){
  // assign x and y values for the item locations
    std::vector<geometry_msgs::Point> itemLocations;
  // item location variables
    geometry_msgs::Point item1;
    geometry_msgs::Point item2;
    geometry_msgs::Point item3;
    geometry_msgs::Point item4;
    geometry_msgs::Point item5;
    geometry_msgs::Point item6;
    geometry_msgs::Point deliveryLocation;
// can accept as many goals but for demonstration only 6 are required...
  item1.x = 1.0;
  item1.y = 1.0;
  item1.z = 0.0;

  item2.x = 4.5;
  item2.y = 3.1;
  item2.z = 0.0;

  item3.x = 6.8;
  item3.y = 9.3;
  item3.z = 0.0;

  item4.x = 6.8;
  item4.y = 9.3;
  item4.z = 0.0;

  item5.x = 6.8;
  item5.y = 9.3;
  item5.z = 0.0;

  item6.x = 6.8;
  item6.y = 9.3;
  item6.z = 0.0;
// location of the hypothetical drop off location to be updated for proper implementation
  deliveryLocation.x = 20.0;
  deliveryLocation.y = 20.0;
  deliveryLocation.z = 0.0;

  // store these items in the vector
  itemLocations.push_back(item1);
  itemLocations.push_back(item2);
  itemLocations.push_back(item3);
  itemLocations.push_back(item4);
  itemLocations.push_back(item5);
  itemLocations.push_back(item6);

}


std::vector<std::pair<geometry_msgs::Point, std::pair<double, double>>> Method::euclideanDistance(const std::vector<geometry_msgs::Point>& itemLocations, 
                                                                                                  const geometry_msgs::Pose& robot1Location, 
                                                                                                  const geometry_msgs::Pose& robot2Location) {
    std::vector<std::pair<geometry_msgs::Point, std::pair<double, double>>> sortedLocationsAndDistances;
    std::vector<bool> assigned(itemLocations.size(), false); // Keep track of assigned points

    for (const auto& point : itemLocations) {
        double item_x1 = point.x - robot1Location.position.x;
        double item_y1 = point.y - robot1Location.position.y;
        double directDistance1 = sqrt(std::pow(item_x1, 2) + std::pow(item_y1, 2));

        double item_x2 = point.x - robot2Location.position.x;
        double item_y2 = point.y - robot2Location.position.y;
        double directDistance2 = sqrt(std::pow(item_x2, 2) + std::pow(item_y2, 2));

        sortedLocationsAndDistances.push_back(std::make_pair(point, std::make_pair(directDistance1, directDistance2)));

        // Mark the point as assigned to the closest robot
        if (directDistance1 < directDistance2) {
            assigned[&point - &itemLocations[0]] = true; // Calculate index of point in itemLocations
        }
    }

    std::sort(sortedLocationsAndDistances.begin(), sortedLocationsAndDistances.end(),
              [](const std::pair<geometry_msgs::Point, std::pair<double, double>>& a, const std::pair<geometry_msgs::Point, std::pair<double, double>>& b) {
                  double distance1a = a.second.first;
                  double distance1b = b.second.first;
                  double distance2a = a.second.second;
                  double distance2b = b.second.second;
                  double totalDistanceA = distance1a + distance2a;
                  double totalDistanceB = distance1b + distance2b;
                  return totalDistanceA < totalDistanceB;
              });

    // Remove points that have been assigned to one of the robots
    sortedLocationsAndDistances.erase(
        std::remove_if(sortedLocationsAndDistances.begin(), sortedLocationsAndDistances.end(),
                       [itemLocations, &assigned](const std::pair<geometry_msgs::Point, std::pair<double, double>>& pair) {
                           return assigned[&pair.first - &itemLocations[0]]; // Calculate index of point in itemLocations
                       }),
        sortedLocationsAndDistances.end());

    return sortedLocationsAndDistances;
}