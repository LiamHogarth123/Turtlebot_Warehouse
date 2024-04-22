#include "sensorprocessing.h"


using namespace std;

Sensorprocessing::Sensorprocessing(){   
    
    Turtlebot_min = 0.02;
    Turtlebot_max = 0.08;
}

void Sensorprocessing::Newdata(sensor_msgs::LaserScan temp_data){
    laserScan = temp_data;
    
}

void Sensorprocessing::PrintLaserSpec(){
    
    std::cout << "min" << std::endl;
    std::cout << laserScan.angle_min << std::endl;
    std::cout << "max" << std::endl;
    std::cout << laserScan.angle_max << std::endl;
    std::cout << "increment" << std::endl;
    std::cout << laserScan.angle_increment << std::endl;
}



double Sensorprocessing::findTurtlebot(){

    std::vector<float> scannedRange = scanningRange(20);
    double distance;
    double midpoint;

    for (int i = 0; i < scannedRange.at(i); i++) {
    
        int startingPt = i;

        while (laserScan.ranges.at(i) < 0.5 ) {
            if (i == laserScan.ranges.size() - 1) {
                break;
            }
            i++;
        }

        geometry_msgs::Point obstacleStart = polarToCart(startingPt);
        geometry_msgs::Point obstacleEnd = polarToCart(i - 1);

        distance = pow(pow((obstacleStart.x - obstacleEnd.x), 2) + pow((obstacleStart.y - obstacleEnd.y), 2), 0.5);

        midpoint = obstacleEnd.y - obstacleStart.y;

    }

    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "distance: " << distance << std::endl;
    std::cout << "midpoint: " << midpoint << std::endl;
    std::cout << "---------------------------------------------" << std::endl;
    
return midpoint;

}

geometry_msgs::Point Sensorprocessing::polarToCart(unsigned int index)
{
    
    float angle = laserScan.angle_min + laserScan.angle_increment*index;// + angle_range/2;
    float range = laserScan.ranges.at(index);
    geometry_msgs::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}



std::vector<float> Sensorprocessing::scanningRange(float scanRange){

    float scanSize = laserScan.ranges.size();
    float degreeIndex = scanSize / 360;
    float scanIndex = round((scanRange/2) * degreeIndex); // The index of the scan at scanRange (degrees), scanRange/2 as there is left and right of 0 degrees
    
    std::vector<float> scanPosDirection(laserScan.ranges.begin(), laserScan.ranges.begin() + scanIndex);
    std::vector<float> scanNegDirection(laserScan.ranges.end() - scanIndex, laserScan.ranges.end());

    // Combine the two vectors
    std::vector<float> combinedVector;
    for (const auto& element : scanPosDirection) {
        scanNegDirection.push_back(element);
    }
    std::cout << "scanindex: " << scanIndex << std::endl;//////////////////////////////////////////////////////////////////////////// continue progress here DC13/4
std::cout << "size: " << laserScan.ranges.size() << std::endl;

    // Display the combined vector
    std::cout << "Combined Vector: ";
    for (const auto& element : combinedVector) {
        std::cout << element << " ";
    }
    std::cout << std::endl;

    return combinedVector;

}