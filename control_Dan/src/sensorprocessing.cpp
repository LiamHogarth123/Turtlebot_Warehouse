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

geometry_msgs::Point Sensorprocessing::polarToCart(unsigned int index)
{
    
    float angle = laserScan.angle_min + laserScan.angle_increment*index;
    float range = laserScan.ranges.at(index);
    geometry_msgs::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}


double Sensorprocessing::findObstacle(){

    std::vector<std::pair<float, int>> scannedRange = scanningRange(20);
    double distance;
    double midpoint;
    int objectCount = 0; // counts how many objects in the scan
    int startingPoint;

    for (int i = 0; i < scannedRange.size(); i++) {
    
        int ObjStartingPt = i;

        if (scannedRange[i].first < 0.5) {
            objectCount++;
        
            while (scannedRange[i].first < 0.5 ) {
                if (i == scannedRange.size() - 1) {
                    break;
                }
                i++;
            }

        }

        if (objectCount == 1) {
            
        }

            geometry_msgs::Point obstacleStart = polarToCart(scannedRange[ObjStartingPt].second);
            geometry_msgs::Point obstacleEnd = polarToCart(scannedRange[i].second);

            distance = pow(pow((obstacleStart.x - obstacleEnd.x), 2) + pow((obstacleStart.y - obstacleEnd.y), 2), 0.5);

            midpoint = obstacleEnd.y - obstacleStart.y;
        

    }

    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "distance: " << distance << std::endl;
    std::cout << "midpoint: " << midpoint << std::endl;
    std::cout << "---------------------------------------------" << std::endl;
    
return midpoint;

}





std::vector<std::pair<float, int>> Sensorprocessing::scanningRange(float scanRange){ //scans from right to left

    float scanSize = laserScan.ranges.size();
    float degreeIndex = scanSize / 360;
    float scanIndex = round((scanRange/2) * degreeIndex); // The index of the scan at scanRange (degrees), scanRange/2 as there is left and right of 0 degrees
    

    // std::cout << "LaserScan.ranges: ";
    //     for (const auto& element : laserScan.ranges) {
    //         std::cout << element << " ";
    //     }
    //     std::cout << std::endl;

    // std::cout << "/////////////////////////////////////////////////////////////" << std::endl;

    // std::vector<float> scanPosDirection(laserScan.ranges.begin(), laserScan.ranges.begin() + scanIndex);
    // std::vector<float> scanNegDirection(laserScan.ranges.end() - scanIndex, laserScan.ranges.end());


    std::vector<std::pair<float, int>> scanPosDirection;
    std::vector<std::pair<float, int>> scanNegDirection;

    // Populate scanPosDirection with range data and indices
    for (int i = 0; i < scanIndex; ++i) {
        scanPosDirection.push_back(std::make_pair(laserScan.ranges[i], i));
    }

    // Populate scanNegDirection with range data and indices
    for (int i = laserScan.ranges.size() - scanIndex; i < laserScan.ranges.size(); ++i) {
        scanNegDirection.push_back(std::make_pair(laserScan.ranges[i], i));
    }



    // Combine the two vectors
    std::vector<std::pair<float, int>> combinedVector = scanNegDirection;
    for (const auto& element : scanPosDirection) {
        combinedVector.push_back(element);
    }

    // Display the combined vector
    std::cout << "Combined Vector: ";
    for (const auto& element : combinedVector) {
        std::cout << "(" << element.first << ", " << element.second << ") ";
    }
    std::cout << std::endl;

    return combinedVector;

}