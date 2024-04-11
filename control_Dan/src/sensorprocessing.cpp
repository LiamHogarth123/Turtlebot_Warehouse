#include "sensorprocessing.h"


using namespace std;

Sensorprocessing::Sensorprocessing(){   
    
    Turtlebot_min = 0.02;
    Turtlebot_max = 0.08;
}

void Sensorprocessing::Newdata(RobotData temp_data){
    Image_data = temp_data;
    
}

void Sensorprocessing::PrintLaserSpec(){
    
    std::cout << "min" << std::endl;
    std::cout << Image_data.laserScan.angle_min << std::endl;
    std::cout << "max" << std::endl;
    std::cout << Image_data.laserScan.angle_max << std::endl;
    std::cout << "increment" << std::endl;
    std::cout << Image_data.laserScan.angle_increment << std::endl;
}



geometry_msgs::Point Sensorprocessing::findTurtlebot(){
    geometry_msgs::Point cones;
    std::vector<std::pair<double, geometry_msgs::Point>> coneDistances;

    float scanAngle = 20;
    float scanSize = Image_data.laserScan.ranges.size();
    //given the scan angle is 180 degrees in the sim
    float scanAngleStart = ((180 - scanAngle) / 2) * (scanSize / 180); // starts scan for truck 
    float scanAngleEnd = (((180 - scanAngle) / 2) + scanAngle) * (scanSize / 180); // starts scan for truck 
    int startIndex = scanAngleStart;
    int endIndex = scanAngleEnd;

    for (int i = startIndex; i < endIndex; i++) {
    
        int startingPt = i;

        while (i < endIndex && Image_data.laserScan.ranges.at(i) < 0.5 ) {
            if (i == Image_data.laserScan.ranges.size() - 1) {
                break;
            }
            i++;
        }

        geometry_msgs::Point obstacleStart = polarToCart(startingPt);
        geometry_msgs::Point obstacleEnd = polarToCart(i - 1);

        double distance = pow(pow((obstacleStart.x - obstacleEnd.x), 2) + pow((obstacleStart.y - obstacleEnd.y), 2), 0.5);

        double midpoint = obstacleEnd.y - obstacleStart.x;
        
    }
    
return cones;

}

geometry_msgs::Point Sensorprocessing::polarToCart(unsigned int index)
{
    
    float angle = Image_data.laserScan.angle_min + Image_data.laserScan.angle_increment*index;// + angle_range/2;
    float range = Image_data.laserScan.ranges.at(index);
    geometry_msgs::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}



std::vector<geometry_msgs::Point> Sensorprocessing::findAllLaserPoints(){
    std::vector<geometry_msgs::Point> pointVector;

    for (int i = 1; i < Image_data.laserScan.ranges.size(); i++) { // for all readings
        if(!std::isinf(Image_data.laserScan.ranges.at(i))){ // if the number isn't infinity
            geometry_msgs::Point pt1 =polarToCart(i);
            pointVector.push_back(pt1);
        }
    }
    return pointVector;
}
