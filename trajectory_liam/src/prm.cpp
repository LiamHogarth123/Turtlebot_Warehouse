// 1. Add start configuration cstart to R(N,E)
// 2. Loop
// 3. Randomly Select New Node c to expand
// 4. Randomly Generate new Node c’ from c
// 5. If edge e from c to c’ is collision-free
// 6. Add (c’, e) to R
// 7. If c’ belongs to endgame region, return path
// 8. Return if stopping criteria is met

#include <vector>
#include <utility> // for std::pair
#include "opencv2/opencv.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include "prm.h"


PRM::PRM() :
{
}


void PRM::setMap(MapSpecs temp){
    slam_map = temp;
}

void PRM::samplePoints(){


    std::vector<geometry_msgs::Point> points;
    std::random_device rd; // Obtain a random number from hardware
    std::mt19937 eng(rd()); // Seed the generator
    std::uniform_real_distribution<> distrX(minX, maxX); // Define the range for x
    std::uniform_real_distribution<> distrY(minY, maxY); // Define the range for y


    while(points.size() < numberOfPoints) {
        geometry_msgs::Point point;
        point.x = distrX(eng)
        point.y - distrY(eng)
        if(ValidPoint(point)) {
            points.push_back(point);

        }
    }          

    
}

bool PRM::ValidPoint(geometry_mgs::Point point){
    int grid_x = static_cast<int>((point.x - origin_x) / resolution);
    int grid_y = static_cast<int>((point.y - origin_y) / resolution);

    
    if(grid_x < 0 || grid_x >= slam_map.occupancyGrid.cols || grid_y < 0 || grid_y >= slam_map.occupancyGrid.rows) {
        return false; 
    }

    // Assuming occupancyGrid is a cv::Mat loaded from a PGM file
    int value = occupancyGrid.at<uchar>(grid_y, grid_x); // Note: y before x due to row-major order

    // Define your occupied threshold here. For simplicity, consider any positive value as occupied
    return value == 0; // Returns true if the cell is free (not occupied)

}
        

void PRM::createNodesAndEdges(){
    //create node check if next nearest node edges are clear


}

void PRM::findPath(int startNodeId, int goalNodeId){
    //A star

}

void setGoalNode(geometry_msgs::Point goal){
    // for nodes
        // if nodes x,y == goal 
            //   goal node = node(i)
}



