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
#include <random>


PRM::PRM() {
}


void PRM::setMap(MapSpecs temp){
    slam_map = temp;
    numberOfPoints_ =  200;
}


void PRM::samplePoints(){
    //graph definition
    Graph.clear();
    int id = 0;
    std::cout <<"map infor" << std::endl;
    std::cout << latestMapMetaData_.height << std::endl;
        std::cout << latestMapMetaData_.width << std::endl;
    //Random generators 
    std::random_device rd; // Obtain a random number from hardware
    std::mt19937 eng(rd()); // Seed the generator
    std::uniform_real_distribution<> distrX(0, latestMapMetaData_.width); // Define the range for x
    std::uniform_real_distribution<> distrY(0, latestMapMetaData_.height); // Define the range for y

    // std::cout << "sampling points" << std::endl;
 
    double Max_y = 0;
    double min_y = 9999999;

    
    while(Graph.size() < numberOfPoints_) {

        // std::cout << "while loop opens" << std::endl;

        geometry_msgs::Point point;
        point.x = distrX(eng);
        point.y = distrY(eng);
        // std::cout << "random data point" << std::endl;
        // std::cout << point.x << std::endl;
        std::cout << point.y << std::endl;
        if(ValidPoint(point)) {
            // std::cout << "if opens" << std::endl;
            Node temp;
            temp.x = point.x;
            temp.y = point.y - 16;   
            temp.id = id;
            Graph.push_back(temp);
            // std::cout << "if finished" << std::endl;

            if (temp.y > Max_y){
                Max_y = temp.y;
            }
            if (temp.y < min_y){
                min_y = temp.y;
            }


        }
        else {
            std::cout << "invalid point" << std::endl;
        }

        // std::cout << "Graph.size"  << std::endl;
    }          
    std::cout << "min max" << std::endl;
    std::cout << Max_y << std::endl;
    std::cout << min_y << std::endl;

    createNodesAndEdges(Graph);



}

bool PRM::ValidPoint(geometry_msgs::Point point){
    int grid_x = static_cast<int>(point.x);
    int grid_y = static_cast<int>(point.y);



    // Calculate index using grid indices
    int index = grid_x + (grid_y * SlamMapData.info.width);
    int above = index + SlamMapData.info.width;
    int below = index - SlamMapData.info.width;


    if (index >= SlamMapData.data.size()){
        return false;
    }


    if (SlamMapData.data.at(index) == 0 && SlamMapData.data.at(above) == 0 && SlamMapData.data.at(below) == 0 && SlamMapData.data.at(index+1) == 0 && SlamMapData.data.at(index-1) == 0 ) {
        return true;
    }
    else {
        return false;
    }
}

bool PRM::pathIsClear(Node Node_A, Node Node_B){
    int m_new = 2 * (Node_B.y - Node_A.y); 
    int slope_error_new = m_new - (Node_B.x - Node_A.x); 
    for (int x = Node_A.x, y = Node_A.y; x <= Node_B.x; x++) { 
        std::cout << "(" << x << "," << y << ")\n"; 

        geometry_msgs::Point temp;
        temp.x = x;
        temp.y = y;


        if (ValidPoint(temp)){
            // Add slope to increment angle formed 
            slope_error_new += m_new; 
  
            // Slope error reached limit, time to 
            // increment y and update slope error. 
            if (slope_error_new >= 0) { 
                y++; 
                slope_error_new -= 2 * (Node_B.x - Node_A.x); 
            } 
        }
        else {
            return false;
        }
    } 
    return true;
}

void PRM::createNodesAndEdges(std::vector<Node> Graph){
    float max_distance = 2;
    for (auto Node_A : Graph){
        for (auto Node_B : Graph){
            if (Node_A.id = Node_B.id){
                continue;   
            }
            float distance = sqrt(pow(Node_A.x - Node_B.x, 2)+ pow(Node_A.x - Node_B.x, 2));
            if (distance < max_distance && pathIsClear(Node_A, Node_B)){
                Node_A.edges.push_back(Node_B.id);
            }
        }

    }

}

void PRM::findPath(int startNodeId, int goalNodeId){
    //A star

}

void PRM::setGoalNode(geometry_msgs::Point goal){
    // for nodes
        // if nodes x,y == goal 
            //   goal node = node(i)
}

void PRM::UpdateMapData(nav_msgs::OccupancyGrid map, nav_msgs::MapMetaData MapMetaData_) {
    std::cout << "PRM map data openning" << std::endl;
    SlamMapData = map;
    numberOfPoints_ =  100;
    latestMapMetaData_ = MapMetaData_;
}

void PRM::visualise_PRM() {
    std::cout << "visualization opens" << std::endl;
    // Load the grayscale map image
    cv::Mat grayscaleMapImage = cv::imread("/home/liam/catkin_ws/src/navigation/map_server/maps/map.pgm", cv::IMREAD_GRAYSCALE);
    if (grayscaleMapImage.empty()) {
        std::cerr << "Could not open or find the map image" << std::endl;
        return; // Exit if the image could not be loaded
    }
    std::cout << "map read" << std::endl;

    // Convert the grayscale image to a BGR (color) image
    cv::Mat mapImage;
    cv::cvtColor(grayscaleMapImage, mapImage, cv::COLOR_GRAY2BGR);





//  test draw big circle/////////////////////////////////////////////////////////
    // cv::Point center(mapImage.cols / 2, mapImage.rows / 2);
    // int radius = 50; // You can adjust this value as needed

    // // Specify the color of the circle in BGR format (red in this case)
    // cv::Scalar color(0, 0, 255); // BGR value for red

    // // Draw the circle
    // cv::circle(mapImage, center, radius, color, -1);



// Draws the nodes without ajustment
///////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Assuming you have a list of nodes from your PRM
    std::vector<Node> nodes = Graph;
    std::cout << Graph.size() << " nodes" << std::endl;

    int radius = 1; // You can adjust this value as needed
    cv::Scalar color(0, 0, 255); // BGR value for red


    //  test draw big circle
    for (auto node : Graph){
    
        cv::Point center(node.x, node.y);

        if (center.x >= 0 && center.x < mapImage.cols && center.y >= 0 && center.y < mapImage.rows) {
            cv::circle(mapImage, center, radius, color, -1);
        } 
        else {
            // The center is outside the image, handle accordingly
            // This could mean adjusting the position, skipping the draw, etc.
            std::cout << "out of bounds" << std::endl;
        }

    }

    
    
    


    // std::vector<Node> nodes = Graph;

    // // Convert nodes' positions to pixel coordinates and draw them
    // float resolution = latestMapMetaData_.resolution; // Map resolution in meters/pixel
    // float origin_x = latestMapMetaData_.origin.position.x; // x-coordinate of the map's origin in meters
    // float origin_y = latestMapMetaData_.origin.position.y; // y-coordinate of the map's origin in meters
    // int radius = 1; // Smaller radius for nodes to be more distinct
    // for (const auto& node : nodes) {
    //     int pixel_x = static_cast<int>((node.x - origin_x) / resolution);
    //     int pixel_y = static_cast<int>((node.y - origin_y) / resolution);

    //     std::cout  << pixel_x << std::endl;
    //     std::cout  << pixel_y << std::endl;

    //     if (pixel_x >= 0 && pixel_x < mapImage.cols && pixel_y >= 0 && pixel_y < mapImage.rows) {
    //         // The center is inside the image, draw the circle
    //         cv::circle(mapImage, cv::Point(pixel_x, pixel_y), radius, cv::Scalar(0, 0, 255), -1);
    //     } else {
    //         // The center is outside the image, handle accordingly
    //         std::cout << "out of bounds" << std::endl;
    //         // This could mean adjusting the position, skipping the draw, etc.
    //     }
        

    //     // Optionally, draw edges if you have the coordinates of connected nodes
    //     // This part is commented out for brevity
    // }
    // std::cout << "loop completed" << std::endl;

    // Display the map with nodes
    cv::namedWindow("SLAM Map with Nodes", cv::WINDOW_AUTOSIZE);
    cv::imshow("SLAM Map with Nodes", mapImage);
    // Optionally, save the color map with nodes to a file
    cv::imwrite("/home/liam/Desktop/map_with_nodes fixed!!!.png", mapImage);

    cv::waitKey(0);
}
