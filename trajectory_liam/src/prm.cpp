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


std::vector<Node> PRM::samplePoints(){
    //graph definition
    Graph.clear();
    int id = 0;
    // std::cout <<"map infor" << std::endl;
    // std::cout << latestMapMetaData_.height << std::endl;
    //     std::cout << latestMapMetaData_.width << std::endl;
    //Random generators 
    std::random_device rd; // Obtain a random number from hardware
    std::mt19937 eng(rd()); // Seed the generator
    std::uniform_real_distribution<> distrX(0, latestMapMetaData_.width); // Define the range for x
    std::uniform_real_distribution<> distrY(0, latestMapMetaData_.height); // Define the range for y

    // std::cout << "sampling points" << std::endl;
 
    double Max_y = 0;
    double min_y = 9999999;

    // std::cout <<numberOfPoints_ << std::endl;
    while(Graph.size() < numberOfPoints_) {
        // std::cout << "loop started"<< std::endl;
        // std::cout << "while loop opens" << std::endl;

        geometry_msgs::Point point;
        point.x = distrX(eng);
        point.y = distrY(eng);
        // std::cout << "random data point" << std::endl;
        // std::cout << point.x << std::endl;
        
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
        }

        // std::cout << "Graph.size"  << std::endl;
    }          
    return Graph;
   



}

bool PRM::ValidPoint(geometry_msgs::Point point){
    int grid_x = static_cast<int>(point.x);
    int grid_y = static_cast<int>(point.y);

    // Calculate index using grid indices
    int index = grid_x + (grid_y * SlamMapData.info.width);
    int above = index + 2*SlamMapData.info.width;
    int below = index - 2*SlamMapData.info.width;

    if (index >= SlamMapData.data.size()){
        return false;
    }


    if (SlamMapData.data.at(index) == 0 && SlamMapData.data.at(above) == 0 && SlamMapData.data.at(below) == 0 && SlamMapData.data.at(index+2) == 0 && SlamMapData.data.at(index-2) == 0 ) {
        return true;
    }
    else {
        return false;
    }
}

// bool PRM::pathIsClear(Node Node_A, Node Node_B){
//     int m_new = 2 * (Node_B.y - Node_A.y); 
//     int slope_error_new = m_new - (Node_B.x - Node_A.x); 
//     for (int x = Node_A.x, y = Node_A.y; x <= Node_B.x; x++) { 
//         // std::cout << "(" << x << "," << y << ")\n"; 
//         std::cout << "x" << x << std::endl;
//         geometry_msgs::Point temp;
//         temp.x = x;
//         temp.y = y;
//         cv::Point tempry;
//         tempry.x = x;
//         tempry.y = y;

//         path_points.push_back(tempry);
        


//         if (ValidPoint(temp)){
//             // Add slope to increment angle formed 
//             slope_error_new += m_new; 
  
//             // Slope error reached limit, time to 
//             // increment y and update slope error. 
//             if (slope_error_new >= 0) { 
//                 y++; 
//                 slope_error_new -= 2 * (Node_B.x - Node_A.x); 
//                 temp.y = y;

//                 if (!ValidPoint(temp)){
//                     return false;
//                     std::cout << "invalided edge method 2" << std::endl;

//                 }
//             } 
//             std::cout << "valided edge" << std::endl;
//         }
//         else {
//             std::cout << "invalided edge" << std::endl;
//             return false;
//         }
//     } 
//     return true;

bool PRM::pathIsClear(Node Node_A, Node Node_B){
    int dx = std::abs(Node_B.x - Node_A.x);
    int dy = -std::abs(Node_B.y - Node_A.y);
    int sx = Node_A.x < Node_B.x ? 1 : -1;
    int sy = Node_A.y < Node_B.y ? 1 : -1;
    int err = dx + dy, e2; /* error value e_xy */
    
    while (true) {
        // Check current position
        geometry_msgs::Point temp;
        temp.x = Node_A.x;
        temp.y = Node_A.y;
        cv::Point tempry;
        tempry.x = temp.x;
        tempry.y = temp.y;

        path_points.push_back(tempry);

        if (!ValidPoint(temp)){
            std::cout << "invalided edge" << std::endl;
            return false;
        }

        if (Node_A.x == Node_B.x && Node_A.y == Node_B.y) break; // Check if end point is reached
        e2 = 2 * err;
        if (e2 >= dy) { 
            err += dy; /* e_xy+e_x > 0 */
            Node_A.x += sx;
        }
        if (e2 <= dx) { /* e_xy+e_y < 0 */
            err += dx;
            Node_A.y += sy;
        }
    }
    return true; // Path is clear if no breaks occurred
}


std::vector<Node> PRM::createNodesAndEdges(std::vector<Node> Graph_){
    float max_distance = 20;
    int edges =0;
    for (size_t k = 0; k < Graph_.size(); k++){
        for (size_t j = 0; j < Graph_.size(); j++){
            // Skip self-comparison
            if (Graph[k].id == Graph[j].id){
                j++;
            }
            
            // Correct distance calculation
            float distance = sqrt(pow(Graph_[k].x - Graph_[j].x, 2) + pow(Graph_[k].y - Graph_[j].y, 2));
            //     Graph[k].edges.push_back(Graph[j].id);
            //     edges++;
                // std::cout << Graph[k].edges.size() << std::endl;
            if (distance < max_distance && pathIsClear(Graph_[k], Graph_[j])){
                Graph_[k].edges.push_back(Graph[j].id);
                edges++;
                std::cout << Graph[k].edges.size() << std::endl;
            }
            else {
                // Graph_[k].edges.push_back(Graph[j].id);
            }
        }
    }

    std::cout << "Edges added: " << edges << std::endl;

    // Use index-based loop for consistent style and efficiency
    for (size_t i = 0; i < Graph_.size(); i++){
        std::cout << "node" << i << ": " << Graph_[i].edges.size() << " edges" << std::endl;
    }
    return Graph_;
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
    numberOfPoints_ =  10;
    latestMapMetaData_ = MapMetaData_;
}

void PRM::visualise_PRM(std::vector<Node> Graph_) {
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


    for (size_t i = 0; i < Graph_.size(); i++){
        std::cout << "node" << i << ": " << Graph_[i].edges.size() << " edges" << std::endl;
    }




    
    int radius = 0; // You can adjust this value as needed
    cv::Scalar color(0, 0, 255); // BGR value for red


    //  test draw big circle
    for (size_t k = 0; k < Graph_.size(); k++){
    
        std::cout << k << std::endl;
        cv::Point center(Graph_[k].x, Graph_[k].y);

        if (center.x >= 0 && center.x < mapImage.cols && center.y >= 0 && center.y < mapImage.rows) {
            
            cv::circle(mapImage, center, radius, color, -1);

   
            for (size_t l = 0; l < Graph_.at(k).edges.size(); l++) {
               std::cout << "l =" << l << std::endl;
                // Safety check
                // if (connected_node_index < 0 || connected_node_index >= Graph.size()) continue;

                int index = Graph_.at(k).edges.at(l);
                // Convert connected node position from world coordinates to pixel coordinates
                const auto& connected_node = Graph_[index];
                
               
              
                cv::Point connected_node_center(connected_node.x, connected_node.y);

                // Draw the line
                cv::line(mapImage, center, connected_node_center, cv::Scalar(255, 0, 0) /*blue*/, 0  /*thickness*/);
            }
            

        } 
        else {
            // The center is outside the image, handle accordingly
            // This could mean adjusting the position, skipping the draw, etc.
            std::cout << "out of bounds" << std::endl;
        }

    }


    
 

    cv::Scalar color3(0, 255, 0); // BGR value for red
    
    for (auto point : path_points){
          cv::circle(mapImage, point, radius, color3, -1);

    }

    for (size_t k = 0; k < Graph_.size(); k++){
    
        cv::Point center(Graph_[k].x, Graph_[k].y);

        if (center.x >= 0 && center.x < mapImage.cols && center.y >= 0 && center.y < mapImage.rows) {
            
            cv::circle(mapImage, center, radius, color, -1);

            // cv::Point start(Graph.at(0).x, Graph.at(0).y);

            // cv::line(mapImage, center, start, cv::Scalar(255, 0, 0) /*blue*/, 1 /*thickness*/);
     
            

        } 
        else {
            // The center is outside the image, handle accordingly
            // This could mean adjusting the position, skipping the draw, etc.
            std::cout << "out of bounds" << std::endl;
        }

    }
    


    // Display the map with nodes
    cv::namedWindow("SLAM Map with Nodes", cv::WINDOW_AUTOSIZE);
    cv::imshow("SLAM Map with Nodes", mapImage);
    // Optionally, save the color map with nodes to a file
    cv::imwrite("/home/liam/Desktop/map_with_nodes fixed!!!.png", mapImage);

    cv::waitKey(0);
}



void PRM::test(){
    
    samplePoints();

    std::vector<Node> ProbablityRoadMap;
    
    ProbablityRoadMap = samplePoints();
    std::cout << "points generated" << std::endl;
    std::vector<Node> ProbablityRoadMap_ = createNodesAndEdges(ProbablityRoadMap);

    visualise_PRM(ProbablityRoadMap_);

}