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
#include <algorithm>
#include <functional>
#include <queue>
#include <unordered_map>
#include <limits>
#include <tf/tf.h>

//To do list
// --- Add diagonal criteria to check points
// --- Add check to see if node edge overlaps another node
// --- Add check that nodes are on top of each themselve

PRM::PRM() {
}


void PRM::GeneratePRM(nav_msgs::OccupancyGrid map, nav_msgs::MapMetaData MapMetaData_) {
    UpdateMapData(map, MapMetaData_);

    std::vector<Node> ProbablityRoadMap;
    ProbablityRoadMap = samplePoints();
    std::vector<Node> ProbablityRoadMap_ = createNodesAndEdges(ProbablityRoadMap);

    ProbablityRoadMap_ = Graph;
}




std::vector<geometry_msgs::Point> PRM::DijkstraToGoal(geometry_msgs::Point start, geometry_msgs::Point goal) {
    std::vector<int> DijkstraNodes;
    
    int start_Id = setGoalNode(start);
    
    int Goal_Id = setGoalNode(goal);
    
    DijkstraNodes = findPathDijkstra(Graph, start_Id, Goal_Id);

    std::vector<geometry_msgs::Point> trajectory;
    
    for (int x = 0; x < DijkstraNodes.size(); x++){
        trajectory.push_back(convertNodeToPoint(Graph.at(DijkstraNodes.at(x))));
    }
    return trajectory;

}


void PRM::UpdateMapData(nav_msgs::OccupancyGrid map, nav_msgs::MapMetaData MapMetaData_) {
    std::cout << "PRM map data openning" << std::endl;
    SlamMapData = map;
    numberOfPoints_ = 3000;
    latestMapMetaData_ = MapMetaData_;
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
        if (true){ //newPoint(point, Graph) adding error??

            if(ValidPoint(point)) {
                // std::cout << "if opens" << std::endl;
                Node temp;
                temp.x = point.x;
                temp.y = point.y;   
                temp.id = id;
                id ++;
                std::cout << "sampling points" << std::endl;
                std::cout << "Point : X" << temp.x << "  y :" << temp.y << std::endl;
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


bool PRM::pathIsClear(Node Node_A, Node Node_B){
    //Generate line between points
    ////////////////////////////////////////////////////////////
    float x1 = Node_A.x;
    float x2 = Node_B.x;
    float y1 = Node_A.y;
    float y2 = Node_B.y;

    std::vector<std::pair<int, int>> BressenhamPoints;

    BressenhamPoints = bresenhamLinePoints(x1, y1, x2, y2);



    // Validating line
    /////////////////////////////////////////////////////
    for (const auto& pair : BressenhamPoints) {
        
        cv::Point point(pair.first, pair.second);
        path_points_withoutValidation.push_back(point);   
    }


    for (const auto& pair : BressenhamPoints) {
        geometry_msgs::Point temp;
        temp.x = pair.first;
        temp.y = pair.second;
        if(ValidPointForPath(pair.first, temp.y)){
            // cv::Point point(pair.first, pair.second);
            // path_points.push_back(point);   
        }
        else{
            std::cout << "invalid Point " << std::endl;
            return false;
        }
        
    }
    
    for (const auto& pair : BressenhamPoints) {
        
        cv::Point point(pair.first, pair.second-16);
        path_points.push_back(point);   
    }

    return true;
}

bool PRM::ValidPointForPath(int x1, int y1){
    
    int grid_x = x1;
    int grid_y = y1;

    // Calculate index using grid indices
    int index = grid_x + (grid_y * SlamMapData.info.width);
    int above = index + 2*SlamMapData.info.width;
    int below = index - 2*SlamMapData.info.width;

    if (index >= SlamMapData.data.size()){
        return false;
    }


    if (SlamMapData.data.at(index) == 0) {
        
        if (SlamMapData.data.at(above) == 0) {
            if (SlamMapData.data.at(below) == 0) {
                if (SlamMapData.data.at(index + 2) == 0) {
                    if (SlamMapData.data.at(index - 2) == 0) {
                        // Your main logic here
                    } 
                    else {
                        std::cout << "Condition index - 2 is not met" << std::endl;
                    }
                } 
                else {
                    std::cout << "Condition index + 2 is not met" << std::endl;
                }
            } 
            else {
                std::cout << "Condition below is not met" << std::endl;
            }
        } 
        else {
            std::cout << "Condition above is not met" << std::endl;
        }
    } 
    else {
    std::cout << "Condition index is not met" << std::endl;
    return false;
}


}


std::vector<std::pair<int, int>> PRM::bresenhamLinePoints(int startX, int startY, int endX, int endY) {
    std::vector<std::pair<int, int>> outputArray;

    int dx = endX - startX;
    int dy = endY - startY;
    int absdx = std::abs(dx);
    int absdy = std::abs(dy);

    int x = startX;
    int y = startY;
    outputArray.push_back({x, y}); // Add starting point

    // Slope < 1
    if (absdx > absdy) {
        int d = 2*absdy - absdx;

        for (int i = 0; i < absdx; ++i) {
            x = dx < 0 ? x - 1 : x + 1;
            if (d < 0) {
                d = d + 2*absdy;
            } else {
                y = dy < 0 ? y - 1 : y + 1;
                d = d + (2*absdy - 2*absdx);
            }
            outputArray.push_back({x, y});
        }
    } else { // Case when slope is greater than or equals to 1
        int d = 2*absdx - absdy;

        for (int i = 0; i < absdy; ++i) {
            y = dy < 0 ? y - 1 : y + 1;
            if (d < 0) {
                d = d + 2*absdx;
            } else {
                x = dx < 0 ? x - 1 : x + 1;
                d = d + (2*absdx - 2*absdy);
            }
            outputArray.push_back({x, y});
        }
    }
    return outputArray;
}



std::vector<Node> PRM::createNodesAndEdges(std::vector<Node> Graph_){
    float max_distance = 1000;
    int edges = 0;
    for (size_t k = 0; k < Graph_.size(); k++){
        
        std::vector<Node> local_nodes;
        std::vector<std::pair<float, size_t>> distances;

        for (size_t m = 0; m < Graph_.size(); m++){
            if (Graph_[m].id != Graph_[k].id) { // Skip the target node itself

                float distance = sqrt(pow(Graph_[k].x - Graph_[m].x, 2) + pow(Graph_[k].y - Graph_[m].y, 2));

                distances.push_back({distance, m});
            }
        }

        size_t count = 30;
        size_t numResults = std::min(count, distances.size());
        std::partial_sort(distances.begin(), distances.begin() + numResults, distances.end());

        // Collect the closest nodes
        std::vector<Node> closestNodes;
    
        for (size_t i = 0; i < numResults; ++i) {
            local_nodes.push_back(Graph_[distances[i].second]);
        }




        for (size_t j = 0; j < local_nodes.size(); j++){
            // std::cout << "inner loop" << j << std::endl;
            // create a temp vector a sorting 


            if (pathIsClear(Graph_[k], local_nodes[j])){

                Graph_[k].edges.push_back(local_nodes[j].id);
                // std::cout << "adding edge " << local_nodes[j].id << std::endl;
                // edges++;
                // std::cout << Graph_[k].edges.size() << std::endl;
            }
        }
    }



    std::cout << edges << std::endl;
    return Graph_;
}



void PRM::visualise_PRM(std::vector<Node> Graph_, std::vector<int> path) {
    //READ Image
    ///////////////////////////////////////////////////////////////////////////////////////////////////
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
    cv::flip(mapImage, mapImage, 0);





// Draws the nodes without ajustment
///////////////////////////////////////////////////////////////////////////////////////////////////////////

    int radius = 0; // You can adjust this value as needed
    cv::Scalar color(0, 0, 255); // BGR value for red
    
    // for (size_t z = 0; z< Graph_.size(); z++){
    //     Graph_[z].y = -Graph_[z].y;
    // }


    for (size_t k = 0; k < Graph_.size(); k++){
    
        // std::cout << k << std::endl;
        cv::Point center(Graph_[k].x, Graph_[k].y);
      
        if (center.x >= 0 && center.x < mapImage.cols && center.y >= 0 && center.y < mapImage.rows) {
            
            for (size_t l = 0; l < Graph_.at(k).edges.size(); l++) {
            //    std::cout << "l =" << l << std::endl;
                int index = Graph_.at(k).edges.at(l);

                const auto& connected_node = Graph_[index];
                
              
                cv::Point connected_node_center(connected_node.x, connected_node.y);

                cv::line(mapImage, center, connected_node_center, cv::Scalar(0, 255, 255) /*blue*/, 0  /*thickness*/);
            }

            // cv::circle(mapImage, center, radius, color, -1);
            
        } 
        else {
            // The center is outside the image, handle accordingly
            // This could mean adjusting the position, skipping the draw, etc.
            std::cout << "out of bounds" << std::endl;
        }
    }
//DRAWING NODES Again
/////////////////////////////////////////////////////////////////////////////
    for (size_t k = 0; k < Graph_.size(); k++){
    
        cv::Point center(Graph_[k].x, Graph_[k].y);

        center.x = Graph_[k].x;
        center.y = Graph_[k].y;
        if (center.x >= 0 && center.x < mapImage.cols && center.y >= 0 && center.y < mapImage.rows) {
            cv::circle(mapImage, center, radius, cv::Scalar(255, 0, 0), -1);
        } 
        else {
            std::cout << "out of bounds" << std::endl;
        }

    }

    // //DRAW Goal
    // //////////////////////////////////////////////////////////////
    // for (auto node : Graph_){
    //     if (node.id = 5){
    //         cv::Point goal;
    //         goal.x = node.x;
    //         goal.y = node.y;
    //         cv::circle(mapImage, goal, 2, cv::Scalar(0, 0, 255), -1);
    //         break;
    //     }
    // }


    //Draw Path
    ////////////////////////////////////////////////////////////////
    
    for (int x = 0; x < path.size(); x++){
        // std::cout << "STUCK" << x << "of " << path.size()  << std::endl;
        cv::Point temp;
        temp.x = Graph_.at(path.at(x)).x;
        temp.y = Graph_.at(path.at(x)).y;
        
        if (x == 0){

        }
        else {
            cv::Point prev;
            prev.x = Graph_.at(path.at(x-1)).x;
            prev.y = Graph_.at(path.at(x-1)).y;

            cv::line(mapImage, temp, prev, cv::Scalar(0, 0, 255) /*blue*/, 1  /*thickness*/);
        }
        cv::circle(mapImage, temp, 0, cv::Scalar(255, 0, 0), -1);
    }

    for (int x = 0; x < path.size(); x++){
        // std::cout << "STUCK" << x << "of " << path.size()  << std::endl;
        cv::Point temp;
        temp.x = Graph_.at(path.at(x)).x;
        temp.y = Graph_.at(path.at(x)).y;
        
        cv::circle(mapImage, temp, 0, cv::Scalar(255, 0, 0), -1);
    }

    // Display the map with nodes
    cv::namedWindow("SLAM Map with Nodes", cv::WINDOW_AUTOSIZE);
    cv::imshow("SLAM Map with Nodes", mapImage);
    // Optionally, save the color map with nodes to a file
    cv::imwrite("/home/liam/Desktop/map_with_nodes fixed!!!.png", mapImage);

    cv::waitKey(0);
}




std::vector<geometry_msgs::Point> PRM::test(){
    
    samplePoints();


    std::vector<Node> ProbablityRoadMap;
    
    ProbablityRoadMap = samplePoints();
   
    std::cout << "points generated" << std::endl;
    std::cout << "s" << std::endl;

    std::cout << ProbablityRoadMap.size() << std::endl;


    std::vector<Node> ProbablityRoadMap_ = createNodesAndEdges(ProbablityRoadMap);

    std::vector<int> path;

   

    path = findPathDijkstra(ProbablityRoadMap_, 2, 10);
    // ProbablityRoadMap_ = rotateNodes(ProbablityRoadMap_, latestMapMetaData_.origin.orientation);
    std::cout << "path size" << path.size() << std::endl; 

   

    visualise_PRM(ProbablityRoadMap_, path);

    std::vector<geometry_msgs::Point> trajectory; 
    
    trajectory = ConvertParthToWorld(path, ProbablityRoadMap_);

    return trajectory;

}



void PRM::findPath(int startNodeId, int goalNodeId){
    //A star

}

std::vector<int> PRM::findPathDijkstra(const std::vector<Node>& graph, int startId, int targetId) {

    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, ComparePair> pq;

    std::unordered_map<int, float> dist; // Distance from start node to each node
    
    std::unordered_map<int, int> prev; // Previous node in optimal path from source
    
    std::vector<int> path; // Store the final path

    // Initialize distances as infinity
    for (const auto& node : graph) {
        dist[node.id] = std::numeric_limits<float>::infinity();
    }

    dist[startId] = 0.0; // Distance from start node to itself is zero
    pq.push({0.0, startId});

    while (!pq.empty()) {
        // std::cout << "OUTERstuck!!!" << std::endl;
        int currentNodeId = pq.top().second;
        pq.pop();

        if (currentNodeId == targetId) { // If target node is reached
            while (currentNodeId != startId) { // Reconstruct the path
                path.push_back(currentNodeId);
                currentNodeId = prev[currentNodeId];
                        // std::cout << "WHILE_stuck!!!" << std::endl;
            }
            path.push_back(startId); // Add start node at the end
            std::reverse(path.begin(), path.end()); // Reverse to get the correct order from start to target
            return path;
        }

        // Explore the neighbors of the current node
        for (const auto& edgeId : graph[currentNodeId].edges) {
            float alt = dist[currentNodeId] + nodeDistance(graph[currentNodeId], graph[edgeId]);
                    // std::cout << "FOR_stuck!!!" << std::endl;
            if (alt < dist[edgeId]) {
                dist[edgeId] = alt;
                prev[edgeId] = currentNodeId;
                pq.push({alt, edgeId});
            }
        }
    }

    return path; // Return an empty path if no path is found
}


float PRM::nodeDistance(const Node& a, const Node& b) {
    // Simple Euclidean distance for now
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}



int PRM::setGoalNode(geometry_msgs::Point goal){
    int goal_id;
    bool goal_filled = false;
    for (auto node : Graph) {
        double dx = node.x - goal.x;
        double dy = node.y - goal.y;
        float distance = sqrt(pow(dx, 2) + pow(dy, 2));
        if (distance < 1){
            goal_id = node.id;
            goal_filled = true;
        }
    }
    if (!goal_filled) {
        if(ValidPoint(goal)) {
            // std::cout << "if opens" << std::endl;
            Node temp;
            temp.x = goal.x;
            temp.y = goal.y - 16;   
            temp.id = Graph.back().id +1;
            goal_id = temp.id;
            Graph.push_back(temp);
        }
        else {
            std::cout << "invalid goal" << std::endl;
        }
    }
    return goal_id;
}


geometry_msgs::Point PRM::convertNodeToPoint(Node temp){
    geometry_msgs::Point world_point;



    world_point.x = ((temp.x * latestMapMetaData_.resolution) + latestMapMetaData_.origin.position.x);
    world_point.y = ((temp.y * latestMapMetaData_.resolution) + latestMapMetaData_.origin.position.y);

    std::cout << "point x" << world_point.x << " y --" << world_point.y << std::endl;

    return world_point;
}


double PRM::getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw = yaw + M_PI/2;
    std::cout << "yaw" << yaw << std::endl;
    return yaw;
}


std::vector<Node> PRM::rotateNodes(std::vector<Node>& graph, const geometry_msgs::Quaternion& orientation, const geometry_msgs::Pose& mapOrigin) {
    double yaw = getYawFromQuaternion(orientation);
    float originX = mapOrigin.position.x; // X coordinate of the origin
    float originY = mapOrigin.position.y; // Y coordinate of the origin

    for (Node& node : graph) {
        // Translate points to rotate around the origin (0,0)
        float translatedX = node.x - originX;
        float translatedY = node.y - originY;

        // Perform rotation
        float rotatedX = translatedX * cos(yaw) - translatedY * sin(yaw);
        float rotatedY = translatedX * sin(yaw) + translatedY * cos(yaw);

        // Translate points back
        node.x = rotatedX + originX;
        node.y = rotatedY + originY;
    }

    return graph;
}



std::vector<geometry_msgs::Point> PRM::ConvertParthToWorld(std::vector<int> path, std::vector<Node> Graph_){
    std::vector<geometry_msgs::Point> path_World;
    Node temp;
    for (int x = 0; x < path.size(); x++){

        for (int y = 0; y < Graph_.size(); y++){

            if (Graph_.at(y).id == path.at(x)){

                temp = Graph_.at(y);
            }
        }
    path_World.push_back(convertNodeToPoint(temp));

    }

    return path_World;
}


bool PRM::newPoint(geometry_msgs::Point point, std::vector<Node> temp) {
    int x1 = static_cast<int>(point.x);
    int y1 = static_cast<int>(point.y);
    for (auto node : temp){
        int x2 = static_cast<int>(node.x);
        int y2 = static_cast<int>(node.y);

        if(x1 == x2 || y1 ==y2){
            return false;
        }
    }
    return true;
}