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
// #include "opencv2/opencv.hpp"
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
// --- Add check that nodes are not on top of each themselve
// --- Fix the goal node assignment
// --- Add multiple different visualisation

PRM::PRM() {
}

//User Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PRM::GeneratePRM(nav_msgs::OccupancyGrid map, nav_msgs::MapMetaData MapMetaData_) {
    UpdateMapData(map, MapMetaData_);

    std::vector<Node> ProbablityRoadMap;
    ProbablityRoadMap = samplePoints();
    std::vector<Node> ProbablityRoadMap_ = createNodesAndEdges(ProbablityRoadMap);
    std::vector<int> emptys;
    Graph = ProbablityRoadMap_;
    // visualise_PRM(Graph, emptys);
}




std::vector<geometry_msgs::Point> PRM::DijkstraToGoal(geometry_msgs::Point start, geometry_msgs::Point goal) {
    std::vector<int> DijkstraNodes;
    std::cout << "Dijkstra_Opens-------------" <<std::endl;
    int start_Id = setGoalNode(start);
    
    int Goal_Id = setGoalNode(goal);
    
    std::cout << "start_Id  -->" << start_Id << std::endl;
        std::cout << "goal_Id  -->" << Goal_Id << std::endl; 
    DijkstraNodes = findPathDijkstra(Graph, start_Id, Goal_Id);
    std::cout << DijkstraNodes.size() << "---------------------------" << std::endl;
    std::vector<geometry_msgs::Point> trajectory;
    
    for (int x = 0; x < DijkstraNodes.size(); x++){
        trajectory.push_back(convertNodeToPoint(Graph.at(DijkstraNodes.at(x))));
    }
    return trajectory;

}

std::vector<geometry_msgs::Point> PRM::A_star_To_Goal(geometry_msgs::Point start, geometry_msgs::Point goal){
    std::vector<int> DijkstraNodes;
    std::cout << "Dijkstra_Opens-------------" <<std::endl;
    int start_Id = setGoalNode(start);
    
    int Goal_Id = setGoalNode(goal);
    
    std::cout << "start_Id  -->" << start_Id << std::endl;
        std::cout << "goal_Id  -->" << Goal_Id << std::endl; 
    DijkstraNodes = findPathAStar(Graph, start_Id, Goal_Id);
    std::cout << DijkstraNodes.size() << "---------------------------" << std::endl;
    std::vector<geometry_msgs::Point> trajectory;
    
    for (int x = 0; x < DijkstraNodes.size(); x++){
        trajectory.push_back(convertNodeToPoint(Graph.at(DijkstraNodes.at(x))));
    }
    return trajectory;
}



void PRM::UpdateMapData(nav_msgs::OccupancyGrid map, nav_msgs::MapMetaData MapMetaData_) {
    std::cout << "PRM map data openning" << std::endl;
    SlamMapData = map;
    numberOfPoints_ = 9000;
    latestMapMetaData_ = MapMetaData_;
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
    // visualise_PRM(ProbablityRoadMap_, path);
    std::cout << "start Node -> x = " << ProbablityRoadMap_.at(2).x << " -> Y=" << ProbablityRoadMap_.at(2).y << std::endl;
    std::cout << "end Node -> x = " << ProbablityRoadMap_.at(10).x << " -> Y=" << ProbablityRoadMap_.at(10).y << std::endl;
    std::cout << "Distance between nodes -> " << nodeDistance(ProbablityRoadMap_.at(2), ProbablityRoadMap_.at(10)) << std::endl;
    std::vector<geometry_msgs::Point> trajectory; 
    trajectory = ConvertParthToWorld(path, ProbablityRoadMap_);
    return trajectory;

}

void PRM::show_Prm(){
    // cv::Mat MapImage = Load_Map();
    // MapImage = visalise_prm(MapImage, Graph);
    // save_map(MapImage);
}










// Gernating PRM
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
                // std::cout << "sampling points" << std::endl;
                // std::cout << "Point : X" << temp.x << "  y :" << temp.y << std::endl;
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

    int index = grid_x + (grid_y * SlamMapData.info.width);
   int grid_size = 5;

    // Loop through the grid_size x grid_size area around the point
    for (int dx = -grid_size/2; dx <= grid_size/2; ++dx) {
        for (int dy = -grid_size/2; dy <= grid_size/2; ++dy) {
            int new_x = grid_x + dx;
            int new_y = grid_y + dy;

            // Check if the new point is within the map boundaries
            if (new_x < 0 || new_x >= SlamMapData.info.width || new_y < 0 || new_y >= SlamMapData.info.height) {
                return false; // If the point is outside the map boundaries, return false
            }

            // Calculate the index of the new point
            int new_index = new_x + (new_y * SlamMapData.info.width);

            // Check if the new point is within the map boundaries
            if (new_index >= SlamMapData.data.size() || new_index < 0) {
                return false;
            }

            // Check if the point is free of obstacles (value 0)
            if (SlamMapData.data.at(new_index) != 0) {
                return false;
            }
        }
    }

    return true; // All points in the grid are free of obstacles

    


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
        
        // cv::Point point(pair.first, pair.second);
        // path_points_withoutValidation.push_back(point);   
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
            // std::cout << "invalid Point " << std::endl;
            return false;
        }
    }
    
    for (const auto& pair : BressenhamPoints) {
        
        // cv::Point point(pair.first, pair.second-16);
        // path_points.push_back(point);   
    }

    return true;
}

bool PRM::ValidPointForPath(int x1, int y1){
    
    int grid_x = x1;
    int grid_y = y1;

    // // Calculate index using grid indices
    int index = grid_x + (grid_y * SlamMapData.info.width);


 

       // Define the size of the grid to check around the point
    int grid_size = 5;

    // Loop through the grid_size x grid_size area around the point
    for (int dx = -grid_size/2; dx <= grid_size/2; ++dx) {
        for (int dy = -grid_size/2; dy <= grid_size/2; ++dy) {
            int new_x = grid_x + dx;
            int new_y = grid_y + dy;

            // Check if the new point is within the map boundaries
            if (new_x < 0 || new_x >= SlamMapData.info.width || new_y < 0 || new_y >= SlamMapData.info.height) {
                return false; // If the point is outside the map boundaries, return false
            }

            // Calculate the index of the new point
            int new_index = new_x + (new_y * SlamMapData.info.width);

            // Check if the new point is within the map boundaries
            if (new_index >= SlamMapData.data.size() || new_index < 0) {
                return false;
            }

            // Check if the point is free of obstacles (value 0)
            if (SlamMapData.data.at(new_index) != 0) {
                return false;
            }
        }
    }

    return true; // All points in the grid are free of obstacles
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


//Visualisation
///////////////////////////////////////////////////////////////////////////////////////////////////////////


// cv::Mat PRM::Load_Map(){
//     //READ Image
//     cv::Mat grayscaleMapImage = cv::imread("/home/liam/catkin_ws/src/navigation/map_server/maps/map.pgm", cv::IMREAD_GRAYSCALE);
//     if (grayscaleMapImage.empty()) {
//         std::cerr << "Could not open or find the map image" << std::endl;
//     }
    
//     cv::Mat mapImage;
//     cv::cvtColor(grayscaleMapImage, mapImage, cv::COLOR_GRAY2BGR);
//     cv::flip(mapImage, mapImage, 0);
//     return mapImage;
// }



// cv::Mat PRM::visalise_prm(cv::Mat mapImage, std::vector<Node> Graph_){
//     int radius = 0; // You can adjust this value as needed
//     cv::Scalar color(0, 0, 255); // BGR value for red
    
//     for (size_t k = 0; k < Graph_.size(); k++){
//         cv::Point center(Graph_[k].x, Graph_[k].y);

//         if (center.x >= 0 && center.x < mapImage.cols && center.y >= 0 && center.y < mapImage.rows) {
            
//             for (size_t l = 0; l < Graph_.at(k).edges.size(); l++) {
//                 int index = Graph_.at(k).edges.at(l);
//                 const auto& connected_node = Graph_[index];
              
//                 cv::Point connected_node_center(connected_node.x, connected_node.y);
//                 cv::line(mapImage, center, connected_node_center, cv::Scalar(0, 255, 255) /*blue*/, 0  /*thickness*/);
//             }            
//         } 
//         else {
//             std::cout << "out of bounds" << std::endl;
//         }
//     }


//     for (size_t k = 0; k < Graph_.size(); k++){
    
//         cv::Point center(Graph_[k].x, Graph_[k].y);

//         center.x = Graph_[k].x;
//         center.y = Graph_[k].y;
//         if (center.x >= 0 && center.x < mapImage.cols && center.y >= 0 && center.y < mapImage.rows) {
//             cv::circle(mapImage, center, radius, cv::Scalar(255, 0, 0), -1);
//         } 
//         else {
//             std::cout << "out of bounds" << std::endl;
//         }

//     }
//     return mapImage;
// }


// cv::Mat PRM::visalise_PRM_With_Path(std::vector<int> path, cv::Mat mapImage, std::vector<Node> Graph_){

   
//     for (int x = 0; x < path.size(); x++){
//         // std::cout << "STUCK" << x << "of " << path.size()  << std::endl;ast
//         cv::Point prev;
//         prev.x = Graph_.at(path.at(x-1)).x;
//         prev.y = Graph_.at(path.at(x-1)).y;
//         cv::Point temp;
//         temp.x = Graph_.at(path.at(x)).x;
//         temp.y = Graph_.at(path.at(x)).y;
//         cv::line(mapImage, temp, prev, cv::Scalar(0, 0, 255) /*blue*/, 2  /*thickness*/);
    
//     }

//     for (int x = 0; x < path.size(); x++){
//         // std::cout << "STUCK" << x << "of " << path.size()  << std::endl;
//         cv::Point temp;
//         temp.x = Graph_.at(path.at(x)).x;
//         temp.y = Graph_.at(path.at(x)).y;
        
//         cv::circle(mapImage, temp, 0, cv::Scalar(255, 0, 0), -1);
//     }

//     return mapImage;
// }


// void PRM::show_map(cv::Mat mapImage){
//     cv::namedWindow("SLAM Map with Nodes", cv::WINDOW_AUTOSIZE);
//     cv::imshow("SLAM Map with Nodes", mapImage);
// }


// void PRM::save_map(cv::Mat mapImage){
//     cv::imwrite("/home/liam/Desktop/map_with_nodes fixed!!!.png", mapImage);
// }




// void PRM::visualise_PRM(std::vector<Node> Graph_, std::vector<int> path) {

// }










//Path Planning
///////////////////////////////////////////////////////////////////////////////////////

void PRM::findPath(int startNodeId, int goalNodeId){
    //A star

}

std::vector<int> PRM::findPathDijkstra(const std::vector<Node>& graph, int startId, int targetId) {
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, ComparePair> pq;
    std::unordered_map<int, float> dist; // Distance from start node to each node
    std::unordered_map<int, int> prev; // Previous node in optimal path from source
    std::vector<int> path; // Store the final path

    std::cout << "PRM size " << graph.size() << std::endl;

    // Initialize distances as infinity
    for (const auto& node : graph) {
        dist[node.id] = std::numeric_limits<float>::infinity();
    }

    dist[startId] = 0.0; // Distance from start node to itself is zero
    pq.push({0.0, startId});

    while (!pq.empty()) {
        int currentNodeId = pq.top().second;
        pq.pop();

        std::cout << "loopping" <<std::endl;
        if (currentNodeId == targetId) { // If target node is reached
            std::cout << "Total distance from start to target: " << dist[targetId] << std::endl; // Print the total distance to target
            while (currentNodeId != startId) { // Reconstruct the path
                path.push_back(currentNodeId);
                currentNodeId = prev[currentNodeId];
            }
            path.push_back(startId); // Add start node at the end
            std::reverse(path.begin(), path.end()); // Reverse to get the correct order from start to target
            return path;
        }

        // Explore the neighbors of the current node
        for (const auto& edgeId : graph[currentNodeId].edges) {
            float alt = dist[currentNodeId] + nodeDistance(graph[currentNodeId], graph[edgeId]);
            if (alt < dist[edgeId]) {
                dist[edgeId] = alt;
                prev[edgeId] = currentNodeId;
                pq.push({alt, edgeId});
            }
        }
    }
    return path; // Return an empty path if no path is found
}


std::vector<int> PRM::findPathAStar(const std::vector<Node>& graph, int startId, int targetId) {
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, ComparePair> pq;
    std::unordered_map<int, float> dist; // Distance from start node to each node
    std::unordered_map<int, int> prev;   // Previous node in optimal path from source
    std::vector<int> path;               // Store the final path

    // Initialize distances as infinity
    for (const auto& node : graph) {
        dist[node.id] = std::numeric_limits<float>::infinity();
    }

    dist[startId] = 0.0; // Distance from start node to itself is zero
    pq.push({0.0 + euclideanDistance(graph[startId], graph[targetId]), startId}); // Include heuristic value in priority queue

    while (!pq.empty()) {
        int currentNodeId = pq.top().second;
        pq.pop();

        if (currentNodeId == targetId) { // If target node is reached
            // Reconstruct the path
            while (currentNodeId != startId) {
                path.push_back(currentNodeId);
                currentNodeId = prev[currentNodeId];
            }
            path.push_back(startId); // Add start node at the end
            std::reverse(path.begin(), path.end()); // Reverse to get the correct order from start to target
            return path;
        }

        // Explore the neighbors of the current node
        for (const auto& edgeId : graph[currentNodeId].edges) {
            float alt = dist[currentNodeId] + euclideanDistance(graph[currentNodeId], graph[edgeId]);
            if (alt < dist[edgeId]) {
                dist[edgeId] = alt;
                prev[edgeId] = currentNodeId;
                pq.push({alt + euclideanDistance(graph[edgeId], graph[targetId]), edgeId}); // Include heuristic value in priority queue
            }
        }
    }

    return path; // Return an empty path if no path is found
}















// Converting cordinate planes
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


float PRM::nodeDistance(const Node& a, const Node& b) {
    // Simple Euclidean distance for now
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

float PRM::euclideanDistance(const Node& node1, const Node& node2) {
    return sqrt(pow(node1.x - node2.x, 2) + pow(node1.y - node2.y, 2));
}


int PRM::setGoalNode(geometry_msgs::Point goal){
     // Convert world coordinates to map coordinates
    std::cout << " Open---------------------------------------" << std::endl;
    geometry_msgs::Point map_point;
    map_point.x = (goal.x - latestMapMetaData_.origin.position.x) / latestMapMetaData_.resolution;
    map_point.y = (goal.y - latestMapMetaData_.origin.position.y) / latestMapMetaData_.resolution;

    std::cout << "goal x" << goal.x << " y --" << goal.y << std::endl;
    std::cout << "map x" << map_point.x << " y --" << map_point.y << std::endl;
    
    int closestNodeId = -1;
    float minDistance = std::numeric_limits<float>::infinity();

    // Iterate through all nodes in the graph to find the closest one
    for (const auto& node : Graph) {
        float dx = node.x - map_point.x;
        float dy = node.y - map_point.y;
        float distance = sqrt(dx * dx + dy * dy);
        if (distance < minDistance) {
            minDistance = distance;
            closestNodeId = node.id;
        }
    }

    return closestNodeId;
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