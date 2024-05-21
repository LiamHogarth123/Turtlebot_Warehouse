#include <vector>
#include <utility> // for std::pair
// #include "opencv2/opencv.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"


struct Node {
    int id;
    float x, y; // Node position
    std::vector<int> edges; // IDs of connected nodes
};

struct ComparePair {
    bool operator()(const std::pair<float, int>& a, const std::pair<float, int>& b) const {
        // Custom comparison logic here
        return a.first > b.first; // Example: Min-heap based on the first element
    }
};


class PRM {
public:
    PRM(); 

    std::vector<Node> samplePoints();

    geometry_msgs::Point convertNodeToPoint(Node temp);
    
    void visualise_PRM(std::vector<Node> Graph_, std::vector<int> path);

    void GeneratePRM(nav_msgs::OccupancyGrid map, nav_msgs::MapMetaData MapMetaData_);

    std::vector<geometry_msgs::Point> DijkstraToGoal(geometry_msgs::Point start, geometry_msgs::Point goal);

    std::vector<Node> createNodesAndEdges(std::vector<Node> Graph_);

    void findPath(int startNodeId, int goalNodeId);

    int setGoalNode(geometry_msgs::Point goal);

    bool ValidPoint(geometry_msgs::Point point);

    void UpdateMapData(nav_msgs::OccupancyGrid map, nav_msgs::MapMetaData MapMetaData_);
    
    bool pathIsClear(Node Node_A, Node Node_B);

    std::vector<std::pair<int, int>> bresenhamLinePoints(int startX, int startY, int endX, int endY);

    bool ValidPointForPath(int x1, int y1);

    std::vector<geometry_msgs::Point> test();

    float nodeDistance(const Node& a, const Node& b);

    float euclideanDistance(const Node& node1, const Node& node2);
    
    std::vector<int> findPathDijkstra(const std::vector<Node>& graph, int startId, int targetId);

    std::vector<geometry_msgs::Point> ConvertParthToWorld(std::vector<int> path, std::vector<Node> Graph_);

    std::vector<int> findPathAStar(const std::vector<Node>& graph, int startId, int targetId);

    double getYawFromQuaternion(const geometry_msgs::Quaternion& quat);
    std::vector<Node> rotateNodes(std::vector<Node>& graph, const geometry_msgs::Quaternion& orientation, const geometry_msgs::Pose& mapOrigin);

    bool newPoint(geometry_msgs::Point point, std::vector<Node> temp);


public:

    std::vector<Node> nodes;
   
    std::vector<Node> Graph;

    nav_msgs::OccupancyGrid SlamMapData;
    nav_msgs::MapMetaData latestMapMetaData_;



    int numberOfPoints_;

    // std::vector<cv::Point> path_points_withoutValidation;
    // std::vector<cv::Point> path_points;
};