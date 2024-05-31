#include <vector>
#include <utility> // for std::pair
#include "opencv2/opencv.hpp"

#include "nav_msgs/MapMetaData.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>


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

struct PrmData {
    std::vector<Node> Exported_Graph;
    nav_msgs::OccupancyGrid map;
    nav_msgs::MapMetaData MapMetaData_;
    
};


class PRM {
public:
    PRM(); 

    //User Functions
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    void GeneratePRM(nav_msgs::OccupancyGrid map, nav_msgs::MapMetaData MapMetaData_, bool User_controlled);

    std::vector<geometry_msgs::Point> DijkstraToGoal(geometry_msgs::Point start, geometry_msgs::Point goal);

    std::vector<geometry_msgs::Point> A_star_To_Goal(geometry_msgs::Point start, geometry_msgs::Point goal);

    void UpdateMapData(nav_msgs::OccupancyGrid map, nav_msgs::MapMetaData MapMetaData_);

    std::vector<geometry_msgs::Point> test();

    void show_Prm();

    PrmData ExportPrmData();

    void Load_PRM(PrmData Imports);

    void User_remove_Nodes();

    bool checkCollision(const std::vector<geometry_msgs::Point>& traj1, const std::vector<geometry_msgs::Point>& traj2);

    // void setoffset(nav_msgs::Odometry start_odom);



private:
    // Gemerating the PRM Functions
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<Node> samplePoints();

    bool ValidPoint(geometry_msgs::Point point);

    bool pathIsClear(Node Node_A, Node Node_B);

    std::vector<std::pair<int, int>> bresenhamLinePoints(int startX, int startY, int endX, int endY);

    bool ValidPointForPath(int x1, int y1);

    std::vector<Node> createNodesAndEdges(std::vector<Node> Graph_);

    

    
    
    // Converting cordinate planes
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    geometry_msgs::Point convertNodeToPoint(Node temp);

    float nodeDistance(const Node& a, const Node& b);

    float euclideanDistance(const Node& node1, const Node& node2);

    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    
    int setGoalNode(geometry_msgs::Point goal);
    
    std::vector<geometry_msgs::Point> ConvertParthToWorld(std::vector<int> path, std::vector<Node> Graph_);


    double getYawFromQuaternion(const geometry_msgs::Quaternion& quat);

    std::vector<Node> rotateNodes(std::vector<Node>& graph, const geometry_msgs::Quaternion& orientation, const geometry_msgs::Pose& mapOrigin);

    bool newPoint(geometry_msgs::Point point, std::vector<Node> temp);


    // Path finding
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<int> findPathAStar(const std::vector<Node>& graph, int startId, int targetId);

    std::vector<int> findPathDijkstra(const std::vector<Node>& graph, int startId, int targetId);

    void findPath(int startNodeId, int goalNodeId);


    // Visualisation Functions
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void visualise_PRM(std::vector<Node> Graph_, std::vector<int> path);

    cv::Mat Load_Map();

    cv::Mat visalise_prm(cv::Mat mapImage, std::vector<Node> Graph_);

    cv::Mat visalise_PRM_With_Path(std::vector<int> path, cv::Mat mapImage, std::vector<Node> Graph_);

    void show_map(cv::Mat mapImage);

    void save_map(cv::Mat mapImage);


    //OPEN CV Define load area 
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    static void staticMouseCallback(int event, int x, int y, int flags, void* userdata);

    void mouseCallback(int event, int x, int y, int flags, void* userdata);
    
    std::vector<cv::Point> getUserDefinedPolygon(const std::string& mapImagePath);
    
    bool isPointInPolygon(const cv::Point& point, const std::vector<cv::Point>& polygon);
    
    std::vector<Node> samplePointsCV();
    
    static std::vector<cv::Point> polygonPoints;


    //User Removal of nodes
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    static void staticRemoveNodeCallback(int event, int x, int y, int flags, void* userdata);

    void removeNodeCallback(int event, int x, int y, int flags, void* userdata);
    
    cv::Mat removeNodes(cv::Mat mapImage, std::vector<Node>& Graph_);



   

    

   


private:

    std::vector<Node> nodes;
   
    std::vector<Node> Graph;

    nav_msgs::OccupancyGrid SlamMapData;
    nav_msgs::MapMetaData latestMapMetaData_;

    double offestx;
    double offsety;
    double offsetyaw;


    int numberOfPoints_;

    std::vector<cv::Point> path_points_withoutValidation;
    std::vector<cv::Point> path_points;
};
