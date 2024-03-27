#include <vector>
#include <utility> // for std::pair
#include "opencv2/opencv.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"

#include "readMap.h"

struct Node {
    int id;
    float x, y; // Node position
    std::vector<int> edges; // IDs of connected nodes
};

class PRM {
public:
    PRM(); 

    void setMap(MapSpecs temp);

    void samplePoints();
    
    void visualise_PRM();

    void createNodesAndEdges(std::vector<Node> Graph);

    void findPath(int startNodeId, int goalNodeId);

    void setGoalNode(geometry_msgs::Point goal);

    bool ValidPoint(geometry_msgs::Point point);

    void UpdateMapData(nav_msgs::OccupancyGrid map, nav_msgs::MapMetaData MapMetaData_);
    
    bool pathIsClear(Node Node_A, Node Node_B);

public:
    std::vector<Node> nodes;
    MapSpecs slam_map;
    std::vector<Node> Graph;

    nav_msgs::OccupancyGrid SlamMapData;
    nav_msgs::MapMetaData latestMapMetaData_;

    int numberOfPoints_;

};
