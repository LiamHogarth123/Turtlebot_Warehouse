#include <vector>
#include <utility> // for std::pair
#include "opencv2/opencv.hpp"
#include <geometry_msgs>

struct Node {
    int id;
    float x, y; // Node position
    std::vector<int> edges; // IDs of connected nodes
};

struct MapSpecs {
    cv::mat mapImage;
    std::string image;
    float resolution;
    std::vector<float> origin;
    int negate;
    float occupied_thresh, free_thresh;
};

class PRM {
public:
    PRM(); 

    void setMap(MapSpecs temp);

    void samplePoints();
        

    void createNodesAndEdges();

    void findPath(int startNodeId, int goalNodeId);

    void setGoalNode(geometry_msgs::Point goal);

    bool ValidPoint(geometry_mgs::Point point);

private:
    std::vector<Node> nodes;
    MapSpecs slam_map;
    std::vector<Node> nodes;
   
};
