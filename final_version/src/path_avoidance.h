#include <vector>
#include <geometry_msgs/Point.h>

struct TurtleBotState {
    geometry_msgs::Point position;
    // double speed; // Speed in meters per second
    double time;  // Timestamp in seconds
};

class path_avoidance_ {
public:
    path_avoidance_();

    void Update_paths(std::vector<std::vector<geometry_msgs::Point>> trajectories);

    bool find_collisions(std::vector<geometry_msgs::Point>& collided_points);


    std::vector<geometry_msgs::Point> get_all_interpolated_points();   

std::vector<std::pair<int, geometry_msgs::Point>> GetCollisionsWithId();

private:
    std::vector<std::vector<geometry_msgs::Point>> trajectories;
    std::vector<std::vector<TurtleBotState>> interpolated_trajectories_;
    std::vector<std::pair<int, geometry_msgs::Point>> collided_points_;

    std::vector<TurtleBotState> interpolatePositions(const std::vector<geometry_msgs::Point>& waypoints, double average_speed);
    void interpolated_trajectories();
    bool will_collide();
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    bool checkCollisions(const std::vector<std::pair<int, geometry_msgs::Point>>& points_with_ids, double threshold);
};