#include <cmath>
#include <vector>
#include <geometry_msgs/Point.h>
#include "path_avoidance.h"
#include <iostream>

path_avoidance_::path_avoidance_() {
}

void path_avoidance_::Update_paths(std::vector<std::vector<geometry_msgs::Point>> input) {
    trajectories = input;
    interpolated_trajectories();
}

bool path_avoidance_::find_collisions(std::vector<geometry_msgs::Point>& collided_points) {
    collided_points.clear();
    bool collision_found = false;
    

    collision_found = will_collide();

    for (const auto& pair : collided_points_) {
        collided_points.push_back(pair.second);
    }

    return collision_found;
}


std::vector<geometry_msgs::Point> path_avoidance_::get_all_interpolated_points() {
    std::vector<geometry_msgs::Point> all_points;
    for (const auto& traj : interpolated_trajectories_) {
        for (const auto& state : traj) {
            all_points.push_back(state.position);
        }
    }
    return all_points;
}



std::vector<TurtleBotState> path_avoidance_::interpolatePositions(const std::vector<geometry_msgs::Point>& waypoints, double average_speed) {
    std::vector<TurtleBotState> interpolated_positions;

    if (waypoints.size() < 2) return interpolated_positions;

    double cumulative_time = 0.0;

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        geometry_msgs::Point start = waypoints[i];
        geometry_msgs::Point end = waypoints[i + 1];
        double distance = calculateDistance(start, end);
        double segment_time = distance / average_speed;

        for (double t = 0; t < segment_time; t += 0.1) { // Interpolating at 0.1 second intervals
            double ratio = t / segment_time;
            geometry_msgs::Point interpolated_point;
            interpolated_point.x = start.x + ratio * (end.x - start.x);
            interpolated_point.y = start.y + ratio * (end.y - start.y);
            interpolated_point.z = start.z + ratio * (end.z - start.z);

            interpolated_positions.push_back({interpolated_point, cumulative_time + t});
        }

        cumulative_time += segment_time;
    }

    // Adding the last waypoint
    interpolated_positions.push_back({waypoints.back(), cumulative_time});

    return interpolated_positions;
}

void path_avoidance_::interpolated_trajectories() {
    interpolated_trajectories_.clear(); // Clear the previous data
    for (const auto& trajectory : trajectories) {
        interpolated_trajectories_.push_back(interpolatePositions(trajectory, 0.2));
    }
}

bool path_avoidance_::will_collide() {
    bool collision = false;
    std::vector<geometry_msgs::Point> temp;
    int max_size = 0;

    // Determine the maximum size of the interpolated trajectories
    for (const auto& traj : interpolated_trajectories_) {
        if (traj.size() > max_size) {
            max_size = traj.size();
        }
    }

    // Check for collisions at each timestep
    for (int k = 0; k < max_size; ++k) {
        temp.clear(); // Clear the temporary vector for each timestep
        for (const auto& traj : interpolated_trajectories_) {
            if (k >= traj.size()) {
                temp.push_back(traj.back().position); // Use the last position if the index is out of bounds
            } else {
                temp.push_back(traj[k].position);
            }
        }
        bool collision2 = checkCollisions(temp, 0.4);
        if (collision2) {
            collision = true;
        }
    }

    return collision;
}

double path_avoidance_::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2) + std::pow(p2.z - p1.z, 2));
}

bool path_avoidance_::checkCollisions(const std::vector<geometry_msgs::Point>& points, double threshold) {
    bool collision = false;
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size(); ++j) {
            if (calculateDistance(points[i], points[j]) < threshold) {
                collided_points_.push_back(std::make_pair(i, points[i]));
                collision = true;

                // Print the points being pushed back
                std::cout << "Collision detected between points: (" 
                          << points[i].x << ", " << points[i].y << ", " << points[i].z 
                          << ") and ("
                          << points[j].x << ", " << points[j].y << ", " << points[j].z
                          << ") - Pushing back point: ("
                          << points[i].x << ", " << points[i].y << ", " << points[i].z
                          << ")" << std::endl;
            }
        }
    }

    // Print if a collision was found
    if (collision) {
        std::cout << "Collision detected and returning true." << std::endl;
    } else {
        std::cout << "No collision detected and returning false." << std::endl;
    }

    return collision;
}
