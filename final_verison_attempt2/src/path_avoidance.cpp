#include <cmath>
#include <vector>
#include <geometry_msgs/Point.h>
#include "path_avoidance.h"
#include <iostream>

path_avoidance_::path_avoidance_() {
    // Constructor
}

void path_avoidance_::Update_paths(std::vector<std::vector<geometry_msgs::Point>> input, std::vector<double> time_offset) {
    trajectories = input;
    time_offset_ = time_offset; // Corrected assignment
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

std::vector<std::pair<int, geometry_msgs::Point>> path_avoidance_::GetCollisionsWithId(){
    return collided_points_;
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

std::vector<TurtleBotState> path_avoidance_::interpolatePositions(const std::vector<geometry_msgs::Point>& waypoints, double average_speed, double wait_times) {
    std::vector<TurtleBotState> interpolated_positions;

    if (waypoints.size() < 2) return interpolated_positions;

    double cumulative_time = 0.0;
    if (wait_times > 0) {
        int waiting_steps = static_cast<int>(wait_times / 0.1); // Number of 0.1 second intervals for wait_times
        for (int i = 0; i < waiting_steps; ++i) {
            cumulative_time += 0.1;
            interpolated_positions.push_back({waypoints.front(), cumulative_time});
        }
    }

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
    for (int i = 0; i < trajectories.size(); i++) {
        std::cout << "Interpolating trajectory " << i << std::endl; // Debugging statement
        interpolated_trajectories_.push_back(interpolatePositions(trajectories.at(i), 0.2, time_offset_.at(i)));
    }
}

bool path_avoidance_::will_collide() {
    bool collision = false;
    collided_points_.clear(); // Ensure this is cleared before use

    std::vector<std::pair<int, geometry_msgs::Point>> temp;
    int max_size = 0;

    // Determine the maximum size of the interpolated trajectories
    for (const auto& traj : interpolated_trajectories_) {
        if (traj.size() > max_size) {
            max_size = traj.size();
        }
    }

    std::cout << "Checking for collisions..." << std::endl; // Debugging statement

    // Check for collisions at each timestep
    for (int k = 0; k < max_size; ++k) {
        temp.clear(); // Clear the temporary vector for each timestep
        for (int l = 0; l < interpolated_trajectories_.size(); ++l) {
            if (l >= interpolated_trajectories_.size() || k >= interpolated_trajectories_.at(l).size()) {
                continue;
            }
            if (k >= interpolated_trajectories_.at(l).size()) {
                temp.push_back(std::make_pair(l, interpolated_trajectories_.at(l).back().position)); // Use the last position if the index is out of bounds
            } else {
                temp.push_back(std::make_pair(l, interpolated_trajectories_.at(l).at(k).position));
            }
        }
        bool collision2 = checkCollisions(temp, 0.4);
        if (collision2) {
            collision = true;
        }
    }

    std::cout << "Collision check completed." << std::endl; // Debugging statement
    return collision;
}

double path_avoidance_::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2) + std::pow(p2.z - p1.z, 2));
}

bool path_avoidance_::checkCollisions(const std::vector<std::pair<int, geometry_msgs::Point>>& points_with_ids, double threshold) {
    bool collision = false;

    for (size_t i = 0; i < points_with_ids.size(); ++i) {
        for (size_t j = i + 1; j < points_with_ids.size(); ++j) {
            if (calculateDistance(points_with_ids[i].second, points_with_ids[j].second) < threshold) {
                collided_points_.push_back(std::make_pair(points_with_ids[i].first, points_with_ids[i].second));
                collided_points_.push_back(std::make_pair(points_with_ids[j].first, points_with_ids[j].second));
                collision = true;
                // std::cout << "Collision detected between TurtleBot " << points_with_ids[i].first
                //           << " and TurtleBot " << points_with_ids[j].first << " at point: ("
                //           << points_with_ids[i].second.x << ", " << points_with_ids[i].second.y << ", " << points_with_ids[i].second.z
                //           << ")" << std::endl; // Debugging statement
            }
        }
    }

    if (collision) {
        // std::cout << "Collision detected." << std::endl; // Debugging statement
    } else {
        // std::cout << "No collisions detected." << std::endl; // Debugging statement
    }

    return collision;
}
