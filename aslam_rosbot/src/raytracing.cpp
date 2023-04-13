#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <iostream>

// Define a struct to represent a cell in the occupancy grid
struct Cell {
    bool occupied;
    // Add any additional fields as needed
};

// Define a function to check if a cell is occupied
bool isOccupied(int x, int y, int width, const std::vector<Cell>& map) {
    if (x < 0 || x >= width || y < 0 || y >= map.size() / width) {
        // Index out of bounds, treat as occupied
        return true;
    } else {
        // Check the occupancy status of the cell
        return map[y * width + x].occupied;
    }
}

// Define a function to perform ray tracing on the occupancy grid
bool rayTrace(float start_x, float start_y, float end_x, float end_y, const nav_msgs::OccupancyGrid& map) {
    // Convert the start and end points to map coordinates
    int start_i = (int)((start_x - map.info.origin.position.x) / map.info.resolution);
    int start_j = (int)((start_y - map.info.origin.position.y) / map.info.resolution);
    int end_i = (int)((end_x - map.info.origin.position.x) / map.info.resolution);
    int end_j = (int)((end_y - map.info.origin.position.y) / map.info.resolution);

    // Use Bresenham's line algorithm to traverse the cells along the ray
    int dx = abs(end_i - start_i);
    int dy = abs(end_j - start_j);
    int sx = (start_i < end_i) ? 1 : -1;
    int sy = (start_j < end_j) ? 1 : -1;
    int err = dx - dy;
    while (true) {
        // Check if the current cell is occupied
        if (isOccupied(start_i, start_j, map.info.width, map.data)) {
            return false; // Obstacle detected, ray is obstructed
        }
        // Check if we've reached the end point
        if (start_i == end_i && start_j == end_j) {
            return true; // End point reached, ray is unobstructed
        }
        // Update the error and cell indices based on the Bresenham algorithm
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            start_i += sx;
        }
        if (e2 < dx) {
            err += dx;
            start_j += sy;
        }
    }
}

// Define a callback function for the occupancy grid map
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    // Perform ray tracing from the robot's position to a goal point
    float start_x = 0.0; // Replace with actual robot position
    float start_y = 0.0; // Replace with actual robot position
    float goal_x = 5.0; // Replace with actual goal position
    float goal_y = 5.0; // Replace with actual goal position
    if (rayTrace(start_x, start_y, goal_x, goal_y, *msg)) {
        ROS_INFO("Goal is visible!");
    } else {
        ROS_WARN("Goal is obstructed.");
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "raytracing");

    // Create a NodeHandle object for the node
    ros::NodeHandle nh;

    // Node code goes here

    return 0;
}
