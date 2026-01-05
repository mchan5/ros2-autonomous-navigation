#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

// ROS 2 Core
#include "rclcpp/rclcpp.hpp"

// ROS 2 Message Types
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

/**
 * @brief Structure representing a node in the A* search tree.
 */
struct AStarNode {
    int x, y;
    double g_cost;  // Cost from start to current node
    double h_cost;  // Heuristic cost from current node to goal
    double f_cost;  // Total cost (g + h)
    std::shared_ptr<AStarNode> parent;

    AStarNode(int x, int y, double g, double h, std::shared_ptr<AStarNode> p)
        : x(x), y(y), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}
};

/**
 * @brief ROS 2 Node that subscribes to a map and odometry to plan a path to a goal point.
 */
class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

private:
    // Callback functions
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void timerCallback();

    // Planning logic
    void planPath();
    bool isValid(int x, int y);
    double calculateHeuristic(int x1, int y1, int x2, int y2);
    void publishPath(std::shared_ptr<AStarNode> goal_node);

    // Enumeration for Node State
    enum class State {
        WAITING_FOR_GOAL,
        PLANNING
    };

    // Subscriptions and Publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Internal Data
    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::Odometry robot_odom_;
    geometry_msgs::msg::PointStamped goal_point_;
    
    State state_{State::WAITING_FOR_GOAL};
    bool map_received_{false};
    bool goal_received_{false};
};

#endif  // PLANNER_NODE_HPP_