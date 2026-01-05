#include "planner_node.hpp"

#include <cmath> 
#include <queue> 
#include <vector> 
#include <algorithm> 

PlannerNode::PlannerNode() : Node("planner") {

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);


  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), 
    std::bind(&PlannerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Planner Node has been initialized.");
}
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = *msg;
  map_received_ = true;
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = *msg;
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_point_ = *msg;
  goal_received_ = true;
  state_ = State::PLANNING; 
  RCLCPP_INFO(this->get_logger(), "New Goal Received: (%.2f, %.2f)", goal_point_.point.x, goal_point_.point.y);

  if (map_received_) {
    planPath(); 
  } else {
    RCLCPP_WARN(this->get_logger(), "Map not received yet. Cannot plan path.");
  }
}

void PlannerNode::timerCallback() 
{
  if (state_ == State::PLANNING) {
    double dx = goal_point_.point.x - robot_odom_.pose.pose.position.x;
    double dy = goal_point_.point.y - robot_odom_.pose.pose.position.y;
    double dist_to_goal = std::sqrt(dx*dx + dy*dy);

    if (dist_to_goal < 0.3) {
      RCLCPP_INFO(this->get_logger(), "Goal Reached!");
      state_ = State::WAITING_FOR_GOAL;
      return;
    }
    planPath();
  }
}

void PlannerNode::planPath()
{
  if (!map_received_ || !goal_received_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Map or Goal not received.");
    return;
  }

  double res = map_.info.resolution; 
  double ox = map_.info.origin.position.x;
  double oy = map_.info.origin.position.y;
  int width = map_.info.width;

  int start_x = static_cast<int>((robot_odom_.pose.pose.position.x - ox) / res);
  int start_y = static_cast<int>((robot_odom_.pose.pose.position.y - oy) / res);
  int goal_x = static_cast<int>((goal_point_.point.x - ox) / res);
  int goal_y = static_cast<int>((goal_point_.point.y - oy) / res);

  if (!isValid(start_x, start_y) || !isValid(goal_x, goal_y)) {
    RCLCPP_ERROR(this->get_logger(), "Start or Goal position is invalid or occupied.");
    return;
  }

  auto cmp = [](std::shared_ptr<AStarNode> left, std::shared_ptr<AStarNode> right) { return left->f_cost > right->f_cost; };
  std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, decltype(cmp)> open_list(cmp);

  std::vector<bool> visited(map_.data.size(), false);

  double h_start = calculateHeuristic(start_x, start_y, goal_x, goal_y); 
  
  std::shared_ptr<AStarNode> start_node = std::make_shared<AStarNode>(start_x, start_y, 0.0, h_start, nullptr);
  open_list.push(start_node);

  RCLCPP_INFO(this->get_logger(), "Planning path from (%d, %d) to (%d, %d)", start_x, start_y, goal_x, goal_y);

  while (!open_list.empty()) {

    std::shared_ptr<AStarNode> current = open_list.top();
    open_list.pop();

    int index = current->y * width + current->x;
    if (visited[index]) continue;
    visited[index] = true;

    // Check if Goal Reached
    if (current->x == goal_x && current->y == goal_y) {
      RCLCPP_INFO(this->get_logger(), "Path Found!");
      publishPath(current);
      return;
    }

    // Check Neighbors (Up, Down, Left, Right)
    int dx[] = {0, 0, 1, -1};
    int dy[] = {1, -1, 0, 0};

    for (int i = 0; i < 4; i++) {
      int nx = current->x + dx[i];
      int ny = current->y + dy[i];

      if (isValid(nx, ny)) {
        int n_index = ny * width + nx;
        if (!visited[n_index]) {
          // G Cost: Cost to get here (parent g + 1 step)
          double new_g = current->g_cost + 1.0; 
          double new_h = calculateHeuristic(nx, ny, goal_x, goal_y);
          
          std::shared_ptr<AStarNode> neighbor = std::make_shared<AStarNode>(nx, ny, new_g, new_h, current);
          open_list.push(neighbor);
        }
      }
    }
  }
  RCLCPP_WARN(this->get_logger(), "No path found to goal.");
}

bool PlannerNode::isValid(int x, int y) {
  int width = map_.info.width;
  int height = map_.info.height;

  if (x < 0 || x >= width || y < 0 || y >= height) return false;

  int index = y * width + x;
  int8_t val = map_.data[index]; 

  return (val < 20 || val == -1); 
}

double PlannerNode::calculateHeuristic(int x1, int y1, int x2, int y2) {
  return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

void PlannerNode::publishPath(std::shared_ptr<AStarNode> goal_node) 
{
  nav_msgs::msg::Path path_msg; 
  path_msg.header.stamp = this->get_clock()->now(); 
  path_msg.header.frame_id = "sim_world"; 

  std::shared_ptr<AStarNode> current = goal_node; 
  while (current != nullptr) 
  {
    geometry_msgs::msg::PoseStamped pose; 
    pose.header = path_msg.header; 

    pose.pose.position.x = (current->x * map_.info.resolution) + map_.info.origin.position.x; 
    pose.pose.position.y = (current->y * map_.info.resolution) + map_.info.origin.position.y;
    pose.pose.orientation.w = 1.0; 
    
    path_msg.poses.push_back(pose);
    current = current->parent;
  }
  std::reverse(path_msg.poses.begin(), path_msg.poses.end()); 
  path_pub_->publish(path_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}