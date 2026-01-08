#include <cmath>
#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory") {
  // Initialize Subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Initialize Publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize Timer (1.0 seconds)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), 
    std::bind(&MapMemoryNode::timerCallback, this));

  // Initialize the Global Map
 
  global_map_.header.frame_id = "sim_world"; 
  global_map_.info.resolution = 0.1;
  global_map_.info.width = 400;  
  global_map_.info.height = 400; 
  
  global_map_.info.origin.position.x = -(global_map_.info.width * global_map_.info.resolution) / 2.0;
  global_map_.info.origin.position.y = -(global_map_.info.height * global_map_.info.resolution) / 2.0;
  global_map_.info.origin.orientation.w = 1.0;
  
  global_map_.data.assign(global_map_.info.width * global_map_.info.height, -1);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = *msg;
}

void MapMemoryNode::timerCallback() {
  if (!costmap_received_) return;

  double current_x = robot_odom_.pose.pose.position.x;
  double current_y = robot_odom_.pose.pose.position.y;
  
  double last_x = last_update_odom_.pose.pose.position.x;
  double last_y = last_update_odom_.pose.pose.position.y;

  double dx = current_x - last_x;
  double dy = current_y - last_y;
  double dist = std::sqrt(dx*dx + dy*dy);

  if (dist >= 1.5 || first_update_) {
    updateGlobalMap();

    last_update_odom_ = robot_odom_;
    first_update_ = false;
    
    global_map_.header.stamp = this->get_clock()->now();
    map_pub_->publish(global_map_);
    RCLCPP_INFO(this->get_logger(), "Global Map Updated");
  }
}

void MapMemoryNode::updateGlobalMap() {

  double yaw = getYawFromOdom(robot_odom_);
  double cos_theta = std::cos(yaw);
  double sin_theta = std::sin(yaw);

  double robot_x = robot_odom_.pose.pose.position.x;
  double robot_y = robot_odom_.pose.pose.position.y;

  int width = latest_costmap_.info.width;
  int height = latest_costmap_.info.height;
  double res = latest_costmap_.info.resolution;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int local_index = y * width + x;
      int8_t cost = latest_costmap_.data[local_index];

      if (cost > 50) { 
        double local_x = (x * res) + latest_costmap_.info.origin.position.x;
        double local_y = (y * res) + latest_costmap_.info.origin.position.y;

        // Rotate and Translate
        double global_x = (local_x * cos_theta - local_y * sin_theta) + robot_x;
        double global_y = (local_x * sin_theta + local_y * cos_theta) + robot_y;

        int gx = static_cast<int>((global_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
        int gy = static_cast<int>((global_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

        if (gx >= 0 && gx < static_cast<int>(global_map_.info.width) &&
            gy >= 0 && gy < static_cast<int>(global_map_.info.height)) {
          
          int global_index = gy * global_map_.info.width + gx;
          global_map_.data[global_index] = 100; 
        }
      }
    }
  }
}


double MapMemoryNode::getYawFromOdom(const nav_msgs::msg::Odometry& odom) {
  double qx = odom.pose.pose.orientation.x;
  double qy = odom.pose.pose.orientation.y;
  double qz = odom.pose.pose.orientation.z;
  double qw = odom.pose.pose.orientation.w;
  
  double siny_cosp = 2 * (qw * qz + qx * qy);
  double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}