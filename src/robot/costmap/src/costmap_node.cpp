#include <chrono>
#include <memory>
#include <cmath>
#include "costmap_node.hpp"


CostmapNode::CostmapNode() : Node("costmap"), costmap_(this->get_logger()) {
  // Initialize publishers
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10); 
  
  // Initialize subscriber
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 
    10, 
    std::bind(&CostmapNode::scanCallback, this, std::placeholders::_1)
  );

  // Initialize timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), 
    std::bind(&CostmapNode::publishMessage, this)
  );
}

void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
  auto map = nav_msgs::msg::OccupancyGrid(); 
  map.header = msg->header;
  map.header.frame_id = "sim_world"; 
  map.info.resolution = 0.1;
  map.info.width = 100; 
  map.info.height = 100; 

  // Center the map
  map.info.origin.position.x = -(map.info.width * map.info.resolution) / 2.0; 
  map.info.origin.position.y = -(map.info.height * map.info.resolution) / 2.0; 
  map.info.origin.position.z = 0.0; 
  map.info.origin.orientation.w = 1.0; 

  // Initialize data to 0
  map.data.assign(map.info.width * map.info.height, 0);

  float angle = msg->angle_min; 
  double inflation_radius = 3.0;
  int max_cost = 100; 

  for (size_t i = 0; i < msg->ranges.size(); i++)
  {
    float dist = msg->ranges[i];
    if (dist > msg->range_min && dist < msg->range_max) 
    {
      float x = dist * std::cos(angle); 
      float y = dist * std::sin(angle); 

      int grid_x = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
      int grid_y = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);

      if (grid_x >= 0 && grid_x < static_cast<int>(map.info.width) &&
          grid_y >= 0 && grid_y < static_cast<int>(map.info.height)) 
      {
        int index = grid_y * map.info.width + grid_x; 
        map.data[index] = 100; 

        // Inflate obstacles
        int radius_cells = static_cast<int>(inflation_radius / map.info.resolution); 
        
        for (int dy = -radius_cells; dy <= radius_cells; dy++)
        {
          for (int dx = -radius_cells; dx <= radius_cells; dx++)
          {
            int neighbour_x = grid_x + dx;
            int neighbour_y = grid_y + dy;
            
            if (neighbour_x >= 0 && neighbour_x < static_cast<int>(map.info.width) &&
                neighbour_y >= 0 && neighbour_y < static_cast<int>(map.info.height))
            {
              double dist = std::sqrt(dx*dx + dy*dy) * map.info.resolution; 
              
              if (dist <= inflation_radius) 
              {
                int cost = static_cast<int>(max_cost * (1.0 - (dist / inflation_radius))); 
                int neighbour_index = neighbour_y * map.info.width + neighbour_x;
                
                if (map.data[neighbour_index] < cost) 
                {
                  map.data[neighbour_index] = cost;
                }
              }
            }
          }
        }
      }
    }
    angle += msg->angle_increment;
  }
  
  costmap_pub_->publish(map); 
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}