#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ControlNode : public rclcpp::Node {
public:
  ControlNode();

private:
  // --- Callbacks ---
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  // --- Helpers ---
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

  // --- ROS Interfaces ---
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Data ---
  nav_msgs::msg::Path current_path_;
  nav_msgs::msg::Odometry robot_odom_;
  
  // --- State Flags ---
  bool path_received_ = false;
  bool odom_received_ = false;
  
  // Alignment Flag: True once we have rotated to face the path start
  bool aligned_to_end_ = false; 

  // --- Tuning Parameters ---
  double lookahead_distance_;
  double linear_speed_;
  double center_offset_; // Dist from Rear Axle to Center of Robot
};

#endif