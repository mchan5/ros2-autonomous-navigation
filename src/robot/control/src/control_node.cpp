#include "control_node.hpp"
#include <cmath> 
#include <algorithm> 
#include <memory> 

ControlNode::ControlNode(): Node("control") {

  // TUNING PARAMETERS
  lookahead_distance_ = 1.0;  
  linear_speed_ = 0.4; // Max speed

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Run at 20Hz (50ms)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), 
    std::bind(&ControlNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Control Node: Regulated Pure Pursuit");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  
  // Check if this is a "New Mission" or just a "Correction"
  bool is_new_mission = false;

  if (current_path_.poses.empty()) {
      // If there was no path before, assume this is a new mission
      is_new_mission = true;
  } 
  else if (!msg->poses.empty()) {
      // If the goal moved > 1.0 meter, it's a new destination.
      double old_end_x = current_path_.poses.back().pose.position.x;
      double old_end_y = current_path_.poses.back().pose.position.y;
      
      double new_end_x = msg->poses.back().pose.position.x;
      double new_end_y = msg->poses.back().pose.position.y;

      double goal_dist = std::sqrt(std::pow(new_end_x - old_end_x, 2) + 
                                   std::pow(new_end_y - old_end_y, 2));
      
      if (goal_dist > 1.0) {
          is_new_mission = true;
      }
  }

  // Update the path data (So we see the curve around the obstacle)
  current_path_ = *msg;
  path_received_ = true;

  // Only Reset Alignment (Stop & Turn) if it's a brand new mission
  if (is_new_mission) {
      aligned_to_end_ = false; 
      RCLCPP_INFO(this->get_logger(), "New Mission Detected (Goal Changed). Re-aligning.");
  } else {
      // If it's just an update, say nothing and keep driving.
      // TimerCallback will automatically finds nearest point
  }
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = *msg;
  odom_received_ = true;
}

void ControlNode::timerCallback() 
{
  if (!path_received_ || !odom_received_ || current_path_.poses.empty()) return; 

  double rx = robot_odom_.pose.pose.position.x; 
  double ry = robot_odom_.pose.pose.position.y; 
  double ryaw = getYawFromQuaternion(robot_odom_.pose.pose.orientation);

  auto cmd = geometry_msgs::msg::Twist(); 

  // This rotates the robot to face the path BEFORE we start driving.
  if (!aligned_to_end_) {
      // Look at a point 10m down the path (not the end, just "forward")
      // We find the first point ~10m away to orient ourselves.
      double target_heading = ryaw; 
      bool found = false;
      for (auto & p : current_path_.poses) {
          double dx = p.pose.position.x - rx;
          double dy = p.pose.position.y - ry;
          if (std::sqrt(dx*dx + dy*dy) > 10) { 
             target_heading = std::atan2(dy, dx);
             found = true;
             break;
          }
      }
      if (!found) target_heading = ryaw; // Just drive if path is tiny

      double error = target_heading - ryaw;
      while (error > M_PI) error -= 2.0 * M_PI;
      while (error < -M_PI) error += 2.0 * M_PI;

      if (std::abs(error) < 0.1) {
          aligned_to_end_ = true; // Done! Start driving.
      } else {
          cmd.linear.x = 0.0;
          cmd.angular.z = 1.0 * (1.0 * error); // Rotate in place
          cmd_vel_pub_->publish(cmd);
          return;
      }
  }

  // Prune passed points (Prevent looping back)
  double min_dist = 1e9;
  int closest_i = -1;
  for (size_t i = 0; i < current_path_.poses.size(); i++) {
      double d = std::hypot(current_path_.poses[i].pose.position.x - rx, 
                            current_path_.poses[i].pose.position.y - ry);
      if (d < min_dist) { min_dist = d; closest_i = i; }
  }
  if (closest_i > 0) {
      current_path_.poses.erase(current_path_.poses.begin(), current_path_.poses.begin() + closest_i);
  }

  // Find next point
  double tx = rx, ty = ry;
  bool target_found = false;
  for (const auto& pose : current_path_.poses) {
      double d = std::hypot(pose.pose.position.x - rx, pose.pose.position.y - ry);
      if (d >= lookahead_distance_) {
          tx = pose.pose.position.x;
          ty = pose.pose.position.y;
          target_found = true;
          break;
      }
  }
  // If we ran out of points, target the very last one
  if (!target_found && !current_path_.poses.empty()) {
      tx = current_path_.poses.back().pose.position.x;
      ty = current_path_.poses.back().pose.position.y;
  }

  // Calculate Steering (Pure Pursuit)
  double dx = tx - rx;
  double dy = ty - ry;
  double local_y = -std::sin(ryaw) * dx + std::cos(ryaw) * dy; // Coordinate transform
  double dist_sq = dx*dx + dy*dy;

  // Curvature = 2y / L^2
  double curvature = 1.0 * (2.0 * local_y) / dist_sq; 
  
  // Base steering command
  cmd.angular.z = curvature * linear_speed_;
  
  double dist_to_goal = std::hypot(current_path_.poses.back().pose.position.x - rx, 
                                   current_path_.poses.back().pose.position.y - ry);

  if (dist_to_goal < 0.3) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200000, "Goal Reached.");
  } else {
      // If angular velocity is 1.0 rad/s, reduce speed by half.
      double regulation_factor = 1.0 + std::abs(cmd.angular.z) * 2.0;
      cmd.linear.x = linear_speed_ / regulation_factor;
      
      // Minimum speed check (don't stall)
      cmd.linear.x = std::max(cmd.linear.x, 0.15); 
  }

  // Final bounds for safety
  cmd.angular.z = std::clamp(cmd.angular.z, -1.5, 1.5);
  
  cmd_vel_pub_->publish(cmd);
}

double ControlNode::getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat) {
  double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}