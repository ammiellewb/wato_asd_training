#include "map_memory_node.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  getParameters();

  // Initialize map memory (subs)
  local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::localCostmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  
  // Initialize publisher
  global_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MapMemoryNode::timerCallback, this));

  map_memory_.initializeMapMemory(resolution_, width_, height_, origin_);

  RCLCPP_INFO(this->get_logger(), "Map Memory Node initialized with resolution: %.2f, width: %d, height: %d", resolution_, width_, height_);
}

void MapMemoryNode::getParameters() {
  this->declare_parameter<int>("map_pub_rate", 500);
  this->declare_parameter<double>("distance_threshold_meters", 1.0);
  this->declare_parameter<double>("global_map.resolution", 0.1);
  this->declare_parameter<int>("global_map.width", 100);
  this->declare_parameter<int>("global_map.height", 100);
  this->declare_parameter<double>("global_map.origin.position.x", -5.0);
  this->declare_parameter<double>("global_map.origin.position.y", -5.0);
  this->declare_parameter<double>("global_map.origin.orientation.w", 1.0);

  map_pub_rate_ = this->get_parameter("map_pub_rate").as_int();
  distance_threshold_meters_ = this->get_parameter("distance_threshold_meters").as_double();
  resolution_ = this->get_parameter("global_map.resolution").as_double();
  width_ = this->get_parameter("global_map.width").as_int();
  height_ = this->get_parameter("global_map.height").as_int();
  origin_.position.x = this->get_parameter("global_map.origin.position.x").as_double();
  origin_.position.y = this->get_parameter("global_map.origin.position.y").as_double();
  origin_.orientation.w = this->get_parameter("global_map.origin.orientation.w").as_double();
}

void MapMemoryNode::localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  bool all_cells_zero = std::all_of(msg->data.begin(), msg->data.end(), [](int8_t cell) { return cell == 0; });

  if (all_cells_zero) {
    RCLCPP_WARN(this->get_logger(), "Received empty costmap, skipping update.");
    return;
  }

  // Check distance traveled since last update
  if (!std::isnan(last_robot_x_)) {
    double dist = std::hypot(robot_x_ - last_robot_x_, robot_y_ - last_robot_y_);
    if (dist < distance_threshold_meters_) {
      RCLCPP_INFO(this->get_logger(), "Distance traveled is less than threshold, skipping update.");
      return;
    }
  }
  last_robot_x_ = robot_x_;
  last_robot_y_ = robot_y_;

  map_memory_.updateMapMemory(msg, robot_x_, robot_y_, robot_theta_);
}
 
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // Compute distance traveled
  double quaternion_x = msg->pose.pose.orientation.x;
  double quaternion_y = msg->pose.pose.orientation.y;
  double quaternion_z = msg->pose.pose.orientation.z;
  double quaternion_w = msg->pose.pose.orientation.w;
  robot_theta_ = quaternionToYaw(quaternion_x, quaternion_y, quaternion_z, quaternion_w);
}

// Timer-based map update
void MapMemoryNode::timerCallback() { 
  // Publish map every 
  nav_msgs::msg::OccupancyGrid map_msg = *map_memory_.publishMapData();
  map_msg.header.stamp = this->now();
  map_msg.header.frame_id = "sim_world";
  global_costmap_pub_->publish(map_msg);
}
 
double MapMemoryNode::quaternionToYaw(double x, double y, double z, double w) {
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
