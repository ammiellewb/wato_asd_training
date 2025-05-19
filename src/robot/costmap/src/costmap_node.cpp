#include "costmap_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <memory>
 
// Constructor for the CostmapNode class
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  getParameters();
  
  // Laser scan data which costmap will subscribe to
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1));
  // Occupany grid which costmap will publish
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  costmap_.initializeCostmap(resolution_, width_, height_, origin_, inflation_radius_, max_cost_);

  RCLCPP_INFO(this->get_logger(), "Costmap Node initialized");
}

// Get parameters from params.yaml
void CostmapNode::getParameters() {
  this->declare_parameter<double>("costmap.resolution", 0.1);
  this->declare_parameter<int>("costmap.width", 100);
  this->declare_parameter<int>("costmap.height", 100);
  this->declare_parameter<double>("costmap.origin.position.x", -5.0);
  this->declare_parameter<double>("costmap.origin.position.y", -5.0);
  this->declare_parameter<double>("costmap.origin.orientation.w", 1.0);
  this->declare_parameter<double>("costmap.inflation_radius", 1.5);
  this->declare_parameter<int>("costmap.max_cost", 100);

  resolution_ = this->get_parameter("costmap.resolution").as_double();
  width_ = this->get_parameter("costmap.width").as_int();
  height_ = this->get_parameter("costmap.height").as_int();
  origin_.position.x = this->get_parameter("costmap.origin.position.x").as_double();
  origin_.position.y = this->get_parameter("costmap.origin.position.y").as_double();
  origin_.orientation.w = this->get_parameter("costmap.origin.orientation.w").as_double();
  inflation_radius_ = this->get_parameter("costmap.inflation_radius").as_double();
  max_cost_ = this->get_parameter("costmap.max_cost").as_int();
}

// Callback function
void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const {
  // Process the laser scan data and publish the costmap
  costmap_.processLaserScan(msg);
  nav_msgs::msg::OccupancyGrid costmap_msg = *costmap_.publishCostmap();
  costmap_msg.header = msg->header;
  costmap_pub_->publish(costmap_msg);
  RCLCPP_INFO(this->get_logger(), "Costmap published");
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}