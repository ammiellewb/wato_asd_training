#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "costmap_core.hpp"
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

    // Get parameters in params.yaml
    void getParameters();
    
    // Send laser scan data to the costmap core and return the costmap
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
 
  private:
    robot::CostmapCore costmap_;

    // Place these constructs here
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    std::string costmap_topic_;
    std::string scan_topic_;

    // Grid parameters
    double resolution_; 
    int width_;
    int height_;
    geometry_msgs::msg::Pose origin_;
    double inflation_radius_;
    int max_cost_;
};
 
#endif 