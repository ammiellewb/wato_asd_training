#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "map_memory_core.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <memory>
#include <cmath>
#include <limits>
#include <vector>

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void getParameters();

    void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    // Convert quaternion to yaw
    double quaternionToYaw(double x, double y, double z, double w);

  private: // Processing global costmap
    robot::MapMemoryCore map_memory_;

    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ROS2 parameters in params.yaml
    int map_pub_rate_;
    double resolution_;
    int width_;
    int height_;
    geometry_msgs::msg::Pose origin_;

    double distance_threshold_meters_;

    double robot_x_;
    double robot_y_;
    double robot_theta_;

    double last_robot_x_;
    double last_robot_y_;
};

#endif 
