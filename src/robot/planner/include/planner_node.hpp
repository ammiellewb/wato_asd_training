#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "planner_core.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <mutex>

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

    void getParameters();

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void timerCallback();

    void publishPath();
    void resetGoal();

  private:
    robot::PlannerCore planner_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Costmap & Mutex
    nav_msgs::msg::OccupancyGrid map_;
    std::mutex map_mutex_;

    // Current goal
    geometry_msgs::msg::PoseStamped current_goal_;
    bool active_goal_;
    rclcpp::Time plan_start_time_;

    // Robot odometry
    bool have_odom_;
    double odom_x_;
    double odom_y_;

    // Parameters
    double smoothing_factor_;
    int iterations_;
    double goal_tolerance_;
    double plan_timeout_;
};

#endif 
