#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

    getParameters();

    double quaternionToYaw(double x, double y, double z, double w);

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void followPath();

    void timerCallback();

  private:
    robot::ControlCore control_;

    // Subscribers and publisher
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cm_vel_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Path and robot state
    double robot_x_;
    double robot_y_;
    double robot_theta_;

    int control_period_ms_;
    double lookahead_distance_;
    double steering_gain_;

    double max_steering_angle_;
    double linear_velocity_;
};

#endif
