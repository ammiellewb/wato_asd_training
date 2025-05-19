#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void initializeControlCore(double lookahead_distance, double max_steering_angle, double steering_gain, double linear_velocity);
  
    void updatePath(nav_msgs::msg::Path path);
    bool isPathEmpty();

    unsigned int findLookaheadPoint(double robot_x, double robot_y, double robot_theta);

    geometry_msgs::msg::Twist computeControlCommand(double robot_x, double robot_y, double robot_theta);

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::Path path_;

    double lookahead_distance_;
    double max_steering_angle_;
    double steering_gain_;
    double linear_velocity_;
};

} 

#endif 
