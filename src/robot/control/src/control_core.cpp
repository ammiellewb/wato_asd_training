#include "control_core.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : path_(nav_msgs::msg::Path()), logger_(logger) {}

void ControlCore::initializeControlCore(
  double lookahead_distance,
  double max_steering_angle,
  double steering_gain,
  double linear_velocity
) {
  lookahead_distance_ = lookahead_distance;
  max_steering_angle_ = max_steering_angle;
  steering_gain_ = steering_gain;
  linear_velocity_ = linear_velocity;
}

void ControlCore::updatePath(nav_msgs::msg::Path path)
{
  RCLCPP_INFO(logger_, "Updating path");
  path_ = path;
}  

bool ControlCore::isPathEmpty() {
  return path_.poses.empty();
}

geometry_msgs::msg::Twist ControlCore::computeControlCommand(
  double robot_x,
  double robot_y,
  double robot_theta
) {
  geometry_msgs::msg::Twist control_command;
  unsigned int lookahead_index = findLookaheadPoint(robot_x, robot_y, robot_theta);

  if (lookahead_index >= path_.poses.size()) {
    RCLCPP_WARN(logger_, "No valid lookahead point found");
    return control_command;
  }

  // Lookahead point coordinates
  double lookahead_x = path_.poses[lookahead_index].pose.position.x;
  double lookahead_y = path_.poses[lookahead_index].pose.position.y;

  // Distance to lookahead point
  double delta_x = lookahead_x - robot_x;
  double delta_y = lookahead_y - robot_y;

  // Angle to lookahead point
  double angle_to_lookahead = std::atan2(delta_y, delta_x);

  // Steering angle calculation (difference between robot's heading and lookahead point)
  // Normalize the angle to be within -pi to pi
  double steering_angle = angle_to_lookahead - robot_theta;

  if (steering_angle > M_PI) {
    steering_angle -= 2 * M_PI;
  } else if (steering_angle < -M_PI) {
    steering_angle += 2 * M_PI;
  }



  if (std::abs(steering_angle) > std::abs(max_steering_angle_)) {
    control_command.linear.x = 0; 
  } else {
    control_command.linear.x = linear_velocity_;
  }

  // Steering angle saturation
  steering_angle = std::max(-max_steering_angle_, std::min(max_steering_angle_, steering_angle));

  // Angular velocity calculation
  double angular_velocity = steering_gain_ * steering_angle;

  control_command.angular.z = angular_velocity;

  return control_command;
}

unsigned int ControlCore::findLookaheadPoint(
  double robot_x,
  double robot_y,
  double robot_theta
) {
  int closest_index = 0;
  double closest_distance = std::numeric_limits<double>::max();
  bool found_forward = false;

  // Loop through the path points to find the closest point to the robot
  for (size_t i = 0; i < path_.poses.size(); ++i) {
    double path_x = path_.poses[i].pose.position.x;
    double path_y = path_.poses[i].pose.position.y;

    // Calculate the distance from the robot to the path point
    double distance = std::hypot(path_x - robot_x, path_y - robot_y);

    // Skip points that within the lookahead distance
    if (distance < lookahead_distance_) continue;

    double angle_to_point = std::atan2(path_y - robot_y, path_x - robot_x);

    // Calculate difference between robot's heading and the angle to the path point
    // Normalize the angle to be within -pi to pi
    double angle_diff = angle_to_point - robot_theta;

    if (angle_diff > M_PI) {
      angle_diff -= 2 * M_PI;
    } else if (angle_diff < -M_PI) {
      angle_diff += 2 * M_PI;
    }

    // Check if angle difference is within the forward direction
    if (std::abs(angle_diff) < M_PI / 2) {
      // If this point is closer than the previous closest point, update the closest index
      if (distance < closest_distance) {
        closest_distance = distance;
        closest_index = i;
        found_forward = true;
      }
    }
  }

  // If no forward point was found, return the last point in the path
  if (!found_forward) {
    // Find the closest point to the robot in the path, even if it's not in the forward direction
    for (size_t i = 0; i < path_.poses.size(); ++i) {
      double path_x = path_.poses[i].pose.position.x;
      double path_y = path_.poses[i].pose.position.y;

      // Calculate the distance from the robot to the path point
      double distance = std::hypot(path_x - robot_x, path_y - robot_y);

      // If this point is closer than the previous closest point, update the closest index
      if (distance < closest_distance) {
        closest_distance = distance;
        closest_index = i;
      }
    }
  }

  return closest_index;

}
}