#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  getParameters();

  // Initialize subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

  planner_.initializePlanner(smoothing_factor_, iterations_);
}

void PlannerNode::getParameters() {
  this->declare_parameter<double>("smoothing_factor", 0.2);
  this->declare_parameter<int>("iterations", 20);
  this->declare_parameter<double>("goal_tolerance", 0.3);
  this->declare_parameter<double>("plan_timeout_seconds", 1.0);

  smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();
  iterations_ = this->get_parameter("iterations").as_int();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  plan_timeout_ = this->get_parameter("plan_timeout_seconds").as_double();

}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ = map_msg;
  }

  if (active_goal_) {
    double elapsed = (now() - plan_start_time_).seconds();
    if (elapsed > plan_timeout_) {
      RCLCPP_WARN(this->get_logger(), "Plan timeout, resetting goal.");
      resetGoal();
    } else {
      RCLCPP_INFO(this->get_logger(), "Planning in progress... (time elapsed: %0.2f seconds)", elapsed);
      publishPath();
    }
  }

}

void PlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg) {

  if (active_goal_) {
    RCLCPP_WARN(this->get_logger(), "Ignoring new goal, since a plan is already in progress...");
    return;
  }

  if (!map_) {
    RCLCPP_WARN(this->get_logger(), "Received empty costmap, so goal cannot be set.");
    return;
  }

  current_goal = *goal_msg;
  active_goal_ = true;
  plan_start_time_ = now();

  RCLCPP_INFO(this->get_logger(), "Received new goal: (%0.2f, %0.2f)", goal_msg->point.x, goal_msg->point.y);
  publishPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  odom_x_ = odom_msg->pose.pose.position.x;
  odom_y_ = odom_msg->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {

  if (active_goal_) {
    double elapsed = (now() - plan_start_time_).seconds();
    if (elapsed > plan_timeout_) {
      RCLCPP_WARN(this->get_logger(), "Plan timeout, resetting goal.");
      resetGoal();
      return;
    } 
    RCLCPP_INFO(this->get_logger(), "Planning in progress... (time elapsed: %0.2f seconds)", elapsed);

    // Check if the robot is close to the goal
    double distance = std::sqrt(std::pow(current_goal_.point.x - odom_x_, 2) +
                       std::pow(current_goal_.point.y - odom_y_, 2));
    
    if (distance < goal_tolerance_) {
      RCLCPP_INFO(this->get_logger(), "Goal reached! Elapsed time: (%.2f). Resetting goal.", elapsed);
      resetGoal();
      return;
    }
  }
}

void PlannerNode::publishPath() {
  if (!have_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry data available, cannot publish path.");
    resetGoal();
    return;
  }

  // Use the latest odometry data as start point
  double start_world_x = odom_x_;
  double start_world_y = odom_y_;

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if(!planner_.planPath(start_world_x, start_world_y, current_goal_.point.x, current_goal_.point.y, map_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan path.");
      resetGoal();
      return;
    }
  }

  nav_msgs::msg::Path path_msg = *planner_.getPath();
  path_msg.header.stamp = now();
  path_msg.header.frame_id = map_->header.frame_id;

  path_pub_->publish(path_msg);
}

void PlannerNode::resetGoal() {
  active_goal_ = false;
  RCLCPP_INFO(this->get_logger(), "Resetting active goal.");

  // Reset the current path to 0 (stop or clear the path)
  nav_msgs::msg::Path empty_path_msg;
  empty_path_msg.header.stamp = now();

  // Use costmap frame if available
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (map_) {
      empty_path_msg.header.frame_id = map_->header.frame_id;
    } else {
      empty_path_msg.header.frame_id = "sim_world";
    }
  }

  path_pub_->publish(empty_path_msg);
  RCLCPP_INFO(this->get_logger(), "Published empty path to stop the robot.");

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
