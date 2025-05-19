#include "costmap_core.hpp"

#include <cmath>
#include <algorithm>
#include <queue>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : costmap_msg_(std::make_shared<nav_msgs::msg::OccupancyGrid>(), logger_(logger)) {}

void CostmapCore::initializeCostmap(double resolution, int width, int height, 
    geometry_msgs::msg::Pose origin, double inflation_radius, int max_cost)
{
  costmap_msg_->info.resolution = resolution;
  costmap_msg_->info.width = width;
  costmap_msg_->info.height = height;
  costmap_msg_->info.origin = origin;
  costmap_msg_->data.assign(width * height, -1);

  inflation_radius_ = inflation_radius;
  max_cost_ = max_cost;
  max_cost_ = static_cast<int>(inflation_radius / resolution);

  RCLCPP_INFO(logger_, "Costmap initialized with resolution: %f, width: %d, height: %d", resolution, width, height);
}

void CostmapCore::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr& laser_scan) const
{
  // Clear the costmap
  std::fill(costmap_msg_->data.begin(), costmap_msg_->data.end(), 0);

  double angle = laser_scan->angle_min;
  
  for (int i = 0; i < static_cast<int>(laser_scan->ranges.size()); ++i) {
    angle += i * laser_scan->angle_increment;

    double range = laser_scan->ranges[i];

    // Check if the range is valid
    if (range >= laser_scan->range_min && range <= laser_scan->range_max) {
        // Calculate obstacle position coordinates
        double x = range * std::cos(angle);
        double y = range * std::sin(angle);

        // Convert to grid coordinates
        int grid_x, grid_y;
        convertToGrid(x, y, grid_x, grid_y);
        
        if(in_grid(grid_x, grid_y)) {
          markObstacle(grid_x, grid_y);
        }
    }
  }

  for (const auto& [grid_x, grid_y] : obstacles) {
    inflateObstacles(grid_x, grid_y);
  }

}

void CostmapCore::convertToGrid(double x, double y, int& grid_x, int& grid_y)
{
  double origin_x = costmap_msg_->info.origin.position.x;
  double origin_y = costmap_msg_->info.origin.position.y;
  double resolution = costmap_msg_->info.resolution;

  grid_x = static_cast<int>((x - origin_x) / resolution);
  grid_y = static_cast<int>((y - origin_y) / resolution);
}

bool CostmapCore::in_grid(int grid_x, int grid_y)
{
  return (grid_x >= 0 && grid_x < static_cast<int>(costmap_msg_->info.width)) &&
          (grid_y >= 0 && grid_y < static_cast<int>(costmap_msg_->info.height));
}

void CostmapCore::markObstacle(int grid_x, int grid_y)
{
  int width_ = costmap_msg_->info.width;
  int index = grid_y * width_ + grid_x;
  costmap_msg_->data[index] = max_cost_; // Mark as occupied
}

void CostmapCore::inflateObstacles(int origin_x, int origin_y) const
{
  stdd::queue<std::pair<int, int>> queue;
  queue.emplace({origin_x, origin_y});

  int width = costmap_msg_->info.width;
  int height = costmap_msg_->info.height;
  int resolution = costmap_msg_->info.resolution;

  std::vector<std::vector<bool>> visited(width, std::vector<bool>(height, false));
  visited[origin_y][origin_x] = true;

  while (!queue.empty()) {
    auto [x, y] = queue.front();
    queue.pop();

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;

        int inflated_x = x + dx;
        int inflated_y = y + dy;

        if (in_grid(inflated_x, inflated_y) && !visited[inflated_y][inflated_x]) {
          // Calculate distance from the original obstacle
          double distance = std::hypot(inflated_x - origin_x, inflated_y - origin_y) * resolution;

          // Check if within inflation radius and mark as inflated, add to queue
          if (distance <= inflation_radius_) {
            int cost = static_cast<int>(1.0 - (distance / inflation_radius_)) * max_cost_;
            int index = inflated_y * width + inflated_x;
            if (cost > costmap_msg_->data[index]) {
              costmap_msg_->data[index] = cost;
            }
            queue.emplace(inflated_x, inflated_y);
          }
        visited[inflated_y][inflated_x] = true;
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::publishCostmap() const {
    return costmap_msg_;
}


}