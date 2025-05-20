#include "map_memory_core.hpp"

#include <eigen3/Eigen/Core>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : global_map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

void MapMemoryCore::initializeMapMemory(double resolution, int width, int height, geometry_msgs::msg::Pose origin) {
  global_map_->info.width = width;
  global_map_->info.height = height;
  global_map_->info.resolution = resolution;
  global_map_->info.origin = origin;
  global_map_->data.assign(width * height, 0);

  RCLCPP_INFO(logger_, "Global Map initialized with resolution: %.2f, width: %d, height: %d", resolution, width, height);
}

void MapMemoryCore::updateMapMemory(nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap, double robot_x, double robot_y, double robot_theta) {
  double local_res = local_costmap->info.resolution;
  double local_origin_x = local_costmap->info.origin.position.x;
  double local_origin_y = local_costmap->info.origin.position.y;
  unsigned int local_width = local_costmap->info.width;
  unsigned int local_height = local_costmap->info.height;
  const auto& local_data = local_costmap->data;

  for (unsigned int i = 0; i < local_height; ++i) {
    for (unsigned int j = 0; j < local_width; ++j) {
      int8_t occ_val = local_data[i * local_width + j];
      if (occ_val < 0) continue;  // Skip unknown cells

      // Transform local cell coordinates to global map coordinates
      double local_cell_x = local_origin_x + (0.5+j) * local_res;
      double local_cell_y = local_origin_y + (0.5+i) * local_res;

      // Use 2D transform:
      //  x' = x * cos(theta) - y * sin(theta) + robot_x
      //  y' = x * sin(theta) + y * cos(theta) + robot_y
      double cos_t = std::cos(robot_theta);
      double sin_t = std::sin(robot_theta);
      double global_cell_x = (local_cell_x * cos_t - local_cell_y * sin_t) + robot_x; 
      double global_cell_y = (local_cell_x * sin_t + local_cell_y * cos_t) + robot_y;

      int map_x, map_y;
      if(!robotToMap(global_cell_x, global_cell_y, map_x, map_y)) {
        RCLCPP_WARN(logger_, "Global cell out of bounds: (%f, %f)", global_cell_x, global_cell_y);
        continue;
      }
      
      // Update global map with local cost
      int8_t& global_cost = global_map_->data[map_y * global_map_->info.width + map_x];

      // If global cell is -1, set it to 0
      int curr_global_cost = (static_cast<int>(global_cost) < 0) ? 0 : static_cast<int>(global_cost);
      int local_cost = static_cast<int>(occ_val);

      // Merge by taking the maximum cost
      int merged_cost = std::max(curr_global_cost, local_cost);
      global_cost = static_cast<int8_t>(merged_cost);
    }
  }
}

bool MapMemoryCore::robotToMap(double robot_x, double robot_y, int& map_x, int& map_y) {
  double map_origin_x = global_map_->info.origin.position.x;
  double map_origin_y = global_map_->info.origin.position.y;
  double map_resolution = global_map_->info.resolution;

  // Offset from origin
  if (robot_x < map_origin_x || robot_y < map_origin_y) return false; // Out of bounds

  // Convert robot coordinates to map coordinates
  map_x = static_cast<int>((robot_x - map_origin_x) / map_resolution);
  map_y = static_cast<int>((robot_y - map_origin_y) / map_resolution);

  // Check if the coordinates are within bounds
  if (in_grid(map_x, map_y)) {
    return false; // Out of bounds
  } else {
    return true; // Valid coordinates
  }
}

bool MapMemoryCore::in_grid(int map_x, int map_y)
{
  return (map_x < 0 || map_x >= static_cast<int>(global_map_->info.width) ||
          map_y < 0 || map_y >= static_cast<int>(global_map_->info.height));
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::publishMapData() const {return global_map_;}

} 
