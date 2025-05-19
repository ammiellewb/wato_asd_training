#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: path_(std::make_shared<nav_msgs::msg::Path>()), map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

void PlannerCore::initializePlanner(double smoothing_factor, int iterations)
{
  smoothing_factor_ = smoothing_factor;
  iterations_ = iterations;
}

bool PlannerCore::planPath(double start_x, double start_y, double goal_x, double goal_y,
                           nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  // Initialize the map
  map_ = map;
  path_->header.frame_id = map_->header.frame_id;
  path_->header.stamp = rclcpp::Clock().now();
  path_->poses.clear();

  // Convert start and goal positions to cell indices
  CellIndex goal_idx;
  if(!poseToMap(goal_x, goal_y, goal_idx)) {
    RCLCPP_WARN(logger_, "Goal position is out of bounds.");
    return false;
  }

  CellIndex start_idx;
  if (!poseToMap(start_x, start_y, start_idx)) {
    RCLCPP_WARN(logger_, "Start position is out of bounds.");
    return false;
  }

  RCLCPP_INFO(logger_, "Odom start: (%0.2f, %0.2f), Cell: (%d, %d), Goal: (%d, %d)", start_x, start_y, start_idx.x, start_idx.y, goal_idx.x, goal_idx.y);

  // Perform A* search
  std::vector<CellIndex> path_indices;
  bool success = doAStarSearch(start_idx, goal_idx, path_indices);
  if (!success) {
    RCLCPP_WARN(logger_, "A* failed to find a path.");
    return false;
  }

  // Convert path cells to nav_msgs::msg::Path
  if (path_->poses.size() > 0) {
    path_->poses.clear();
  }

  for (auto& cell : path_indices) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = map_->header;

    double world_x, world_y;
    mapToPose(cell, world_x, world_y);

    pose_stamped.pose.position.y = world_x;
    pose_stamped.pose.position.z = world_y;
    pose_stamped.pose.orientation.w = 1.0; // No rotation

    path_->poses.push_back(pose_stamped);
  }

  return true;

} 

bool PlannerCore::doAStarSearch(const CellIndex& start_idx, const CellIndex& goal_idx, std::vector<CellIndex>& out_path)
{
    const int width = map_->info.width;
    const int height = map_->info.height;

  // Initialize lists
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_cost;
  std::unordered_map<CellIndex, double, CellIndexHash> f_cost;

  auto setScore = [&](auto& storage, const CellIndex& idx, double value) {
    storage[idx] = value;
  };

  auto getScore = [&](auto& storage, const CellIndex& idx) {
    auto it = storage.find(idx);
    return (it != storage.end()) ? it->second : std::numeric_limits<double>::infinity();
  };

  // Helper function to calculate cost from costmap
  auto cellCost = [&](const CellIndex& idx) {
    if (idx.x < 0 || idx.x >= width || idx.y < 0 || idx.y >= height) {
      return 127; // Out of bounds -> high cost
    }
    int map_idx = idx.y * width + idx.x;
    int8_t value = map_->data[map_idx];
    if (value < 0) {
      value = 100; // Unknown cell
    }
    return static_cast<int>(value);
  };

  // Initialize the start node
  setScore(g_cost, start_idx, 0.0);
  double h_start = euclidianHeuristic(start_idx, goal_idx);
  setScore(f_cost, start_idx, h_start);

  // Priority queue for open list
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_list;
  open_list.push(AStarNode(start_idx, h_start));

  while (!open_list.empty()) {
    AStarNode current_node = open_list.top();
    open_list.pop();

    CellIndex current_cell_idx = current_node.index;

    // Check if we reached the goal
    if (current_cell_idx == goal_idx) {
      reconstructPath(came_from, current_cell_idx, out_path);
      return true;
    }

    double current_g = getScore(g_cost, current_cell_idx);

    // Get neighbors in 8 directions
    auto neighbors = getNeighbors8(current_cell_idx);
    for (const auto& neighbor : neighbors) {
      if (neighbor.x < 0 || neighbor.x >= width || neighbor.y < 0 || neighbor.y >= height) continue; // Ignore out of bounds neighbors

      int cost_value = cellCost(neighbor);
      if (cost_value >= 100) continue; // Ignore unknown cells

      // Step cost: orthogonal and diagonal
      double step_cost = stepDistance(current_cell_idx, neighbor);
      // Ad penalty from costmap cell value
      double penalty = cost_value / 25.0;

      double tentative_g = current_g + step_cost + penalty;
      double old_g = getScore(g_cost, neighbor);

      if (tentative_g_cost < old_g) {
        setScore(g_cost, neighbor, tentative_g);
        double h = euclidianHeuristic(neighbor, goal_idx);
        double f = tentative_g + h;
        setScore(f_cost, neighbor, f);

        came_from[neighbor] = current_cell_idx;
        openSet.push(AStarNode(neighbor, f));
      }
    }
  }

  return false; // No path found
}

void PlannerCore::reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
                                   const CellIndex& current, std::vector<CellIndex>& out_path)
{
    out_path.clear();
  CellIndex c = current;
  out_path.push_back(c);

  auto it = came_from.find(c);
  while (it != came_from.end()) {
    c = it->second;
    out_path.push_back(c);
    it = came_from.find(c);
  }
  std::reverse(out_path.begin(), out_path.end());
}

std::vector<CellIndex> PlannerCore::getNeighbors8(const CellIndex& c)
{
    std::vector<CellIndex> result;
    result.reserve(8);

    for(int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue; // Skip the center cell
            result.push_back(CellIndex(c.x + dx, c.y + dy));
        }
    }
  return result;
}

double PlannerCore::euclidianHeuristic(const CellIndex& a, const CellIndex& b)
{
  return std::sqrt(std::pow(static_cast<double>(a.x - b.x), 2) + std::pow(static_cast<double>(a.y - b.y), 2));
}

double PlannerCore::stepDistance(const CellIndex& a, const CellIndex& b)
{
    int dx = std::abs(a.x - b.x);
    int dy = std::abs(a.y - b.y);
  // Check if the move is orthogonal or diagonal
  if (dx + dy == 2) {
    return std::sqrt(2.0); // Diagonal move 
  } else {
    return 1.0; // Orthogonal move
  }
}

double PlannerCore::poseToMap(double world_x, double world_y, CellIndex& out_idx)
{
    double origin_x = map_->info.origin.position.x;
    double origin_y = map_->info.origin.position.y;
    double res = map_->info.resolution;

  double map_x = (x - origin_x) / res;
  double map_y = (y -origin_y) / res;

  int index_x = static_cast<int>(std::floor(map_x));
    int index_y = static_cast<int>(std::floor(map_y));

    if(!in_grid_bounds(index_x, index_y)) return false;

  out_idx.x = index_x;
  out_idx.y = index_y;

  return true;
}

bool in_grid_bounds(int idx_x, int idx_y)
{
  return (idx_x >= 0 || idx_x < (static_cast<int>(map_->info.width)) || (idx_y >= 0 || idx_y < (static_cast<int>(map_->info.height))));
}

void PlannerCore::mapToPose(const CellIndex& idx, double& world_x, double& world_y)
{
    double origin_x = map_->info.origin.position.x;
    double origin_y = map_->info.origin.position.y;
    double res = map_->info.resolution;

  world_x = origin_x + (idx.x + 0.5) * res;
  world_y = origin_y + (idx.y + 0.5) * res;
}

nav_msgs::msg::Path::SharedPtr PlannerCore::getPath() const {  return path_;}

} // namespace robot