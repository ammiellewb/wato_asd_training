#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cmath>
#include <queue>
#include <vector>
#include <limits>
#include <unordered_map>
#include <chrono>

namespace robot
{

struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex& other) const {
    return x == other.x && y == other.y;
  }

  bool operator!=(const CellIndex& other) const {
    return (x!=other.x || y!=other.y);
  }

};

struct CellIndexHash {
  std::size_t operator()(const CellIndex& index) const {
    return std::hash<int>()(index.x) ^ (std::hash<int>()(index.y) << 1);
  }
};

struct AStarNode {
  CellIndex index;
  double f_cost; // Total cost (g + h)

  AStarNode(CellIndex idx, double f) : index(idx), f_cost(f) {}
};

struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) {
    return a.f_cost > b.f_cost; // Min-heap
  }
}

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    void initializePlanner(double smoothing_factor, int iterations);

    // Function to set the map
    bool planPath(double start_x, double start_y, double goal_x, double goal_y,
                  nav_msgs::msg::OccupancyGrid::SharedPtr map);

    // Function to get the path
    bool doAStarSearch(const CellIndex& start_idx, const CellIndex& goal_idx, std::vector<CellIndex>& out_path);

    // Helper function to get the neighbors of a cell
    void reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
                         const CellIndex& current, std::vector<CellIndex>& out_path);

    // 8-direction neighbor offsets
    std::vector<CellIndex> getNeighbors8(const CellIndex& c);
    // Euclidean distance
    double euclidianHeuristic(const CellIndex& a, const CellIndex& b);
    // Step cost for orthogonal and diagonal moves
    double stepDistance(const CellIndex& a, const CellIndex& b);
    
    void lineOfSightSmoothing(std::vector<CellIndex>& path_cells);

    // Convert world coordinates to grid indices
    bool poseToMap(double world_x, double world_y, CellIndex& out_idx);
    bool in_grid_bounds(int index_x, int index_y);

    // Convert map cell to world coordinates
    bool mapToPose(const CellIndex& idx, double& world_x, double& world_y);

    bool lineOfSight(const CellIndex& start, const CellIndex& end);
    
    bool isCellFree(int x, int y);

    nav_msgs::msg::Path::SharedPtr publishPathMsg() const;
  
  private:
    double smoothing_factor_;
    int iterations_;

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::Path::SharedPtr path_;
};

}  

#endif  
