#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Initialize the costmap with given parameters in params.yaml
    void initializeCostmap(double resolution, int width, int height, 
      const geometry_msgs::msg::Pose origin, double inflation_radius, int max_cost);
      
    // Update the costmap with new laser scan data
    void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr& laser_scan);
    void convertToGrid(double x, double y, int& grid_x, int& grid_y);
    bool in_grid(int grid_x, int grid_y);
    void markObstacle(int grid_x, int grid_y);

    // Inflate the obstacles in the costmap which are within the inflation radius
    void inflateObstacles(int origin_x, int origin_y);

    // Publish the costmap to a topic
    nav_msgs::msg::OccupancyGrid::SharedPtr publishCostmap() const;

  private:
  // Costmap container
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_msg_;
    rclcpp::Logger logger_;

    // Inflation parameters
    double inflation_radius_;
    int max_cost_;
};

}  

#endif  