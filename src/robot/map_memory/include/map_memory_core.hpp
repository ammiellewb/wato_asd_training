#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void initializeMapMemory(double resolution, int width, int height, geometry_msgs::msg::Pose origin); 
    
    void updateMapMemory(nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap, 
      double robot_x, double robot_y, double robot_theta);
    
    bool in_grid(int map_x, int map_y);
    bool robotToMap(double robot_x, double robot_y, int& map_x, int& map_y);

    // Publish the global map
    nav_msgs::msg::OccupancyGrid::SharedPtr publishMapData() const;
  
  private:
    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
    rclcpp::Logger logger_;
};

}  

#endif  
