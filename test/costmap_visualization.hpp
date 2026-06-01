#include "matplot/matplot.h"
#include "matplot/util/keywords.h"

#include <autoware/freespace_planning_algorithms/abstract_algorithm.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

void plot_planning_result(const nav_msgs::msg::OccupancyGrid & occupancy_grid, 
  geometry_msgs::msg::Pose start_pose, 
  geometry_msgs::msg::Pose goal_pose, 
  std::vector<geometry_msgs::msg::Pose> intermediate_poses, 
  autoware::freespace_planning_algorithms::VehicleShape collision_vehicle_shape) 
{
  (void)start_pose; 
  (void)goal_pose; 
  (void)intermediate_poses; 
  (void)collision_vehicle_shape;
    
  // occupancy grid

  std::vector<std::vector<float>> grid_2d; 
  // uint8_t occupancy_grid_pixels = occupancy_grid.data.size();
  for (int col = 0; col < static_cast<int>(occupancy_grid.info.width); ++col) {
    std::vector<float> tmp_row;
    for (int row = 0; row < static_cast<int>(occupancy_grid.info.height); ++row) {
      tmp_row.emplace_back(occupancy_grid.data.at(col*occupancy_grid.info.width + row));
    }
    grid_2d.emplace_back(tmp_row);
  } 
  matplot::image(grid_2d);
  matplot::colorbar();

  matplot::hold(matplot::on);

  std::vector<float> x_coords;
  std::vector<float> y_coords;
  for (auto & intermediate_pose : intermediate_poses) {
    x_coords.push_back(intermediate_pose.position.x);
    y_coords.push_back(intermediate_pose.position.y);
  }

  matplot::plot(x_coords, y_coords, "--xr");
  
  // matplot::plot(x, y, "-o");
  // matplot::plot(x, transform(y, [](auto y) { return -y; }), "--xr");
  // matplot::plot(x, transform(x, [](auto x) { return x / pi - 1.; }), "-:gs");
  // matplot::plot({1.0, 0.7, 0.4, 0.0, -0.4, -0.7, -1}, "k");
  
  matplot::axis("equal");
  matplot::show();

}
