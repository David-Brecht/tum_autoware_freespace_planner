#include "matplot/matplot.h"
#include <autoware/freespace_planning_algorithms/abstract_algorithm.hpp>

void plot_planning_result(const nav_msgs::msg::OccupancyGrid & occupancy_grid, 
    geometry_msgs::msg::Pose start_pose, 
    geometry_msgs::msg::Pose goal_pose, 
    PlannerWaypoints waypoints, 
    autoware::freespace_planning_algorithms::VehicleShape collision_vehicle_shape) {
  std::vector<double> x = linspace(0, 2 * pi);
  std::vector<double> y = transform(x, [](auto x) { return sin(x); });

  matplot::plot(x, y, "-o");
  matplot::hold(on);
  matplot::plot(x, transform(y, [](auto y) { return -y; }), "--xr");
  matplot::plot(x, transform(x, [](auto x) { return x / pi - 1.; }), "-:gs");
  matplot::plot({1.0, 0.7, 0.4, 0.0, -0.4, -0.7, -1}, "k");

  matplot::show();
}
