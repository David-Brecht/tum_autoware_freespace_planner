#pragma once

#include "matplot/core/figure_registry.h"
#include "matplot/matplot.h"

#include <autoware/freespace_planning_algorithms/abstract_algorithm.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <algorithm>
#include <string>
#include <vector>
#include <cstdint>

void plot_planning_result(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const geometry_msgs::msg::Pose & start_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const std::vector<geometry_msgs::msg::Pose> & intermediate_poses,
  const autoware::freespace_planning_algorithms::VehicleShape & collision_vehicle_shape,
  const std::string & output_path = "/workspace/data/planning_result.pdf")
{
  (void)collision_vehicle_shape;
  (void)output_path;

  const int width = static_cast<int>(occupancy_grid.info.width);
  const int height = static_cast<int>(occupancy_grid.info.height);
  const double resolution = occupancy_grid.info.resolution;

  const double origin_x = occupancy_grid.info.origin.position.x;
  const double origin_y = occupancy_grid.info.origin.position.y;

  std::vector<std::vector<double>> grid_2d(height, std::vector<double>(width));

  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      const int index = row * width + col;
      const int8_t value = occupancy_grid.data.at(index);

      if (value < 0) {
        grid_2d[row][col] = 50.0;  // unknown stays gray
      } else {
        grid_2d[row][col] = 100.0 - static_cast<double>(value);
      }
    }
  }

  const double min_x = origin_x;
  const double max_x = origin_x + static_cast<double>(width) * resolution;
  const double min_y = origin_y;
  const double max_y = origin_y + static_cast<double>(height) * resolution;

  matplot::image(min_x, max_x, min_y, max_y, grid_2d, true);
  matplot::gca()->y_axis().reverse(false);

  matplot::colormap(matplot::palette::gray());
  matplot::hold(matplot::on);

  std::vector<double> path_x;
  std::vector<double> path_y;

  // for (const auto & pose : intermediate_poses) {
  //   path_x.push_back(pose.position.x);
  //   path_y.push_back(pose.position.y);
  // }

  // if (!path_x.empty()) {
  //   matplot::plot(path_x, path_y, "xr");
  // }

  // TODO Plot here the vehicles footprints

  matplot::plot(
    std::vector<double>{start_pose.position.x},
    std::vector<double>{start_pose.position.y},
    "go");

  matplot::plot(
    std::vector<double>{goal_pose.position.x},
    std::vector<double>{goal_pose.position.y},
    "bo");

  double plot_min_x = min_x;
  double plot_max_x = max_x;
  double plot_min_y = min_y;
  double plot_max_y = max_y;

  auto include_point = [&](double x, double y) {
    plot_min_x = std::min(plot_min_x, x);
    plot_max_x = std::max(plot_max_x, x);
    plot_min_y = std::min(plot_min_y, y);
    plot_max_y = std::max(plot_max_y, y);
  };

  include_point(start_pose.position.x, start_pose.position.y);
  include_point(goal_pose.position.x, goal_pose.position.y);

  for (const auto & pose : intermediate_poses) {
    include_point(pose.position.x, pose.position.y);
  }

  const double margin = 1.0;
  const double x_center = (plot_min_x + plot_max_x) / 2.0;
  const double y_center = (plot_min_y + plot_max_y) / 2.0;
  const double half_range =
    std::max(plot_max_x - plot_min_x, plot_max_y - plot_min_y) / 2.0 + margin;

  matplot::xlim({x_center - half_range, x_center + half_range});
  matplot::ylim({y_center - half_range, y_center + half_range});

  matplot::xlabel("x [m]");
  matplot::ylabel("y [m]");
  matplot::title("Freespace planning result");

  matplot::gcf()->size(1000, 1000);

  matplot::axis(matplot::equal);

  // matplot::save(output_path);
  matplot::show();
}

