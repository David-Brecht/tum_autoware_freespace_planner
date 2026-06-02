#include <algorithm>
#include <cmath>
#include <memory>
#include <chrono>
#include <iostream>
#include <string>
#include <filesystem>

#include <CLI/CLI.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>
#include "autoware/freespace_planning_algorithms/astar_search.hpp"

#include "rosbag_loader.hpp"
#include "rosbag_writer.hpp"
#include "yaml_param_loader.hpp"

// todo make a ROS parameter later
// But then again there might be some vehicle blocking 
float kDistanceFromBoundary = 7.0;

autoware::freespace_planning_algorithms::PlannerCommonParam planner_common_params {
  // base configs
  30000.0,
  // search configs
  120, 
  0.5,
  0.7,
  2.0,
  0.5,
  1.0,
  6.0,
  0.7,
  1,
  // costmap configs
  100
};

autoware::freespace_planning_algorithms::AstarParam a_star_params {
  // base configs
  "forward", // options: forward, backward
  false, // solutions should be behind the goal
  true, // backward search
  true, 
  0.5, 
  3.0, 
  // search configs
  2.0, // obstacle threshold on grid [0,255]
  0.5, 
  3.0, 
  5.0
};

autoware::freespace_planning_algorithms::VehicleShape collision_vehicle_shape {
  4.973, // length = front_overhang + wheel_base + rear_overhang = 4.89 m
  2.2252, // width = wheel_tread + left + right overhang = 1.896 m
  3.124, // base_length = wheel_base [m]
  0.70, // max_steering [rad]
  0.897 // base2back = rear_overhang [m]
};

struct PathPoint {
  geometry_msgs::msg::Pose pose;
  float cumulative_distance = 0.0f;
};

geometry_msgs::msg::Pose get_goal_pose(
  const autoware_planning_msgs::msg::Path & path,
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const geometry_msgs::msg::Pose & start_pose)
{
  if (path.points.empty()) {
    return start_pose;
  }

  std::vector<PathPoint> path_points;

  PathPoint first_point;
  first_point.pose = path.points.at(0).pose;
  first_point.cumulative_distance = 0.0f;
  path_points.push_back(first_point);

  float cumulative_distance = 0.0f;
  int path_points_n = path.points.size();

  for (int idx = 1; idx < path_points_n; ++idx) {
    float dx = path.points.at(idx).pose.position.x - path.points.at(idx-1).pose.position.x;
    float dy = path.points.at(idx).pose.position.y - path.points.at(idx-1).pose.position.y;
    float dz = path.points.at(idx).pose.position.z - path.points.at(idx-1).pose.position.z;

    cumulative_distance += std::hypot(dx, dy, dz);

    PathPoint point;
    point.pose = path.points.at(idx).pose;
    point.cumulative_distance = cumulative_distance;
    path_points.push_back(point);
  }

  const auto x_min = static_cast<float>(occupancy_grid.info.origin.position.x);
  const auto y_min = static_cast<float>(occupancy_grid.info.origin.position.y);
  const float x_max = x_min + static_cast<float>(occupancy_grid.info.width) * occupancy_grid.info.resolution;
  const float y_max = y_min + static_cast<float>(occupancy_grid.info.height) * occupancy_grid.info.resolution;

  auto is_outside_grid = [&](const PathPoint & pp) {
    return pp.pose.position.x < x_min || pp.pose.position.x >= x_max ||
           pp.pose.position.y < y_min || pp.pose.position.y >= y_max;
  };

  auto nearest_it = std::min_element(path_points.begin(), path_points.end(),
    [&](const PathPoint & a, const PathPoint & b) {
      auto dist_sq = [&](const PathPoint & pp) {
        auto dx = static_cast<float>(pp.pose.position.x - start_pose.position.x);
        auto dy = static_cast<float>(pp.pose.position.y - start_pose.position.y);
        return dx * dx + dy * dy;
      };
      return dist_sq(a) < dist_sq(b);
    });

  auto exit_it = std::find_if(nearest_it, path_points.end(), is_outside_grid);

  if (exit_it == path_points.end()) {
    return path_points.back().pose;
  }

  const float target_dist = exit_it->cumulative_distance - kDistanceFromBoundary;

  if (target_dist <= 0.0f) {
    return path_points.front().pose;
  }

  auto goal_it = std::lower_bound(path_points.begin(), exit_it, target_dist,
    [](const PathPoint & pp, float dist) { return pp.cumulative_distance < dist; });

  return goal_it->pose;
}

int main(int argc, char ** argv){
  CLI::App app{"Standalone A star remote planner"};
  std::string rosbag_input_path;
  std::string yaml_param_path;
  std::string rosbag_output_path;
  app.add_option(
    "-i,--input-path", 
    rosbag_input_path, 
    "Path to input rosbag describing the scene. Use tum_autoware_freespace_planner/scripts/ros2_snapshot_bag.py to obtain"
  )->required();
  app.add_option(
    "-p,--param-path",
    yaml_param_path,
    "Path to a yaml file that holds the parameters of the planner. See tum_autoware_freespace_planner/test/config/planner_config.yaml for reference"
  )->required();
  app.add_option(
    "-o,--output-path", 
    rosbag_output_path, 
    "Path to the final rosbag which can be visualized using tum_autoware_freespace_planner/scripts/debug_plot.py"
  );
  CLI11_PARSE(app, argc, argv);

  if (rosbag_output_path.empty()) {
    rosbag_output_path = (std::filesystem::path(rosbag_input_path) / "output").string();
  }

  const auto planner_param_configs = read_params_from_file(yaml_param_path);
  std::cout << "Loaded " << planner_param_configs.size() << " planner parameter configurations."
            << std::endl;

  BagData bag_data = load_data_from_mcap(rosbag_input_path);
  geometry_msgs::msg::Pose start_pose = bag_data.odometry.pose.pose;
  geometry_msgs::msg::Pose goal_pose = get_goal_pose(bag_data.path, bag_data.occupancy_grid, start_pose);

  for (int idx = 0; idx < static_cast<int>(planner_param_configs.size()); ++idx) {
    const auto & planner_param_config = planner_param_configs.at(idx);
    std::cout << "Running planner parameter configuration: " << planner_param_config.identifier
              << std::endl;

    auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    auto planner = std::make_unique<autoware::freespace_planning_algorithms::AstarSearch>(
      planner_param_config.planner_common_params, 
      planner_param_config.vehicle_shape, 
      planner_param_config.a_star_params, 
      ros_clock);
    
    planner->setMap(bag_data.occupancy_grid);
    
    // run and time the planner
    auto start = std::chrono::steady_clock::now();
    try {
      planner->makePlan(start_pose, goal_pose);
    } catch (const std::exception & e) {
      auto end = std::chrono::steady_clock::now();
      auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
      std::cerr << "Planner parameter configuration failed: " << planner_param_config.identifier
                << " after " << elapsed_ms.count() << " ms. Reason: " << e.what() << std::endl;
      continue;
    }

    auto end = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
    std::cout << "time to find a solution using planner: " << elapsed_ms.count() << " ms"
              << std::endl;

    auto waypoints = planner->getWaypoints();

    write_data_to_mcap(
      elapsed_ms.count(), 
      waypoints,
      start_pose,
      goal_pose,
      bag_data.occupancy_grid,
      collision_vehicle_shape,
      rosbag_output_path + "_" + planner_param_config.identifier
    );
  }

  return 0;
}
