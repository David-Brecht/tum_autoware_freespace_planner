#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"
#include "autoware/freespace_planning_algorithms/astar_search.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/motion_utils/trajectory/interpolation.hpp"
#include "rosbag_loader.hpp"
#include "rosbag_writer.hpp"
#include "yaml_param_loader.hpp"

#include <CLI/CLI.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>
#include <rclcpp/rclcpp.hpp>
#include <autoware_planning_msgs/msg/detail/path_point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

// todo make a ROS parameter later
// But then again there might be some vehicle blocking
float kDistanceFromBoundary = 20.0;

autoware::freespace_planning_algorithms::PlannerCommonParam planner_common_params{
  // base configs
  30000.0,
  // search configs
  120, 0.5, 0.7, 2.0, 0.5, 1.0, 6.0, 0.7, 1,
  // costmap configs
  100};

autoware::freespace_planning_algorithms::AstarParam a_star_params{
  // base configs
  "forward",  // options: forward, backward
  false,      // solutions should be behind the goal
  true,       // backward search
  true, 0.5, 3.0,
  // search configs
  2.0,  // obstacle threshold on grid [0,255]
  0.5, 3.0, 5.0};

autoware::freespace_planning_algorithms::VehicleShape collision_vehicle_shape{
  4.973,   // length = front_overhang + wheel_base + rear_overhang = 4.89 m
  2.2252,  // width = wheel_tread + left + right overhang = 1.896 m
  3.124,   // base_length = wheel_base [m]
  0.70,    // max_steering [rad]
  0.897    // base2back = rear_overhang [m]
};

struct PathPoint
{
  geometry_msgs::msg::Pose pose;
  float cumulative_distance = 0.0f;
};

geometry_msgs::msg::Pose get_goal_pose(
  const autoware_planning_msgs::msg::Path & path,
  const nav_msgs::msg::OccupancyGrid & occupancy_grid, const geometry_msgs::msg::Pose & start_pose)
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
    float dx = path.points.at(idx).pose.position.x - path.points.at(idx - 1).pose.position.x;
    float dy = path.points.at(idx).pose.position.y - path.points.at(idx - 1).pose.position.y;
    float dz = path.points.at(idx).pose.position.z - path.points.at(idx - 1).pose.position.z;

    cumulative_distance += std::hypot(dx, dy, dz);

    PathPoint point;
    point.pose = path.points.at(idx).pose;
    point.cumulative_distance = cumulative_distance;
    path_points.push_back(point);
  }

  const auto x_min = static_cast<float>(occupancy_grid.info.origin.position.x);
  const auto y_min = static_cast<float>(occupancy_grid.info.origin.position.y);
  const float x_max =
    x_min + static_cast<float>(occupancy_grid.info.width) * occupancy_grid.info.resolution;
  const float y_max =
    y_min + static_cast<float>(occupancy_grid.info.height) * occupancy_grid.info.resolution;

  auto is_outside_grid = [&](const PathPoint & pp) {
    return pp.pose.position.x < x_min || pp.pose.position.x >= x_max ||
           pp.pose.position.y < y_min || pp.pose.position.y >= y_max;
  };

  auto nearest_it = std::min_element(
    path_points.begin(), path_points.end(), [&](const PathPoint & a, const PathPoint & b) {
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

  auto goal_it = std::lower_bound(
    path_points.begin(), exit_it, target_dist,
    [](const PathPoint & pp, float dist) { return pp.cumulative_distance < dist; });

  return goal_it->pose;
}

std::vector<geometry_msgs::msg::Pose> get_goal_poses(
  const autoware_planning_msgs::msg::Path & path,
  const geometry_msgs::msg::Pose & start_pose, 
  const std::vector<float> distance_along_path)
{
  std::vector<geometry_msgs::msg::Pose> goal_poses;
  const size_t pose_idx = autoware::motion_utils::findNearestIndex(path.points, start_pose.position);

  std::vector<autoware_planning_msgs::msg::PathPoint> points_from_start (path.points.begin()+pose_idx, path.points.end());

  for (const auto & distance : distance_along_path) {
    geometry_msgs::msg::Pose pose = autoware::motion_utils::calcInterpolatedPose(points_from_start, distance);
    goal_poses.push_back(pose);
  }

  return goal_poses;
}

std::vector<float> compute_path_distance_map(
  const autoware_planning_msgs::msg::Path & path,
  const nav_msgs::msg::OccupancyGrid & occupancy_grid, const std::vector<bool> & obstacle_table)
{
  using autoware::freespace_planning_algorithms::global2local;
  using autoware::freespace_planning_algorithms::pose2index;

  const int width = static_cast<int>(occupancy_grid.info.width);
  const int height = static_cast<int>(occupancy_grid.info.height);
  const int cells = width * height;
  const float resolution = occupancy_grid.info.resolution;

  if (width <= 0 || height <= 0 || resolution <= 0.0f || path.points.empty()) {
    return {};
  }
  if (static_cast<int>(obstacle_table.size()) != cells) {
    throw std::runtime_error("obstacle table size does not match occupancy grid size");
  }

  std::vector<float> distance_map(cells, std::numeric_limits<float>::max());

  using QueueEntry = std::pair<float, int>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;

  const auto try_seed = [&](const geometry_msgs::msg::Pose & pose_global) {
    const auto pose_local = global2local(occupancy_grid, pose_global);
    const auto index = pose2index(occupancy_grid, pose_local, 1);
    if (index.x < 0 || index.x >= width || index.y < 0 || index.y >= height) {
      return;
    }

    const int id = index.y * width + index.x;
    if (obstacle_table[id] || distance_map[id] == 0.0f) {
      return;
    }

    distance_map[id] = 0.0f;
    queue.push({0.0f, id});
  };

  for (size_t i = 0; i + 1 < path.points.size(); ++i) {
    const auto & p0 = path.points.at(i).pose.position;
    const auto & p1 = path.points.at(i + 1).pose.position;
    const float dx = static_cast<float>(p1.x - p0.x);
    const float dy = static_cast<float>(p1.y - p0.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / resolution)));

    for (int step = 0; step <= steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      auto seed_pose = path.points.at(i).pose;
      seed_pose.position.x = p0.x + t * (p1.x - p0.x);
      seed_pose.position.y = p0.y + t * (p1.y - p0.y);
      seed_pose.position.z = p0.z + t * (p1.z - p0.z);
      try_seed(seed_pose);
    }
  }
  try_seed(path.points.back().pose);

  constexpr std::array<int, 8> dx = {1, -1, 0, 0, 1, -1, 1, -1};
  constexpr std::array<int, 8> dy = {0, 0, 1, -1, 1, 1, -1, -1};
  const float diagonal_step = resolution * std::sqrt(2.0f);
  const std::array<float, 8> step_cost = {resolution,    resolution,    resolution,
                                          resolution,    diagonal_step, diagonal_step,
                                          diagonal_step, diagonal_step};

  while (!queue.empty()) {
    const auto [distance, id] = queue.top();
    queue.pop();
    if (distance > distance_map[id]) {
      continue;
    }

    const int cell_x = id % width;
    const int cell_y = id / width;
    for (int k = 0; k < 8; ++k) {
      const int next_x = cell_x + dx[k];
      const int next_y = cell_y + dy[k];
      if (next_x < 0 || next_x >= width || next_y < 0 || next_y >= height) {
        continue;
      }

      const int next_id = next_y * width + next_x;
      if (obstacle_table[next_id]) {
        continue;
      }

      const float next_distance = distance + step_cost[k];
      if (next_distance < distance_map[next_id]) {
        distance_map[next_id] = next_distance;
        queue.push({next_distance, next_id});
      }
    }
  }

  return distance_map;
}

int main(int argc, char ** argv)
{
  CLI::App app{"Standalone A star remote planner"};
  std::string rosbag_input_path;
  std::string yaml_param_path;
  std::string rosbag_output_path;
  app.add_option(
      "-i,--input-path", rosbag_input_path,
      "Path to input rosbag describing the scene. Use "
      "tum_autoware_freespace_planner/scripts/ros2_snapshot_bag.py to obtain")
    ->required();
  app.add_option(
      "-p,--param-path", yaml_param_path,
      "Path to a yaml file that holds the parameters of the planner. See "
      "tum_autoware_freespace_planner/test/config/planner_config.yaml for reference")
    ->required();
  app.add_option(
    "-o,--output-path", rosbag_output_path,
    "Path to the final rosbag which can be visualized using "
    "tum_autoware_freespace_planner/scripts/debug_plot.py");
  CLI11_PARSE(app, argc, argv);

  if (rosbag_output_path.empty()) {
    rosbag_output_path = (std::filesystem::path(rosbag_input_path) / "output").string();
  }

  const auto planner_param_configs = read_params_from_file(yaml_param_path);

  BagData bag_data = load_data_from_mcap(rosbag_input_path);
  geometry_msgs::msg::Pose start_pose = bag_data.odometry.pose.pose;
  
  for (int idx = 0; idx < static_cast<int>(planner_param_configs.size()); ++idx) {
    const auto & planner_param_config = planner_param_configs.at(idx);
    std::cout << "\n=======================\nRunning planner parameter configuration: " << planner_param_config.identifier
              << std::endl;

    auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    auto planner = std::make_unique<autoware::freespace_planning_algorithms::AstarSearch>(
      planner_param_config.planner_common_params, planner_param_config.vehicle_shape,
      planner_param_config.a_star_params, ros_clock);

    planner->setMap(bag_data.occupancy_grid);
    if (planner_param_config.use_path_distance_cost) {
      auto path_distance_map = compute_path_distance_map(
        bag_data.path, bag_data.occupancy_grid, planner->getObstacleTable());
      planner->setPathDistanceCost(
        std::move(path_distance_map), planner_param_config.path_distance_weight);
      std::cout << "Enabled path distance cost with weight "
                << planner_param_config.path_distance_weight << std::endl;
    } else {
      planner->setPathDistanceCost({}, 0.0);
    }

    // run and time the planner
    std::vector<float> distance_along_path{15.0, 20.0, 25.0, 30.0, 35.0};  // TODO make param
    std::vector<geometry_msgs::msg::Pose> goal_poses =
      get_goal_poses(bag_data.path, start_pose, distance_along_path);

    std::vector<std::pair<float, autoware::freespace_planning_algorithms::PlannerWaypoints>>
      trajectory_results;
    const rclcpp::Time t_start = ros_clock->now();
    for (size_t i = 0; i < goal_poses.size(); ++i) {
      try {
        if (planner->makePlan(start_pose, goal_poses[i])) {
          trajectory_results.emplace_back(distance_along_path[i], planner->getWaypoints());
        }
      } catch (const std::exception & e) {
        std::cerr << "No path for " << distance_along_path[i] << " m: " << e.what() << std::endl;
      }
    }
    const int64_t elapsed_ms = (ros_clock->now() - t_start).nanoseconds() / 1000000;
    std::cout << "Statistics:\n"; 
    std::cout << "  - Time to find solutions: " << elapsed_ms << " ms\n";
    std::cout << "  - Paths found: " << trajectory_results.size() << "/" << goal_poses.size() << "\n";
    std::cout << "=======================\n";
    
    if (trajectory_results.empty()) {
      std::cerr << "Planner parameter configuration failed: " << planner_param_config.identifier
                << std::endl;
      continue;
    }

    write_data_to_mcap(
      static_cast<float>(elapsed_ms), trajectory_results, start_pose, bag_data.occupancy_grid,
      planner_param_config.vehicle_shape, rosbag_output_path + "_" + planner_param_config.identifier);
  }

  return 0;
}
