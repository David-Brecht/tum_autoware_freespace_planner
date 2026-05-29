// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTOWARE__FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_
#define AUTOWARE__FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_

#include "autoware_utils/ros/logger_level_configure.hpp"

#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

class TestFreespacePlanner;

namespace autoware::freespace_planner
{
using autoware::freespace_planning_algorithms::AbstractPlanningAlgorithm;
using autoware::freespace_planning_algorithms::AstarParam;
using autoware::freespace_planning_algorithms::AstarSearch;
using autoware::freespace_planning_algorithms::PlannerCommonParam;
using autoware::freespace_planning_algorithms::RRTStar;
using autoware::freespace_planning_algorithms::RRTStarParam;
using autoware::freespace_planning_algorithms::VehicleShape;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_internal_planning_msgs::msg::GeneratorInfo;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using unique_identifier_msgs::msg::UUID;

struct NodeParam
{
  std::string planning_algorithm;
  double waypoints_velocity;  // constant velocity on planned waypoints [km/h]
  double update_rate;         // replanning and publishing rate [Hz]
  double th_arrived_distance_m;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
  double th_course_out_distance_m;  // collision margin [m]
  double th_obstacle_time_sec;
  double vehicle_shape_margin_m;
  bool replan_when_obstacle_found;
  bool replan_when_course_out;
  double max_planning_velocity;      // plan only when ego speed is below this threshold [m/s]
  bool use_path_distance_cost;       // enable reference-path attraction cost layer
  double path_distance_weight;       // weight for path distance cost (linear: weight * dist_m)
  double path_distance_viz_cap;      // distance [m] that maps to full cost in the debug costmap
  double sampling_start_m;           // nearest sampled goal distance [m]
  double sampling_step_m;            // step between goal samples [m]
  double sampling_end_m;             // farthest sampled goal distance [m]
  double stuck_replan_time_sec;      // seconds stopped before reverting LOCKED→SAMPLING
};

// One planning result: the goal distance used and the resulting trajectory.
struct CandidateEntry
{
  double goal_distance_m;
  double path_length_m;   // arc length for display
  Trajectory trajectory;
};

class FreespacePlannerNode : public rclcpp::Node
{
public:
  explicit FreespacePlannerNode(const rclcpp::NodeOptions & node_options);

private:
  // ── Publishers ──────────────────────────────────────────────────────────
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_pose_array_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_partial_pose_array_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_completed_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    processing_time_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr debug_obstacle_cost_pub_;
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr candidate_trajectories_pub_;

  // ── Subscribers ─────────────────────────────────────────────────────────
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr trajectory_select_sub_;

  autoware_utils::InterProcessPollingSubscriber<OccupancyGrid> occupancy_grid_sub_{
    this, "~/input/occupancy_grid"};
  autoware_utils::InterProcessPollingSubscriber<Odometry, autoware_utils::polling_policy::All>
    odom_sub_{this, "~/input/odometry", rclcpp::QoS{100}};

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── Params ──────────────────────────────────────────────────────────────
  NodeParam node_param_;
  VehicleShape vehicle_shape_;

  // ── Planning state ──────────────────────────────────────────────────────
  std::unique_ptr<AbstractPlanningAlgorithm> algo_;
  PoseStamped current_pose_;

  Trajectory trajectory_;
  Trajectory partial_trajectory_;
  std::vector<size_t> reversing_indices_;
  size_t prev_target_index_;
  size_t target_index_;
  bool is_completed_ = false;
  boost::optional<rclcpp::Time> obs_found_time_;

  // ── Multi-candidate state ────────────────────────────────────────────────
  // Protects candidates_ and locked_goal_distance_m_ against concurrent
  // access from the selection callback and the timer thread.
  mutable std::mutex candidates_mutex_;
  std::vector<CandidateEntry> candidates_;
  double locked_goal_distance_m_ = -1.0;   // -1 = SAMPLING_MODE
  boost::optional<rclcpp::Time> stuck_start_time_;
  // Stable UUIDs per goal-distance slot so downstream consumers can track by ID across replans.
  std::map<double, UUID> slot_uuids_;

  // ── Incoming data ────────────────────────────────────────────────────────
  Path::ConstSharedPtr path_;
  OccupancyGrid::ConstSharedPtr occupancy_grid_;
  Odometry::ConstSharedPtr odom_;

  std::deque<Odometry::ConstSharedPtr> odom_buffer_;

  // ── Path distance cost layer ─────────────────────────────────────────────
  std::vector<float> path_distance_map_;  // free-space distance raster from reference path (m)

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_cb_;

  // ── Internal helpers ─────────────────────────────────────────────────────
  PlannerCommonParam getPlannerCommonParam();

  void onPath(const Path::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onSelectTrajectory(const std_msgs::msg::Int8::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  void onTimer();
  void updateData();
  void reset();
  void initializePlanningAlgorithm();
  bool isDataReady();

  // Returns sampled goal poses: (distance_m, pose) sorted ascending by distance.
  // In LOCKED_MODE returns only the single entry for locked_goal_distance_m_.
  std::vector<std::pair<double, PoseStamped>> sampleGoalsFromPath();

  // Runs A* for each sampled goal, populates candidates_, publishes results.
  void planCandidateTrajectories();

  // Builds and publishes the CandidateTrajectories message from candidates_.
  void publishCandidates();

  // Populates trajectory_ from the candidate matching locked_goal_distance_m_.
  // Returns false if no matching candidate is found → reverts to SAMPLING_MODE.
  bool activateLockedTrajectory();

  /**
   * @brief Checks if a new trajectory planning is required.
   */
  bool isPlanRequired();

  /**
   * @brief Sets the target index along the current trajectory points.
   */
  void updateTargetIndex();

  /**
   * @brief Checks if current trajectory is colliding with an object.
   */
  bool checkCurrentTrajectoryCollision();

  TransformStamped getTransform(const std::string & from, const std::string & to);
  void computePathDistanceMap();
  void publishDebugCostmap();

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;

  friend class ::TestFreespacePlanner;
};
}  // namespace autoware::freespace_planner

#endif  // AUTOWARE__FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_
