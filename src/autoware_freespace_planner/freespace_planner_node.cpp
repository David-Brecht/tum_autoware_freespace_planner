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
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include "autoware/freespace_planner/freespace_planner_node.hpp"

#include "autoware/freespace_planner/utils.hpp"
#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace autoware::freespace_planner
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

FreespacePlannerNode::FreespacePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("freespace_planner", node_options)
{
  using std::placeholders::_1;

  // NodeParam
  {
    auto & p = node_param_;
    p.planning_algorithm = declare_parameter<std::string>("planning_algorithm");
    p.waypoints_velocity = declare_parameter<double>("waypoints_velocity");
    p.update_rate = declare_parameter<double>("update_rate");
    p.th_arrived_distance_m = declare_parameter<double>("th_arrived_distance_m");
    p.th_stopped_time_sec = declare_parameter<double>("th_stopped_time_sec");
    p.th_stopped_velocity_mps = declare_parameter<double>("th_stopped_velocity_mps");
    p.th_course_out_distance_m = declare_parameter<double>("th_course_out_distance_m");
    p.th_obstacle_time_sec = declare_parameter<double>("th_obstacle_time_sec");
    p.vehicle_shape_margin_m = declare_parameter<double>("vehicle_shape_margin_m");
    p.replan_when_obstacle_found = declare_parameter<bool>("replan_when_obstacle_found");
    p.replan_when_course_out = declare_parameter<bool>("replan_when_course_out");
    p.max_planning_velocity = declare_parameter<double>("max_planning_velocity");
    p.use_path_distance_cost = declare_parameter<bool>("use_path_distance_cost");
    p.path_distance_weight = declare_parameter<double>("path_distance_weight");
    p.path_distance_viz_cap = declare_parameter<double>("path_distance_viz_cap");
    p.sampling_start_m = declare_parameter<double>("sampling_start_m");
    p.sampling_step_m = declare_parameter<double>("sampling_step_m");
    p.sampling_end_m = declare_parameter<double>("sampling_end_m");
    p.stuck_replan_time_sec = declare_parameter<double>("stuck_replan_time_sec");
  }

  // set vehicle_info
  {
    const auto vehicle_info =
      autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
    vehicle_shape_.length = vehicle_info.vehicle_length_m;
    vehicle_shape_.width = vehicle_info.vehicle_width_m;
    vehicle_shape_.base_length = vehicle_info.wheel_base_m;
    vehicle_shape_.max_steering = vehicle_info.max_steer_angle_rad;
    vehicle_shape_.base2back = vehicle_info.rear_overhang_m;
  }

  // Declare planner/algo params once; read via get_parameter() on every (re)init.
  {
    declare_parameter<double>("time_limit");
    declare_parameter<int>("theta_size");
    declare_parameter<double>("angle_goal_range");
    declare_parameter<double>("curve_weight");
    declare_parameter<double>("reverse_weight");
    declare_parameter<double>("direction_change_weight");
    declare_parameter<double>("lateral_goal_range");
    declare_parameter<double>("longitudinal_goal_range");
    declare_parameter<double>("max_turning_ratio");
    declare_parameter<int>("turning_steps");
    declare_parameter<int>("obstacle_threshold");
    declare_parameter<std::string>("astar.search_method");
    declare_parameter<bool>("astar.only_behind_solutions");
    declare_parameter<bool>("astar.use_back");
    declare_parameter<bool>("astar.adapt_expansion_distance");
    declare_parameter<double>("astar.expansion_distance");
    declare_parameter<double>("astar.near_goal_distance");
    declare_parameter<double>("astar.distance_heuristic_weight");
    declare_parameter<double>("astar.smoothness_weight");
    declare_parameter<double>("astar.obstacle_distance_weight");
    declare_parameter<double>("astar.goal_lat_distance_weight");
    declare_parameter<bool>("rrtstar.enable_update");
    declare_parameter<bool>("rrtstar.use_informed_sampling");
    declare_parameter<double>("rrtstar.max_planning_time");
    declare_parameter<double>("rrtstar.neighbor_radius");
    declare_parameter<double>("rrtstar.margin");
  }

  // Planning
  initializePlanningAlgorithm();

  // Subscribers
  path_sub_ = create_subscription<Path>(
    "~/input/path", rclcpp::QoS{1},
    std::bind(&FreespacePlannerNode::onPath, this, _1));

  trajectory_select_sub_ = create_subscription<std_msgs::msg::Int8>(
    "/external/remote/freespace_planner/input/selected_trajectory_index",
    rclcpp::QoS{1},
    std::bind(&FreespacePlannerNode::onSelectTrajectory, this, _1));

  // Publishers
  {
    rclcpp::QoS qos{1};
    qos.transient_local();
    trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos);
    debug_pose_array_pub_ = create_publisher<PoseArray>("~/debug/pose_array", qos);
    debug_partial_pose_array_pub_ = create_publisher<PoseArray>("~/debug/partial_pose_array", qos);
    is_completed_pub_ = create_publisher<std_msgs::msg::Bool>("is_completed", qos);
    processing_time_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/processing_time_ms", 1);
    debug_obstacle_cost_pub_ =
      create_publisher<OccupancyGrid>("/external/remote/freespace_planner/debug/costmap", 1);
    candidate_trajectories_pub_ = create_publisher<CandidateTrajectories>(
      "/external/remote/freespace_planner/output/candidate_trajectories", 1);
  }

  // TF
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Timer
  {
    const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&FreespacePlannerNode::onTimer, this));
  }

  on_set_params_cb_ = add_on_set_parameters_callback(
    std::bind(&FreespacePlannerNode::onSetParameters, this, std::placeholders::_1));

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
}

// ---------------------------------------------------------------------------
// Algorithm parameter helpers
// ---------------------------------------------------------------------------

PlannerCommonParam FreespacePlannerNode::getPlannerCommonParam()
{
  PlannerCommonParam p;
  p.time_limit = get_parameter("time_limit").as_double();
  p.theta_size = static_cast<int>(get_parameter("theta_size").as_int());
  p.angle_goal_range = get_parameter("angle_goal_range").as_double();
  p.curve_weight = get_parameter("curve_weight").as_double();
  p.reverse_weight = get_parameter("reverse_weight").as_double();
  p.direction_change_weight = get_parameter("direction_change_weight").as_double();
  p.lateral_goal_range = get_parameter("lateral_goal_range").as_double();
  p.longitudinal_goal_range = get_parameter("longitudinal_goal_range").as_double();
  p.max_turning_ratio = get_parameter("max_turning_ratio").as_double();
  p.turning_steps = static_cast<int>(get_parameter("turning_steps").as_int());
  p.obstacle_threshold = static_cast<int>(get_parameter("obstacle_threshold").as_int());
  return p;
}

// ---------------------------------------------------------------------------
// Replan / collision checks
// ---------------------------------------------------------------------------

bool FreespacePlannerNode::isPlanRequired()
{
  // In SAMPLING_MODE with valid candidates: hold until the operator selects.
  // Without this guard, isPlanRequired() fires every tick (trajectory_ is empty),
  // causing constant A* re-runs that can clear candidates before the selection callback.
  {
    std::lock_guard<std::mutex> lock(candidates_mutex_);
    if (locked_goal_distance_m_ < 0.0 && !candidates_.empty()) {
      return false;
    }
  }

  if (trajectory_.points.empty()) {
    return true;
  }

  if (node_param_.replan_when_obstacle_found && checkCurrentTrajectoryCollision()) {
    RCLCPP_DEBUG(get_logger(), "Found obstacle on trajectory");
    return true;
  }

  if (node_param_.replan_when_course_out) {
    const bool is_course_out = utils::calc_distance_2d(trajectory_, current_pose_.pose) >
                               node_param_.th_course_out_distance_m;
    if (is_course_out) {
      RCLCPP_INFO(get_logger(), "Course out");
      return true;
    }
  }

  return false;
}

bool FreespacePlannerNode::checkCurrentTrajectoryCollision()
{
  if (partial_trajectory_.points.empty()) {
    return false;
  }

  algo_->setMap(*occupancy_grid_);

  const size_t nearest_index_partial = autoware::motion_utils::findNearestIndex(
    partial_trajectory_.points, current_pose_.pose.position);
  const size_t end_index_partial = partial_trajectory_.points.size() - 1;
  const auto forward_trajectory = utils::get_partial_trajectory(
    partial_trajectory_, nearest_index_partial, end_index_partial, get_clock());

  const bool is_obs_found =
    algo_->hasObstacleOnTrajectory(utils::trajectory_to_pose_array(forward_trajectory));

  if (!is_obs_found) {
    obs_found_time_ = {};
    return false;
  }

  if (!obs_found_time_) obs_found_time_ = get_clock()->now();

  return (get_clock()->now() - obs_found_time_.get()).seconds() > node_param_.th_obstacle_time_sec;
}

// ---------------------------------------------------------------------------
// Target index / segment tracking
// ---------------------------------------------------------------------------

void FreespacePlannerNode::updateTargetIndex()
{
  if (!utils::is_stopped(odom_buffer_, node_param_.th_stopped_velocity_mps)) {
    return;
  }

  const auto is_near_target = utils::is_near_target(
    trajectory_.points.at(target_index_).pose, current_pose_.pose,
    node_param_.th_arrived_distance_m);

  if (!is_near_target) return;

  const auto new_target_index =
    utils::get_next_target_index(trajectory_.points.size(), reversing_indices_, target_index_);

  if (new_target_index == target_index_) {
    is_completed_ = true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Freespace segment completed");
    std_msgs::msg::Bool is_completed_msg;
    is_completed_msg.data = true;
    is_completed_pub_->publish(is_completed_msg);
  } else {
    prev_target_index_ = target_index_;
    target_index_ = new_target_index;
  }
}

// ---------------------------------------------------------------------------
// Subscriptions
// ---------------------------------------------------------------------------

void FreespacePlannerNode::onPath(const Path::ConstSharedPtr msg)
{
  const auto is_same_path_geometry = [](
                                      const Path::ConstSharedPtr & lhs,
                                      const Path::ConstSharedPtr & rhs) {
    if (!lhs || !rhs) return false;
    if (lhs->header.frame_id != rhs->header.frame_id || lhs->points.size() != rhs->points.size()) {
      return false;
    }

    constexpr double position_tolerance = 0.05;
    constexpr double yaw_tolerance = 0.02;
    for (size_t i = 0; i < lhs->points.size(); ++i) {
      const auto & a = lhs->points[i].pose;
      const auto & b = rhs->points[i].pose;
      const double dx = a.position.x - b.position.x;
      const double dy = a.position.y - b.position.y;
      const double dz = a.position.z - b.position.z;
      if (std::hypot(std::hypot(dx, dy), dz) > position_tolerance) return false;
      if (std::abs(tf2::getYaw(a.orientation) - tf2::getYaw(b.orientation)) > yaw_tolerance) {
        return false;
      }
    }
    return true;
  };

  const bool path_changed = !is_same_path_geometry(path_, msg);
  path_ = msg;

  // The behavior planner can republish an unchanged path continuously. Only an
  // actual geometry change invalidates operator selection.
  bool was_locked = false;
  {
    std::lock_guard<std::mutex> lock(candidates_mutex_);
    was_locked = path_changed && locked_goal_distance_m_ >= 0.0;
    if (was_locked) {
      RCLCPP_INFO(get_logger(), "Path geometry changed while locked — releasing goal lock");
      locked_goal_distance_m_ = -1.0;
    }
  }

  if (was_locked) {
    stuck_start_time_ = boost::none;
    reset();
  } else if (path_changed) {
    std::lock_guard<std::mutex> lock(candidates_mutex_);
    candidates_.clear();
  }
}

void FreespacePlannerNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
  odom_buffer_.push_back(msg);

  while (true) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(odom_buffer_.front()->header.stamp);
    if (time_diff.seconds() < node_param_.th_stopped_time_sec) break;
    odom_buffer_.pop_front();
  }
}

void FreespacePlannerNode::onSelectTrajectory(const std_msgs::msg::Int8::SharedPtr msg)
{
  const int idx = static_cast<int>(msg->data);
  std::lock_guard<std::mutex> lock(candidates_mutex_);

  if (idx == -1) {
    locked_goal_distance_m_ = -1.0;
    stuck_start_time_ = boost::none;
    trajectory_ = Trajectory();
    partial_trajectory_ = Trajectory();
    RCLCPP_INFO(get_logger(), "Goal lock released — returning to sampling mode");
    return;
  }

  if (idx < 0 || static_cast<size_t>(idx) >= candidates_.size()) {
    RCLCPP_WARN(
      get_logger(), "Selected index %d out of range (have %zu candidates)", idx, candidates_.size());
    return;
  }

  const auto & entry = candidates_[idx];
  locked_goal_distance_m_ = entry.goal_distance_m;
  stuck_start_time_ = boost::none;

  // Activate immediately from the already-computed candidates so the trajectory
  // appears on the output trajectory topic without waiting for the next replan.
  trajectory_ = entry.trajectory;
  reversing_indices_ = utils::get_reversing_indices(trajectory_);
  prev_target_index_ = 0;
  target_index_ = utils::get_next_target_index(
    trajectory_.points.size(), reversing_indices_, prev_target_index_);
  is_completed_ = false;

  RCLCPP_INFO(
    get_logger(), "Goal locked at %.1f m (index %d, path %.1f m, %zu pts)",
    locked_goal_distance_m_, idx, entry.path_length_m, trajectory_.points.size());

  // Publish the full trajectory immediately so downstream receives it
  // without waiting for the next timer tick.
  partial_trajectory_ =
    utils::get_partial_trajectory(trajectory_, prev_target_index_, target_index_, get_clock());
  trajectory_pub_->publish(partial_trajectory_);
  RCLCPP_INFO(
    get_logger(), "Published trajectory immediately: %zu points",
    partial_trajectory_.points.size());
}

// ---------------------------------------------------------------------------
// Goal sampling
// ---------------------------------------------------------------------------

std::vector<std::pair<double, PoseStamped>> FreespacePlannerNode::sampleGoalsFromPath()
{
  std::vector<std::pair<double, PoseStamped>> goals;
  if (!path_ || path_->points.empty()) return goals;

  const auto & pts = path_->points;

  // Find nearest path point to current pose
  size_t nearest_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < pts.size(); ++i) {
    const double dx = pts[i].pose.position.x - current_pose_.pose.position.x;
    const double dy = pts[i].pose.position.y - current_pose_.pose.position.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < min_dist_sq) { min_dist_sq = d2; nearest_idx = i; }
  }

  // Read lock state
  double locked_dist;
  {
    std::lock_guard<std::mutex> lock(candidates_mutex_);
    locked_dist = locked_goal_distance_m_;
  }

  // Build list of target distances
  std::vector<double> target_distances;
  if (locked_dist >= 0.0) {
    target_distances.push_back(locked_dist);
  } else {
    for (double d = node_param_.sampling_start_m;
         d <= node_param_.sampling_end_m + 1e-6;
         d += node_param_.sampling_step_m) {
      target_distances.push_back(d);
    }
  }

  // Walk along path for each target distance
  for (const double target_dist : target_distances) {
    double accumulated = 0.0;
    size_t goal_idx = nearest_idx;
    bool reached_target = target_dist <= 0.0;
    for (size_t i = nearest_idx + 1; i < pts.size(); ++i) {
      const double dx = pts[i].pose.position.x - pts[i - 1].pose.position.x;
      const double dy = pts[i].pose.position.y - pts[i - 1].pose.position.y;
      accumulated += std::sqrt(dx * dx + dy * dy);
      goal_idx = i;
      if (accumulated >= target_dist) {
        reached_target = true;
        break;
      }
    }
    if (!reached_target) continue;

    PoseStamped goal;
    goal.header = path_->header;
    goal.pose = pts[goal_idx].pose;
    goals.emplace_back(target_dist, goal);
  }

  return goals;
}

// ---------------------------------------------------------------------------
// Data / readiness
// ---------------------------------------------------------------------------

void FreespacePlannerNode::updateData()
{
  occupancy_grid_ = occupancy_grid_sub_.take_data();

  auto msgs = odom_sub_.take_data();
  for (const auto & msg : msgs) onOdometry(msg);
}

bool FreespacePlannerNode::isDataReady()
{
  bool ready = true;
  if (!path_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for path data.");
    ready = false;
  }
  if (!occupancy_grid_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for occupancy grid.");
    ready = false;
  }
  if (!odom_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odometry.");
    ready = false;
  }
  return ready;
}

// ---------------------------------------------------------------------------
// Main timer
// ---------------------------------------------------------------------------

void FreespacePlannerNode::onTimer()
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  updateData();
  if (!isDataReady()) return;

  current_pose_.pose = odom_->pose.pose;
  current_pose_.header = odom_->header;
  if (current_pose_.header.frame_id.empty()) return;

  const auto & lin = odom_->twist.twist.linear;
  const double current_speed = std::hypot(lin.x, lin.y);

  // ── Stuck detection (LOCKED_MODE) ────────────────────────────────────────
  bool should_reset = false;
  {
    std::lock_guard<std::mutex> lock(candidates_mutex_);
    if (locked_goal_distance_m_ >= 0.0) {
      if (current_speed < node_param_.th_stopped_velocity_mps) {
        if (!stuck_start_time_) {
          stuck_start_time_ = get_clock()->now();
        } else if (
          (get_clock()->now() - *stuck_start_time_).seconds() >=
          node_param_.stuck_replan_time_sec) {
          RCLCPP_WARN(
            get_logger(),
            "Stuck for %.1f s at locked goal %.1f m — reverting to sampling mode",
            node_param_.stuck_replan_time_sec, locked_goal_distance_m_);
          locked_goal_distance_m_ = -1.0;
          stuck_start_time_ = boost::none;
          should_reset = true;
        }
      } else {
        stuck_start_time_ = boost::none;
      }
    }
  }
  if (should_reset) reset();

  // ── Segment completion → back to sampling ────────────────────────────────
  if (is_completed_) {
    {
      std::lock_guard<std::mutex> lock(candidates_mutex_);
      if (locked_goal_distance_m_ >= 0.0) {
        RCLCPP_INFO(get_logger(), "Locked trajectory completed — returning to sampling mode");
        locked_goal_distance_m_ = -1.0;
        stuck_start_time_ = boost::none;
      }
    }
    reset();
  }

  // ── Replan ───────────────────────────────────────────────────────────────
  if (isPlanRequired()) {
    if (current_speed < node_param_.max_planning_velocity) {
      planCandidateTrajectories();
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping replan: speed %.2f m/s exceeds max_planning_velocity %.2f m/s",
        current_speed, node_param_.max_planning_velocity);
    }
  }

  // ── Trajectory output — always publish at 10 Hz ──────────────────────────
  // In LOCKED_MODE: follow the active trajectory.
  // In SAMPLING_MODE: publish a stop-at-current-position trajectory so downstream
  //   nodes see a valid 10 Hz stream while the operator is choosing.
  const bool is_locked = [&]() {
    std::lock_guard<std::mutex> lock(candidates_mutex_);
    return locked_goal_distance_m_ >= 0.0;
  }();

  if (is_locked && trajectory_.points.size() > 1) {
    updateTargetIndex();
    partial_trajectory_ =
      utils::get_partial_trajectory(trajectory_, prev_target_index_, target_index_, get_clock());
    debug_pose_array_pub_->publish(utils::trajectory_to_pose_array(trajectory_));
    debug_partial_pose_array_pub_->publish(utils::trajectory_to_pose_array(partial_trajectory_));
  } else {
    // SAMPLING_MODE or no trajectory yet: hold position.
    partial_trajectory_ = utils::create_stop_trajectory(current_pose_, get_clock());
  }

  trajectory_pub_->publish(partial_trajectory_);

  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  processing_time_pub_->publish(processing_time_msg);
}

// ---------------------------------------------------------------------------
// Multi-candidate planning
// ---------------------------------------------------------------------------

void FreespacePlannerNode::planCandidateTrajectories()
{
  if (!occupancy_grid_) return;

  // Shared map setup — done once for all candidates
  algo_->setMap(*occupancy_grid_);
  computePathDistanceMap();
  algo_->setPathDistanceCost(path_distance_map_, node_param_.path_distance_weight);
  publishDebugCostmap();

  const auto goals = sampleGoalsFromPath();
  if (goals.empty()) {
    bool was_locked = false;
    {
      std::lock_guard<std::mutex> lock(candidates_mutex_);
      was_locked = locked_goal_distance_m_ >= 0.0;
      if (was_locked) {
        locked_goal_distance_m_ = -1.0;
        stuck_start_time_ = boost::none;
      }
    }
    if (was_locked) {
      RCLCPP_WARN(get_logger(), "Locked goal is beyond the available path — reverting to sampling mode");
      reset();
    }
    return;
  }

  const auto tf =
    getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id);
  const auto start_local = utils::transform_pose(current_pose_.pose, tf);

  const auto & costmap_info = occupancy_grid_->info;
  const double costmap_w = costmap_info.width * costmap_info.resolution;
  const double costmap_h = costmap_info.height * costmap_info.resolution;

  std::vector<CandidateEntry> new_candidates;
  new_candidates.reserve(goals.size());

  for (const auto & [dist_m, goal] : goals) {
    const auto goal_local = utils::transform_pose(
      goal.pose, getTransform(occupancy_grid_->header.frame_id, goal.header.frame_id));

    const rclcpp::Time t0 = get_clock()->now();
    bool result = false;
    try {
      result = algo_->makePlan(start_local, goal_local);
    } catch (const std::exception & e) {
      // Gather diagnostic context
      const double start_obs_dist = algo_->getDistanceToObstacle(current_pose_.pose);
      const double goal_obs_dist = algo_->getDistanceToObstacle(goal.pose);
      const double yaw_start = tf2::getYaw(current_pose_.pose.orientation) * 180.0 / M_PI;
      const double yaw_goal = tf2::getYaw(goal.pose.orientation) * 180.0 / M_PI;
      const double gx_local = goal_local.position.x - costmap_info.origin.position.x;
      const double gy_local = goal_local.position.y - costmap_info.origin.position.y;
      const bool out_of_map =
        gx_local < 0.0 || gy_local < 0.0 || gx_local > costmap_w || gy_local > costmap_h;

      RCLCPP_WARN(
        get_logger(),
        "Goal %.1f m failed: %s\n"
        "  Start : (%.2f, %.2f) yaw=%.1f deg  obs_dist=%.2f m\n"
        "  Goal  : (%.2f, %.2f) yaw=%.1f deg  obs_dist=%.2f m%s\n"
        "  Costmap: %.1f x %.1f m  origin=(%.1f, %.1f)  goal_local=(%.2f, %.2f)",
        dist_m, e.what(),
        current_pose_.pose.position.x, current_pose_.pose.position.y, yaw_start, start_obs_dist,
        goal.pose.position.x, goal.pose.position.y, yaw_goal, goal_obs_dist,
        out_of_map ? "  <-- OUTSIDE COSTMAP" : "",
        costmap_w, costmap_h,
        costmap_info.origin.position.x, costmap_info.origin.position.y,
        gx_local, gy_local);
      continue;
    }

    const double elapsed = (get_clock()->now() - t0).seconds();

    if (!result) {
      RCLCPP_DEBUG(get_logger(), "No path to %.1f m goal (%.3f s)", dist_m, elapsed);
      continue;
    }

    Trajectory traj = utils::create_trajectory(
      current_pose_, algo_->getWaypoints(), node_param_.waypoints_velocity);

    // Compute arc length for operator display
    double path_len = 0.0;
    for (size_t i = 1; i < traj.points.size(); ++i) {
      const auto & a = traj.points[i - 1].pose.position;
      const auto & b = traj.points[i].pose.position;
      path_len += std::hypot(b.x - a.x, b.y - a.y);
    }

    RCLCPP_DEBUG(get_logger(), "Found path to %.1f m goal: %.1f m arc in %.3f s",
                 dist_m, path_len, elapsed);

    new_candidates.push_back({dist_m, path_len, std::move(traj)});
  }

  // Update shared candidate list
  bool no_candidates = false;
  {
    std::lock_guard<std::mutex> lock(candidates_mutex_);
    candidates_ = std::move(new_candidates);
    no_candidates = candidates_.empty();
  }

  if (no_candidates) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No candidate trajectories found");
    reset();
    return;
  }

  publishCandidates();

  // In LOCKED_MODE: activate the locked candidate
  double locked_dist;
  {
    std::lock_guard<std::mutex> lock(candidates_mutex_);
    locked_dist = locked_goal_distance_m_;
  }
  if (locked_dist >= 0.0) {
    if (!activateLockedTrajectory()) {
      RCLCPP_WARN(
        get_logger(),
        "Locked goal %.1f m is no longer reachable — reverting to sampling mode", locked_dist);
      {
        std::lock_guard<std::mutex> lock(candidates_mutex_);
        locked_goal_distance_m_ = -1.0;
        stuck_start_time_ = boost::none;
      }
      reset();
    }
  }
}

bool FreespacePlannerNode::activateLockedTrajectory()
{
  std::lock_guard<std::mutex> lock(candidates_mutex_);

  if (candidates_.empty() || locked_goal_distance_m_ < 0.0) return false;

  // Find the candidate closest to the locked distance
  const CandidateEntry * best = nullptr;
  double best_diff = std::numeric_limits<double>::max();
  for (const auto & c : candidates_) {
    const double diff = std::abs(c.goal_distance_m - locked_goal_distance_m_);
    if (diff < best_diff) { best_diff = diff; best = &c; }
  }

  if (!best || best_diff > node_param_.sampling_step_m * 0.5 + 1e-6) return false;

  trajectory_ = best->trajectory;
  reversing_indices_ = utils::get_reversing_indices(trajectory_);
  prev_target_index_ = 0;
  target_index_ =
    utils::get_next_target_index(trajectory_.points.size(), reversing_indices_, prev_target_index_);
  return true;
}

void FreespacePlannerNode::publishCandidates()
{
  CandidateTrajectories msg;

  std::lock_guard<std::mutex> lock(candidates_mutex_);
  for (const auto & entry : candidates_) {
    // Stable UUID per goal-distance slot (generated once, reused across replans)
    auto it = slot_uuids_.find(entry.goal_distance_m);
    if (it == slot_uuids_.end()) {
      slot_uuids_[entry.goal_distance_m] = autoware_utils_uuid::generate_uuid();
      it = slot_uuids_.find(entry.goal_distance_m);
    }
    const UUID & uid = it->second;

    // GeneratorInfo
    GeneratorInfo info;
    info.generator_id = uid;
    info.generator_name.data =
      "freespace_planner/goal_" + std::to_string(static_cast<int>(entry.goal_distance_m)) +
      "m_path_" + std::to_string(static_cast<int>(entry.path_length_m)) + "m";
    msg.generator_info.push_back(info);

    // CandidateTrajectory
    CandidateTrajectory ct;
    ct.header = entry.trajectory.header;
    ct.generator_id = uid;
    ct.points = entry.trajectory.points;
    msg.candidate_trajectories.push_back(ct);
  }

  candidate_trajectories_pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// Path distance cost layer (BFS raster)
// ---------------------------------------------------------------------------

void FreespacePlannerNode::computePathDistanceMap()
{
  if (!node_param_.use_path_distance_cost || !path_ || !algo_) {
    path_distance_map_.clear();
    return;
  }

  const auto & costmap = algo_->getCostmap();
  const auto & obstacle_table = algo_->getObstacleTable();
  const int W = static_cast<int>(costmap.info.width);
  const int H = static_cast<int>(costmap.info.height);
  const int N = W * H;
  const float res = costmap.info.resolution;
  const float ox = static_cast<float>(costmap.info.origin.position.x);
  const float oy = static_cast<float>(costmap.info.origin.position.y);

  path_distance_map_.assign(N, std::numeric_limits<float>::max());

  using Entry = std::pair<float, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> heap;

  const auto try_seed = [&](const float px, const float py) {
    const int cx = static_cast<int>((px - ox) / res);
    const int cy = static_cast<int>((py - oy) / res);
    if (cx < 0 || cx >= W || cy < 0 || cy >= H) return;
    const int id = cy * W + cx;
    if (obstacle_table[id] || path_distance_map_[id] == 0.0f) return;
    path_distance_map_[id] = 0.0f;
    heap.push({0.0f, id});
  };

  for (size_t i = 0; i + 1 < path_->points.size(); ++i) {
    const float x0 = static_cast<float>(path_->points[i].pose.position.x);
    const float y0 = static_cast<float>(path_->points[i].pose.position.y);
    const float x1 = static_cast<float>(path_->points[i + 1].pose.position.x);
    const float y1 = static_cast<float>(path_->points[i + 1].pose.position.y);
    const float dx = x1 - x0, dy = y1 - y0;
    const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / res)));
    for (int s = 0; s <= steps; ++s) {
      const float t = static_cast<float>(s) / static_cast<float>(steps);
      try_seed(x0 + t * dx, y0 + t * dy);
    }
  }
  if (!path_->points.empty()) {
    try_seed(
      static_cast<float>(path_->points.back().pose.position.x),
      static_cast<float>(path_->points.back().pose.position.y));
  }

  constexpr std::array<int, 8> DX = {1, -1, 0, 0, 1, -1, 1, -1};
  constexpr std::array<int, 8> DY = {0, 0, 1, -1, 1, 1, -1, -1};
  const float diag = res * 1.41421356f;
  const std::array<float, 8> STEP = {res, res, res, res, diag, diag, diag, diag};

  while (!heap.empty()) {
    const auto [dist, id] = heap.top();
    heap.pop();
    if (dist > path_distance_map_[id]) continue;
    const int cx = id % W, cy = id / W;
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + DX[k], ny = cy + DY[k];
      if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
      const int nid = ny * W + nx;
      if (obstacle_table[nid]) continue;
      const float nd = dist + STEP[k];
      if (nd < path_distance_map_[nid]) {
        path_distance_map_[nid] = nd;
        heap.push({nd, nid});
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Debug costmap visualisation
// ---------------------------------------------------------------------------

void FreespacePlannerNode::publishDebugCostmap()
{
  if (!algo_ || debug_obstacle_cost_pub_->get_subscription_count() == 0) return;

  const auto & costmap = algo_->getCostmap();
  const auto & edt_map = algo_->getEDTMap();
  const auto & obstacle_table = algo_->getObstacleTable();

  const int n = static_cast<int>(costmap.info.width * costmap.info.height);
  if (static_cast<int>(edt_map.size()) != n || static_cast<int>(obstacle_table.size()) != n) {
    return;
  }

  const double obs_cap = vehicle_shape_.max_dimension + 1.0;
  const double path_cap = node_param_.path_distance_viz_cap;
  const bool has_path_layer =
    node_param_.use_path_distance_cost && static_cast<int>(path_distance_map_.size()) == n;

  OccupancyGrid debug_grid;
  debug_grid.header = costmap.header;
  debug_grid.info = costmap.info;
  debug_grid.data.resize(n);

  for (int i = 0; i < n; ++i) {
    if (obstacle_table[i]) {
      debug_grid.data[i] = 100;
      continue;
    }
    const double obs_f = std::max(0.0, 1.0 - edt_map[i].distance / obs_cap);
    double path_f = 0.0;
    if (has_path_layer) {
      const float pd = path_distance_map_[i];
      path_f = (pd < std::numeric_limits<float>::max())
               ? std::min(1.0, static_cast<double>(pd) / path_cap) : 1.0;
    }
    const double combined = std::min(1.0, obs_f + path_f);
    debug_grid.data[i] = static_cast<int8_t>(std::lround(combined * 99.0));
  }

  debug_obstacle_cost_pub_->publish(debug_grid);
}

// ---------------------------------------------------------------------------
// Parameter callbacks
// ---------------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult FreespacePlannerNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  static const std::vector<std::string> algo_param_names = {
    "time_limit", "theta_size", "angle_goal_range", "curve_weight", "reverse_weight",
    "direction_change_weight", "lateral_goal_range", "longitudinal_goal_range",
    "max_turning_ratio", "turning_steps", "obstacle_threshold",
    "astar.search_method", "astar.only_behind_solutions", "astar.use_back",
    "astar.adapt_expansion_distance", "astar.expansion_distance", "astar.near_goal_distance",
    "astar.distance_heuristic_weight", "astar.smoothness_weight",
    "astar.obstacle_distance_weight", "astar.goal_lat_distance_weight",
    "rrtstar.enable_update", "rrtstar.use_informed_sampling", "rrtstar.max_planning_time",
    "rrtstar.neighbor_radius", "rrtstar.margin"};

  bool algo_reinit_needed = false;

  for (const auto & p : parameters) {
    const auto & name = p.get_name();
    if (name == "waypoints_velocity") {
      node_param_.waypoints_velocity = p.as_double();
    } else if (name == "th_arrived_distance_m") {
      node_param_.th_arrived_distance_m = p.as_double();
    } else if (name == "th_stopped_time_sec") {
      node_param_.th_stopped_time_sec = p.as_double();
    } else if (name == "th_stopped_velocity_mps") {
      node_param_.th_stopped_velocity_mps = p.as_double();
    } else if (name == "th_course_out_distance_m") {
      node_param_.th_course_out_distance_m = p.as_double();
    } else if (name == "th_obstacle_time_sec") {
      node_param_.th_obstacle_time_sec = p.as_double();
    } else if (name == "replan_when_obstacle_found") {
      node_param_.replan_when_obstacle_found = p.as_bool();
    } else if (name == "replan_when_course_out") {
      node_param_.replan_when_course_out = p.as_bool();
    } else if (name == "max_planning_velocity") {
      node_param_.max_planning_velocity = p.as_double();
    } else if (name == "use_path_distance_cost") {
      node_param_.use_path_distance_cost = p.as_bool();
    } else if (name == "path_distance_weight") {
      node_param_.path_distance_weight = p.as_double();
    } else if (name == "path_distance_viz_cap") {
      node_param_.path_distance_viz_cap = p.as_double();
    } else if (name == "sampling_start_m") {
      node_param_.sampling_start_m = p.as_double();
    } else if (name == "sampling_step_m") {
      node_param_.sampling_step_m = p.as_double();
    } else if (name == "sampling_end_m") {
      node_param_.sampling_end_m = p.as_double();
    } else if (name == "stuck_replan_time_sec") {
      node_param_.stuck_replan_time_sec = p.as_double();
    } else if (
      std::find(algo_param_names.begin(), algo_param_names.end(), name) !=
      algo_param_names.end()) {
      algo_reinit_needed = true;
    }
    // planning_algorithm, update_rate, vehicle_shape_margin_m require a node restart.
  }

  if (algo_reinit_needed) {
    RCLCPP_INFO(get_logger(), "Planner algorithm params changed — reinitializing.");
    initializePlanningAlgorithm();
    reset();
  }

  return result;
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void FreespacePlannerNode::reset()
{
  trajectory_ = Trajectory();
  partial_trajectory_ = Trajectory();
  is_completed_ = false;
  std_msgs::msg::Bool is_completed_msg;
  is_completed_msg.data = false;
  is_completed_pub_->publish(is_completed_msg);
  obs_found_time_ = {};

  std::lock_guard<std::mutex> lock(candidates_mutex_);
  candidates_.clear();
}

// ---------------------------------------------------------------------------
// TF helper
// ---------------------------------------------------------------------------

TransformStamped FreespacePlannerNode::getTransform(
  const std::string & from, const std::string & to)
{
  TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(from, to, tf2::TimePointZero, tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}

// ---------------------------------------------------------------------------
// Algorithm initialisation
// ---------------------------------------------------------------------------

void FreespacePlannerNode::initializePlanningAlgorithm()
{
  autoware::freespace_planning_algorithms::VehicleShape extended_vehicle_shape = vehicle_shape_;
  const double margin = node_param_.vehicle_shape_margin_m;
  extended_vehicle_shape.length += margin;
  extended_vehicle_shape.width += margin;
  extended_vehicle_shape.base2back += margin / 2;
  extended_vehicle_shape.setMinMaxDimension();

  const auto planner_common_param = getPlannerCommonParam();
  const auto algo_name = node_param_.planning_algorithm;

  if (algo_name == "astar") {
    AstarParam astar_param;
    astar_param.search_method = get_parameter("astar.search_method").as_string();
    astar_param.only_behind_solutions = get_parameter("astar.only_behind_solutions").as_bool();
    astar_param.use_back = get_parameter("astar.use_back").as_bool();
    astar_param.adapt_expansion_distance = get_parameter("astar.adapt_expansion_distance").as_bool();
    astar_param.expansion_distance = get_parameter("astar.expansion_distance").as_double();
    astar_param.near_goal_distance = get_parameter("astar.near_goal_distance").as_double();
    astar_param.distance_heuristic_weight = get_parameter("astar.distance_heuristic_weight").as_double();
    astar_param.smoothness_weight = get_parameter("astar.smoothness_weight").as_double();
    astar_param.obstacle_distance_weight = get_parameter("astar.obstacle_distance_weight").as_double();
    astar_param.goal_lat_distance_weight = get_parameter("astar.goal_lat_distance_weight").as_double();
    algo_ = std::make_unique<AstarSearch>(
      planner_common_param, extended_vehicle_shape, astar_param, get_clock());
  } else if (algo_name == "rrtstar") {
    RRTStarParam rrtstar_param;
    rrtstar_param.enable_update = get_parameter("rrtstar.enable_update").as_bool();
    rrtstar_param.use_informed_sampling = get_parameter("rrtstar.use_informed_sampling").as_bool();
    rrtstar_param.max_planning_time = get_parameter("rrtstar.max_planning_time").as_double();
    rrtstar_param.neighbor_radius = get_parameter("rrtstar.neighbor_radius").as_double();
    rrtstar_param.margin = get_parameter("rrtstar.margin").as_double();
    algo_ = std::make_unique<RRTStar>(
      planner_common_param, extended_vehicle_shape, rrtstar_param, get_clock());
  } else {
    throw std::runtime_error("No such algorithm named " + algo_name + " exists.");
  }
  RCLCPP_INFO_STREAM(get_logger(), "initialize planning algorithm: " << algo_name);
}

}  // namespace autoware::freespace_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::freespace_planner::FreespacePlannerNode)
