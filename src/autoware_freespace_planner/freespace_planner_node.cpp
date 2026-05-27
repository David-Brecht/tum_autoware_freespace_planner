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

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::freespace_planner
{
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
    p.path_lookup_distance = declare_parameter<double>("path_lookup_distance");
    p.max_planning_velocity = declare_parameter<double>("max_planning_velocity");
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

  // Planning
  initializePlanningAlgorithm();

  // Subscribers
  path_sub_ = create_subscription<Path>(
    "~/input/path", rclcpp::QoS{1},
    std::bind(&FreespacePlannerNode::onPath, this, _1));

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

PlannerCommonParam FreespacePlannerNode::getPlannerCommonParam()
{
  PlannerCommonParam p;

  // search configs
  p.time_limit = declare_parameter<double>("time_limit");

  p.theta_size = declare_parameter<int>("theta_size");
  p.angle_goal_range = declare_parameter<double>("angle_goal_range");
  p.curve_weight = declare_parameter<double>("curve_weight");
  p.reverse_weight = declare_parameter<double>("reverse_weight");
  p.direction_change_weight = declare_parameter<double>("direction_change_weight");
  p.lateral_goal_range = declare_parameter<double>("lateral_goal_range");
  p.longitudinal_goal_range = declare_parameter<double>("longitudinal_goal_range");
  p.max_turning_ratio = declare_parameter<double>("max_turning_ratio");
  p.turning_steps = declare_parameter<int>("turning_steps");

  // costmap configs
  p.obstacle_threshold = declare_parameter<int>("obstacle_threshold");

  return p;
}

bool FreespacePlannerNode::isPlanRequired()
{
  if (trajectory_.points.empty()) {
    return true;
  }

  if (node_param_.replan_when_obstacle_found && checkCurrentTrajectoryCollision()) {
    RCLCPP_DEBUG(get_logger(), "Found obstacle");
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
    // Reached the end of the current planned segment — reset so the next tick replans toward
    // the updated rolling goal.
    is_completed_ = true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Freespace segment completed, replanning toward updated goal");
    std_msgs::msg::Bool is_completed_msg;
    is_completed_msg.data = true;
    is_completed_pub_->publish(is_completed_msg);
  } else {
    // Switch to next partial trajectory
    prev_target_index_ = target_index_;
    target_index_ = new_target_index;
  }
}

void FreespacePlannerNode::onPath(const Path::ConstSharedPtr msg)
{
  path_ = msg;
  reset();
}

void FreespacePlannerNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  odom_ = msg;

  odom_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(odom_buffer_.front()->header.stamp);

    if (time_diff.seconds() < node_param_.th_stopped_time_sec) {
      break;
    }

    odom_buffer_.pop_front();
  }
}

PoseStamped FreespacePlannerNode::computeGoalFromPath()
{
  const auto & pts = path_->points;

  // Find nearest point on the path to the current vehicle pose
  size_t nearest_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < pts.size(); ++i) {
    const double dx = pts[i].pose.position.x - current_pose_.pose.position.x;
    const double dy = pts[i].pose.position.y - current_pose_.pose.position.y;
    const double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      nearest_idx = i;
    }
  }

  // Walk forward along the path until path_lookup_distance is accumulated
  double accumulated = 0.0;
  size_t goal_idx = nearest_idx;
  for (size_t i = nearest_idx + 1; i < pts.size(); ++i) {
    const double dx = pts[i].pose.position.x - pts[i - 1].pose.position.x;
    const double dy = pts[i].pose.position.y - pts[i - 1].pose.position.y;
    accumulated += std::sqrt(dx * dx + dy * dy);
    goal_idx = i;
    if (accumulated >= node_param_.path_lookup_distance) {
      break;
    }
  }

  PoseStamped goal;
  goal.header = path_->header;
  goal.pose = pts[goal_idx].pose;
  return goal;
}

void FreespacePlannerNode::updateData()
{
  occupancy_grid_ = occupancy_grid_sub_.take_data();

  {
    auto msgs = odom_sub_.take_data();
    for (const auto & msg : msgs) {
      onOdometry(msg);
    }
  }
}

bool FreespacePlannerNode::isDataReady()
{
  bool is_ready = true;

  if (!path_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for path data.");
    is_ready = false;
  }

  if (!occupancy_grid_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for occupancy grid.");
    is_ready = false;
  }

  if (!odom_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odometry.");
    is_ready = false;
  }

  return is_ready;
}

void FreespacePlannerNode::onTimer()
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  updateData();

  if (!isDataReady()) {
    return;
  }

  // Update current pose
  current_pose_.pose = odom_->pose.pose;
  current_pose_.header = odom_->header;

  if (current_pose_.header.frame_id.empty()) {
    return;
  }

  // Update rolling goal: the point path_lookup_distance ahead along the reference path
  goal_pose_ = computeGoalFromPath();

  // If the previous segment was completed, reset so we replan toward the updated goal
  if (is_completed_) {
    reset();
  }

  // Compute current speed from odometry
  const auto & lin = odom_->twist.twist.linear;
  const double current_speed = std::sqrt(lin.x * lin.x + lin.y * lin.y);
  const bool is_below_max_velocity = current_speed < node_param_.max_planning_velocity;

  // Replan when required and ego is slow enough for planning
  if (isPlanRequired()) {
    if (is_below_max_velocity) {
      planTrajectory();
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping replan: speed %.2f m/s exceeds max_planning_velocity %.2f m/s",
        current_speed, node_param_.max_planning_velocity);
    }
  }

  if (trajectory_.points.size() <= 1) {
    return;
  }

  // Update partial trajectory
  updateTargetIndex();
  partial_trajectory_ =
    utils::get_partial_trajectory(trajectory_, prev_target_index_, target_index_, get_clock());

  // Publish messages
  trajectory_pub_->publish(partial_trajectory_);
  debug_pose_array_pub_->publish(utils::trajectory_to_pose_array(trajectory_));
  debug_partial_pose_array_pub_->publish(utils::trajectory_to_pose_array(partial_trajectory_));

  // Publish ProcessingTime
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  processing_time_pub_->publish(processing_time_msg);
}

void FreespacePlannerNode::planTrajectory()
{
  if (occupancy_grid_ == nullptr) {
    return;
  }

  // Provide robot shape and map for the planner
  algo_->setMap(*occupancy_grid_);

  // Calculate poses in costmap frame
  const auto current_pose_in_costmap_frame = utils::transform_pose(
    current_pose_.pose,
    getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));

  const auto goal_pose_in_costmap_frame = utils::transform_pose(
    goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));

  // execute planning
  const rclcpp::Time start = get_clock()->now();
  std::string error_msg;
  bool result = false;
  try {
    result = algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
  } catch (const std::exception & e) {
    error_msg = e.what();
  }
  const rclcpp::Time end = get_clock()->now();

  RCLCPP_DEBUG(get_logger(), "Freespace planning: %f [s]", (end - start).seconds());

  if (result) {
    RCLCPP_DEBUG(get_logger(), "Found goal!");
    trajectory_ = utils::create_trajectory(
      current_pose_, algo_->getWaypoints(), node_param_.waypoints_velocity);
    reversing_indices_ = utils::get_reversing_indices(trajectory_);
    prev_target_index_ = 0;
    target_index_ = utils::get_next_target_index(
      trajectory_.points.size(), reversing_indices_, prev_target_index_);
  } else {
    RCLCPP_INFO(get_logger(), "Can't find goal: %s", error_msg.c_str());
    reset();
  }
}

rcl_interfaces::msg::SetParametersResult FreespacePlannerNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : parameters) {
    if (p.get_name() == "waypoints_velocity") {
      node_param_.waypoints_velocity = p.as_double();
    } else if (p.get_name() == "th_arrived_distance_m") {
      node_param_.th_arrived_distance_m = p.as_double();
    } else if (p.get_name() == "th_stopped_time_sec") {
      node_param_.th_stopped_time_sec = p.as_double();
    } else if (p.get_name() == "th_stopped_velocity_mps") {
      node_param_.th_stopped_velocity_mps = p.as_double();
    } else if (p.get_name() == "th_course_out_distance_m") {
      node_param_.th_course_out_distance_m = p.as_double();
    } else if (p.get_name() == "th_obstacle_time_sec") {
      node_param_.th_obstacle_time_sec = p.as_double();
    } else if (p.get_name() == "replan_when_obstacle_found") {
      node_param_.replan_when_obstacle_found = p.as_bool();
    } else if (p.get_name() == "replan_when_course_out") {
      node_param_.replan_when_course_out = p.as_bool();
    } else if (p.get_name() == "path_lookup_distance") {
      node_param_.path_lookup_distance = p.as_double();
    } else if (p.get_name() == "max_planning_velocity") {
      node_param_.max_planning_velocity = p.as_double();
    }
    // planning_algorithm, update_rate, vehicle_shape_margin_m, and all
    // planner algorithm params require a node restart to take effect.
  }

  return result;
}

void FreespacePlannerNode::reset()
{
  trajectory_ = Trajectory();
  partial_trajectory_ = Trajectory();
  is_completed_ = false;
  std_msgs::msg::Bool is_completed_msg;
  is_completed_msg.data = false;
  is_completed_pub_->publish(is_completed_msg);
  obs_found_time_ = {};
}

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

void FreespacePlannerNode::initializePlanningAlgorithm()
{
  // Extend robot shape
  autoware::freespace_planning_algorithms::VehicleShape extended_vehicle_shape = vehicle_shape_;
  const double margin = node_param_.vehicle_shape_margin_m;
  extended_vehicle_shape.length += margin;
  extended_vehicle_shape.width += margin;
  extended_vehicle_shape.base2back += margin / 2;
  extended_vehicle_shape.setMinMaxDimension();

  const auto planner_common_param = getPlannerCommonParam();

  const auto algo_name = node_param_.planning_algorithm;

  // initialize specified algorithm
  if (algo_name == "astar") {
    algo_ = std::make_unique<AstarSearch>(planner_common_param, extended_vehicle_shape, *this);
  } else if (algo_name == "rrtstar") {
    algo_ = std::make_unique<RRTStar>(planner_common_param, extended_vehicle_shape, *this);
  } else {
    throw std::runtime_error("No such algorithm named " + algo_name + " exists.");
  }
  RCLCPP_INFO_STREAM(get_logger(), "initialize planning algorithm: " << algo_name);
}
}  // namespace autoware::freespace_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::freespace_planner::FreespacePlannerNode)
