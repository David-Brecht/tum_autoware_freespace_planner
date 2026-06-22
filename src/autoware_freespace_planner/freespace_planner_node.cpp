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
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::freespace_planner
{

namespace {

const char * toString(const FreespacePlannerNode::State state) {
  switch (state) {
    case FreespacePlannerNode::State::WAITING_FOR_INPUT:
      return "WAITING_FOR_INPUT";
    case FreespacePlannerNode::State::IDLE:
      return "IDLE";
    case FreespacePlannerNode::State::PLANNING:
      return "PLANNING";
    case FreespacePlannerNode::State::AWAITING_TRAJECTORY_SELECTION:
      return "AWAITING_TRAJECTORY_SELECTION";
    case FreespacePlannerNode::State::EXECUTING_TRAJECTORY:
      return "EXECUTING_TRAJECTORY";
    case FreespacePlannerNode::State::EXECUTION_FINISHED:
      return "EXECUTION_FINISHED";
  }
  return "UNKNOWN";
}

}

FreespacePlannerNode::FreespacePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("freespace_planner", node_options)
{
  using std::placeholders::_1;

  // NodeParam
  {
    auto & p = node_param_;
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
    p.goal_distances_along_path_m =
      declare_parameter<std::vector<double>>("goal_distances_along_path_m");
    p.extend_path_distance_m = declare_parameter<double>("extend_path_distance_m");
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
    "~/input/path", rclcpp::QoS{1}.durability_volatile(),
    std::bind(&FreespacePlannerNode::onPath, this, _1));

  trajectory_identifier_sub_ = create_subscription<std_msgs::msg::Int32>(
    "~/input/desired_trajectory_index", rclcpp::QoS{1}.durability_volatile(),
    [this](const std_msgs::msg::Int32 & msg) {
      if (msg.data < 0) {
        desired_trajectory_index_.reset();
        return;
      }
      desired_trajectory_index_ = static_cast<size_t>(msg.data);
    }
  );

  operation_mode_state_sub_ = create_subscription<OperationModeState>("/api/operation_mode/state", 
    rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local(), 
    [this] (const OperationModeState & msg) {
      operation_mode_ = msg.mode;
    }
  );  


  // Publishers
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos);
    path_pub_ = create_publisher<Path>("~/output/path", qos);
    candidate_trajectories_pub_ = create_publisher<CandidateTrajectories>("~/output/candidate_trajectories", qos);
    debug_goal_poses_pub_ = create_publisher<PoseArray>("~/debug/goal_poses", qos);
    current_state_pub_ = create_publisher<std_msgs::msg::String>("~/output/current_state", qos);
    processing_time_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/processing_time_ms", 1);
  }

  // Services
  replan_srv_ = create_service<Trigger>("~/trigger_replan", std::bind( 
    &FreespacePlannerNode::onTriggerReplan, this, std::placeholders::_1, std::placeholders::_2));

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

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
  state_msg_.data = "UNDEFINED";
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
  // if (planning_requested_) {
  //   RCLCPP_INFO(get_logger(), "User requested planning.");
  //   planning_requested_ = false;
  //   return true;
  // }

  // if (candidate_trajectories_.candidate_trajectories.empty()) {
  //   RCLCPP_INFO(get_logger(), "No candidate trajectories. Triggering planning.");
  //   return true;
  // }

  // if (node_param_.replan_when_obstacle_found && hasPersistentObstacle()) {
  //   RCLCPP_INFO(get_logger(), "Found obstacle on trajectory. Triggering planning.");
  //   return true;
  // }

  // if (node_param_.replan_when_course_out) {
  //   const bool is_course_out = utils::calc_distance_2d(trajectory_, current_pose_.pose) >
  //                              node_param_.th_course_out_distance_m;
  //   if (is_course_out) {
  //     RCLCPP_INFO(get_logger(), "Out of course. Triggering planning.");
  //     return true;
  //   }
  // }

  return false;
}

bool FreespacePlannerNode::hasPersistentObstacle(const std::vector<PoseArray> & pose_arrays)
{
  if (!occupancy_grid_) {
    RCLCPP_ERROR(this->get_logger(), "Function called without valid occupancy grid map.");
    return false;
  }

  algo_->setMap(*occupancy_grid_);
  
  const bool any_blocked = std::any_of(pose_arrays.begin(), pose_arrays.end(),
        [&](const PoseArray & pa) { return algo_->hasObstacleOnTrajectory(pa); });

  if (!any_blocked) { 
    obs_found_time_ = {}; 
    return false; 
  }
  
  if (!obs_found_time_) {
    obs_found_time_ = get_clock()->now();
  }
  
  return (get_clock()->now() - obs_found_time_.get()).seconds() > node_param_.th_obstacle_time_sec;
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

void FreespacePlannerNode::onPath(const Path::ConstSharedPtr msg)
{
  if (msg->points.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received an empty path. Ignoring path and not computing goal points.");
    return;
  }

  if (!odom_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Received path before odometry. Ignoring path and not computing goal points.");
    return;
  }

  const auto goal_poses =
    getGoalPoses(*msg, odom_->pose.pose, node_param_.goal_distances_along_path_m);
  if (goal_poses.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Could not compute goal poses from path. Ignoring path.");
    return;
  }

  path_ = msg;
  goal_poses_.header = msg->header;
  goal_poses_.poses = goal_poses;
}

void FreespacePlannerNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  if (msg->header.frame_id == "") {
    RCLCPP_WARN(this->get_logger(), "Odometry does not provide reference frame in header.");
  }

  odom_ = msg;

  current_pose_.pose = odom_->pose.pose;
  current_pose_.header = odom_->header; 

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

void FreespacePlannerNode::onTriggerReplan(
  const std::shared_ptr<Trigger::Request> request,
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;

  planning_requested_ = true;

  response->success = true;
  response->message = "Replanning requested.";
}

std::vector<Pose> FreespacePlannerNode::getGoalPoses(
  const Path & path, const Pose & start_pose,
  const std::vector<double> & distance_along_path)
{
  std::vector<Pose> goal_poses;
  if (path.points.empty()) {
    return goal_poses;
  }

  goal_poses.reserve(distance_along_path.size());
  const size_t pose_idx = autoware::motion_utils::findNearestIndex(path.points, start_pose.position);

  std::vector<PathPoint> points_from_start (path.points.begin()+pose_idx, path.points.end());
  if (points_from_start.empty()) {
    return goal_poses;
  }

  for (const auto & distance : distance_along_path) {
    Pose pose = autoware::motion_utils::calcInterpolatedPose(points_from_start, distance);
    goal_poses.push_back(pose);
  }

  return goal_poses;
}

// =================================================================================================
// === Methods related to handling states and their transitions ====================================
// =================================================================================================
void FreespacePlannerNode::handleWaitingForInput() 
{
  if (isDataReady()) {
    setState(State::IDLE);
  }
}

bool FreespacePlannerNode::isDataReady()
{
  bool is_ready = true;

  if (!path_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for path data.");
    is_ready = false;
  }

  if (goal_poses_.poses.empty()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for goal poses.");
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

void FreespacePlannerNode::handleIdle() 
{
  trajectory_ = utils::create_stop_trajectory(current_pose_, get_clock());
  path_out_ = utils::convert_to_path(trajectory_);

  if (!planning_requested_) {
    return;
  }

  planning_requested_ = false;

  if (operation_mode_ != OperationModeState::REMOTE) {
    RCLCPP_INFO(this->get_logger(), "Not yet in Remote Mode. Planning cannot be activated. Set autoware to Remote Mode and send planning request again.");
    return;
  }

  RCLCPP_INFO(get_logger(), "User requested planning.");
  setState(State::PLANNING);
}

void FreespacePlannerNode::handlePlanning() 
{
  // In the planning state, we stand still
  trajectory_ = utils::create_stop_trajectory(current_pose_, get_clock()); 
  path_out_ = utils::convert_to_path(trajectory_);

  const bool is_ego_stopped = utils::is_stopped(odom_buffer_, node_param_.th_stopped_velocity_mps);
  if (is_ego_stopped) {
    // TODO right now we are searching until a candidate will be found. But there might be 
    // scenarios where no trajectory candidate can be found → we are stuck. Solve this via waypoint 
    // guidance or similar
    bool found_trajectory = false;
    while (rclcpp::ok(get_node_base_interface()->get_context())) {
      RCLCPP_INFO(this->get_logger(), "===========================================");
      RCLCPP_INFO(this->get_logger(), "Planning candidate trajectories.");
      found_trajectory = planTrajectories();
      if (found_trajectory) {
        break;
      }
    }

    if (!found_trajectory) {
      return;
    }

    setState(State::AWAITING_TRAJECTORY_SELECTION);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 500,
      "Waiting for the vehicle to stop before generating a new trajectory.");
  }

  // TODO user may also do the automation handover here if the automation trajectory now suits their liking
}

void FreespacePlannerNode::handleAwaitingTrajectorySelection() 
{
  trajectory_ = utils::create_stop_trajectory(current_pose_, get_clock()); 
  path_out_ = utils::convert_to_path(trajectory_);
  
  // If a planning request is received here (e.g. the user likes none of the trajectories and the
  // environment has changed since last initiation), allow going back to the planning step
  if (planning_requested_) {
    RCLCPP_INFO(get_logger(), "User requested planning.");
    planning_requested_ = false;
    setState(State::PLANNING);
    return;
  }

  // TODO - we likely need here a logic that compares timestamps of desired trajectory message and 
  // trajectories to get a indication of where to set the goal point

  if (!desired_trajectory_index_) {
    return;
  }

  const auto desired_trajectory_index = desired_trajectory_index_.value();
  if (desired_trajectory_index >= candidate_trajectories_.candidate_trajectories.size()) {
    RCLCPP_INFO(this->get_logger(), "Desired trajectory index too large.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Populating trajectory with data from candidate trajectory.");
  trajectory_.header = candidate_trajectories_.candidate_trajectories.at(desired_trajectory_index).header;
  trajectory_.points = candidate_trajectories_.candidate_trajectories.at(desired_trajectory_index).points;
  desired_trajectory_index_.reset();

  RCLCPP_INFO(this->get_logger(), "converting trajectory to path");
  path_out_ = utils::convert_to_path(trajectory_);
  if (node_param_.extend_path_distance_m > 1e-6) {
    utils::append_reference_path(path_out_, path_, node_param_.extend_path_distance_m);
  }
  RCLCPP_INFO(this->get_logger(), "converted trajectory to path");
  
  candidate_trajectories_ = CandidateTrajectories();

  setState(State::EXECUTING_TRAJECTORY);
}

void FreespacePlannerNode::handleExecutingTrajectory() 
{
  // TODO Handle... 
  // Emergency brake
  // Replanning, including the goal point and also repeated selection (maybe only if below a certain velocity threshold)
  
  // Keep the headers stamp up to date
  trajectory_.header.stamp=get_clock()->now();
  path_out_.header.stamp=get_clock()->now();

  const bool is_ego_stopped = utils::is_stopped(odom_buffer_, node_param_.th_stopped_velocity_mps);
  // TODO make based on the goal point itself? Think about the case where we may get to automation handover as soon as we are stopping along the trajectory
  const bool is_near_goal = utils::is_near_target(
    trajectory_.points.back().pose, current_pose_.pose, node_param_.th_arrived_distance_m);

  if (is_ego_stopped && is_near_goal) {
    RCLCPP_INFO(this->get_logger(), "Vehicle near goal. Automation Handover may be triggered");
    setState(State::EXECUTION_FINISHED);
  }
}

void FreespacePlannerNode::handleExecutionFinished() 
{
  // Keep the vehicle stopped in this state
  trajectory_ = utils::create_stop_trajectory(current_pose_, get_clock()); 
  path_out_ = utils::convert_to_path(trajectory_);

  // User may trigger replanning if vehicles position is not as desired
  if (planning_requested_) {
    RCLCPP_INFO(get_logger(), "User requested planning.");
    planning_requested_ = false;
    setState(State::PLANNING);
  }  

  // If autoware mode is changed to auto (teleoperation is not needed anymore), jump back to idle
  // TODO - Give a final approval for handover?
  if(operation_mode_ == OperationModeState::AUTONOMOUS) {
    setState(State::IDLE);
  }
}

// =================================================================================================
// === Timer with state management =================================================================
// =================================================================================================
void FreespacePlannerNode::onTimer()
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  updateData();

  switch (state_) {
    case State::WAITING_FOR_INPUT:
      handleWaitingForInput();
      break;
    case State::IDLE:
      handleIdle();
      break;
    case State::PLANNING:
      handlePlanning();
      break;
    case State::AWAITING_TRAJECTORY_SELECTION:
      handleAwaitingTrajectorySelection();
      break;
    case State::EXECUTING_TRAJECTORY:
      handleExecutingTrajectory();
      break;
    case State::EXECUTION_FINISHED:
      handleExecutionFinished();
      break;
    default:
      RCLCPP_INFO(this->get_logger(), "No state identified.\n");
  }

  // TODO - publish the topics here or in the states themselve? 
  trajectory_pub_->publish(trajectory_);
  path_pub_->publish(path_out_);
  candidate_trajectories_pub_->publish(candidate_trajectories_);
  debug_goal_poses_pub_->publish(goal_poses_);
  current_state_pub_->publish(state_msg_);

  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  processing_time_pub_->publish(processing_time_msg);
}

bool FreespacePlannerNode::planTrajectories()
{
  if (!occupancy_grid_) {
    RCLCPP_ERROR(this->get_logger(), "Function called without valid occupancy grid map.");
    return false;
  }

  algo_->setMap(*occupancy_grid_);

  const auto current_pose_in_costmap_frame = utils::transform_pose(
    current_pose_.pose,
    getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));

  if (occupancy_grid_->header.frame_id != goal_poses_.header.frame_id) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Costmap frame (" << occupancy_grid_->header.frame_id 
      << " ) and goal poses frame (" << goal_poses_.header.frame_id << ") are different. returning.");
  }

  const rclcpp::Time start = get_clock()->now();
  std::string error_msg;
  bool any_result = false;
  candidate_trajectories_.candidate_trajectories.clear();

  for (size_t i = 0; i < goal_poses_.poses.size(); ++i) {
    const auto & goal_pose_in_costmap_frame = goal_poses_.poses.at(i);
    const rclcpp::Time goal_start = get_clock()->now();
    const double goal_distance = node_param_.goal_distances_along_path_m.at(i);
    try {
      algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);

      any_result = true;

      autoware_internal_planning_msgs::msg::CandidateTrajectory candidate_trajectory;
      candidate_trajectory.header.stamp = start; // simplification
      candidate_trajectory.header.frame_id = goal_poses_.header.frame_id;
      // candidate_trajectory.uuid = // TODO needed?
            
      const auto tmp_trajectory_ = utils::create_trajectory(current_pose_, algo_->getWaypoints(), node_param_.waypoints_velocity);

      candidate_trajectory.points = tmp_trajectory_.points;

      candidate_trajectories_.candidate_trajectories.push_back(candidate_trajectory);

      RCLCPP_INFO(
        get_logger(), "Goal Point %.2f m: Found goal in %f seconds", goal_distance,
        (get_clock()->now() - goal_start).seconds());
    } catch (const std::exception & e) {
      error_msg = e.what();
      RCLCPP_INFO(
        get_logger(), "Goal Point %.2f m: Unable to find goal. %s", goal_distance,
        error_msg.c_str());
    }
  } 
  const rclcpp::Time end = get_clock()->now();
  RCLCPP_INFO(get_logger(), "Freespace planner total time: %f seconds", (end - start).seconds());
  RCLCPP_INFO(get_logger(), "===========================================");

  if (!any_result) {
    RCLCPP_INFO(get_logger(), "Can't find goal: %s", error_msg.c_str());
    reset();
  }

  return any_result;
}

void FreespacePlannerNode::reset()
{
  trajectory_ = Trajectory();
  path_out_ = Path();
  candidate_trajectories_ = CandidateTrajectories();
  is_completed_ = false;
  std_msgs::msg::Bool is_completed_msg;
  is_completed_msg.data = is_completed_;
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

void FreespacePlannerNode::setState(const State & target_state) 
{
  if (target_state == state_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Target state is current state. Check state management!");
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "State transition: " << toString(state_) << " to " << toString(target_state));
  
  state_ = target_state;
  
  state_msg_.data = toString(state_);
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

  // initialize specified algorithm
  algo_ = std::make_unique<AstarSearch>(planner_common_param, extended_vehicle_shape, *this);

  RCLCPP_INFO_STREAM(get_logger(), "initialize astar planning algorithm");
}
}  // namespace autoware::freespace_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::freespace_planner::FreespacePlannerNode)
