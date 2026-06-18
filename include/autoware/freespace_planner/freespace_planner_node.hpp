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
#include <autoware/freespace_planning_algorithms/reeds_shepp.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/detail/operation_mode_state__struct.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/detail/pose_array__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/detail/trigger__struct.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <iostream>
#include <memory>
#include <string>
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
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::PathPoint;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using std_srvs::srv::Trigger;

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
};

class FreespacePlannerNode : public rclcpp::Node
{
public:
  explicit FreespacePlannerNode(const rclcpp::NodeOptions & node_options);
  enum class State {
    WAITING_FOR_INPUT,
    IDLE,
    PLANNING,
    AWAITING_TRAJECTORY_SELECTION,
    EXECUTING_TRAJECTORY,
    EXECUTION_FINISHED
  };

private:
  // ros
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_goal_poses_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_state_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr processing_time_pub_;
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr candidate_trajectories_pub_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trajectory_identifier_sub_;
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Subscription<OperationModeState>::SharedPtr operation_mode_state_sub_;
  autoware_utils::InterProcessPollingSubscriber<OccupancyGrid> occupancy_grid_sub_{
    this, "~/input/occupancy_grid"};
  autoware_utils::InterProcessPollingSubscriber<Odometry, autoware_utils::polling_policy::All>
    odom_sub_{this, "~/input/odometry", rclcpp::QoS{100}};
    
  rclcpp::Service<Trigger>::SharedPtr replan_srv_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // params
  NodeParam node_param_;
  VehicleShape vehicle_shape_;

  // variables
  std::unique_ptr<AbstractPlanningAlgorithm> algo_;
  PoseStamped current_pose_;
  PoseArray goal_poses_;
  CandidateTrajectories candidate_trajectories_; 
  std::vector<float> goal_distances_along_path_ {15.0, 20.0, 25.0, 30.0, 35.0}; // TODO read in as param

  Trajectory trajectory_;
  Path path_out_;
  bool is_completed_ = false;
  // bool reset_in_progress_ = false;
  bool planning_requested_ = false;
  bool obstacle_on_trajectory_;
  boost::optional<rclcpp::Time> obs_found_time_;

  Path::ConstSharedPtr path_;
  OccupancyGrid::ConstSharedPtr occupancy_grid_;
  Odometry::ConstSharedPtr odom_;
  std_msgs::msg::String state_msg_;
  std::optional<size_t> desired_trajectory_index_;
  uint8_t operation_mode_;

  std::deque<Odometry::ConstSharedPtr> odom_buffer_;

  // functions used in the constructor
  PlannerCommonParam getPlannerCommonParam();

  // functions, callback
  void onPath(const Path::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);

  void onTimer();
  void updateData();
  void reset();
  bool planTrajectories();
  void initializePlanningAlgorithm();
  bool isDataReady();

  /**
   * @brief Checks if a new trajectory planning is required.
   * @details A new trajectory planning is required if:
   *           - Current trajectory points are empty, or
   *           - Current trajectory collides with an object, or
   *           - Ego deviates from current trajectory
   * @return true if any of the conditions are met.
   */
  bool isPlanRequired();


  TransformStamped getTransform(const std::string & from, const std::string & to);

  static std::vector<Pose> getGoalPoses(
    const Path & path,
    const Pose & start_pose, 
    const std::vector<float> & distance_along_path);

  void onTriggerReplan(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;

  // Checks if there is a persistent obstacle on the target trajectory; characterized through a pose array
  bool hasPersistentObstacle(const std::vector<PoseArray> & pose_arrays);

  State state_{State::WAITING_FOR_INPUT};
  void setState(const State & target_state);
  void handleWaitingForInput();
  void handleIdle();
  void handlePlanning();
  void handleAwaitingTrajectorySelection();
  void handleExecutingTrajectory();
  void handleExecutionFinished();
  

  friend class ::TestFreespacePlanner;
};
}  // namespace autoware::freespace_planner

#endif  // AUTOWARE__FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_
