#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>
#include "autoware/freespace_planning_algorithms/astar_search.hpp"

#include "rosbag_loader.hpp"

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
  0.5,
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
  1.75, 
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

geometry_msgs::msg::Pose get_goal_pose(const autoware_planning_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & occupancy_grid) {
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
  }

  // Get the pose with the minimum distance to the grid
  geometry_msgs::msg::Pose pose;
  return pose;
}

int main(int argc, char ** argv){
  // std::cout << "number of argc: " << argc << std::endl;
  // for (int idx = 0; idx < argc; ++idx){ 
  //   std::cout << "string of argv: " << argv[idx] << std::endl;
  // }

  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  auto planner = std::make_unique<autoware::freespace_planning_algorithms::AstarSearch>(
    planner_common_params, 
    collision_vehicle_shape, 
    a_star_params, 
    ros_clock);

  BagData bag_data = load_data_from_mcap("/workspace/snapshot_20260601_164941/snapshot_20260601_164941_0.mcap");
  
  planner->setMap(bag_data.occupancy_grid);

  geometry_msgs::msg::Pose start_pose = bag_data.odometry.pose.pose;
  
  // TODO - find out the goal pose through interception of path and gridmap
  geometry_msgs::msg::Pose goal_pose = get_goal_pose(bag_data.path, bag_data.occupancy_grid);
  

  planner->makePlan(start_pose, goal_pose);
  auto waypoints = planner->getWaypoints();

  int idx = 0;

  for (const auto & waypoint : waypoints.waypoints) {
    std::cout << "waypoint no " << idx << ": x: " << waypoint.pose.position.x << " y: " << waypoint.pose.position.y << std::endl;
    ++idx;
  } 

  return 0;
}