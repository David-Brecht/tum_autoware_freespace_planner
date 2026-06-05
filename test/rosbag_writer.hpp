
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"

#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <tf2_msgs/msg/detail/tf_message__struct.hpp>

using namespace autoware::freespace_planning_algorithms;

void write_data_to_mcap(
  const float elapsed_time,
  const std::vector<std::pair<float, PlannerWaypoints>> & trajectory_results,
  const geometry_msgs::msg::Pose & start_pose,
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const VehicleShape & vehicle_shape,
  const std::string & rosbag_output_path)
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = rosbag_output_path;
  storage_options.storage_id = "mcap";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::Writer writer;
  writer.open(storage_options, converter_options);

  auto time_stamp = rclcpp::Time(0);

  std_msgs::msg::Float64 elapsed_time_msg;
  elapsed_time_msg.data = elapsed_time;
  writer.write(elapsed_time_msg, "elapsed_time", time_stamp);

  std_msgs::msg::Float64 vehicle_length_msg;
  vehicle_length_msg.data = vehicle_shape.length;
  writer.write(vehicle_length_msg, "vehicle_length", time_stamp);
  // std::cout << "vehicle length: " << vehicle_shape.length << std::endl;

  std_msgs::msg::Float64 vehicle_width_msg;
  vehicle_width_msg.data = vehicle_shape.width;
  writer.write(vehicle_width_msg, "vehicle_width", time_stamp);
  // std::cout << "vehicle width: " << vehicle_shape.width << std::endl;

  std_msgs::msg::Float64 vehicle_base2back_msg;
  vehicle_base2back_msg.data = vehicle_shape.base2back;
  writer.write(vehicle_base2back_msg, "vehicle_base2back", time_stamp);
  // std::cout << "vehicle base2back: " << vehicle_shape.base2back << std::endl;

  tf2_msgs::msg::TFMessage tf_msg;
  geometry_msgs::msg::TransformStamped base_link_transform;
  base_link_transform.header.frame_id = "map";
  base_link_transform.child_frame_id = "base_link";
  base_link_transform.transform.translation.x = start_pose.position.x;
  base_link_transform.transform.translation.y = start_pose.position.y;
  base_link_transform.transform.translation.z = start_pose.position.z;
  base_link_transform.transform.rotation.x = start_pose.orientation.x;
  base_link_transform.transform.rotation.y = start_pose.orientation.y;
  base_link_transform.transform.rotation.z = start_pose.orientation.z;
  base_link_transform.transform.rotation.w = start_pose.orientation.w;
  tf_msg.transforms.push_back(base_link_transform);
  writer.write(tf_msg, "/tf", time_stamp);

  writer.write(start_pose, "start", time_stamp);
  writer.write(occupancy_grid, "costmap", time_stamp);

  for (const auto & [distance_m, waypoints] : trajectory_results) {
    const std::string dist_tag = std::to_string(static_cast<int>(std::round(distance_m))) + "_m";

    geometry_msgs::msg::PoseArray trajectory_msg;
    trajectory_msg.header.frame_id = "map";
    for (const auto & waypoint : waypoints.waypoints) {
      trajectory_msg.poses.push_back(waypoint.pose.pose);
    }
    writer.write(trajectory_msg, "trajectory_" + dist_tag, time_stamp);
  }
}