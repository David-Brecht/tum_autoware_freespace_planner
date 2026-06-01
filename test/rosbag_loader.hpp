#pragma once

// Standard library
#include <string>

// Third-party / framework
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"

// Message types
#include "autoware_planning_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"


struct BagData {
  autoware_planning_msgs::msg::Path path;
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  nav_msgs::msg::Odometry odometry;
};

BagData load_data_from_mcap(const std::string & input_file) {
  rosbag2_cpp::Reader reader;

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = input_file;
  storage_options.storage_id = "mcap";

  reader.open(storage_options);

  rclcpp::Serialization<autoware_planning_msgs::msg::Path> path_ser;
  rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> grid_ser;
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_ser;

  BagData bag_data;
  
  while(reader.has_next()) {
    auto bag_msg = reader.read_next();

    if(bag_msg->topic_name == "/planning/scenario_planning/lane_driving/behavior_planning/path") {
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      autoware_planning_msgs::msg::Path msg;
      path_ser.deserialize_message(&serialized, &msg);
      bag_data.path = msg;
    } else if (bag_msg->topic_name == "/planning/scenario_planning/parking/costmap_generator/occupancy_grid") {
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      nav_msgs::msg::OccupancyGrid msg;
      grid_ser.deserialize_message(&serialized, &msg);
      bag_data.occupancy_grid = msg;
    } else if (bag_msg->topic_name == "/localization/kinematic_state") {
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      nav_msgs::msg::Odometry msg;
      odom_ser.deserialize_message(&serialized, &msg);
      bag_data.odometry = msg;
    }
  }
  
  return bag_data;
}