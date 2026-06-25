cmake_minimum_required(VERSION 3.14)
project(tum_autoware_freespace_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(freespace_planner_node SHARED
  src/autoware_freespace_planner/freespace_planner_node.cpp
  src/autoware_freespace_planner/utils.cpp
)
rclcpp_components_register_node(freespace_planner_node
  PLUGIN "autoware::freespace_planner::FreespacePlannerNode"
  EXECUTABLE freespace_planner
)

## TODO delete this for deployment
install(PROGRAMS
  scripts/trajectory_selector_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

autoware_ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
