cmake_minimum_required(VERSION 3.14)
project(tum_autoware_freespace_planner)

find_package(autoware_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(CLI11 REQUIRED)
find_package(yaml-cpp REQUIRED)
autoware_package()

ament_auto_add_library(freespace_planner_node SHARED
  src/autoware_freespace_planner/freespace_planner_node.cpp
  src/autoware_freespace_planner/utils.cpp
)
rclcpp_components_register_node(freespace_planner_node
  PLUGIN "autoware::freespace_planner::FreespacePlannerNode"
  EXECUTABLE freespace_planner
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/rosbag_loader.hpp
    test/rosbag_writer.hpp
    test/test_astar_planner_remote.cpp
    # test/yaml_param_loader.hpp
    # test/test_freespace_planner_node_interface.cpp
    # test/test_freespace_planner_utils.cpp
    # test/test_freespace_planner.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    freespace_planner_node
    ${OpenCV_LIBS}
    rosbag2_cpp::rosbag2_cpp
    CLI11::CLI11
    yaml-cpp
  )

  target_include_directories(test_${PROJECT_NAME}
    PRIVATE ${OpenCV_INCLUDE_DIRS}
  )

endif()

autoware_ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
