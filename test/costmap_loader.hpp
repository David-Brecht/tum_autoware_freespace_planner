#pragma once

#include <cstdint>
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>


nav_msgs::msg::OccupancyGrid load_costmap_from_png(const std::string & png_path, double resolution, int obstacle_threshold=128) {     // if brightness smaller → Obstacle 0 

  cv::Mat img = cv::imread(png_path, cv::IMREAD_GRAYSCALE);
  cv::flip(img, img, 0);


  if ( img.data == NULL) {
    std::cerr << "No image could be loaded from the png path. Check the path and included png.\n";
    return nav_msgs::msg::OccupancyGrid();
  }
  
  nav_msgs::msg::OccupancyGrid map; 

  map.header.frame_id = "map";

  map.info.height = img.rows;
  map.info.width = img.cols;
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.y = 0.0;
  map.info.origin.orientation.z = 0.0;
  map.info.origin.orientation.w = 1.0;
  map.info.resolution = resolution;
  
  for (int row = 0; row < img.rows; ++row) {
    for (int col = 0; col < img.cols; ++ col) {
      map.data.push_back(img.at<uint8_t>(row, col) < obstacle_threshold ? 100 : 0);
    }
  }

  return map; 
}