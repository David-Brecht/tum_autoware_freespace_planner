#pragma once

#include <iostream>
#include <yaml-cpp/yaml.h>

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"
#include <autoware/freespace_planning_algorithms/astar_search.hpp>

using namespace autoware::freespace_planning_algorithms;

struct PlannerParams {
  PlannerCommonParam planner_common_params;
  AstarParam a_star_params;
  VehicleShape vehicle_shape;
};

template <typename T>
std::vector<T> read_as_vector(const YAML::Node & node) {
  if (node.IsSequence()) {
    return node.as<std::vector<T>>();
  } 
  return {node.as<T>()};
}

std::vector<PlannerParams> read_params_from_file(const std::string & path) {

  // PlannerParams planner_params{};

  YAML::Node config = YAML::LoadFile(path);

  const auto time_limits   = read_as_vector<int>(config["PlannerCommonParam.time_limit"]);
  const auto theta_sizes   = read_as_vector<int>(config["PlannerCommonParam.theta_size"]);
  const auto curve_weights = read_as_vector<int>(config["PlannerCommonParam.curve_weight"]);
  
  int port = config["port"].as<int>();

  // TODO here we need a logic that goes through the file line by 
  // lines and if it starts with the respective key, appends the 
  // params


  return planner_params;
}