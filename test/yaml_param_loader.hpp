#pragma once

#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"
#include <autoware/freespace_planning_algorithms/astar_search.hpp>

struct PlannerParams {
  autoware::freespace_planning_algorithms::PlannerCommonParam planner_common_params;
  autoware::freespace_planning_algorithms::AstarParam a_star_params;
  autoware::freespace_planning_algorithms::VehicleShape vehicle_shape;
  std::string identifier;     // A identifier telling how this parameter set is set up
};

namespace yaml_param_loader_detail
{

template <typename T>
std::vector<T> read_as_vector(const YAML::Node & node)
{
  if (!node) {
    throw std::runtime_error("Missing YAML parameter");
  }

  if (!node.IsSequence()) {
    return {node.as<T>()};
  }

  return node.as<std::vector<T>>();
}

inline YAML::Node get_param_node(
  const YAML::Node & config, const std::string & group, const std::string & key)
{
  const auto flat_key = group + "." + key;
  if (config[flat_key]) {
    return config[flat_key];
  }

  const auto group_node = config[group];
  if (!group_node) {
    throw std::runtime_error("Missing YAML group: " + group);
  }

  if (group_node.IsMap() && group_node[key]) {
    return group_node[key];
  }

  if (group_node.IsSequence()) {
    for (const auto & entry : group_node) {
      if (entry[key]) {
        return entry[key];
      }
    }
  }

  throw std::runtime_error("Missing YAML parameter: " + flat_key);
}

template <typename T>
std::string value_to_identifier(const T & value)
{
  std::ostringstream oss;
  if constexpr (std::is_same_v<T, bool>) {
    oss << std::boolalpha << value;
  } else {
    oss << value;
  }

  auto text = oss.str();
  for (auto & c : text) {
    if (c == '/' || c == '\\' || c == ' ' || c == ':') {
      c = '_';
    }
  }
  return text;
}

struct SweepVariant
{
  std::function<void(PlannerParams &)> apply;
  std::string identifier_part;
};

template <typename T, typename Setter>
std::vector<SweepVariant> make_sweep_field(
  const YAML::Node & config, const std::string & group, const std::string & key, Setter setter)
{
  const auto values = read_as_vector<T>(get_param_node(config, group, key));
  if (values.empty()) {
    throw std::runtime_error("YAML parameter has no values: " + group + "." + key);
  }

  const bool include_in_identifier = values.size() > 1;

  std::vector<SweepVariant> variants;
  variants.reserve(values.size());

  for (const auto & value : values) {
    SweepVariant variant;
    variant.apply = [value, setter](PlannerParams & params) { setter(params, value); };
    if (include_in_identifier) {
      variant.identifier_part = key + "_" + value_to_identifier(value);
    }
    variants.push_back(std::move(variant));
  }

  return variants;
}

inline void append_identifier_part(std::string & identifier, const std::string & part)
{
  if (part.empty()) {
    return;
  }

  if (!identifier.empty()) {
    identifier += "_";
  }
  identifier += part;
}

inline void build_param_combinations(
  const std::vector<std::vector<SweepVariant>> & fields, const std::size_t field_index,
  const PlannerParams & current, const std::string & identifier,
  std::vector<PlannerParams> & output)
{
  if (field_index == fields.size()) {
    auto params = current;
    params.vehicle_shape.setMinMaxDimension();
    params.identifier = identifier.empty() ? "default" : identifier;
    std::cout << "Planner parameter identifier: " << params.identifier << std::endl;
    output.push_back(std::move(params));
    return;
  }

  for (const auto & variant : fields.at(field_index)) {
    auto next = current;
    auto next_identifier = identifier;
    variant.apply(next);
    append_identifier_part(next_identifier, variant.identifier_part);
    build_param_combinations(fields, field_index + 1, next, next_identifier, output);
  }
}

}  // namespace yaml_param_loader_detail

inline std::vector<PlannerParams> read_params_from_file(const std::string & path)
{
  const YAML::Node config = YAML::LoadFile(path);

  using yaml_param_loader_detail::SweepVariant;
  using yaml_param_loader_detail::build_param_combinations;
  using yaml_param_loader_detail::make_sweep_field;

  std::vector<std::vector<SweepVariant>> fields;

  fields.push_back(make_sweep_field<double>(
    config, "PlannerCommonParam", "time_limit",
    [](PlannerParams & p, const double v) { p.planner_common_params.time_limit = v; }));
  fields.push_back(make_sweep_field<int>(
    config, "PlannerCommonParam", "theta_size",
    [](PlannerParams & p, const int v) { p.planner_common_params.theta_size = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "PlannerCommonParam", "curve_weight",
    [](PlannerParams & p, const double v) { p.planner_common_params.curve_weight = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "PlannerCommonParam", "reverse_weight",
    [](PlannerParams & p, const double v) { p.planner_common_params.reverse_weight = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "PlannerCommonParam", "direction_change_weight",
    [](PlannerParams & p, const double v) {
      p.planner_common_params.direction_change_weight = v;
    }));
  fields.push_back(make_sweep_field<double>(
    config, "PlannerCommonParam", "lateral_goal_range",
    [](PlannerParams & p, const double v) { p.planner_common_params.lateral_goal_range = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "PlannerCommonParam", "longitudinal_goal_range",
    [](PlannerParams & p, const double v) {
      p.planner_common_params.longitudinal_goal_range = v;
    }));
  fields.push_back(make_sweep_field<double>(
    config, "PlannerCommonParam", "angle_goal_range",
    [](PlannerParams & p, const double v) { p.planner_common_params.angle_goal_range = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "PlannerCommonParam", "max_turning_ratio",
    [](PlannerParams & p, const double v) { p.planner_common_params.max_turning_ratio = v; }));
  fields.push_back(make_sweep_field<int>(
    config, "PlannerCommonParam", "turning_steps",
    [](PlannerParams & p, const int v) { p.planner_common_params.turning_steps = v; }));
  fields.push_back(make_sweep_field<int>(
    config, "PlannerCommonParam", "obstacle_threshold",
    [](PlannerParams & p, const int v) { p.planner_common_params.obstacle_threshold = v; }));

  fields.push_back(make_sweep_field<std::string>(
    config, "AstarParam", "search_method",
    [](PlannerParams & p, const std::string & v) { p.a_star_params.search_method = v; }));
  fields.push_back(make_sweep_field<bool>(
    config, "AstarParam", "only_behind_solutions",
    [](PlannerParams & p, const bool v) { p.a_star_params.only_behind_solutions = v; }));
  fields.push_back(make_sweep_field<bool>(
    config, "AstarParam", "use_back",
    [](PlannerParams & p, const bool v) { p.a_star_params.use_back = v; }));
  fields.push_back(make_sweep_field<bool>(
    config, "AstarParam", "adapt_expansion_distance",
    [](PlannerParams & p, const bool v) { p.a_star_params.adapt_expansion_distance = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "AstarParam", "expansion_distance",
    [](PlannerParams & p, const double v) { p.a_star_params.expansion_distance = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "AstarParam", "near_goal_distance",
    [](PlannerParams & p, const double v) { p.a_star_params.near_goal_distance = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "AstarParam", "distance_heuristic_weight",
    [](PlannerParams & p, const double v) { p.a_star_params.distance_heuristic_weight = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "AstarParam", "smoothness_weight",
    [](PlannerParams & p, const double v) { p.a_star_params.smoothness_weight = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "AstarParam", "obstacle_distance_weight",
    [](PlannerParams & p, const double v) { p.a_star_params.obstacle_distance_weight = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "AstarParam", "goal_lat_distance_weight",
    [](PlannerParams & p, const double v) { p.a_star_params.goal_lat_distance_weight = v; }));

  fields.push_back(make_sweep_field<double>(
    config, "VehicleShape", "length",
    [](PlannerParams & p, const double v) { p.vehicle_shape.length = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "VehicleShape", "width",
    [](PlannerParams & p, const double v) { p.vehicle_shape.width = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "VehicleShape", "base_length",
    [](PlannerParams & p, const double v) { p.vehicle_shape.base_length = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "VehicleShape", "max_steering",
    [](PlannerParams & p, const double v) { p.vehicle_shape.max_steering = v; }));
  fields.push_back(make_sweep_field<double>(
    config, "VehicleShape", "base2back",
    [](PlannerParams & p, const double v) { p.vehicle_shape.base2back = v; }));

  std::vector<PlannerParams> planner_params;
  PlannerParams base_params{};
  build_param_combinations(fields, 0, base_params, "", planner_params);

  return planner_params;
}
