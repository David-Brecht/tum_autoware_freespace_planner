#!/bin/bash

# give let the user define a range of paths here
scenario_path="/workspace/scenarios/snapshot_20260601_164941" 
# scenario_path="/workspace/scenarios/snapshot_20260602_172142" 
input_path="${scenario_path}/input"
output_path_rosbag="${scenario_path}/output"
output_path_plot="${scenario_path}/plot"

# The parameters for the planner
param_path="/workspace/src/universe/external/tum_autoware_freespace_planner/test/config/planner_config.yaml"

# mkdir -p "${output_path_rosbag}"
rm -rf "${output_path_rosbag}"
mkdir -p "${output_path_plot}"

echo "=======================" 
echo "=== Running planner ===" 
echo "=======================" 
/workspace/build/tum_autoware_freespace_planner/test_tum_autoware_freespace_planner \
  --input-path "${input_path}" \
  --param-path "${param_path}" \
  --output-path "${output_path_rosbag}" &&

echo "=============================" 
echo "=== Running visualization ===" 
echo "=============================" 
python3 /workspace/src/universe/external/tum_autoware_freespace_planner/scripts/debug_plot.py \
  --input-path "${output_path_rosbag}" \
  --output-path "${output_path_plot}" \
  --interactive &
plot_pid=$!

echo "======================"
echo "=== Opening output ===" 
echo "======================" 
evince "${output_path_plot}/plot.pdf" --fullscreen &
evince_pid=$!

read -n 1 -s -r -p "Press any key to stop..."

kill "$plot_pid" "$evince_pid"