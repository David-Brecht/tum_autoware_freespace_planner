#!/bin/bash

# give let the user define a range of paths here
# scenario_path="/workspace/scenarios/snapshot_20260601_164941"       # vehicle in ego lane. prefer this
# scenario_path="/workspace/scenarios/snapshot_20260602_125319"       # bus in ego lane
# scenario_path="/workspace/scenarios/snapshot_20260602_234306"       # bus slightly in ego lane
# scenario_path="/workspace/scenarios/snapshot_20260604_112549"       # parked vehicle inside curve
# scenario_path="/workspace/scenarios/snapshot_20260604_114408"
scenario_paths=(
  # "/workspace/scenarios/snapshot_20260601_164941"
  # "/workspace/scenarios/snapshot_20260605_145120"
  # "/workspace/scenarios/snapshot_20260605_163947"
  # "/workspace/scenarios/snapshot_20260604_180140"
  # "/workspace/scenarios/snapshot_20260602_125319"
  "/workspace/scenarios/snapshot_20260604_112549"     # left turn
  # "/workspace/scenarios/snapshot_20260605_143657"     # left turn, 0.5 m lanelet margin
  # "/workspace/scenarios/snapshot_20260608_104534"
)

# The parameters for the planner
param_path="/workspace/src/universe/external/tum_autoware_freespace_planner/test/config/planner_config.yaml"

plot_pids=()

for scenario_path in "${scenario_paths[@]}"; do
  input_path="${scenario_path}/input"
  output_path_rosbag="${scenario_path}/output"
  output_path_plot="${scenario_path}/plot"

  rm -rf "${output_path_rosbag}"*
  mkdir -p "${output_path_plot}"

  echo ""
  echo "========================================"
  echo "=== Running planner: ${scenario_path} ==="
  echo "========================================"
  /workspace/build/tum_autoware_freespace_planner/test_tum_autoware_freespace_planner \
    --input-path "${input_path}" \
    --param-path "${param_path}" \
    --output-path "${output_path_rosbag}" || { echo "Planner failed for ${scenario_path}, skipping."; continue; }

  echo ""
  echo "=============================================="
  echo "=== Running visualization: ${scenario_path} ==="
  echo "=============================================="
  python3 /workspace/src/universe/external/tum_autoware_freespace_planner/scripts/debug_plot.py \
    --input-path "${output_path_rosbag}" \
    --output-path "${output_path_plot}" \
    --interactive &
  plot_pids+=($!)
done

echo ""
echo "========================="
echo "=== All scenarios done ==="
echo "========================="
read -n 1 -s -r -p "Press any key to stop all visualizations..."

kill "${plot_pids[@]}"