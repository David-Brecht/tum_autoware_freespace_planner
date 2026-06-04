#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2021 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
from dataclasses import dataclass
from math import asin
from math import atan2
from math import cos
from math import sin
import os
import re
import subprocess
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float64


OUTPUT_PREFIX = "output_"


TRAJ_TOPIC_PREFIX = "trajectory_"


@dataclass
class ProblemDescription:
    costmap: OccupancyGrid
    start: Pose
    trajectories: Dict[str, PoseArray]  # key: "15_m", "20_m", …
    vehicle_length: Float64
    vehicle_width: Float64
    vehicle_base2back: Float64
    elapsed_time: Float64
    goal: Optional[Pose] = None  # no longer written by the planner, kept for older bags

    @classmethod
    def from_rosbag_path(cls, path: str) -> "ProblemDescription":
        # ref: rosbag2/rosbag2_py/test/test_sequential_reader.py
        storage_options, converter_options = cls.get_rosbag_options(path)
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        topic_types = reader.get_all_topics_and_types()

        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
        message_map = {}

        while reader.has_next():
            topic, data, t = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            message_map[topic] = msg

        trajectories = {
            k[len(TRAJ_TOPIC_PREFIX):]: v
            for k, v in message_map.items()
            if k.startswith(TRAJ_TOPIC_PREFIX)
        }
        scalar_fields = {
            k: v for k, v in message_map.items()
            if k in cls.__dataclass_fields__ and k != "trajectories"
        }
        return cls(trajectories=trajectories, **scalar_fields)

    @staticmethod
    def get_rosbag_options(path: str, serialization_format="cdr"):
        # copied from rosbag2/rosbag2_py/test/test_sequential_reader.py
        storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="mcap") # TODO make mcap

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format=serialization_format,
            output_serialization_format=serialization_format,
        )

        return storage_options, converter_options


@dataclass
class VehicleModel:
    length: float
    width: float
    base2back: float

    @classmethod
    def from_problem_description(cls, pd: ProblemDescription) -> "VehicleModel":
        return cls(pd.vehicle_length.data, pd.vehicle_width.data, pd.vehicle_base2back.data)

    # cspell: ignore nparr
    # nparr means "numpy array" (maybe)
    def get_vertices(self, pose: Pose) -> np.ndarray:
        x, y, yaw = self.pose_msg_to_nparr(pose)

        back = -1.0 * self.base2back
        front = self.length - self.base2back
        right = -0.5 * self.width
        left = 0.5 * self.width
        vertices_local = np.array([[back, left], [back, right], [front, right], [front, left]])

        R_mat = np.array([[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]])
        vertices_global = vertices_local.dot(R_mat.T) + np.array([x, y])
        return vertices_global

    def plot_pose(self, pose: Pose, ax, color="black", lw=1) -> List:
        x = pose.position.x
        y = pose.position.y
        V = self.get_vertices(pose)
        artists = [ax.scatter(x, y, color=color, s=2)]
        for idx_pair in [[0, 1], [1, 2], [2, 3], [3, 0]]:
            i, j = idx_pair
            line, = ax.plot([V[i, 0], V[j, 0]], [V[i, 1], V[j, 1]], color=color, linewidth=lw)
            artists.append(line)
        return artists

    @staticmethod
    def euler_from_quaternion(quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sin_roll_cos_pitch = 2 * (w * x + y * z)
        cos_roll_cos_pitch = 1 - 2 * (x * x + y * y)
        roll = atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

        sin_pitch = 2 * (w * y - z * x)
        pitch = asin(sin_pitch)

        sin_yaw_cos_pitch = 2 * (w * z + x * y)
        cos_yaw_cos_pitch = 1 - 2 * (y * y + z * z)
        yaw = atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
        return roll, pitch, yaw

    @staticmethod
    def pose_msg_to_nparr(pose_msg: Pose) -> Tuple[float, float, float]:
        _, _, yaw = VehicleModel.euler_from_quaternion(pose_msg.orientation)
        return pose_msg.position.x, pose_msg.position.y, yaw


@dataclass
class PlannerRun:
    identifier: str
    path: str
    problem: ProblemDescription


def natural_key(text: str) -> List:
    return [int(part) if part.isdigit() else part for part in re.split(r"(\d+)", text)]


def is_rosbag_dir(path: str) -> bool:
    return os.path.isdir(path) and os.path.exists(os.path.join(path, "metadata.yaml"))


def identifier_from_output_path(path: str) -> str:
    name = os.path.basename(os.path.normpath(path))
    if name.startswith(OUTPUT_PREFIX):
        return name[len(OUTPUT_PREFIX):]
    return name


def discover_output_bags(input_path: str) -> List[Tuple[str, str]]:
    if is_rosbag_dir(input_path):
        return [(identifier_from_output_path(input_path), input_path)]

    search_dir = input_path
    if not os.path.isdir(search_dir):
        search_dir = os.path.dirname(os.path.normpath(input_path))

    if os.path.basename(os.path.normpath(input_path)) == "output":
        search_dir = os.path.dirname(os.path.normpath(input_path))

    bags = []
    for entry in os.listdir(search_dir):
        path = os.path.join(search_dir, entry)
        if entry.startswith(OUTPUT_PREFIX) and is_rosbag_dir(path):
            bags.append((entry[len(OUTPUT_PREFIX):], path))

    return sorted(bags, key=lambda item: natural_key(item[0]))


def load_planner_runs(input_path: str) -> List[PlannerRun]:
    bag_infos = discover_output_bags(input_path)
    if not bag_infos:
        raise RuntimeError("No output rosbag directories found for input path: {}".format(input_path))

    runs = []
    for identifier, path in bag_infos:
        print("Loading output bag '{}' from {}".format(identifier, path))
        runs.append(PlannerRun(identifier, path, ProblemDescription.from_rosbag_path(path)))

    return runs


def plot_costmap(pd: ProblemDescription, ax) -> List:
    info = pd.costmap.info
    n_grid = np.array([info.width, info.height])
    res = info.resolution
    origin = info.origin
    arr = np.array(pd.costmap.data).reshape((n_grid[1], n_grid[0]))

    # Build corner grids (n_rows+1, n_cols+1) so pcolormesh draws each cell flat.
    # Account for origin orientation so rotated costmaps render correctly.
    _, _, yaw = VehicleModel.euler_from_quaternion(origin.orientation)
    cos_yaw, sin_yaw = cos(yaw), sin(yaw)
    ox, oy = origin.position.x, origin.position.y

    cols = np.arange(n_grid[0] + 1) * res
    rows = np.arange(n_grid[1] + 1) * res
    C, R = np.meshgrid(cols, rows)
    X = cos_yaw * C - sin_yaw * R + ox
    Y = sin_yaw * C + cos_yaw * R + oy

    return [ax.pcolormesh(X, Y, arr, cmap="Greys", shading="flat", rasterized=True)]


def plot_start_and_goal(pd: ProblemDescription, ax) -> Dict[str, List]:
    vehicle_model = VehicleModel.from_problem_description(pd)
    result: Dict[str, List] = {"Start": vehicle_model.plot_pose(pd.start, ax, "green", 4)}
    if pd.goal is not None:
        result["Goal"] = vehicle_model.plot_pose(pd.goal, ax, "red", 4)
    return result


def plot_trajectory(traj: PoseArray, vehicle_model: "VehicleModel", ax, color: str) -> List:
    traj_artists = []
    xs = [pose.position.x for pose in traj.poses]
    ys = [pose.position.y for pose in traj.poses]
    if xs and ys:
        line, = ax.plot(xs, ys, color=color, linewidth=2.0)
        traj_artists.append(line)

    step = max(len(traj.poses) // 40, 1)
    for pose in traj.poses[::step]:
        traj_artists.extend(vehicle_model.plot_pose(pose, ax, color, 0.5))

    return traj_artists


def trajectory_color(index: int):
    colors = plt.get_cmap("tab10").colors
    return colors[index % len(colors)]


def trajectory_label(run_identifier: str, dist_key: str, multi_run: bool) -> str:
    display = dist_key.replace("_", " ")
    return f"{run_identifier} – {display}" if multi_run else display


def set_view_limits(runs: List[PlannerRun], ax):
    # Zoom to a square bounding box around start and all trajectories.
    key_poses = []
    max_vehicle_dimension = 0.0
    for run in runs:
        pd = run.problem
        vehicle_model = VehicleModel.from_problem_description(pd)
        max_vehicle_dimension = max(max_vehicle_dimension, vehicle_model.length, vehicle_model.width)
        key_poses.append(pd.start)
        if pd.goal is not None:
            key_poses.append(pd.goal)
        for traj in pd.trajectories.values():
            key_poses.extend(traj.poses)

    xs = [p.position.x for p in key_poses]
    ys = [p.position.y for p in key_poses]
    margin = max_vehicle_dimension * 2
    cx = (min(xs) + max(xs)) / 2
    cy = (min(ys) + max(ys)) / 2
    half_span = max((max(xs) - min(xs)) / 2, (max(ys) - min(ys)) / 2) + margin

    ax.set_aspect("equal")
    ax.set_xlim([cx - half_span, cx + half_span])
    ax.set_ylim([cy - half_span, cy + half_span])


def plot_problem(runs: List[PlannerRun], ax, meta_info: Optional[str] = None) -> Dict[str, List]:
    base_pd = runs[0].problem
    multi_run = len(runs) > 1

    groups: Dict[str, List] = {"Costmap": plot_costmap(base_pd, ax)}
    groups.update(plot_start_and_goal(base_pd, ax))

    color_idx = 0
    for run in runs:
        vehicle_model = VehicleModel.from_problem_description(run.problem)
        for dist_key in sorted(run.problem.trajectories.keys(), key=natural_key):
            label = trajectory_label(run.identifier, dist_key, multi_run)
            traj = run.problem.trajectories[dist_key]
            groups[label] = plot_trajectory(traj, vehicle_model, ax, trajectory_color(color_idx))
            color_idx += 1

    set_view_limits(runs, ax)

    if meta_info is None:
        meta_info = "{} output bag{}".format(len(runs), "" if len(runs) == 1 else "s")
    ax.set_title(meta_info, color="black")

    return groups


def create_concat_png(src_list, dest, is_horizontal):
    opt = "+append" if is_horizontal else "-append"
    cmd = ["convert", opt]
    for src in src_list:
        cmd.append(src)
    cmd.append(dest)
    subprocess.Popen(cmd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--concat", action="store_true", help="concat png images (requires imagemagick)")
    parser.add_argument("--input-path", default="/workspace", help="the path to the rosbag containing all information for plotting")
    parser.add_argument("--output-path", default="/workspace/output", help="the path to the rosbag containing all information for plotting")
    parser.add_argument("--interactive", action="store_true", help="open interactive window with layer toggles instead of saving to PDF")
    args = parser.parse_args()
    concat = args.concat
    input_path = args.input_path
    output_path = args.output_path
    os.makedirs(output_path, exist_ok=True)

    runs = load_planner_runs(input_path)
    meta_info = "{} output bag{}".format(len(runs), "" if len(runs) == 1 else "s")

    if args.interactive:
        fig, ax = plt.subplots(figsize=(12, 8))
        fig.subplots_adjust(right=0.66)

        groups = plot_problem(runs, ax, meta_info)

        labels = list(groups.keys())
        check_ax = fig.add_axes([0.68, 0.12, 0.30, 0.78])
        check = CheckButtons(check_ax, labels, actives=[True] * len(labels))
        label_colors = {"Costmap": "black", "Start": "green", "Goal": "red"}
        color_idx = 0
        for run in runs:
            for dist_key in sorted(run.problem.trajectories.keys(), key=natural_key):
                label = trajectory_label(run.identifier, dist_key, len(runs) > 1)
                label_colors[label] = trajectory_color(color_idx)
                color_idx += 1
        for label in check.labels:
            label.set_color(label_colors.get(label.get_text(), "black"))
            label.set_fontsize(8)

        def on_toggle(label):
            for artist in groups[label]:
                artist.set_visible(not artist.get_visible())
            fig.canvas.draw_idle()

        check.on_clicked(on_toggle)

        file_name = os.path.join(output_path, "plot.pdf")
        plt.savefig(file_name)
        print("saved to {}".format(file_name))

        plt.show()
    else:
        fig, ax = plt.subplots(figsize=(12, 8))
        fig.subplots_adjust(right=0.66)
        groups = plot_problem(runs, ax, meta_info)

        labels = list(groups.keys())
        check_ax = fig.add_axes([0.68, 0.12, 0.30, 0.78])
        check_ax.axis("off")
        label_colors = {"Costmap": "black", "Start": "green", "Goal": "red"}
        color_idx = 0
        for run in runs:
            for dist_key in sorted(run.problem.trajectories.keys(), key=natural_key):
                label = trajectory_label(run.identifier, dist_key, len(runs) > 1)
                label_colors[label] = trajectory_color(color_idx)
                color_idx += 1
        for idx, label in enumerate(labels):
            y = 1.0 - idx * 0.055
            if y < 0:
                break
            check_ax.text(0.0, y, label, fontsize=8, va="top", color=label_colors.get(label, "black"))

        file_name = os.path.join(output_path, "plot.pdf")
        plt.savefig(file_name)
        print("saved to {}".format(file_name))
