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
from typing import Tuple

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float64


@dataclass
class ProblemDescription:
    costmap: OccupancyGrid
    start: Pose
    goal: Pose
    trajectory: PoseArray
    vehicle_length: Float64
    vehicle_width: Float64
    vehicle_base2back: Float64
    elapsed_time: Float64

    @classmethod
    def from_rosbag_path(cls, path: str) -> "ProblemDescription":
        # ref: rosbag2/rosbag2_py/test/test_sequential_reader.py
        storage_options, converter_options = cls.get_rosbag_options(path)
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        topic_types = reader.get_all_topics_and_types()

        print(topic_types)

        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
        message_map = {}
        print(type_map)

        while reader.has_next():
            topic, data, t = reader.read_next()
            print("topic", topic)
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            message_map[topic] = msg

        return cls(**{k: v for k, v in message_map.items() if k in cls.__dataclass_fields__})

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

    def plot_pose(self, pose: Pose, ax, color="black", lw=1):
        x = pose.position.x
        y = pose.position.y
        V = self.get_vertices(pose)
        ax.scatter(x, y, c=color, s=2)
        for idx_pair in [[0, 1], [1, 2], [2, 3], [3, 0]]:
            i, j = idx_pair
            ax.plot([V[i, 0], V[j, 0]], [V[i, 1], V[j, 1]], color=color, linewidth=lw)

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


def plot_problem(pd: ProblemDescription, ax, meta_info):
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

    ax.pcolormesh(X, Y, arr, cmap="Greys", shading="flat", rasterized=True)

    vehicle_model = VehicleModel.from_problem_description(pd)
    vehicle_model.plot_pose(pd.start, ax, "green", 4)
    vehicle_model.plot_pose(pd.goal, ax, "red", 4)

    for pose in pd.trajectory.poses:
        vehicle_model.plot_pose(pose, ax, "blue", 0.5)

    # Zoom to a square bounding box around start, goal, and trajectory.
    key_poses = [pd.start, pd.goal] + list(pd.trajectory.poses)
    xs = [p.position.x for p in key_poses]
    ys = [p.position.y for p in key_poses]
    margin = max(vehicle_model.length, vehicle_model.width) * 2
    cx = (min(xs) + max(xs)) / 2
    cy = (min(ys) + max(ys)) / 2
    half_span = max((max(xs) - min(xs)) / 2, (max(ys) - min(ys)) / 2) + margin

    ax.set_aspect("equal")
    ax.set_xlim([cx - half_span, cx + half_span])
    ax.set_ylim([cy - half_span, cy + half_span])

    elapsed_ms = int(round(pd.elapsed_time.data))
    ax.set_title("{} | elapsed: {} ms".format(meta_info, elapsed_ms), color="black")


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
    args = parser.parse_args()
    # print(args)
    concat = args.concat
    input_path = args.input_path
    output_path = args.output_path

    fig, ax = plt.subplots()
    pd = ProblemDescription.from_rosbag_path(os.path.join(input_path))

    meta_info = "test"
    plot_problem(pd, ax, meta_info)
    fig.tight_layout()

    file_name = os.path.join(output_path, "plot.pdf")
    plt.savefig(file_name)
    print("saved to {}".format(file_name))
