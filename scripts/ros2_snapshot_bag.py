#!/usr/bin/env python3
"""Record exactly one message from each target topic into an .mcap bag."""

from __future__ import annotations

import argparse
import sys
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py

from autoware_planning_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid, Odometry


TOPICS: dict[str, dict] = {
    '/planning/scenario_planning/lane_driving/behavior_planning/path': {
        'type': 'autoware_planning_msgs/msg/Path',
        'cls': Path,
    },
    '/planning/scenario_planning/parking/costmap_generator/occupancy_grid': {
        'type': 'nav_msgs/msg/OccupancyGrid',
        'cls': OccupancyGrid,
    },
    '/localization/kinematic_state': {
        'type': 'nav_msgs/msg/Odometry',
        'cls': Odometry,
    },
}


class SnapshotRecorder(Node):
    def __init__(self, writer: rosbag2_py.SequentialWriter) -> None:
        super().__init__('ros2_snapshot_bag')
        self._writer = writer
        self._received: dict[str, bool] = {t: False for t in TOPICS}

        for topic, info in TOPICS.items():
            self.create_subscription(
                info['cls'],
                topic,
                lambda msg, t=topic: self._on_msg(t, msg),
                rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value,
            )
            self.get_logger().info(f'Waiting for: {topic}')

    def _on_msg(self, topic: str, msg) -> None:
        if self._received[topic]:
            return
        self._received[topic] = True
        stamp_ns = self.get_clock().now().nanoseconds
        self._writer.write(topic, serialize_message(msg), stamp_ns)
        remaining = sum(1 for v in self._received.values() if not v)
        self.get_logger().info(f'[{3 - remaining}/3] Captured {topic}')

    @property
    def done(self) -> bool:
        return all(self._received.values())

    def missing(self) -> list[str]:
        return [t for t, v in self._received.items() if not v]


def _make_topic_metadata(name: str, type_str: str) -> rosbag2_py.TopicMetadata:
    try:
        # ROS2 Iron+ requires positional id as first arg
        return rosbag2_py.TopicMetadata(
            id=0,
            name=name,
            type=type_str,
            serialization_format='cdr',
        )
    except TypeError:
        return rosbag2_py.TopicMetadata(
            name=name,
            type=type_str,
            serialization_format='cdr',
        )


def open_writer(output: str) -> rosbag2_py.SequentialWriter:
    writer = rosbag2_py.SequentialWriter()
    storage_opts = rosbag2_py.StorageOptions(uri=output, storage_id='mcap')
    converter_opts = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    writer.open(storage_opts, converter_opts)
    for topic, info in TOPICS.items():
        writer.create_topic(_make_topic_metadata(topic, info['type']))
    return writer


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Capture one message per topic and write to an .mcap bag.'
    )
    parser.add_argument(
        '-o', '--output',
        default=f'/workspace/scenarios/snapshot_{datetime.now().strftime("%Y%m%d_%H%M%S")}/input',
        help='Output bag path (default: snapshot_<timestamp>)',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=30.0,
        help='Seconds to wait before giving up (default: 30)',
    )
    args = parser.parse_args()

    rclpy.init()
    writer = open_writer(args.output)
    node = SnapshotRecorder(writer)

    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds > deadline:
                node.get_logger().error(
                    f'Timeout after {args.timeout}s. Still missing:\n'
                    + '\n'.join(f'  {t}' for t in node.missing())
                )
                sys.exit(1)
    except KeyboardInterrupt:
        if not node.done:
            node.get_logger().warn(
                'Interrupted. Missing:\n'
                + '\n'.join(f'  {t}' for t in node.missing())
            )
    finally:
        node.get_logger().info(f'Bag written to: {args.output}')
        node.destroy_node()
        rclpy.shutdown()
        del writer  # flushes and closes the bag


if __name__ == '__main__':
    main()
