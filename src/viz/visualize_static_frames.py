#!/usr/bin/env python3
"""
yaml_static_tf_publisher.py
Read a YAML file that lists rigid transforms and publish them as static TF2 frames.

Run (example):
  ros2 run <your_pkg> yaml_static_tf_publisher \
        --ros-args -p yaml_file:=/path/to/transforms.yaml
"""

import os
import sys
from typing import List

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_matrix
import yaml
import numpy as np


class YAMLStaticTFPublisher(Node):
    """Node that loads transforms from YAML and publishes them once as /tf_static."""

    def __init__(self):
        super().__init__("yaml_static_tf_publisher")

        # Declare and read the 'yaml_file' parameter.
        self.declare_parameter("yaml_file", "")
        yaml_path = self.get_parameter("yaml_file").get_parameter_value().string_value

        if not yaml_path:
            self.get_logger().error("Parameter 'yaml_file' is empty.")
            rclpy.shutdown()
            return

        if not os.path.isfile(yaml_path):
            self.get_logger().error(f"YAML file not found: {yaml_path}")
            rclpy.shutdown()
            return

        transforms = self._load_transforms(yaml_path)
        if not transforms:
            self.get_logger().error("No transforms found in YAML; nothing to publish.")
            rclpy.shutdown()
            return

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(transforms)

        self.get_logger().info(
            f"Published {len(transforms)} static transform(s) from '{yaml_path}'."
        )

        # Keep the node alive so the latched /tf_static message remains available.
        # (If you prefer, you could call rclpy.shutdown() right away.)
        self._timer = self.create_timer(1.0, lambda: None)

    # ------------------------------------------------------------------ helpers

    def _load_transforms(self, yaml_path: str) -> List[TransformStamped]:
        """Parse YAML and build a list of TransformStamped messages."""
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        transforms: List[TransformStamped] = []
        now = self.get_clock().now().to_msg()

        for parent_frame, children in data.items():
            if not isinstance(children, dict):
                self.get_logger().warn(
                    f"Ignoring non-mapping under parent frame '{parent_frame}'."
                )
                continue

            for child_frame, tf_dict in children.items():
                try:
                    pos = tf_dict["pos"]
                    rot = tf_dict["rot"]  # 3 × 3 list of lists
                    T = np.identity(4)
                    T[:3, :3] = np.array(rot, dtype=float)
                    q = quaternion_from_matrix(T)

                    msg = TransformStamped()
                    msg.header.stamp = now
                    msg.header.frame_id = parent_frame
                    msg.child_frame_id = child_frame
                    msg.transform.translation.x = float(pos[0])
                    msg.transform.translation.y = float(pos[1])
                    msg.transform.translation.z = float(pos[2])
                    msg.transform.rotation.x = float(q[0])
                    msg.transform.rotation.y = float(q[1])
                    msg.transform.rotation.z = float(q[2])
                    msg.transform.rotation.w = float(q[3])
                    transforms.append(msg)

                except (KeyError, ValueError, TypeError) as e:
                    self.get_logger().warn(
                        f"Failed to read transform '{parent_frame}->{child_frame}': {e}"
                    )

        return transforms


def main(argv: List[str] = None):
    rclpy.init(args=argv)
    node = YAMLStaticTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
