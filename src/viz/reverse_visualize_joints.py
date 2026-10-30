#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

import yaml
from typing import List, Tuple


class JointStateToPolicyOutputNode(Node):
    """
    Does the reverse of visualize_joints.py:
      - Subscribes to "joint_states" (sensor_msgs/JointState)
      - Reconstructs the vector (Float32MultiArray) using the same scheme YAML
      - Publishes to "hand/policy_output" (std_msgs/Float32MultiArray)
    Useful if you want to use the joint_state_publisher_gui to control the robot hand

    Notes:
      - The forward mapping in visualize_joints.py is: y_k = factor_k * x_tendon_idx
        where each JointState entry depends on exactly one tendon value (no mixing).
      - To invert it, we compute x = y / factor for every matching joint name and
        average multiple estimates for the same tendon.
      - Topic names are *relative*, so namespaces/remaps work the same way as in the original node.
    """

    def __init__(self):
        super().__init__("jointstate_to_policy_output_node")

        # Match parameters used by visualize_joints.py for naming behavior
        self.declare_parameter("viz/hand_chirality", "")
        self.declare_parameter("viz/joint_name_contains_chirality", False)

        # Same scheme parameter name as visualize_joints.py
        self.declare_parameter("scheme_path", "")

        self.hand_chirality: str = self.get_parameter("viz/hand_chirality").value
        self.joint_name_contains_chirality: bool = bool(
            self.get_parameter("viz/joint_name_contains_chirality").value
        )

        self.joint_name_prefix = ""
        if self.hand_chirality and self.joint_name_contains_chirality:
            self.joint_name_prefix = f"{self.hand_chirality}_"

        scheme_path = self.get_parameter("scheme_path").value
        if not scheme_path:
            raise RuntimeError('Parameter "scheme_path" must be set (path to scheme_*.yaml).')

        self.get_logger().info(f"Reading hand scheme from: {scheme_path}")
        with open(scheme_path, "r") as f:
            scheme = yaml.safe_load(f)

        tendons = scheme["gc_tendons"]  # dict: tendon_name -> {joint_name: factor, ...}

        # Tendon ordering must match visualize_joints.py (dict iteration order)
        self.tendon_names: List[str] = list(tendons.keys())
        self.num_tendons = len(self.tendon_names)

        # Build lookup: joint_index -> (joint_name, factor)
        # This mirrors how visualize_joints.py builds its jacobian_list and joint_names.
        self.index_to_name_factor: List[Tuple[str, float]] = []
        for main_joint_name in tendons.keys():
            rolling_contact_factor = 0.5 if main_joint_name.endswith("_virt") else 1.0

            tendon_joint_name = f"{self.joint_name_prefix}{main_joint_name}"
            self.index_to_name_factor.append((tendon_joint_name, rolling_contact_factor))
            # ignore the other joints listed under this tendon, just use the main one

        # Reverse of the original topics:
        self.sub = self.create_subscription(JointState, "joint_states", self._cb_joint_states, 10)
        self.pub = self.create_publisher(Float32MultiArray, "hand/policy_output", 10)

        self.get_logger().info('Subscribing to "joint_states"')
        self.get_logger().info('Publishing to "hand/policy_output"')

        self.joint_angles = [0.] * len(self.index_to_name_factor)
        self.out_msg = Float32MultiArray()

    def _cb_joint_states(self, msg: JointState) -> None:
        # Reconstruct Float32MultiArray message from joint states
        for i, (name, factor) in enumerate(self.index_to_name_factor):
            try:
                i_in_msg = msg.name.index(name)
            except ValueError:
                raise RuntimeError(f'Joint name "{name}" not found in JointState message.')
            self.joint_angles[i] = msg.position[i_in_msg] / factor
        self.out_msg.data = self.joint_angles
        self.pub.publish(self.out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateToPolicyOutputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
