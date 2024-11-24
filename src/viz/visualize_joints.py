#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import yaml


class VisualizeJointsNode(Node):
    def __init__(self):
        super().__init__("visualize_joints_node")
        self.subscription = self.create_subscription(
            Float32MultiArray, "/hand/policy_output", self.policy_output_callback, 10
        )
        self.publisher_ = self.create_publisher(JointState, "/joint_states", 10)
        self.get_logger().info('Subscribing to "/hand/policy_output"')
        self.get_logger().info('Publishing to "/joint_states"')
        
        self.declare_parameter('scheme_path', "")
        scheme_path = self.get_parameter("scheme_path").value
        print(f"Reading hand scheme from {scheme_path}")
        # Read the YAML file directly
        with open(scheme_path, 'r') as f:
            self.hand_scheme = yaml.safe_load(f)

        self.tendons = self.hand_scheme["gc_tendons"]
        # a list representation of the jacobian matrix. Each element is a tuple (tendon_name, factor)
        self.jacobian_list = []
        self.joint_names = []
        for i, (tendon_name, joints) in enumerate(self.tendons.items()):
            self.joint_names.append(tendon_name)
            rolling_contact_factor = 0.5 if tendon_name.endswith("_virt") else 1.0
            self.jacobian_list.append((i, rolling_contact_factor))
            for joint_name, factor in joints.items():
                self.joint_names.append(joint_name)
                self.jacobian_list.append((i, factor * rolling_contact_factor))

        self.js_msg = JointState()
        self.js_msg.name = self.joint_names

    def policy_output_callback(self, msg):
        
        self.js_msg.header.stamp = self.get_clock().now().to_msg()
        joint_states = self.policy_output2urdf_joint_states(msg.data)
        self.js_msg.position = joint_states
        self.publisher_.publish(self.js_msg)
        # self.get_logger().info('Publishing joint states: "%s"' % self.js_msg)

    def policy_output2urdf_joint_states(self, joint_values):
        """
        Process joint values to create a vector where each joint's value is halved.
        If the joint has a corresponding virtual joint (virt), duplicate the value.

        :param joint_values: List of joint values with length N (in this case, 16).
        :param has_virt_joint: List indicating if each joint has a virtual counterpart.
        :return: List with processed joint values, doubled for those with virtual joints.
        """
        # Initialize the output list for processed joint values
        assert (len(joint_values) == len(self.tendons)), f"The length of joint values {len(joint_values)} should match the number of tendons {len(self.tendons)}"
        # Iterate over each joint value and corresponding virtual status
        processed_values = [joint_values[ind] * factor for ind, factor in self.jacobian_list]
        return processed_values


def main(args=None):
    rclpy.init(args=args)
    node = VisualizeJointsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
