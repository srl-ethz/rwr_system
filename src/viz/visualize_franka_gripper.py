#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import yaml
from array import array  # Import array module for conversion


class VisualizeJointsNode(Node):
    def __init__(self):
        super().__init__("visualize_joints_node")
        self.subscription = self.create_subscription(
            Float32MultiArray, "/hand/policy_output", self.policy_output_callback, 10
        )
        self.publisher_ = self.create_publisher(JointState, "/joint_states", 10)
        self.get_logger().info('Subscribing to "/hand/policy_output"')
        self.get_logger().info('Publishing to "/joint_states"')
        
        self.js_msg = JointState()


    def policy_output_callback(self, msg):
        self.js_msg.header.stamp = self.get_clock().now().to_msg()
        print(f"Received policy output: {msg.data}")
        self.js_msg.position = array('d', msg.data)
        self.js_msg.name = ["panda_finger_joint1"]
        self.publisher_.publish(self.js_msg)
        print(f"Published joint states: {self.js_msg.position}")
        
    def width2joint(self, width):
        joint = width/2
        return joint



def main(args=None):
    rclpy.init(args=args)
    node = VisualizeJointsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
