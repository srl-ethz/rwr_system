#!/usr/bin/env python3
import subprocess
import rclpy
from rclpy.node import Node

class RosbagNode(Node):
    def __init__(self):
        super().__init__('rosbag_player')
        
        # Get the rosbag path from the parameters
        self.declare_parameter("rosbag_path", rclpy.Parameter.Type.STRING)
        rosbag_path = self.get_parameter("rosbag_path").value

        print(type(rosbag_path))
        if rosbag_path is None:
            self.get_logger().error('rosbag_path is not set! Please provide a valid rosbag path.')
            raise ValueError('rosbag_path cannot be None.')

        # Proceed with playing the rosbag
        self.get_logger().info(f'Playing rosbag from path: {rosbag_path}')
        self.process = subprocess.Popen(["ros2", "bag", "play", rosbag_path, "--loop"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def destroy_node(self):
        self.process.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RosbagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()