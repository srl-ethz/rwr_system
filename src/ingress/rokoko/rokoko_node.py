#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rokoko_ingress import RokokoTracker
from faive_system.src.common.utils import numpy_to_float32_multiarray    


class RokokoNode(Node):
    def __init__(self, debug=False):
        super().__init__("rokoko_node")

        # start tracker
        self.declare_parameter("rokoko_tracker/ip", "0.0.0.0")
        self.declare_parameter("rokoko_tracker/port", 14043)
        self.declare_parameter("rokoko_tracker/use_coil", True)

        ip = self.get_parameter("rokoko_tracker/ip").value
        port = self.get_parameter("rokoko_tracker/port").value
        self.use_coil = self.get_parameter("rokoko_tracker/use_coil").value

        self.tracker = RokokoTracker(ip=ip, port=port, use_coil=self.use_coil)
        self.tracker.start()

        ingress_period = 0.005  # Timer period in seconds
        self.timer = self.create_timer(ingress_period, self.timer_publish_cb)

        self.ingress_mano_pub = self.create_publisher(
            Float32MultiArray, "/ingress/mano", 10
        )
        self.ingress_wrist_pub = self.create_publisher(
            PoseStamped, "/ingress/wrist", 10
        )
        self.debug = debug

    def timer_publish_cb(self):

        key_points = self.tracker.get_keypoint_positions()
        wait_cnt = 1
        while (key_points is None):
            if (not (wait_cnt % 100000)):
                print("waiting for hand tracker", wait_cnt//100000)
            wait_cnt+=1
            key_points = self.tracker.get_keypoint_positions()
        keypoint_positions, timestamp = key_points

        keypoint_positions_msg = numpy_to_float32_multiarray(keypoint_positions)
        self.ingress_mano_pub.publish(keypoint_positions_msg)

        if self.use_coil:
            wrist_pos, wrist_rot = self.tracker.get_wrist_pose()
            # Create a PoseStamped message
            wrist_msg = PoseStamped()
            wrist_msg.header.frame_id = "coil"
            wrist_msg.header.stamp = self.get_clock().now().to_msg()

            # Assign position using Point
            wrist_msg.pose.position = Point(
                x=wrist_pos[0], y=wrist_pos[1], z=wrist_pos[2]
            )

            # Assign orientation using Quaternion
            wrist_msg.pose.orientation = Quaternion(
                x=wrist_rot[0], y=wrist_rot[1], z=wrist_rot[2], w=wrist_rot[3]
            )

            # Publish the message
            self.ingress_wrist_pub.publish(wrist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RokokoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
