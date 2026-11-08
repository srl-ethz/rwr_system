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
        self.declare_parameter("rokoko_tracker/enable_right", True)
        self.declare_parameter("rokoko_tracker/enable_left", True)

        ip = self.get_parameter("rokoko_tracker/ip").value
        port = self.get_parameter("rokoko_tracker/port").value
        self.use_coil = self.get_parameter("rokoko_tracker/use_coil").value
        self.enable_right = self.get_parameter("rokoko_tracker/enable_right").value
        self.enable_left = self.get_parameter("rokoko_tracker/enable_left").value

        self.tracker = RokokoTracker(ip=ip, port=port, use_coil=self.use_coil, 
                                   enable_right=self.enable_right, enable_left=self.enable_left)
        self.tracker.start()

        ingress_period = 0.005  # Timer period in seconds
        self.timer = self.create_timer(ingress_period, self.timer_publish_cb)

        ingress_right_mano_pub = self.create_publisher(
            Float32MultiArray, "/ingress/right/mano", 5
        )
        ingress_right_wrist_pub = self.create_publisher(
            PoseStamped, "/ingress/right/wrist", 5
        )
        ingress_right_forearm_pub = self.create_publisher(
            PoseStamped, "/ingress/right/forearm", 5
        )
        ingress_left_mano_pub = self.create_publisher(
            Float32MultiArray, "/ingress/left/mano", 5
        )
        ingress_left_wrist_pub = self.create_publisher(
            PoseStamped, "/ingress/left/wrist", 5
        )
        ingress_left_forearm_pub = self.create_publisher(
            PoseStamped, "/ingress/left/forearm", 5
        )
        self.wrist_pubs = [ingress_left_wrist_pub, ingress_right_wrist_pub]
        self.forearm_pubs = [ingress_left_forearm_pub, ingress_right_forearm_pub]
        self.mano_pubs = [ingress_left_mano_pub, ingress_right_mano_pub]

        self.debug = debug

    def timer_publish_cb(self):

        key_points, timestamp = self.tracker.get_keypoint_positions()
        if key_points is None:
            return
        left_keypoint_positions, right_keypoint_positions = key_points

        if self.enable_left:
            left_keypoint_positions_msg = numpy_to_float32_multiarray(left_keypoint_positions)
            self.mano_pubs[0].publish(left_keypoint_positions_msg)

        if self.enable_right:
            right_keypoint_positions_msg = numpy_to_float32_multiarray(right_keypoint_positions)
            self.mano_pubs[1].publish(right_keypoint_positions_msg)

        if self.use_coil:
            wrist_poses = self.tracker.get_wrist_poses()
            forearm_poses = self.tracker.get_arm_poses()
            if wrist_poses is None or forearm_poses is None:
                return

            # concatenate: [left wrist, right wrist, left forearm, right forearm]
            both_poses = wrist_poses + forearm_poses
            both_pubs = self.wrist_pubs + self.forearm_pubs

            # Publish right and left (wrist+arm) poses
            for i, (pose, pub) in enumerate(zip(both_poses, both_pubs)):
                # Skip publishing if hand is disabled
                is_left = (i % 2 == 0)
                is_right = not is_left
                if (is_left and not self.enable_left) or (is_right and not self.enable_right):
                    continue
                
                pos, rot = pose

                # Create a PoseStamped message
                msg = PoseStamped()
                msg.header.frame_id = "coil"
                msg.header.stamp = self.get_clock().now().to_msg()

                # Assign position using Point
                msg.pose.position = Point(
                    x=pos[0], y=pos[1], z=pos[2]
                )

                # Assign orientation using Quaternion
                msg.pose.orientation = Quaternion(
                    x=rot[0], y=rot[1], z=rot[2], w=rot[3]
                )

                # Publish the message
                pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RokokoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.tracker.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
