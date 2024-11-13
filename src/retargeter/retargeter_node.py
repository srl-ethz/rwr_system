#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from faive_system.src.retargeter import Retargeter
from faive_system.src.common.utils import numpy_to_float32_multiarray
import os
from faive_system.src.viz.visualize_mano import ManoHandVisualizer

class RetargeterNode(Node):
    def __init__(self, debug=False):
        super().__init__("rokoko_node")

        # start retargeter
        self.declare_parameter("retarget/mjcf_filepath", rclpy.Parameter.Type.STRING)
        self.declare_parameter("retarget/urdf_filepath", rclpy.Parameter.Type.STRING)
        self.declare_parameter("retarget/hand_scheme", rclpy.Parameter.Type.STRING)
        self.declare_parameter("debug", rclpy.Parameter.Type.BOOL)

        try:
            mjcf_filepath = self.get_parameter("retarget/mjcf_filepath").value
        except:
            mjcf_filepath = None
        
        try:
            urdf_filepath = self.get_parameter("retarget/urdf_filepath").value
        except:
            urdf_filepath = None
        hand_scheme = self.get_parameter("retarget/hand_scheme").value
        debug = self.get_parameter("debug").value
        
        # subscribe to ingress topics
        self.ingress_mano_sub = self.create_subscription(
            Float32MultiArray, "/ingress/mano", self.ingress_mano_cb, 10
        )
        
        self.retargeter = Retargeter(
            device="cuda",  mjcf_filepath= mjcf_filepath, urdf_filepath=urdf_filepath, hand_scheme=hand_scheme
        )
        
        self.joints_pub = self.create_publisher(
            Float32MultiArray, "/hand/policy_output", 10
        )
        self.debug = debug
        if self.debug:
            self.rviz_pub = self.create_publisher(MarkerArray, 'retarget/normalized_mano_points', 10)
            self.mano_hand_visualizer = ManoHandVisualizer(self.rviz_pub)
            
        
        self.timer = self.create_timer(0.005, self.timer_publish_cb)
    
    def ingress_mano_cb(self, msg):
        self.keypoint_positions = np.array(msg.data).reshape(-1, 3)
    
        
    def timer_publish_cb(self):
        try:
            if self.debug:
                self.mano_hand_visualizer.reset_markers()

            debug_dict = {}
            joint_angles, debug_dict = self.retargeter.retarget(self.keypoint_positions, debug_dict)

            if self.debug:
                self.mano_hand_visualizer.generate_hand_markers(
                    debug_dict["normalized_joint_pos"],
                    stamp=self.get_clock().now().to_msg(),
                )

            self.joints_pub.publish(
                numpy_to_float32_multiarray(np.deg2rad(joint_angles))
            )

            if self.debug:
                self.mano_hand_visualizer.publish_markers()
        except Exception as e:
            print(f"Error in timer_publish_cb: {e}")
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RetargeterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
