#!/usr/bin/env python3

"""
This script is used to test the retargeter module.
Instead of receiving mano keypoints from ingress nodes, it reads from a JSON file and calls the retargeter module.
It publishes the joint angles of the retargeted hand to the topic "/hand/policy_output".
The JSON file is either a recording of experiments/rokoko_recorder.py ([left, right, timestamp] per pose,
the right hand is used) or the older single-hand format ([handpose, timestamp] per pose).
"""

import rclpy
from rclpy.node import Node
import torch
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from faive_system.src.retargeter import Retargeter
from faive_system.src.common.utils import numpy_to_float32_multiarray
import os
from faive_system.src.viz.visualize_mano import ManoHandVisualizer
import json
from ament_index_python.packages import get_package_share_directory
import time

class RetargeterNode(Node):
    def __init__(self, debug=False):
        super().__init__("retargeter_test_node")

        # start retargeter
        self.declare_parameter("retarget/mjcf_filepath", rclpy.Parameter.Type.STRING)
        self.declare_parameter("retarget/urdf_filepath", rclpy.Parameter.Type.STRING)
        # URDF content (e.g. rendered from a xacro); if non-empty, it is used instead of urdf_filepath
        self.declare_parameter("robot_description", "")
        self.declare_parameter("handposes_file", "handposes_20241127222652.json")
        self.declare_parameter("retarget/hand_scheme", rclpy.Parameter.Type.STRING)
        self.declare_parameter("retarget/mano_adjustments", "")
        self.declare_parameter("retarget/retargeter_cfg", "")
        self.declare_parameter("debug", True)

        try:
            mjcf_filepath = self.get_parameter("retarget/mjcf_filepath").value
        except:
            mjcf_filepath = None
        
        urdf_string = self.get_parameter("robot_description").value or None
        try:
            urdf_filepath = self.get_parameter("retarget/urdf_filepath").value
        except:
            urdf_filepath = None
        if urdf_string is not None:
            urdf_filepath = None
        hand_scheme = self.get_parameter("retarget/hand_scheme").value
        # mano_adjustments's default value is None
        mano_adjustments = self.get_parameter("retarget/mano_adjustments").value
        if mano_adjustments == "":
            mano_adjustments = None

        retargeter_cfg = self.get_parameter("retarget/retargeter_cfg").value
        if retargeter_cfg == "":
            retargeter_cfg = None

        debug = self.get_parameter("debug").value

        # Get the path to the share directory of the package
        package_name = 'faive_test'
        share_dir = get_package_share_directory(package_name)

        # Define the relative path within the share directory
        relative_path = os.path.join(share_dir, 'artifacts', self.get_parameter("handposes_file").value)
        self.load_mano_handposes(relative_path)

        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.retargeter = Retargeter(
            device=device,  mjcf_filepath= mjcf_filepath, urdf_filepath=urdf_filepath, urdf_string=urdf_string,
            hand_scheme=hand_scheme, mano_adjustments=mano_adjustments, retargeter_cfg=retargeter_cfg
        )

        self.joints_pub = self.create_publisher(
            Float32MultiArray, "/hand/policy_output", 10
        )
        self.debug = debug
        if self.debug:
            self.rviz_pub = self.create_publisher(MarkerArray, 'retarget/normalized_mano_points', 10)
            self.mano_hand_visualizer = ManoHandVisualizer(self.rviz_pub)

        self.timer = self.create_timer(0.005, self.timer_publish_cb)
        self.curr_pose_start_time = time.monotonic()
        self.hand_pose_idx = 0

    def load_mano_handposes(self, filepath):
        with open(filepath, "r") as f:
            handposes_w_time_stamps = json.load(f)

        if len(handposes_w_time_stamps[0]) == 3:
            # rokoko_recorder.py format: [left, right, timestamp], use the right hand
            self.mano_handposes = [right for left, right, timestamp in handposes_w_time_stamps]
        else:
            self.mano_handposes = [handpose for handpose, timestamp in handposes_w_time_stamps]
        self.mano_handposes = [np.array(handpose, dtype=np.float32) for handpose in self.mano_handposes]
        if self.mano_handposes[0].shape[0] == 21:
            # the retargeter needs 22 keypoints: forearm, then the 21 MANO points (wrist first).
            # Older recordings have no forearm point, so place one 15 cm behind the wrist,
            # away from the middle finger's base.
            self.get_logger().warn("The hand poses have no forearm keypoint, using an approximate one")
            self.mano_handposes = [self.add_forearm_keypoint(p) for p in self.mano_handposes]

    @staticmethod
    def add_forearm_keypoint(handpose):
        direction = handpose[0] - handpose[9]
        direction /= np.linalg.norm(direction)
        return np.vstack([handpose[0] + 0.15 * direction, handpose]).astype(np.float32)

    def timer_publish_cb(self):
        pose_switch_time = 1.0  # seconds
        
        curr_time = time.monotonic()
        if curr_time - self.curr_pose_start_time > pose_switch_time:
            self.hand_pose_idx += 1
            self.hand_pose_idx = self.hand_pose_idx % len(self.mano_handposes)
            self.curr_pose_start_time = curr_time
        
        keypoint_positions = self.mano_handposes[self.hand_pose_idx]

        if self.debug:
            self.mano_hand_visualizer.reset_markers()

        debug_dict = {}
        joint_angles, debug_dict = self.retargeter.retarget(keypoint_positions, debug_dict)

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


def main(args=None):
    rclpy.init(args=args)
    node = RetargeterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()