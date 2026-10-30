#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import torch
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import SetBool
from faive_system.src.retargeter.retargeter import Retargeter
from faive_system.src.common.utils import numpy_to_float32_multiarray
from faive_system.src.common.enums import HandChirality
from std_msgs.msg import Bool

import os
from faive_system.src.viz.visualize_mano import ManoHandVisualizer
import time

class RetargeterNode(Node):
    def __init__(self, debug=False):
        super().__init__("retargeter_node")

        # start retargeter
        self.declare_parameter("retarget/mjcf_filepath", rclpy.Parameter.Type.STRING)
        self.declare_parameter("retarget/urdf_filepath", rclpy.Parameter.Type.STRING)
        # Preferred: pass the URDF as an in-memory string (same convention as
        # robot_state_publisher's robot_description). Avoids materializing a
        # rendered xacro on disk. If non-empty, urdf_filepath/mjcf_filepath
        # are ignored.
        self.declare_parameter("robot_description", "")
        self.declare_parameter("retarget/hand_scheme", rclpy.Parameter.Type.STRING)
        self.declare_parameter("retarget/mano_adjustments", "")
        self.declare_parameter("retarget/retargeter_cfg", "")
        self.declare_parameter("retarget/joint_angle_mea", 0.7)
        self.declare_parameter("retarget/optimizer", "RMSprop")
        self.declare_parameter("enable_at_start", True)
        self.declare_parameter("debug", True)
        self.declare_parameter("retarget/hand_chirality", HandChirality.RIGHT)
        self.hand_chirality = self.get_parameter("retarget/hand_chirality").value

        urdf_string = self.get_parameter("robot_description").value or None
        try:
            mjcf_filepath = self.get_parameter("retarget/mjcf_filepath").value
        except:
            mjcf_filepath = None
        try:
            urdf_filepath = self.get_parameter("retarget/urdf_filepath").value
        except:
            urdf_filepath = None
        if urdf_string is not None:
            # robot_description wins over urdf_filepath/mjcf_filepath.
            urdf_filepath = None
            mjcf_filepath = None
        
        hand_scheme = self.get_parameter("retarget/hand_scheme").value
        if hand_scheme == "":
            hand_scheme = None
        # mano_adjustments's default value is None
        mano_adjustments = self.get_parameter("retarget/mano_adjustments").value
        if mano_adjustments == "":
            mano_adjustments = None

        retargeter_cfg = self.get_parameter("retarget/retargeter_cfg").value
        if retargeter_cfg == "":
            retargeter_cfg = None

        optimizer = self.get_parameter("retarget/optimizer").value
        if optimizer == "":
            optimizer = "RMSprop"

        self._joint_angle_mea = self.get_parameter("retarget/joint_angle_mea").value
        self._joint_angles = 0.0

        debug = self.get_parameter("debug").value
        
        # subscribe to ingress topics
        self.ingress_mano_sub = self.create_subscription(
            Float32MultiArray, f"/ingress/{self.hand_chirality}/mano", self.ingress_mano_cb, 5
        )
        
        device = "cuda" if torch.cuda.is_available() else "cpu"

        self.retargeter = Retargeter(
            device=device, mjcf_filepath=mjcf_filepath, urdf_filepath=urdf_filepath,
            urdf_string=urdf_string,
            hand_scheme=hand_scheme, mano_adjustments=mano_adjustments, retargeter_cfg=retargeter_cfg,
            optimizer=optimizer,
        )
        
        self.joints_pub = self.create_publisher(
            Float32MultiArray, f"/{self.hand_chirality}/hand/policy_output", 5
        )

        self.set_enabled_service = self.create_service(
            SetBool, 
            f"/{self.hand_chirality}/retargeter/set_enabled",  
            self.set_enable_callback
        )

        self.debug = debug
        if self.debug:
            self.rviz_pub = self.create_publisher(MarkerArray, f"retarget/{self.hand_chirality}/normalized_mano_points", 10)
            self.mano_hand_visualizer = ManoHandVisualizer(self.rviz_pub, self.retargeter.anchor_name)
            
        self.timer = self.create_timer(0.005, self.timer_publish_cb)
        self.keypoint_positions = None
        self.enabled = self.get_parameter("enable_at_start").value
    
    def ingress_mano_cb(self, msg):
        self.keypoint_positions = np.array(msg.data).reshape(-1, 3)

    def set_enable_callback(self, request, response):
        self.enabled = request.data
        print(f"Retargeter enabled: {self.enabled}")
        response.success = True
        return response
 

    def timer_publish_cb(self):
        if self.keypoint_positions is None:
            self.get_logger().warn("No keypoints received yet")
            time.sleep(1)
            return
        try:
            if self.debug:
                self.mano_hand_visualizer.reset_markers()

            debug_dict = dict()
            joint_angles, debug_dict = self.retargeter.retarget(self.keypoint_positions, debug_dict)
            self._joint_angles = joint_angles*self._joint_angle_mea + self._joint_angles*(1-self._joint_angle_mea)

            if self.debug:
                self.mano_hand_visualizer.generate_hand_markers(
                    debug_dict["normalized_joint_pos"],
                    stamp=self.get_clock().now().to_msg(),
                )
            if self.enabled:
                # if retargeter is disabled, do not publish it
                # (but still run the retargeter calculations so it can smoothly be re-enabled)
                self.joints_pub.publish(
                    numpy_to_float32_multiarray(np.deg2rad(self._joint_angles))
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
