#!/usr/bin/env python3
"""
The ros node that helps to config the retargeter_cfgs and retargeter_adjustments
compared to the original retargeter node. It offers a lot more visualization and allows hot reloading of the retargeter_cfgs and adjustments.
"""

import rclpy
from rclpy.node import Node
import torch
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from faive_system.src.retargeter.retargeter import Retargeter, retarget_utils
from faive_system.src.common.utils import numpy_to_float32_multiarray
from faive_system.src.common.enums import HandChirality
import argparse
import yaml

import os
from faive_system.src.viz.visualize_mano import ManoHandVisualizer
import time
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from std_srvs.srv import SetBool




def get_saved_keypoints(name):
    if name == "hand_open":
        return np.array([[-0.16236758,  0.11668789, -0.06047356,],
                        [-0.0181196,   0.06287539, -0.30507904,],
                        [ 0.01121986,  0.06297629, -0.32465568,],
                        [ 0.04015163,  0.09084391, -0.3238375, ],
                        [ 0.06092609,  0.10848117, -0.3375254, ],
                        [ 0.08236857,  0.13036478, -0.35411108,],
                        [ 0.017886,    0.07402944, -0.38295108,],
                        [ 0.05791114,  0.07656229, -0.40638953,],
                        [ 0.08248338,  0.08214188, -0.41987282,],
                        [ 0.10625045,  0.09017849, -0.43232024,],
                        [-0.0071618,   0.07889569, -0.38547528,],
                        [ 0.02393873,  0.07709062, -0.4215814, ],
                        [ 0.04514837,  0.07998443, -0.4446274, ],
                        [ 0.06590658,  0.08539295, -0.46619803,],
                        [-0.03151482,  0.08502817, -0.3812824, ],
                        [-0.01220944,  0.08083391, -0.42120415,],
                        [ 0.00084494,  0.08206344, -0.4467635, ],
                        [ 0.01381145,  0.08580565, -0.47126028,],
                        [-0.05206574,  0.08939457, -0.3704004, ],
                        [-0.04681917,  0.08623552, -0.40591866,],
                        [-0.0431573,   0.08481049, -0.42823035,],
                        [-0.03918179,  0.08374059, -0.45093685,]])

class RetargeterPlayerNode(Node):
    def __init__(self):
        super().__init__("retargeter_palyer_node")


        # start retargeter
        self.declare_parameter("retarget/mjcf_filepath", rclpy.Parameter.Type.STRING)
        self.declare_parameter("retarget/urdf_filepath", rclpy.Parameter.Type.STRING)
        # Preferred: pass the URDF as an in-memory string (same convention as
        # robot_state_publisher's robot_description). Avoids materializing a
        # rendered xacro on disk.
        self.declare_parameter("robot_description", "")
        self.declare_parameter("retarget/hand_scheme", rclpy.Parameter.Type.STRING)
        self.declare_parameter("retarget/mano_adjustments", "")
        self.declare_parameter("retarget/retargeter_cfg", "")
        self.declare_parameter("retarget/joint_angle_mea", 0.7)
        self.declare_parameter("debug", True)
        self.declare_parameter("retarget/publish_retarget_result", True)
        self.declare_parameter("retarget/hand_chirality", HandChirality.RIGHT)
        self.declare_parameter("retarget/use_saved_keypoint", "")


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
            urdf_filepath = None
            mjcf_filepath = None
        
        hand_scheme = self.get_parameter("retarget/hand_scheme").value
        if hand_scheme == "":
            hand_scheme = None

        self.mano_adjustments_path =  self.get_parameter("retarget/mano_adjustments").value
        self.retargeter_cfg_path = self.get_parameter("retarget/retargeter_cfg").value
        if os.path.exists(self.mano_adjustments_path):
            with open(self.mano_adjustments_path, "r") as f:
                mano_adjustments = yaml.safe_load(f)
        else:
            mano_adjustments = {
                k : dict(
                    translation = [0.0, 0.0, 0.0],
                    rotation = [0.0, 0.0, 0.0],
                    scale = [1.0, 1.0, 1.0]
                )
                for k in ["thumb", "index", "middle", "ring", "pinky"]   
            }
            with open(self.mano_adjustments_path, "w") as f:
                yaml.dump(mano_adjustments, f)

        if os.path.exists(self.retargeter_cfg_path):
            with open(self.retargeter_cfg_path, "r") as f:
                retargeter_cfg = yaml.safe_load(f)  
        else:
            retargeter_cfg = {
                "lr": 2.5,
                "loss_func_cfg": {
                    "keyvector_fingers2palm": 2.0,
                },
            }
            with open(self.retargeter_cfg_path, "w") as f:
                yaml.dump(retargeter_cfg, f)
        self.mano_adjustments_filestamp = os.path.getmtime(self.mano_adjustments_path)
        self.retargeter_cfg_filestamp = os.path.getmtime(self.retargeter_cfg_path)
        self.get_logger().info("\033[92m" + "This node support hot reloading of the retargeter_cfg and mano_adjustments. Please edit the following files." + "\033[0m")
        self.get_logger().info("\033[92m" + f"Retargeter cfg: {self.retargeter_cfg_path}" + "\033[0m" )
        self.get_logger().info("\033[92m" + f"Mano adjustments: {self.mano_adjustments_path}"  + "\033[0m" )

        debug = True
        self.publish_retarget_result = self.get_parameter("retarget/publish_retarget_result").value

        self.use_saved_keypoint = self.get_parameter("retarget/use_saved_keypoint").value
        if self.use_saved_keypoint != "":
            self.keypoint_positions = get_saved_keypoints(self.use_saved_keypoint)
            print(f"Using saved keypoints: {self.keypoint_positions}")
        else:
            # subscribe to ingress topics
            self.ingress_mano_sub = self.create_subscription(
                Float32MultiArray, f"/ingress/{self.hand_chirality}/mano", self.ingress_mano_cb, 5
            )
            self.keypoint_positions = None
        
        device = "cuda" if torch.cuda.is_available() else "cpu"

        self.retargeter = Retargeter(
            device=device, mjcf_filepath=mjcf_filepath, urdf_filepath=urdf_filepath,
            urdf_string=urdf_string,
            hand_scheme=hand_scheme, mano_adjustments=mano_adjustments, retargeter_cfg=retargeter_cfg,
            optimizer="RMSprop"
        )
        
        if self.publish_retarget_result:
            self.joints_pub = self.create_publisher(
                Float32MultiArray, f"/{self.hand_chirality}/hand/policy_output", 5
            )
        self.debug = debug
        if self.debug:
            self.rviz_pub = self.create_publisher(MarkerArray, f"retarget/{self.hand_chirality}/normalized_mano_points", 10)
            self.mano_hand_visualizer = ManoHandVisualizer(self.rviz_pub, self.retargeter.anchor_name)
            
        # Create a publisher for Marker messages with a queue size of 10
        self._marker_pub = self.create_publisher(Marker, 'mano_marker', 10)
        # Create a publisher for PoseStamped messages with a queue size of 10
        self._pose_pubs = {k : self.create_publisher(PoseStamped, f'pose_{k}', 10)
                           for k in ["all", "thumb", "index", "middle", "ring", "pinky"]}

        self.timer = self.create_timer(0.005, self.timer_publish_cb)
    
        self.set_enabled_service = self.create_service(
            SetBool, 
            f"/{self.hand_chirality}/retargeter/set_enabled",  
            self.set_enable_callback
        )

        self.enabled = True

    def ingress_mano_cb(self, msg):
        self.keypoint_positions = np.array(msg.data).reshape(-1, 3)
    
    def timer_publish_cb(self):
        if self.keypoint_positions is None:
            print("No keypoints received yet")
            time.sleep(1)
            return
        try:

            # Hot reload the retargeter_cfg and mano_adjustments
            if os.path.getmtime(self.mano_adjustments_path) != self.mano_adjustments_filestamp:
                with open(self.mano_adjustments_path, "r") as f:
                    mano_adjustments = yaml.safe_load(f)
                self.retargeter.mano_adjustments = mano_adjustments
                self.mano_adjustments_filestamp = os.path.getmtime(self.mano_adjustments_path)
                print(f"Reloaded mano adjustments from {self.mano_adjustments_path} with timestamp {self.mano_adjustments_filestamp}")
            if os.path.getmtime(self.retargeter_cfg_path) != self.retargeter_cfg_filestamp:
                with open(self.retargeter_cfg_path, "r") as f:
                    retargeter_cfg = yaml.safe_load(f)
                self.retargeter.retargeter_cfg = retargeter_cfg
                self.retargeter_cfg_filestamp = os.path.getmtime(self.retargeter_cfg_path)
                print(f"Reloaded retargeter cfg from {self.retargeter_cfg_path} with timestamp {self.retargeter_cfg_filestamp}")

            if self.debug:
                self.mano_hand_visualizer.reset_markers()
            debug_dict = dict()
            if self.enabled:
                joint_angles, debug_dict = self.retargeter.retarget(self.keypoint_positions, debug_dict)

                if self.debug:
                    self.mano_hand_visualizer.generate_hand_markers(
                        debug_dict["normalized_joint_pos"],
                        stamp=self.get_clock().now().to_msg(),
                    )
                    self.visualize_adjustments(self.retargeter.mano_adjustments, debug_dict["raw_joint_pos"], 
                                            debug_dict["model_center_and_rot"], hand_anchor_frame = "palm")
                if self.publish_retarget_result:
                    self.joints_pub.publish(
                        numpy_to_float32_multiarray(np.deg2rad(joint_angles))
                    )

                if self.debug:
                    self.mano_hand_visualizer.publish_markers()
        except Exception as e:
            print(f"Error in timer_publish_cb: {e}")
            pass

    def set_enable_callback(self, request, response):
        self.enabled = request.data
        print(f"Retargeter enabled: {self.enabled}")
        response.success = True
        return response


    def visualize_adjustments(self, mano_adjustments, joints, model_center_and_rot, hand_anchor_frame=None):
        """
        Visualize the MANO adjustments in RViz2 by publishing 6 markers and corresponding pose topics.
        
        The markers are “squares” (rendered as very thin cubes) which show:
        • The overall (all) transformation on a square originally at the origin.
        • For each finger (thumb, index, middle, ring, pinky), a square whose pose is computed
            by applying first the 'all' adjustment and then the finger-specific adjustment.
        
        Assumptions:
        - mano_adjustments is a dict with keys "all", "thumb", "index", etc.
        - Each adjustment dict may contain keys "translation" (np.ndarray shape (3,)),
            "rotation" (np.ndarray shape (3,) in radians), and "scale" (np.ndarray shape (3,)).
        - joints is an array of 3D joint coordinates; the helper function
            `retarget_utils.get_mano_joints_dict(joints)` returns a dict where each finger’s joints
            are stored in an array (with index 0 corresponding to the finger “root”).
        - This function is a method of a ROS2 node that has already created publishers:
                self._marker_pub (publishing Marker messages) and
                self._pose_pub   (publishing PoseStamped messages).
        """
        hand_anchor_frame = hand_anchor_frame if hand_anchor_frame else "hand_root"
        model_center, model_rotation = model_center_and_rot
        # Prepare a header for all published messages.
        header = Header()
        header.frame_id = hand_anchor_frame # Change to your fixed frame as needed
        header.stamp = self.get_clock().now().to_msg()

        # =========================
        # Overall "all" transformation
        # =========================
        # Extract overall adjustment parameters (with default values)
        all_adj = mano_adjustments.get("all", {})
        t_all = np.array(all_adj.get("translation", np.zeros(3)))
        r_all_angles = np.array(all_adj.get("rotation", np.zeros(3)))
        s_all = np.array(all_adj.get("scale", np.ones(3)))
        r_all = Rotation.from_euler("xyz", r_all_angles, degrees=False)

        # For visualization of "all", we consider a unit square located at the origin.
        # Because the adjustments are applied as:
        #     p' = ( p * s_all + t_all ) rotated by r_all,
        # the transformed origin becomes:
        #     p_all = r_all.apply( 0*s_all + t_all ) = r_all.apply(t_all)
        p_all = (t_all.reshape(1,3) @ model_rotation.T + model_center)[0]

        # Create the marker for the "all" square.
        marker_all = Marker()
        marker_all.header = header
        marker_all.ns = "mano_adjustments"
        marker_all.id = 0
        marker_all.type = Marker.CUBE  # Use a cube to represent a square (with z thickness set very low)
        marker_all.action = Marker.ADD

        marker_all.pose.position.x = float(p_all[0])
        marker_all.pose.position.y = float(p_all[1])
        marker_all.pose.position.z = float(p_all[2])
        # Use the rotation from r_all directly.
        quat_all = Rotation.from_matrix(r_all.as_matrix() @ model_rotation.T).as_quat()  # returns [x, y, z, w]
        marker_all.pose.orientation = Quaternion(
            x=float(quat_all[0]),
            y=float(quat_all[1]),
            z=float(quat_all[2]),
            w=float(quat_all[3])
        )
        # The marker’s scale in x and y corresponds to the scaling factors (for a unit square).
        marker_all.scale.x = float(abs(s_all[0]))*0.03
        marker_all.scale.y = float(abs(s_all[1]))*0.03
        marker_all.scale.z = float(abs(s_all[2]))*0.03  # thin square (adjust thickness as needed)

        # Set the marker color to be translucent (alpha < 1)
        marker_all.color.a = 0.5
        marker_all.color.r = 0.0
        marker_all.color.g = 1.0
        marker_all.color.b = 0.0

        # Publish the "all" marker and a corresponding pose.
        self._marker_pub.publish(marker_all)
        pose_all = PoseStamped()
        pose_all.header = header
        pose_all.pose = marker_all.pose
        self._pose_pubs['all'].publish(pose_all)

        joints = joints * s_all + t_all
        joints = joints @ r_all.as_matrix().T  # Apply rotation

        # =========================
        # Visualize each finger transformation
        # =========================
        # Get finger joint positions using a helper function (make sure retarget_utils is imported)
        joints_dict = retarget_utils.get_mano_joints_dict(joints)
        finger_names = ["thumb", "index", "middle", "ring", "pinky"]
        marker_id = 1

        for finger in finger_names:
            # For each finger, obtain the finger joints.
            # The "root" of the finger is the first joint.
            if finger not in joints_dict or len(joints_dict[finger]) == 0:
                continue
            base_point = joints_dict[finger][0]  # Finger root point (np.array with shape (3,))
            # Finger-specific adjustment (if any)
            finger_adj = mano_adjustments.get(finger, {})
            t_f = np.array(finger_adj.get("translation", np.zeros(3)))
            r_f_angles = np.array(finger_adj.get("rotation", np.zeros(3)))
            s_f = np.array(finger_adj.get("scale", np.ones(3)))
            r_f = Rotation.from_euler("xyz", r_f_angles, degrees=False)
            # In the finger adjustment, the base (zero local offset) remains the same except for an added translation.
            p_final = ((base_point + t_f).reshape(1,3) @ model_rotation.T + model_center)[0]

            # The overall orientation for the finger square is the composition of r_all and r_f.
            # (Note: scipy’s Rotation multiplication composes rotations: r_total = r_all * r_f)
            quat_total = Rotation.from_matrix(r_f.as_matrix() @ model_rotation.T).as_quat()


            marker_f = Marker()
            marker_f.header = header
            marker_f.ns = "mano_adjustments"
            marker_f.id = marker_id
            marker_id += 1
            marker_f.type = Marker.CUBE
            marker_f.action = Marker.ADD

            marker_f.pose.position.x = float(p_final[0])
            marker_f.pose.position.y = float(p_final[1])
            marker_f.pose.position.z = float(p_final[2])
            marker_f.pose.orientation = Quaternion(
                x=float(quat_total[0]),
                y=float(quat_total[1]),
                z=float(quat_total[2]),
                w=float(quat_total[3])
            )
            marker_f.scale.x = float(abs(s_f[0]))* 0.01
            marker_f.scale.y = float(abs(s_f[1]))* 0.01
            marker_f.scale.z = float(abs(s_f[2]))* 0.01

            # Give each finger a distinctive color for clarity.
            marker_f.color.a = 0.5
            if finger == "thumb":
                marker_f.color.r, marker_f.color.g, marker_f.color.b = 1.0, 0.5, 0.0
            elif finger == "index":
                marker_f.color.r, marker_f.color.g, marker_f.color.b = 0.0, 1.0, 0.0
            elif finger == "middle":
                marker_f.color.r, marker_f.color.g, marker_f.color.b = 0.0, 0.0, 1.0
            elif finger == "ring":
                marker_f.color.r, marker_f.color.g, marker_f.color.b = 1.0, 1.0, 0.0
            elif finger == "pinky":
                marker_f.color.r, marker_f.color.g, marker_f.color.b = 1.0, 0.0, 1.0

            self._marker_pub.publish(marker_f)
            # Also publish a pose topic for this finger’s square.
            pose_f = PoseStamped()
            pose_f.header = header
            pose_f.pose = marker_f.pose
            self._pose_pubs[finger].publish(pose_f)


def main(args=None):
    rclpy.init(args=args)
    node = RetargeterPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
