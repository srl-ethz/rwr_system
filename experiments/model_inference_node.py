#!/bin/env python3
"""
Usage:
ros2 topic pub /model_inference_command std_msgs/msg/String "{data: 'stop'}"

This node is data-driven from the inference config yaml (see
experiments/cfgs/inference_config_*.yaml). The config declares:
  - camera_topics / camera_names   : the camera streams to subscribe to
  - observations                   : model obs key -> ros topic to read it from
  - actions                        : model action key -> {topic, init}
Keys containing "arm" are treated as a 7-dim arm pose (PoseStamped:
position + quaternion); keys containing "hand" as a Float32MultiArray of
joint angles. Single- vs dual-arm is just how many entries the dicts have,
so the same node handles both unimanual and bimanual policies.
"""
from time import sleep
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_msgs.msg import String as ROSString
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from threading import Lock
from functools import partial
import cv2
from scipy.spatial.transform import Rotation

import torch
import yaml

from faive_system.src.common.ros_utils import numpy_to_float32_multiarray, float32_multiarray_to_numpy
from faive_system.src.common.transformation_utils import rigid_transform, complete_rotation_matrix
from srl_il.export.policy import get_policy_from_ckpt
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

import h5py
import os
import threading
import sys
import select

class CameraListener(Node):
    def __init__(self, camera_topic, name, node):
        self.camera_topic = camera_topic
        self.lock = Lock()
        self.image = None
        self.image_stamp = None
        self.name = name
        self.im_subscriber = node.create_subscription(
            Image, self.camera_topic, self.recv_im,
            QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        )

    def recv_im(self, msg: Image):
        with self.lock:
            self.image = msg
            self.image_stamp = msg.header.stamp

    def get_im(self):
        with self.lock:
            return deepcopy(self.image), self.image_stamp


class PolicyPlayerAgent(Node):
    @staticmethod
    def _is_arm_key(key):
        """Arm keys carry a 7-dim PoseStamped; hand keys a Float32MultiArray."""
        return "arm" in key

    def __init__(self):
        super().__init__("policy_publisher")

        # --- Declare and get parameters ---
        self.declare_parameter("policy_ckpt_path", "")   # assume the policy ckpt is saved with its config
        self.declare_parameter("hand_qpos_dim", 17) # The dimension of the hand_qpos, we need this because we need to broadcast an all zero command to the hand at the beginning
        self.declare_parameter("logfile_root", "")      # root directory for logging

        self.declare_parameter("inference_cfg_file", "")
        self.policy_ckpt_path = self.get_parameter("policy_ckpt_path").value
        self.hand_qpos_dim = self.get_parameter("hand_qpos_dim").value
        self.logfile_root = self.get_parameter("logfile_root").value

        self.control_freq_hz = 10 #hz

        # --- Load the inference config (cameras / observation / action mapping) ---
        self.inference_cfg_file = self.get_parameter("inference_cfg_file").value
        if self.inference_cfg_file == "":
            raise RuntimeError("inference_cfg_file parameter is required")
        with open(self.inference_cfg_file, "r") as f:
            inference_config = yaml.safe_load(f)
        self.rgb_process_dict = inference_config.get("rgb_process_dict", {}) or {}
        self.resize_image_size = tuple(inference_config.get("rgb_resize_size", [224, 224]))
        self.hand_qpos_dim = inference_config.get("hand_qpos_dim", self.hand_qpos_dim)
        camera_topics = inference_config["camera_topics"]
        camera_names = inference_config["camera_names"]
        self.observations_cfg = inference_config["observations"]  # obs_key -> ros topic
        self.actions_cfg = inference_config["actions"]            # action_key -> {topic, init}

        self.lock = Lock()

        # --- Action publishers (one per action key) ---
        self.action_pubs = {}
        for key, spec in self.actions_cfg.items():
            msg_type = PoseStamped if self._is_arm_key(key) else Float32MultiArray
            self.action_pubs[key] = self.create_publisher(msg_type, spec["topic"], 10)

        # --- Observation subscribers + latest-value buffers (one per obs key) ---
        self.obs_values = {key: None for key in self.observations_cfg}
        self.obs_stamps = {key: None for key in self.observations_cfg}
        self.obs_subs = {}
        for key, topic in self.observations_cfg.items():
            if self._is_arm_key(key):
                self.obs_subs[key] = self.create_subscription(
                    PoseStamped, topic, partial(self.arm_obs_callback, key=key), 10
                )
            else:
                self.obs_subs[key] = self.create_subscription(
                    Float32MultiArray, topic, partial(self.hand_obs_callback, key=key), 10
                )

        # --- Camera listeners ---
        self.camera_listeners = [
            CameraListener(camera_topic, camera_name, self)
            for camera_topic, camera_name in zip(camera_topics, camera_names)
        ]

        # --- Utils ---
        self.bridge = CvBridge()

        # --- Load policy ---
        print(f"About to load policy")
        self.policy = get_policy_from_ckpt(self.policy_ckpt_path)
        self.policy.reset_policy()

        # --- State variables ---
        self.running_policy = False
        self.log_file = None
        self.log_filename = None
        self.step_idx = 0
        self.policy_stamp = None

        # --- Command Subscriber ---
        self.command_sub = self.create_subscription(
            ROSString, "/model_inference_command", self.command_callback, 10
        )

        print("About to go_to_default_init_state")
        # --- Initialize to default state ---
        self.go_to_default_init_state()

        # --- Timer for policy execution ---
        self.policy_timer = self.create_timer(1 / self.control_freq_hz, self.run_policy_cb)
        # Maintain a step counter to name groups incrementally

    def command_callback(self, msg: ROSString):
        """Handle commands received via the /policy_command topic."""
        print("received command", msg)
        command = msg.data.strip().lower()
        if command == "start":
            if not self.running_policy:
                self.start_logging()
                self.policy.reset_policy()
                self.running_policy = True
                self.get_logger().info("Policy started.")
        elif command == "stop":
            if self.running_policy:
                self.running_policy = False
                self.stop_logging()
                self.get_logger().info("Policy stopped.")
        elif command == "reset":
            if self.running_policy:
                self.get_logger().info("Cannot reset when policy is running")
                return
            self.get_logger().info("Resetting to default state.")
            self.go_to_default_init_state()
        elif command == "quit":
            self.get_logger().info("Exiting.")
            self.stop_logging()
            rclpy.shutdown()
            sys.exit(0)
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    def start_logging(self):
        """Start a new log file for the policy."""
        if self.log_file is not None:
            self.log_file.close()
            self.get_logger().info(f"Closed previous log file: {self.log_filename}")

        parent_dir = os.path.dirname(self.policy_ckpt_path)         # -> /.../20250117-231131_egocentric/checkpoints
        parent_parent_dir = os.path.dirname(parent_dir)             # -> /.../20250117-231131_egocentric
        experiment_name = os.path.basename(parent_parent_dir)       # -> 20250117-231131_egocentric

        ckpt_filename = os.path.basename(self.policy_ckpt_path)     # -> checkpoint_2150.pth
        ckpt_name_no_ext = os.path.splitext(ckpt_filename)[0]       # -> checkpoint_2150

        # 2) Use current time as the experiment timestamp
        current_time = time.strftime("%Y%m%d-%H%M%S")

        # 3) Construct the log filename
        self.logfile_folder = f"{experiment_name}-{ckpt_name_no_ext}"
        self.logfile_name = f"{current_time}.hdf5"
        os.makedirs(os.path.join(self.logfile_root, self.logfile_folder), exist_ok=True)
        # 4) Now open your HDF5 log file
        self.log_filename = os.path.join(self.logfile_root, self.logfile_folder, self.logfile_name)
        self.log_file = h5py.File(self.log_filename, "w")
        self.get_logger().info(f"Started new log file: {self.log_filename}")
        self.step_idx = 0  # Reset step index for the new log file

    def stop_logging(self):
        """Stop logging and close the log file."""
        if self.log_file is not None:
            self.log_file.close()
            self.get_logger().info(f"Closed log file: {self.log_filename}")
            self.log_file = None
            self.log_filename = None

    def rosimg_to_numpy_resized(self, img, crop,  K=None, D=None, rot90_k=0):
        """
        crop: top, bottom, left, right
        rot90_k, calles np.rot90, counterclockwise
        """
        img = self.bridge.imgmsg_to_cv2(img, "rgb8")
        h, w = img.shape[:2]
        DIM = (w, h)

        if K is not None and D is not None:
            K = np.array(K).reshape(3, 3)
            D = np.array(D).reshape(4, 1)

            new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K, D, DIM, np.eye(3), balance=0.1
            )
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                K, D, np.eye(3), new_K, DIM, cv2.CV_16SC2
            )
            undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)
            img = undistorted_img

        img = img[crop[0]:crop[1], crop[2]:crop[3]]

        resized = cv2.resize(img, self.resize_image_size).transpose(2, 0, 1) / 255.0
        if rot90_k != 0:
            resized = np.ascontiguousarray(np.rot90(resized, k=rot90_k, axes=(1, 2)))
        return resized

    def __del__(self):
        """Close the log file in destructor to handle Ctrl+C shutdown."""
        if hasattr(self, 'log_file') and self.log_file is not None:
            self.log_file.close()
            print(f"HDF5 file {self.log_filename} closed.")

    def default_init_actions(self):
        """Build the default init action dict from the config `init` fields.

        Arm actions store a 3-vec position in the config; the orientation uses
        the fixed default rotation. Hand actions default to zeros(hand_qpos_dim)
        when `init` is null.
        """
        action_dict = {}
        start_wrist_rot = complete_rotation_matrix(x=[1, 0, 0], z=[0, 0, -1])
        start_wrist_quat = np.array(Rotation.from_matrix(start_wrist_rot).as_quat())
        for key, spec in self.actions_cfg.items():
            init = spec.get("init", None)
            if self._is_arm_key(key):
                pos = np.array(init, dtype=np.float64)
                action_dict[key] = np.concatenate([pos, start_wrist_quat])
            else:
                if init is None:
                    action_dict[key] = np.zeros(self.hand_qpos_dim)
                else:
                    action_dict[key] = np.array(init, dtype=np.float64)
        return action_dict

    def go_to_default_init_state(self):
        """Initialize the arm(s) and hand(s) to the config-defined default state."""
        # Forget stale observations so we don't act on pre-reset state.
        with self.lock:
            for key in self.obs_values:
                self.obs_values[key] = None

        self.publish(self.default_init_actions())
        time.sleep(15)

    def publish(self, action_dict: dict):
        """Publish each action in `action_dict` to its configured ros topic."""
        self.policy_stamp = self.get_clock().now().to_msg()
        for key, action in action_dict.items():
            pub = self.action_pubs[key]
            if self._is_arm_key(key):
                # Skip invalid (effectively zero) arm poses.
                if (np.abs(action) < 0.01).all():
                    continue
                wrist_msg = PoseStamped()
                wrist_msg.pose.position.x, wrist_msg.pose.position.y, wrist_msg.pose.position.z = action[:3].astype(np.float64)
                (
                    wrist_msg.pose.orientation.x,
                    wrist_msg.pose.orientation.y,
                    wrist_msg.pose.orientation.z,
                    wrist_msg.pose.orientation.w,
                ) = action[3:7].astype(np.float64)
                wrist_msg.header.stamp = self.policy_stamp
                wrist_msg.header.frame_id = "panda_link0"
                pub.publish(wrist_msg)
            else:
                pub.publish(numpy_to_float32_multiarray(action))

    def arm_obs_callback(self, msg: PoseStamped, key: str):
        """Callback to record the current pose for an arm observation key."""
        pose = msg.pose
        position = [pose.position.x, pose.position.y, pose.position.z]
        quaternion = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        with self.lock:
            self.obs_values[key] = np.concatenate([position, quaternion])
            self.obs_stamps[key] = msg.header.stamp

    def hand_obs_callback(self, msg: Float32MultiArray, key: str):
        """Callback to record the current joint state for a hand observation key."""
        state = float32_multiarray_to_numpy(msg)
        value = np.zeros(self.hand_qpos_dim)
        value[:len(state)] = state
        with self.lock:
            self.obs_values[key] = value
            self.obs_stamps[key] = self.get_clock().now().to_msg()

    def get_current_observations(self):
        """Gather current observations from cameras, arm(s), and hand(s)."""
        obs_dict = {}
        stamp_dict = {}
        get_data_success = True

        # 1) Retrieve images
        images_and_stamps = {cam.name: cam.get_im() for cam in self.camera_listeners}
        if any([im is None for im, _ in images_and_stamps.values()]):
            get_data_success = False
            self.get_logger().error("Missing camera images "+ str([im is not None for im in images_and_stamps.values()]))
            return get_data_success, obs_dict, stamp_dict

        # Convert ROS images to numpy arrays
        images = {k: v[0] for k, v in images_and_stamps.items()}
        img_stamps = {k: v[1] for k, v in images_and_stamps.items()}
        if missing_keys:=(set(self.rgb_process_dict.keys()) - set(images.keys())):
            self.get_logger().warning(f"image keys {missing_keys} are defined in rgb_process_dict but not used. \nCheck if the match {images.keys()}" )
        for k,v in images.items():
            crop = (None,None,None,None)
            rot90_k = 0
            K, D = None, None
            if k in self.rgb_process_dict:
                for operations in self.rgb_process_dict[k]:
                    if operations["type"] == "crop":
                        # Crop the image
                        crop = (operations["top"], operations["bottom"],
                                operations["left"], operations["right"])
                    if operations["type"] == "rotate90":
                        # Rotate the image
                        rot90_k = operations["k"]
                    if operations["type"] == "undistort":
                        # Resize the image
                        K, D = operations["K"], operations["D"]
            images[k] = self.rosimg_to_numpy_resized(v, crop, K, D, rot90_k)

        # 2) Retrieve arm/hand state observations
        with self.lock:
            state_obs = {key: self.obs_values[key] for key in self.observations_cfg}
            state_stamps = {key: self.obs_stamps[key] for key in self.observations_cfg}
        for key, value in state_obs.items():
            if value is None:
                print(f"missing observation {key}")
                return False, obs_dict, stamp_dict

        # 3) Populate obs_dict
        obs_dict.update(images)
        obs_dict.update(state_obs)

        stamp_dict.update(state_stamps)
        stamp_dict.update(img_stamps)

        return get_data_success, obs_dict, stamp_dict

    def run_policy_cb(self):
        """Main callback to fetch observations, run policy, and publish commands."""
        if not self.running_policy:
            return

        get_data_success, obs_dict, stamp_dict = self.get_current_observations()
        if not get_data_success:
            self.get_logger().info("No observations available. Sleeping for 1 second.")
            sleep(1)
            return

        # Check the observation timestamps are up to date (2x control period)
        current_time = self.get_clock().now().to_msg()
        control_period_sec = 1.0 / self.control_freq_hz
        max_age_sec = 2 * control_period_sec
        for k, stamp in stamp_dict.items():
            time_diff = rclpy.time.Time.from_msg(current_time) - rclpy.time.Time.from_msg(stamp)
            if time_diff.nanoseconds / 1e9 > max_age_sec:
                self.get_logger().warn(f"Observation {k} is too old ({time_diff.nanoseconds / 1e9:.3f} s). ")

        dbg_info = {} if self.logfile_root != "" else None
        # Convert to torch tensors, run policy
        with torch.inference_mode():
            obs_torch = {k: torch.tensor(v).float().unsqueeze(0) for k, v in obs_dict.items()} # add batch dimension
            actions = self.policy.predict_action(obs_torch, dbg_info)
            action_dict = {key: actions[key][0].cpu().numpy() for key in self.actions_cfg}

        # check if the policy calculation took too long
        if self.policy_stamp is not None:
            time_diff = rclpy.time.Time.from_msg(self.get_clock().now().to_msg()) - rclpy.time.Time.from_msg(self.policy_stamp)
            if time_diff.nanoseconds / 1e9 > max_age_sec:
                self.get_logger().warn(f"Policy execution is too slow ({time_diff.nanoseconds / 1e9:.3f} s).")
        self.publish(action_dict)

        # Log the actions if logging is enabled
        if self.log_file is not None:
            # Log actions in actions/{key} format
            action_group = self.log_file.create_group(f"step_{self.step_idx}/actions")
            for k, v in action_dict.items():
                action_group.create_dataset(f"{k}", data=v)

            # Log observations in obs_{k} format
            obs_group = self.log_file.create_group(f"step_{self.step_idx}/observations")
            for k, v in obs_dict.items():
                obs_group.create_dataset(f"{k}", data=v)

            # Log debug information in dbg/{k} format (if available)
            if dbg_info is not None:
                dbg_group = self.log_file.create_group(f"step_{self.step_idx}/dbg")
                for k, v in dbg_info.items():
                    if isinstance(v, torch.Tensor) or isinstance(v, np.ndarray):
                        if isinstance(v, torch.Tensor):
                            v = v.detach().cpu()
                        dbg_group.create_dataset(f"{k}", data=v)
            self.log_file.flush()
            self.step_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = PolicyPlayerAgent()
    import threading

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.command_callback(ROSString(data = "start") )

    spin_thread.join()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
