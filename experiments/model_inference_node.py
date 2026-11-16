#!/bin/env python3
"""
Usage:
ros2 topic pub /model_inference_command std_msgs/msg/String "{data: 'stop'}"

This node is data-driven from the inference config yaml (see
experiments/cfgs/inference_config_*.yaml). The config declares:
  - camera_topics / camera_names   : the camera streams to subscribe to
  - observations                   : model obs key -> ros topic to read it from
  - actions                        : model action key -> {topic} (hardware destination)
Keys containing "arm" are treated as a 7-dim arm pose (PoseStamped:
position + quaternion); keys containing "hand" as a Float32MultiArray of
joint angles. Single- vs dual-arm is just how many entries the dicts have,
so the same node handles both unimanual and bimanual policies.
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String as ROSString
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
from threading import Lock
from functools import partial
import cv2
from scipy.spatial.transform import Rotation

import torch
import yaml

from faive_system.src.common.ros_utils import numpy_to_float32_multiarray, float32_multiarray_to_numpy
from srl_il.export.policy import get_policy_from_ckpt
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

import h5py
import os

class CameraListener(Node):
    def __init__(self, camera_topic, name, node):
        self.camera_topic = camera_topic
        self.lock = Lock()
        self.image = None
        self.image_stamp = None
        self.name = name
        self.im_subscriber = node.create_subscription(
            Image, self.camera_topic, self.recv_im,
            QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE),
            callback_group=node.observation_callbacks,
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
        self.declare_parameter("hand_qpos_dim", 17)
        self.declare_parameter("preview", False)
        self.declare_parameter("allow_arm_init", False)
        self.declare_parameter("arm_init_pose", "")
        self.preview = self.get_parameter("preview").value
        self.allow_arm_init = self.get_parameter("allow_arm_init").value and not self.preview
        arm_init_pose = self.get_parameter("arm_init_pose").value
        self.arm_init_target = None
        if self.allow_arm_init and arm_init_pose:
            target = np.asarray(yaml.safe_load(arm_init_pose), dtype=np.float64)
            if target.shape != (7,) or not np.isfinite(target).all() or not np.isclose(
                np.linalg.norm(target[3:]), 1.0, atol=0.001
            ):
                raise ValueError("arm_init_pose must be [x, y, z, qx, qy, qz, qw] with a unit quaternion")
            self.arm_init_target = target
        self.declare_parameter("logfile_root", "")      # root directory for logging

        self.declare_parameter("inference_cfg_file", "")
        self.policy_ckpt_path = self.get_parameter("policy_ckpt_path").value
        self.hand_qpos_dim = self.get_parameter("hand_qpos_dim").value
        self.logfile_root = self.get_parameter("logfile_root").value

        self.control_freq_hz = 10
        self.max_observation_age = 2.0 / self.control_freq_hz

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
        self.actions_cfg = inference_config["actions"]
        self.arm_command_frame = inference_config.get("arm_command_frame")

        self.lock = Lock()
        self.observation_callbacks = MutuallyExclusiveCallbackGroup()

        # Relay only messages generated by this node. External messages sent
        # to the observable policy topics cannot pass through to the robot.
        self.action_pubs = {}
        self.control_pubs = {}
        for key, spec in self.actions_cfg.items():
            msg_type = PoseStamped if self._is_arm_key(key) else Float32MultiArray
            self.action_pubs[key] = self.create_publisher(msg_type, f"/policy_inference/{key}", 10)
            if not self.preview:
                self.control_pubs[key] = self.create_publisher(msg_type, spec["topic"], 1)

        # --- Observation subscribers + latest-value buffers (one per obs key) ---
        self.obs_values = {key: None for key in self.observations_cfg}
        self.obs_stamps = {key: None for key in self.observations_cfg}
        self.arm_observation_frames = {}
        self.obs_subs = {}
        for key, topic in self.observations_cfg.items():
            if self._is_arm_key(key):
                self.obs_subs[key] = self.create_subscription(
                    PoseStamped, topic, partial(self.arm_obs_callback, key=key), 10,
                    callback_group=self.observation_callbacks,
                )
            else:
                self.obs_subs[key] = self.create_subscription(
                    Float32MultiArray, topic, partial(self.hand_obs_callback, key=key), 10,
                    callback_group=self.observation_callbacks,
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
        self.arm_init_deadline = None
        self.arm_init_settled_at = None
        self.arm_init_failed = False

        # --- Command Subscriber ---
        self.command_sub = self.create_subscription(
            ROSString, "/model_inference_command", self.command_callback, 10
        )

        # --- Timer for policy execution ---
        self.policy_timer = self.create_timer(1 / self.control_freq_hz, self.run_policy_cb)
        # Maintain a step counter to name groups incrementally

    def command_callback(self, msg: ROSString):
        """Handle commands received via /model_inference_command."""
        command = msg.data.strip().lower()
        if command == "start":
            if self.arm_init_deadline is not None or self.arm_init_failed:
                self.get_logger().warn("Arm initialization is pending or failed. Finish it, retry init_arm, or reset.")
                return
            if not self.running_policy:
                self.start_logging()
                self.policy.reset_policy()
                self.policy_stamp = None
                self.running_policy = True
                self.get_logger().info("Policy started.")
        elif command == "stop":
            if self.arm_init_deadline is not None:
                self.abort_arm_init("stopped by operator")
            if self.running_policy:
                self.running_policy = False
                self.stop_logging()
                self.get_logger().info("Policy stopped.")
        elif command == "reset":
            if self.running_policy or self.arm_init_deadline is not None:
                self.get_logger().warn("Stop policy execution or arm initialization before resetting.")
                return
            self.policy.reset_policy()
            self.policy_stamp = None
            self.arm_init_failed = False
            self.get_logger().info("Policy state reset.")
        elif command == "init_arm":
            self.start_arm_init()
        elif command == "quit":
            self.get_logger().info("Exiting.")
            rclpy.try_shutdown()
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

    def start_arm_init(self):
        if not self.allow_arm_init or self.running_policy or self.arm_init_deadline is not None:
            self.get_logger().warn("init_arm requires live mode, allow_arm_init, and an idle policy.")
            return
        output = self.control_pubs.get("actions_arm")
        if (
            self.arm_init_target is None
            or not self.arm_command_frame
            or self.actions_cfg.get("actions_arm", {}).get("topic") != "/right/arm/end_effector_pose_cmd"
            or self.observations_cfg.get("qpos_arm") != "/right/arm/end_effector_pose"
            or output is None
            or output.get_subscription_count() == 0
        ):
            self.abort_arm_init("set a verified arm_init_pose and frame and check the right-arm controller connection")
            return
        self.arm_init_failed = False
        self.arm_init_settled_at = None
        self.arm_init_deadline = time.monotonic() + 90.0
        self.get_logger().info("Arm initialization started.")

    def abort_arm_init(self, reason):
        self.arm_init_deadline = None
        self.arm_init_failed = True
        self.get_logger().error(f"Arm initialization stopped: {reason}.")

    def run_arm_init(self):
        now = time.monotonic()
        if now >= self.arm_init_deadline:
            self.abort_arm_init("target not reached within 90 s")
            return
        with self.lock:
            current = self.obs_values["qpos_arm"]
            stamp = self.obs_stamps["qpos_arm"]
            frame = self.arm_observation_frames.get("qpos_arm")
        if (
            current is None or not np.isfinite(current).all()
            or not np.isclose(np.linalg.norm(current[3:]), 1.0, atol=0.001)
            or self.observation_problems({"qpos_arm": stamp})
            or frame != self.arm_command_frame
        ):
            self.abort_arm_init("right-arm feedback is missing, stale, or in a different frame")
            return

        target = self.arm_init_target
        offset = target[:3] - current[:3]
        distance = np.linalg.norm(offset)
        rotation = Rotation.from_quat(current[3:])
        correction = (Rotation.from_quat(target[3:]) * rotation.inv()).as_rotvec()
        angle = np.linalg.norm(correction)
        if distance <= 0.005 and angle <= np.deg2rad(3.0):
            if self.arm_init_settled_at is None:
                self.arm_init_settled_at = now
            elif now - self.arm_init_settled_at >= 0.3:
                self.arm_init_deadline = None
                self.get_logger().info("Arm reached init pose. Ready for start.")
            return

        self.arm_init_settled_at = None
        # Limit each requested displacement relative to the measured pose.
        # A stalled arm cannot accumulate commands further along the path.
        if distance > 0.002:
            offset *= 0.002 / distance
        if angle > 0.015:
            correction *= 0.015 / angle
        quaternion = (Rotation.from_rotvec(correction) * rotation).as_quat()
        next_pose = np.concatenate([current[:3] + offset, quaternion])
        self.control_pubs["actions_arm"].publish(
            self.pose_message(next_pose, self.get_clock().now().to_msg())
        )

    def pose_message(self, action, stamp):
        msg = PoseStamped()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = action[:3].astype(np.float64)
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = action[3:7].astype(np.float64)
        msg.header.stamp = stamp
        msg.header.frame_id = self.arm_command_frame or "panda_link0"
        return msg

    def publish(self, action_dict: dict):
        """Publish each generated action and, in live mode, relay it to the controller."""
        stamp = self.get_clock().now().to_msg()
        messages = {}
        for key, action in action_dict.items():
            if not np.isfinite(action).all():
                self.discard_actions(f"invalid {key} action")
                return False
            if self._is_arm_key(key):
                if action.shape != (7,) or not np.isclose(np.linalg.norm(action[3:]), 1.0, atol=0.001):
                    self.discard_actions(f"invalid {key} pose")
                    return False
                messages[key] = self.pose_message(action, stamp)
            else:
                messages[key] = numpy_to_float32_multiarray(action)

        self.policy_stamp = stamp
        for key, msg in messages.items():
            self.action_pubs[key].publish(msg)
            if key in self.control_pubs:
                self.control_pubs[key].publish(msg)
        return True

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
        frame = msg.header.frame_id
        if self.arm_command_frame and frame != self.arm_command_frame:
            with self.lock:
                self.obs_values[key] = None
                self.obs_stamps[key] = None
            self.get_logger().warn(
                f"{key} frame {frame!r} differs from {self.arm_command_frame!r}",
                throttle_duration_sec=2.0,
            )
            return
        with self.lock:
            self.obs_values[key] = np.concatenate([position, quaternion])
            self.obs_stamps[key] = msg.header.stamp
            self.arm_observation_frames[key] = frame

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
        images_and_stamps = {cam.name: cam.get_im() for cam in self.camera_listeners}
        with self.lock:
            state_obs = dict(self.obs_values)
            stamp_dict = dict(self.obs_stamps)
        images = {key: value[0] for key, value in images_and_stamps.items()}
        stamp_dict.update({key: value[1] for key, value in images_and_stamps.items()})
        observations = {**images, **state_obs}
        missing = [key for key, value in observations.items() if value is None]
        if missing or self.observation_problems(stamp_dict):
            return None, stamp_dict

        # Convert ROS images to numpy arrays only after all inputs are available.
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

        return {**images, **state_obs}, stamp_dict

    def observation_problems(self, stamp_dict):
        now = self.get_clock().now()
        problems = []
        for key, stamp in stamp_dict.items():
            if stamp is None:
                problems.append(f"{key} missing")
                continue
            age = (now - rclpy.time.Time.from_msg(stamp)).nanoseconds / 1e9
            if not 0.0 <= age <= self.max_observation_age:
                problems.append(f"{key} age {age:.3f} s")
        return problems

    def discard_actions(self, reason):
        self.policy.reset_policy()
        self.policy_stamp = None
        self.get_logger().warn(f"Policy output paused: {reason}", throttle_duration_sec=2.0)

    def run_policy_cb(self):
        """Compute an action only when every observation is fresh."""
        if self.arm_init_deadline is not None:
            self.run_arm_init()
            return
        if not self.running_policy:
            return

        obs_dict, stamp_dict = self.get_current_observations()
        problems = self.observation_problems(stamp_dict)
        if obs_dict is None or problems:
            self.discard_actions(", ".join(problems) or "missing observation")
            return

        dbg_info = {} if self.logfile_root != "" else None
        # Convert to torch tensors, run policy
        with torch.inference_mode():
            obs_torch = {k: torch.tensor(v).float().unsqueeze(0) for k, v in obs_dict.items()} # add batch dimension
            actions = self.policy.predict_action(obs_torch, dbg_info)
            action_dict = {key: actions[key][0].cpu().numpy() for key in self.actions_cfg}

        problems = self.observation_problems(stamp_dict)
        if problems:
            self.discard_actions(", ".join(problems))
            return

        # Warn about the gap between action publications.
        if self.policy_stamp is not None:
            time_diff = rclpy.time.Time.from_msg(self.get_clock().now().to_msg()) - rclpy.time.Time.from_msg(self.policy_stamp)
            if time_diff.nanoseconds / 1e9 > self.max_observation_age:
                self.get_logger().warn(f"Policy execution is too slow ({time_diff.nanoseconds / 1e9:.3f} s).")
        if not self.publish(action_dict):
            return

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
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.stop_logging()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
