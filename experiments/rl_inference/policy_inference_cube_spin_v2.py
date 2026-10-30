#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Policy Inference Node for the ORCA v2 hand - runs an rsl_rl policy trained in faive_lab
(task orca-v2-inhand-repose-v0, proprioception only) on the real hand.

The policy is the TorchScript file that faive_lab's play.py exports (logs/rsl_rl/orca_v2_repose/<run>/exported/policy.pt);
it includes the observation normalisation, so it takes the raw observation vector.

Every 1/policy_hz seconds:
1. Read the hand's joint positions and velocities (radians, in the order of the hand's config.yaml joint_ids)
2. Build the policy observation exactly as in the simulation (USD joint order, 74 values):
       joint_pos       joint positions normalised to [-1, 1] over the joint limits
       joint_vel       joint velocities * 0.2
       goal_command    [0, 0, -0.05, 0, 0, angular velocity about z]
       last_action     the policy's previous raw output
       last_joint_cmd  the previous joint position command (radians)
3. Run the policy, map its output onto the joint limits and smooth it with the same moving average as the simulation
   (target = alpha * new + (1 - alpha) * previous)
4. Publish the joint position command on /right/hand/policy_output, which hand_control_node executes (unless dry_run)

Subtask 2 (exteroception): every step the node also reads the cube pose from TF (hand_root -> cube, published by
cube_spin_v2.launch.py and cube_detection_node) and converts it to object_pos / object_quat as in the simulation. They are
logged, but not part of the observation (commented out in process_once).

The joint limits, observation order and scales below must match the training task (orca_v2_repose_env_cfg.py).

Usage (hand driver running, e.g. `ros2 launch hand_control orcahand_v2_controller_launch.yaml`):
    python policy_inference_cube_spin_v2.py --ros-args -p policy_ckpt_path:=/path/to/exported/policy.pt
"""

import csv
from pathlib import Path

import numpy as np
import rclpy
import rclpy.executors
import tf2_ros
import torch
import yaml
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float32MultiArray

from faive_system.src.common.ros_utils import numpy_to_float32_multiarray, float32_multiarray_to_numpy

DEFAULT_HAND_CONFIG = (
    Path(__file__).resolve().parents[2] / "src" / "hand_control" / "orca" / "configs" / "orcahand_v2" / "right" / "config.yaml"
)

# Joint order of the v2 USD model in Isaac Lab (= order of the policy's observations and actions), with the joint limits
# the task normalises over (soft limits of the v2 USD, degrees).
JOINT_LIMITS_USD_DEG = {
    "wrist": (-65.0, 35.0),
    "index_abd": (-25.0, 30.0),
    "middle_abd": (-27.0, 27.0),
    "pinky_abd": (-30.0, 30.0),
    "ring_abd": (-27.0, 27.0),
    "thumb_cmc": (-45.0, 33.0),
    "index_mcp": (-25.0, 100.0),
    "middle_mcp": (-25.0, 100.0),
    "pinky_mcp": (-25.0, 100.0),
    "ring_mcp": (-25.0, 100.0),
    "thumb_abd": (-18.0, 55.0),
    "index_pip": (-15.0, 107.0),
    "middle_pip": (-15.0, 107.0),
    "pinky_pip": (-15.0, 107.0),
    "ring_pip": (-15.0, 107.0),
    "thumb_mcp": (-25.0, 100.0),
    "thumb_dip": (-15.0, 107.0),
}
OBS_DIM = 17 + 17 + 6 + 17 + 17


class PolicyInferenceNodeV2(Node):
    def __init__(self):
        super().__init__("policy_inference_node_v2")

        # ========== ROS2 Parameters ==========
        self.declare_parameter("policy_ckpt_path", "")
        self.declare_parameter("hand_config_path", str(DEFAULT_HAND_CONFIG))
        self.declare_parameter("policy_hz", 20.0)          # the task runs the policy at 20 Hz (decimation 6, dt 1/120)
        self.declare_parameter("action_alpha", 0.2)        # moving average of the task's action term
        self.declare_parameter("velocity_obs_scale", 0.2)  # joint_vel observation scale in the task
        self.declare_parameter("goal_ang_vel", 1.0)        # rad/s about z; trained on |w| in [0.9, 1.5] (either sign = direction)
        self.declare_parameter("goal_flip_period_s", 0.0)  # > 0: flip the spin direction every this many seconds
        self.declare_parameter("dry_run", False)           # true: compute and log actions, but do not command the hand
        self.declare_parameter("log_period_s", 1.0)        # print the observation summary this often (0: never)
        self.declare_parameter("csv_log_path", "")         # non-empty: log observations and commands to this CSV
        self.declare_parameter("command_topic", "/right/hand/policy_output")  # what hand_control_node subscribes to

        # Frame IDs
        self.declare_parameter("hand_frame_id", "hand_root")
        self.declare_parameter("cube_frame_id", "cube")
        self.declare_parameter("cube_pose_timeout_s", 0.5)  # an older cube pose counts as not tracked

        self.policy_ckpt_path = self.get_parameter("policy_ckpt_path").value
        self.policy_hz = float(self.get_parameter("policy_hz").value)
        self.action_alpha = float(self.get_parameter("action_alpha").value)
        self.velocity_obs_scale = float(self.get_parameter("velocity_obs_scale").value)
        self.goal_flip_period_s = float(self.get_parameter("goal_flip_period_s").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.log_period_s = float(self.get_parameter("log_period_s").value)
        self.hand_frame = self.get_parameter("hand_frame_id").value
        self.cube_frame = self.get_parameter("cube_frame_id").value
        self.cube_pose_timeout_s = float(self.get_parameter("cube_pose_timeout_s").value)

        # ========== Joint orders and limits ==========
        # the hand driver publishes and accepts joints in the order of config.yaml's joint_ids
        hand_config = yaml.safe_load(Path(self.get_parameter("hand_config_path").value).read_text())
        self.joint_order = list(hand_config["joint_ids"])
        self.joint_order_usd = list(JOINT_LIMITS_USD_DEG)
        assert set(self.joint_order) == set(self.joint_order_usd), (
            f"hand config joints {self.joint_order} do not match the policy's joints {self.joint_order_usd}"
        )
        self.robot_to_usd_idx = [self.joint_order.index(name) for name in self.joint_order_usd]
        self.usd_to_robot_idx = [self.joint_order_usd.index(name) for name in self.joint_order]
        self.limits_usd = np.deg2rad(np.array(list(JOINT_LIMITS_USD_DEG.values()), dtype=np.float32))

        # ========== Policy ==========
        if not self.policy_ckpt_path:
            raise ValueError("Set policy_ckpt_path to the exported policy.pt (faive_lab logs/.../exported/policy.pt)")
        self.policy = torch.jit.load(self.policy_ckpt_path, map_location="cpu").eval()
        with torch.no_grad():
            out = self.policy(torch.zeros(1, OBS_DIM))
        assert out.shape[-1] == 17, f"policy outputs {out.shape[-1]} actions, expected 17"
        self.get_logger().info(f"Loaded policy {self.policy_ckpt_path} ({OBS_DIM} observations -> 17 actions)")

        # ========== ROS2 Publishers and Subscribers ==========
        self.hand_pub = self.create_publisher(Float32MultiArray, self.get_parameter("command_topic").value, 10)
        self.create_subscription(Float32MultiArray, "/right/hand/joint_position_from_motor", self.hand_pos_callback, 10)
        self.create_subscription(Float32MultiArray, "/right/hand/joint_velocity_from_motor", self.hand_vel_callback, 10)

        # ========== TF2 Listener ==========
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.last_object_quat = None

        # ========== State (USD order) ==========
        self.current_joint_pos = None  # robot order, radians
        self.current_joint_vel = None
        self.last_action_usd = np.zeros(17, dtype=np.float32)  # raw policy output, as the task's last_action
        self.last_joint_cmd_usd = None                          # set from the measured pose on the first step, as at a reset
        self.goal_command = np.array([0.0, 0.0, -0.05, 0.0, 0.0, float(self.get_parameter("goal_ang_vel").value)],
                                     dtype=np.float32)
        self.start_time = None
        self.last_flip_time = None
        self.last_log_time = None

        self.csv_writer = None
        csv_log_path = self.get_parameter("csv_log_path").value
        if csv_log_path:
            self.csv_file = open(csv_log_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["time"] + [f"obs_{i}" for i in range(OBS_DIM)]
                                     + [f"cmd_{name}" for name in self.joint_order_usd]
                                     + ["cube_x", "cube_y", "cube_z", "cube_qw", "cube_qx", "cube_qy", "cube_qz"])

        self.create_timer(1.0 / self.policy_hz, self.process_once)
        mode = "DRY RUN (not commanding the hand)" if self.dry_run else "COMMANDING THE HAND"
        self.get_logger().info(f"Policy inference v2 running at {self.policy_hz} Hz, alpha {self.action_alpha}: {mode}")

    def hand_pos_callback(self, msg: Float32MultiArray):
        self.current_joint_pos = float32_multiarray_to_numpy(msg)

    def hand_vel_callback(self, msg: Float32MultiArray):
        self.current_joint_vel = float32_multiarray_to_numpy(msg)

    def normalize(self, joint_pos_usd: np.ndarray) -> np.ndarray:
        """Isaac Lab's scale_transform (joint_pos_limit_normalized): limits -> [-1, 1], not clipped."""
        low, high = self.limits_usd[:, 0], self.limits_usd[:, 1]
        return (2.0 * (joint_pos_usd - (low + high) / 2) / (high - low)).astype(np.float32)

    def denormalize(self, action_usd: np.ndarray) -> np.ndarray:
        """Isaac Lab's unscale_transform (rescale_to_limits): [-1, 1] -> limits."""
        low, high = self.limits_usd[:, 0], self.limits_usd[:, 1]
        return (low + (action_usd + 1.0) / 2 * (high - low)).astype(np.float32)

    def lookup_transform_from_tf(self, parent_frame, child_frame):
        """Latest parent_T_child from TF as a 4x4 matrix with its time stamp (seconds), or (None, None)."""
        try:
            tf_msg = self.tf_buffer.lookup_transform(parent_frame, child_frame, Time())
        except tf2_ros.TransformException:
            return None, None
        t, q = tf_msg.transform.translation, tf_msg.transform.rotation
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()  # ROS order is (x, y, z, w)
        transform[:3, 3] = [t.x, t.y, t.z]
        return transform, Time.from_msg(tf_msg.header.stamp).nanoseconds * 1e-9

    def process_once(self):
        if self.current_joint_pos is None or self.current_joint_vel is None:
            self.get_logger().info("Waiting for hand state...", throttle_duration_sec=2.0)
            return
        joint_pos_usd = np.asarray(self.current_joint_pos, dtype=np.float32)[self.robot_to_usd_idx]
        joint_vel_usd = np.asarray(self.current_joint_vel, dtype=np.float32)[self.robot_to_usd_idx]
        if not (np.all(np.isfinite(joint_pos_usd)) and np.all(np.isfinite(joint_vel_usd))):
            self.get_logger().warn("Non-finite joint state, skipping this step", throttle_duration_sec=1.0)
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            # as at a reset in the task: the moving average starts from the current pose, last action is zero
            self.start_time = self.last_flip_time = self.last_log_time = now
            self.last_joint_cmd_usd = joint_pos_usd.copy()
        if self.goal_flip_period_s > 0 and now - self.last_flip_time >= self.goal_flip_period_s:
            self.goal_command[5] *= -1.0
            self.last_flip_time = now
            self.get_logger().info(f"Spin direction flipped: goal angular velocity {self.goal_command[5]:+.2f} rad/s")

        # === cube pose (Subtask 2) ===
        object_pos, object_quat = None, None  # stay None while the cube is not tracked
        hand_T_cube, stamp = self.lookup_transform_from_tf(self.hand_frame, self.cube_frame)
        if hand_T_cube is not None and now - stamp <= self.cube_pose_timeout_s:
            # If in your simulation environment you use `mdp.root_pos_w` for the cube position,
            # you need to represent your cube position in the simulator's world frame.
            # Read your robot position from the env cfg and set the obs_T_hand accordingly.
            # Here: robot init_state pos (0.0004, 0.1512, -0.0187), rot (w, x, y, z) (-0.2607, 0.9654, 0, 0) of the
            # root link right_forearm, and hand_root -> forearm 90 deg about x (orcahand_v2.urdf.xacro): 120.2 deg about x.
            # If your simulator observes the cube relative to the hand instead, note that the robot's root link is
            # forearm, not hand_root: use forearm_T_hand_root (90 deg about -x) @ hand_T_cube.
            obs_T_hand = np.array([
                [1.0,  0.0,     0.0,     0.0004],
                [0.0, -0.5034, -0.8641,  0.1512],
                [0.0,  0.8641, -0.5034, -0.0187],
                [0.0,  0.0,     0.0,     1.0],
            ])
            obs_T_cube = obs_T_hand @ hand_T_cube

            object_pos = obs_T_cube[:3, 3].astype(np.float32)
            # Convert quaternion from ROS (x,y,z,w) to policy (w,x,y,z)
            x, y, z, w = Rotation.from_matrix(obs_T_cube[:3, :3]).as_quat()
            object_quat = np.array([w, x, y, z], dtype=np.float32)
            # q and -q are the same rotation; the simulation's quaternion changes continuously, so keep the previous sign
            if self.last_object_quat is not None and np.dot(object_quat, self.last_object_quat) < 0:
                object_quat = -object_quat
            self.last_object_quat = object_quat

        # === observation, in the task's order ===
        obs = np.concatenate([
            self.normalize(joint_pos_usd),
            joint_vel_usd * self.velocity_obs_scale,
            # object_pos,
            # object_quat,
            self.goal_command,
            self.last_action_usd,
            self.last_joint_cmd_usd,
        ]).astype(np.float32)

        # === policy ===
        with torch.no_grad():
            action_usd = self.policy(torch.from_numpy(obs).unsqueeze(0))[0].numpy().astype(np.float32)
        self.last_action_usd = action_usd

        # === action processing, as the task's EMAJointPositionToLimitsAction ===
        target_usd = self.denormalize(np.clip(action_usd, -1.0, 1.0))
        target_usd = self.action_alpha * target_usd + (1.0 - self.action_alpha) * self.last_joint_cmd_usd
        target_usd = np.clip(target_usd, self.limits_usd[:, 0], self.limits_usd[:, 1]).astype(np.float32)
        self.last_joint_cmd_usd = target_usd

        if not self.dry_run:
            self.hand_pub.publish(numpy_to_float32_multiarray(target_usd[self.usd_to_robot_idx]))
        tracked = object_pos is not None
        if self.csv_writer is not None:
            cube = np.concatenate([object_pos, object_quat]).tolist() if tracked else [float("nan")] * 7
            self.csv_writer.writerow([now - self.start_time] + obs.tolist() + target_usd.tolist() + cube)
        if self.log_period_s > 0 and now - self.last_log_time >= self.log_period_s:
            self.last_log_time = now
            norm = self.normalize(joint_pos_usd)
            cube = (f"pos {np.round(object_pos, 3)} quat {np.round(object_quat, 2)}" if tracked else "not tracked")
            self.get_logger().info(
                "joint_pos (normalised): " + " ".join(f"{n}={v:+.2f}" for n, v in zip(self.joint_order_usd, norm))
                + f" | |action| max {np.abs(action_usd).max():.2f} | cube {cube}"
            )

    def destroy_node(self):
        if self.csv_writer is not None:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PolicyInferenceNodeV2()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\nShutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():  # a signal may already have shut the context down
            rclpy.shutdown()


if __name__ == "__main__":
    main()
