#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cube Detection Node - Real-time cube pose estimation using MaskRCNN and PnP

This node performs the following tasks:
1. Subscribes to camera images and intrinsics
2. Detects cube keypoints using MaskRCNN
3. Estimates 6D pose of the cube using robust PnP (RANSAC + refinement)
4. Publishes TF transformation: camera_frame → cube

PUBLISHED TF FRAMES:
====================
    camera_frame → cube (dynamic, detected cube pose in camera frame)

The cube pose in world frame is computed by other nodes using the chain:
    world → tag (from launch file) → camera (via calibration) → cube (from this node)
"""

import os

from faive_system.src.common.ros_utils import restore_library_path_on_macos

restore_library_path_on_macos()

import numpy as np
import cv2

# Apple Silicon: Mask R-CNN's nms has no MPS implementation, run it on the CPU. Must be set before importing torch.
os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")
import torch

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

# For lightweight pose filtering
import time

# === pose_estimation ===
from pose_estimation.maskrcnn_detection import maskrcnn_detection

# === utils === 
from rotation_utils import (
    rotation_matrix_to_quaternion,
    create_transform_matrix,
)

# Import shared camera listener
from camera_utils import CameraListener


class CubeDetectionNode(Node):
    """
    ROS2 node for cube detection and pose estimation.
    
    Pipeline:
    1. Receive camera images
    2. Detect cube keypoints with MaskRCNN
    3. Estimate 6D pose using robust PnP (RANSAC + iterative refinement)
    4. Publish TF: camera_frame → cube
    """
    
    def __init__(self):
        super().__init__("cube_detection_node")

        # ========== ROS2 Parameters ==========
        self.declare_parameter("model_path", "")
        self.declare_parameter("camera_topic", '/oakd_bottom_view/color')
        self.declare_parameter("camera_name", 'oakd_bottom_view')
        self.declare_parameter("cube_edge_length", 0.045)  # meters
        self.declare_parameter("process_hz", 20.0)
        self.declare_parameter("keypoint_order", [0,1,2,3,4,5,6,7])
        self.declare_parameter("show_windows", True)
        self.declare_parameter("cube_frame_id", "cube")
        self.declare_parameter("use_kalman_filter", True)
        self.declare_parameter("kalman_process_noise", 1e-3)
        self.declare_parameter("kalman_measurement_noise", 3e-3)
        self.declare_parameter("orientation_slerp_alpha", 0.25)
        
        # Visualization
        self.declare_parameter("publish_marker", True)
        self.declare_parameter("mesh_resource", "package://viz/models/dexcube/dex_cube_instanceable.dae")
        
        # Robust PnP parameters
        self.declare_parameter("ransac_iterations", 5000)
        self.declare_parameter("ransac_reprojection_error",3.0)  # 3 pixels is reasonable for noisy keypoints
        self.declare_parameter("ransac_confidence", 0.99)
        self.declare_parameter("min_inliers", 3)

        # Get parameter values
        model_path = self.get_parameter("model_path").value
        self.camera_topic = self.get_parameter("camera_topic").value
        self.camera_name = self.get_parameter("camera_name").value
        self.cube_L = float(self.get_parameter("cube_edge_length").value)
        self.process_hz = float(self.get_parameter("process_hz").value)
        self.kpt_order = list(self.get_parameter("keypoint_order").value)
        self.show_windows = bool(self.get_parameter("show_windows").value)
        self.cube_frame = self.get_parameter("cube_frame_id").value
        self.use_kalman = bool(self.get_parameter("use_kalman_filter").value)
        self.kalman_q = float(self.get_parameter("kalman_process_noise").value)
        self.kalman_r = float(self.get_parameter("kalman_measurement_noise").value)
        self.orientation_alpha = float(self.get_parameter("orientation_slerp_alpha").value)
        self.publish_marker = bool(self.get_parameter("publish_marker").value)
        self.mesh_resource = self.get_parameter("mesh_resource").value
        
        self.ransac_iters = int(self.get_parameter("ransac_iterations").value)
        self.ransac_reproj_err = float(self.get_parameter("ransac_reprojection_error").value)
        self.ransac_confidence = float(self.get_parameter("ransac_confidence").value)
        self.min_inliers = int(self.get_parameter("min_inliers").value)

        # ========== Load MaskRCNN Detection Model ==========
        self.get_logger().info(f"Loading detection model from: {model_path}")
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        elif torch.backends.mps.is_available():
            self.device = torch.device("mps")
        else:
            self.device = torch.device("cpu")
        self.model = maskrcnn_detection.load_from_checkpoint(model_path).to(self.device)
        self.model.eval()

        # ========== Camera Listener ==========
        self.get_logger().info(f"Creating camera listener for {self.camera_name}: {self.camera_topic}")
        self.listener = CameraListener(self, self.camera_topic, self.camera_name)
        
        # ========== TF Broadcasting and Visualization ==========
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, "~/cube_markers", 1)
        self.mesh_scale_xyz = (self.cube_L/0.06, self.cube_L/0.06, self.cube_L/0.06)  # Scale to actual size (mesh is 6cm)

        # ========== Processing Timer ==========
        period = 1.0 / max(1e-3, self.process_hz)
        self.timer = self.create_timer(period, self.process_once)

        # ========== Cube Model for PnP ==========
        L = self.cube_L
        model_corners = np.array([
            [ L/2,  L/2,  L/2],
            [-L/2,  L/2,  L/2],
            [ L/2, -L/2,  L/2],
            [-L/2, -L/2,  L/2],
            [ L/2,  L/2, -L/2],
            [-L/2,  L/2, -L/2],
            [ L/2, -L/2, -L/2],
            [-L/2, -L/2, -L/2],
        ], dtype=np.float64)
        self.model_pts = model_corners[self.kpt_order, :]

        # ========== Pose tracking for initial guess ==========
        self.last_valid_pose = None  # Store last valid pose for PnP initial guess
        self.last_pose_time = None   # Timestamp of last valid pose
        self.pose_timeout = 2.0      # Reset pose if not updated for 2 seconds
        self.consecutive_failures = 0  # Track consecutive detection failures
        self.max_failures_before_reset = 5  # Reset last pose after N failures
        
        # ========== Performance tracking ==========
        self.last_processed_frame_time = None  # Track last frame timestamp to avoid reprocessing
        self.frame_count = 0

        # ========== Kalman filter for translation ==========
        self.kf = None
        self.kf_initialized = False
        self.kf_last_time = None
        self.last_filtered_quat = None

        self.get_logger().info("Cube detection node initialized successfully")
        self.get_logger().info(f"Detection device: {self.device}")
        self.get_logger().info(f"Process rate: {self.process_hz} Hz")

    def _init_kalman(self, pos_xyz):
        """Initialize constant-velocity Kalman filter for position."""
        self.kf = cv2.KalmanFilter(6, 3)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ], dtype=np.float32)
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * self.kalman_q
        self.kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * self.kalman_r
        self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 1e-2

        self.kf.statePost = np.array([
            [pos_xyz[0]],
            [pos_xyz[1]],
            [pos_xyz[2]],
            [0.0],
            [0.0],
            [0.0],
        ], dtype=np.float32)
        self.kf_initialized = True
        self.kf_last_time = time.time()

    def _predict_kalman(self, dt):
        """Predict step with variable dt."""
        self.kf.transitionMatrix = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ], dtype=np.float32)
        return self.kf.predict()

    def _update_kalman(self, pos_xyz):
        meas = np.array([[pos_xyz[0]], [pos_xyz[1]], [pos_xyz[2]]], dtype=np.float32)
        return self.kf.correct(meas)

    def _slerp_quat(self, q0, q1, alpha):
        """Fast SLERP between quaternions q0 -> q1."""
        q0 = np.array(q0, dtype=np.float64)
        q1 = np.array(q1, dtype=np.float64)
        dot = np.dot(q0, q1)
        if dot < 0.0:
            q1 = -q1
            dot = -dot
        dot = np.clip(dot, -1.0, 1.0)
        if dot > 0.9995:
            q = q0 + alpha * (q1 - q0)
            return q / np.linalg.norm(q)
        theta_0 = np.arccos(dot)
        sin_theta_0 = np.sin(theta_0)
        theta = theta_0 * alpha
        sin_theta = np.sin(theta)
        s0 = np.sin(theta_0 - theta) / sin_theta_0
        s1 = sin_theta / sin_theta_0
        q = (s0 * q0) + (s1 * q1)
        return q / np.linalg.norm(q)

    def process_once(self):
        """
        Main processing loop:
        1. Get camera frame
        2. Detect cube keypoints
        3. Estimate pose with robust PnP
        4. Publish TF: camera_frame → cube
        """
        # === Step 1: Get Latest Camera Frame ===
        img_bgr, hdr = self.listener.get_frame()
        if img_bgr is None or not self.listener.is_ready():
            return
        
        # Check if this is a new frame (avoid processing same frame multiple times)
        frame_time = hdr.stamp.sec + hdr.stamp.nanosec * 1e-9
        if self.last_processed_frame_time is not None and frame_time == self.last_processed_frame_time:
            return  # Skip, already processed this frame
        self.last_processed_frame_time = frame_time
        self.frame_count += 1
        
        # Reset last valid pose if too old (prevents getting stuck with stale pose)
        if self.last_pose_time is not None and (frame_time - self.last_pose_time) > self.pose_timeout:
            self.get_logger().info(f"[{self.listener.name}] Resetting last pose (timeout)")
            self.last_valid_pose = None
            self.last_pose_time = None
            self.consecutive_failures = 0

        # === Step 2: Run MaskRCNN Detection ===
        rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        rgb_tensor = (torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0).to(self.device)

        with torch.no_grad():
            output = self.model.get_detections([rgb_tensor])[0]

        is_failure = False
        if output is None or len(output["keypoints"]) == 0:
            self.get_logger().debug(f"[{self.listener.name}] No cube detected")
            self.consecutive_failures += 1
            is_failure = True
        scores = output["scores"].detach().cpu()
        if len(scores) == 0:
            is_failure = True
        else:
            best_idx = int(scores.argmax().item())
            if  scores[best_idx].item() < 0.3:
                is_failure = True
        
        if self.consecutive_failures >= self.max_failures_before_reset:
            self.get_logger().info(f"[{self.listener.name}] Too many failures, resetting last pose")
            self.last_valid_pose = None
            self.last_pose_time = None
            self.consecutive_failures = 0
        if is_failure:
            return
        # === Step 3: Extract Keypoints and Run Robust PnP ===
        kpts = output["keypoints"].cpu().numpy()[0]
        pts3d = self.model_pts.astype(np.float32)
        pts2d = kpts[:, :2].astype(np.float32)
        K = self.listener.mtx.astype(np.float32)
        dist = None if self.listener.dist is None else self.listener.dist.astype(np.float32)

        # RANSAC for outlier rejection
        ret, rvec, tvec, inliers = cv2.solvePnPRansac(
            objectPoints=pts3d,
            imagePoints=pts2d,
            cameraMatrix=K,
            distCoeffs=dist,
            iterationsCount=self.ransac_iters,
            reprojectionError=self.ransac_reproj_err,
            confidence=self.ransac_confidence,
            flags=cv2.SOLVEPNP_AP3P
        )

        still_success = True
        if not ret or inliers is None or len(inliers) < self.min_inliers:
            self.get_logger().warn(f"[{self.listener.name}] PnP RANSAC failed or too few inliers")
            self.consecutive_failures += 1
            if self.consecutive_failures >= self.max_failures_before_reset:
                self.get_logger().info(f"[{self.listener.name}] Too many RANSAC failures, resetting last pose")
                self.last_valid_pose = None
                self.last_pose_time = None
                self.consecutive_failures = 0
            still_success = False

        if still_success:
            # Keep only inliers
            inl_idx = inliers.ravel()
            inl_obj = pts3d[inl_idx]
            inl_img = pts2d[inl_idx]
            
            # Store RANSAC result as backup
            ransac_rvec = rvec.copy()
            ransac_tvec = tvec.copy()

            # Initial guess: use last valid pose if available
            use_guess = False
            if self.last_valid_pose is not None:
                last_rvec, _ = cv2.Rodrigues(self.last_valid_pose[:3, :3].astype(np.float32))
                last_tvec = self.last_valid_pose[:3, 3].astype(np.float32).reshape(3, 1)
                r_init, t_init, use_guess = last_rvec, last_tvec, True
            else:
                r_init, t_init, use_guess = rvec, tvec, True

            # Nonlinear refinement (ITERATIVE)
            ret, rvec_refined, tvec_refined = cv2.solvePnP(
                objectPoints=inl_obj,
                imagePoints=inl_img,
                cameraMatrix=K,
                distCoeffs=dist,
                rvec=r_init,
                tvec=t_init,
                useExtrinsicGuess=use_guess,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            
            if not ret:
                self.get_logger().warn(f"[{self.listener.name}] PnP refinement failed")
                self.consecutive_failures += 1
                if self.consecutive_failures >= self.max_failures_before_reset:
                    self.get_logger().info(f"[{self.listener.name}] Too many refinement failures, resetting last pose")
                    self.last_valid_pose = None
                    self.last_pose_time = None
                    self.consecutive_failures = 0
                still_success = False
            
        # Validate refined result by comparing with RANSAC result
        # If they differ too much, prefer RANSAC (it's more robust to bad initial guesses)
        if still_success:
            translation_diff = np.linalg.norm(tvec_refined.flatten() - ransac_tvec.flatten())
            
            # If refinement diverged significantly from RANSAC, use RANSAC result instead
            if translation_diff > 0.1:  # 10cm threshold
                self.get_logger().warn(f"[{self.listener.name}] Refined pose diverged from RANSAC ({translation_diff:.3f}m), using RANSAC")
                rvec_final = ransac_rvec
                tvec_final = ransac_tvec
                # Don't update last_valid_pose - force RANSAC-only on next frame
                self.last_valid_pose = None
                self.last_pose_time = None
            else:
                # Refinement is close to RANSAC, use refined result
                rvec_final = rvec_refined
                tvec_final = tvec_refined
                
            # === Step 4: Create Transform and Publish TF ===
            R, _ = cv2.Rodrigues(rvec_final)
            t = tvec_final.flatten()
            cam_T_cube = create_transform_matrix(R, t)
            
            # Validate pose is reasonable (e.g., not too far from camera)
            distance = np.linalg.norm(t)
            if distance > 2.0 or distance < 0.05:  # Between 5cm and 2m
                self.get_logger().warn(f"[{self.listener.name}] Unreasonable distance: {distance:.3f}m, skipping")
                self.consecutive_failures += 1
                if self.consecutive_failures >= self.max_failures_before_reset:
                    self.last_valid_pose = None
                    self.last_pose_time = None
                    self.consecutive_failures = 0
                still_success = False
            
        # Success! Reset failure counter and update last valid pose
        if still_success:
            self.consecutive_failures = 0
            if translation_diff <= 0.1:  # Only update if refinement was good
                self.last_valid_pose = cam_T_cube.copy()
                self.last_pose_time = frame_time
        
            # Publish TF: camera_frame → cube
            qx, qy, qz, qw = rotation_matrix_to_quaternion(cam_T_cube[:3, :3])
            t_cam_cube = cam_T_cube[:3, 3]

            # === Optional Kalman filtering for pose ===
            if self.use_kalman:
                if not self.kf_initialized:
                    self._init_kalman(t_cam_cube)
                else:
                    now = time.time()
                    dt = max(1e-3, now - (self.kf_last_time or now))
                    self._predict_kalman(dt)
                    self.kf_last_time = now
                state = self._update_kalman(t_cam_cube)
                t_cam_cube = state[:3, 0].astype(np.float64)

                quat = np.array([qx, qy, qz, qw], dtype=np.float64)
                if self.last_filtered_quat is None:
                    self.last_filtered_quat = quat
                else:
                    self.last_filtered_quat = self._slerp_quat(
                        self.last_filtered_quat, quat, self.orientation_alpha
                    )
                qx, qy, qz, qw = self.last_filtered_quat.tolist()
        
            cube_tf = TransformStamped()
            cube_tf.header.stamp = hdr.stamp
            cube_tf.header.frame_id = f"{self.camera_name}_frame"
            cube_tf.child_frame_id = self.cube_frame
            cube_tf.transform.translation.x = float(t_cam_cube[0])
            cube_tf.transform.translation.y = float(t_cam_cube[1])
            cube_tf.transform.translation.z = float(t_cam_cube[2])
            cube_tf.transform.rotation.x = qx
            cube_tf.transform.rotation.y = qy
            cube_tf.transform.rotation.z = qz
            cube_tf.transform.rotation.w = qw
            
            self.tf_broadcaster.sendTransform(cube_tf)

            # === Step 5: Publish RViz Marker ===
            if self.publish_marker:
                markers = MarkerArray()
                marker = self.make_mesh_marker(
                    hdr,              # header with timestamp
                    "cube",           # namespace
                    0,                # marker_id
                    t_cam_cube,       # position in camera frame
                    [qx, qy, qz, qw]  # orientation (quaternion)
                )
                markers.markers.append(marker)
                self.marker_pub.publish(markers)

        # === Step 6: Optional Debug Windows ===
        if self.show_windows:
            #plot the 2d keypoimts on the image
            if still_success:
                for pt in pts2d:
                    cv2.circle(img_bgr, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)
            cv2.imshow(f"cube_detection_{self.listener.name}", img_bgr)
            cv2.waitKey(1)

    def make_mesh_marker(self, header, ns, marker_id, pos_xyz, quat_xyzw):
        """
        Create a mesh marker for RViz visualization.
        
        Args:
            header: ROS message header with timestamp
            ns: Marker namespace
            marker_id: Unique marker identifier
            pos_xyz: Position [x, y, z] in camera frame
            quat_xyzw: Orientation quaternion [x, y, z, w]
            
        Returns:
            Marker: RViz marker message
        """
        marker = Marker()
        marker.header.stamp = header.stamp
        marker.header.frame_id = f"{self.camera_name}_frame"  # Publish in camera frame
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = self.mesh_resource
        marker.mesh_use_embedded_materials = True
        marker.scale.x = self.mesh_scale_xyz[0]
        marker.scale.y = self.mesh_scale_xyz[1]
        marker.scale.z = self.mesh_scale_xyz[2]
        marker.action = Marker.ADD
        marker.pose.position.x = float(pos_xyz[0])
        marker.pose.position.y = float(pos_xyz[1])
        marker.pose.position.z = float(pos_xyz[2])
        marker.pose.orientation.x = float(quat_xyzw[0])
        marker.pose.orientation.y = float(quat_xyzw[1])
        marker.pose.orientation.z = float(quat_xyzw[2])
        marker.pose.orientation.w = float(quat_xyzw[3])
        return marker

    def destroy_node(self):
        """Clean shutdown"""
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = CubeDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⚠ Keyboard interrupt received, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

