#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AprilTag Camera Calibration - Computes camera-to-tag transformation using AprilTag detection

This script:
1. Subscribes to camera images and intrinsics
2. Detects AprilTag markers in the camera view
3. Estimates camera pose relative to the tag (cam_T_tag)
4. Averages multiple samples for robustness
5. Saves cam_T_tag to file for use in cube_repose_inference_node.py

TRANSFORMATION OUTPUT:
=====================
    camera → tag (detected via AprilTag pose estimation)
    ───────────────────────────────────────
    cam_T_tag (saved to file)

The ROS node will handle world_T_tag transformation separately.

USAGE:
======
    python3 apriltag_calibration.py \
        --camera-topic /oakd_top_camera/color \
        --tag-family tag36h11 \
        --tag-size 0.08 \
        --tag-id 25 \
        --output camera_extrinsics.npy \
        --visualize
"""

import os
import argparse
import threading
from typing import Optional, Tuple
from copy import deepcopy

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from ingress.msg import CameraIntrinsics
from cv_bridge import CvBridge

# AprilTag detection
try:
    from dt_apriltags import Detector
    APRILTAG_AVAILABLE = True
except ImportError:
    print("⚠ Warning: dt_apriltags not installed. Install with: pip install dt-apriltags")
    APRILTAG_AVAILABLE = False

# Import rotation utilities
from rotation_utils import (
    rotation_matrix_to_quaternion,
    quaternion_to_rot_matrix,
    create_transform_matrix,
    euler_angles_to_rotation_matrix
)

# Import shared camera listener
from camera_utils import CameraListener


# ======================== AprilTag Calibration Node ========================

class AprilTagCalibration(Node):
    """
    ROS2 node for camera-to-tag calibration using AprilTag detection.
    
    Detects AprilTag in camera view and computes cam_T_tag transformation.
    """
    
    def __init__(self, args):
        super().__init__("apriltag_calibration")
        
        # Store arguments
        self.camera_topic = args.camera_topic
        self.camera_name = args.camera_name
        self.tag_family = args.tag_family
        self.tag_size = args.tag_size
        self.tag_id = args.tag_id
        self.output_file = args.output_file
        self.num_samples = args.num_samples
        self.visualize = args.visualize
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("📍 AprilTag Camera-to-Tag Calibration")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Tag Family: {self.tag_family}")
        self.get_logger().info(f"Tag Size: {self.tag_size:.4f} m")
        self.get_logger().info(f"Tag ID: {self.tag_id}")
        self.get_logger().info(f"Samples to collect: {self.num_samples}")
        self.get_logger().info(f"Output file: {self.output_file}")
        self.get_logger().info("=" * 60)
        
        # Check AprilTag library
        if not APRILTAG_AVAILABLE:
            self.get_logger().error("❌ dt_apriltags not available!")
            raise ImportError("Please install: pip install dt-apriltags")
        
        # Create camera listener
        self.listener = CameraListener(self, self.camera_topic, self.camera_name)
        
        # AprilTag detector (will be initialized when camera intrinsics arrive)
        self.detector = None
        
        # Storage for multiple samples
        self.cam_T_tag_samples = []
        self.calibration_complete = False
        
        # Create timer for processing
        self.timer = self.create_timer(0.1, self.process_once)  # 10 Hz
        
        self.get_logger().info("\n🎯 Waiting for camera stream and intrinsics...")
    
    
    def initialize_detector(self):
        """Initialize AprilTag detector once camera intrinsics are available."""
        if self.detector is not None:
            return
            
        if self.listener.mtx is None:
            return
        
        # Create detector
        self.detector = Detector(
            families=self.tag_family,
            nthreads=4,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        
        self.get_logger().info(f"✓ AprilTag detector initialized ({self.tag_family})")
    
    def process_once(self):
        """Main processing loop - detect tag and collect samples."""
        
        # Check if calibration is complete
        if self.calibration_complete:
            return
        
        # Wait for camera to be ready
        if not self.listener.is_ready():
            return
        
        # Initialize detector if needed
        self.initialize_detector()
        if self.detector is None:
            self.get_logger().error("❌ Failed to initialize AprilTag detector")
            return
        
        # Get latest frame
        img_bgr, header = self.listener.get_frame()
        if img_bgr is None:
            self.get_logger().error("❌ Failed to get latest frame")
            return
        
        # Convert to grayscale for AprilTag detection
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        self.get_logger().info(f"running detector...")
        # Detect tags
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[
                self.listener.fx,
                self.listener.fy,
                self.listener.cx,
                self.listener.cy
            ],
            tag_size=self.tag_size
        )
        self.get_logger().info(f"detector finished")
        # Filter for our specific tag ID
        target_detections = [d for d in detections if d.tag_id == self.tag_id]
        self.get_logger().info(f"target_detections: {len(target_detections)}")
        if len(target_detections) == 0:
            if self.visualize:
                # Show frame with "no detection" message
                vis = img_bgr.copy()
                cv2.putText(vis, f"Searching for tag {self.tag_id}...", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(vis, f"Samples: {len(self.cam_T_tag_samples)}/{self.num_samples}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.imshow("AprilTag Calibration", vis)
                cv2.waitKey(1)
            return
        
        # Get the detection
        detection = target_detections[0]
        
        # Extract camera_T_tag from detection
        R_cam_tag = detection.pose_R  # 3x3 rotation matrix
        t_cam_tag = detection.pose_t.flatten()  # 3x1 translation vector
        cam_T_tag = create_transform_matrix(R_cam_tag, t_cam_tag)
        
        # Store sample
        self.cam_T_tag_samples.append(cam_T_tag)
        
        self.get_logger().info(
            f"✓ Detected tag {self.tag_id} - Sample {len(self.cam_T_tag_samples)}/{self.num_samples} "
            f"(distance: {np.linalg.norm(t_cam_tag):.3f}m)"
        )
        
        # Visualize if requested
        if self.visualize:
            vis = img_bgr.copy()
            
            # Draw tag outline
            corners = detection.corners.astype(int)
            cv2.polylines(vis, [corners], True, (0, 255, 0), 3)
            
            # Draw tag ID
            center = tuple(corners.mean(axis=0).astype(int))
            cv2.putText(vis, f"ID: {detection.tag_id}", center,
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            
            # Draw axes (X=red, Y=green, Z=blue)
            self.draw_axis(vis, cam_T_tag, self.listener.mtx, self.listener.dist, self.tag_size)
            
            # Show progress
            cv2.putText(vis, f"Samples: {len(self.cam_T_tag_samples)}/{self.num_samples}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(vis, "Hold tag steady...", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            cv2.imshow("AprilTag Calibration", vis)
            cv2.waitKey(1)
        
        # Check if we have enough samples
        if len(self.cam_T_tag_samples) >= self.num_samples:
            self.compute_and_save_calibration()
    
    def draw_axis(self, img, cam_T_tag, K, dist, size):
        """Draw 3D coordinate axes on the image."""
        # Define axis points in tag frame
        axis_length = size * 0.5
        axis_points = np.array([
            [0, 0, 0],  # origin
            [axis_length, 0, 0],  # X axis (red)
            [0, axis_length, 0],  # Y axis (green)
            [0, 0, axis_length],  # Z axis (blue)
        ], dtype=np.float32)
        
        # Transform to camera frame
        R = cam_T_tag[:3, :3]
        t = cam_T_tag[:3, 3]
        
        # Project to image
        rvec, _ = cv2.Rodrigues(R)
        imgpts, _ = cv2.projectPoints(axis_points, rvec, t, K, dist)
        imgpts = imgpts.reshape(-1, 2).astype(int)
        
        # Draw axes
        origin = tuple(imgpts[0])
        cv2.line(img, origin, tuple(imgpts[1]), (0, 0, 255), 3)  # X - red
        cv2.line(img, origin, tuple(imgpts[2]), (0, 255, 0), 3)  # Y - green
        cv2.line(img, origin, tuple(imgpts[3]), (255, 0, 0), 3)  # Z - blue
    
    def compute_and_save_calibration(self):
        """Average samples and compute final camera-to-tag transformation."""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("📊 Computing camera-to-tag transformation from samples...")
        self.get_logger().info("=" * 60)
        
        # Average rotation and translation separately
        # For rotations, we'll average the rotation matrices (simple average is ok for small variations)
        rotations = [T[:3, :3] for T in self.cam_T_tag_samples]
        translations = [T[:3, 3] for T in self.cam_T_tag_samples]
        
        avg_rotation = np.mean(rotations, axis=0)
        avg_translation = np.mean(translations, axis=0)
        
        # Ensure rotation matrix is valid (orthogonalize via SVD)
        U, _, Vt = np.linalg.svd(avg_rotation)
        avg_rotation = U @ Vt
        
        # Create averaged cam_T_tag
        cam_T_tag_avg = create_transform_matrix(avg_rotation, avg_translation)
        
        # Compute standard deviations for quality check
        translation_std = np.std(translations, axis=0)
        
        self.get_logger().info(f"Average cam_T_tag translation: {avg_translation}")
        self.get_logger().info(f"Translation std: {translation_std}")
        self.get_logger().info(f"Max translation std: {np.max(translation_std):.6f} m")
        
        if np.max(translation_std) > 0.01:  # 1cm threshold
            self.get_logger().warn("⚠ High variation in samples! Consider recalibrating.")
        
        self.get_logger().info("\n📐 Final Camera-to-Tag Transformation (cam_T_tag):")
        self.get_logger().info(f"\n{cam_T_tag_avg}")
        
        # Extract pose for readability
        t = cam_T_tag_avg[:3, 3]
        qx, qy, qz, qw = rotation_matrix_to_quaternion(cam_T_tag_avg[:3, :3])
        
        self.get_logger().info(f"\nTag position in camera frame:")
        self.get_logger().info(f"  x={t[0]:.4f}, y={t[1]:.4f}, z={t[2]:.4f}")
        self.get_logger().info(f"Tag orientation in camera frame (quaternion x,y,z,w):")
        self.get_logger().info(f"  {qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}")
        
        # Save to file
        try:
            np.save(self.output_file, cam_T_tag_avg)
            self.get_logger().info(f"\n✅ Camera-to-tag transformation saved to: {self.output_file}")
            self.get_logger().info("=" * 60)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save calibration: {e}")
            return
        
        # Show success message
        if self.visualize:
            img = np.zeros((400, 800, 3), dtype=np.uint8)
            cv2.putText(img, "CAMERA-TO-TAG CALIBRATION COMPLETE!", (50, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
            cv2.putText(img, f"Saved to: {os.path.basename(self.output_file)}", (150, 200),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(img, "Press any key to exit", (250, 300),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("AprilTag Calibration", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        
        self.calibration_complete = True
        self.get_logger().info("🛑 Shutting down...")
        rclpy.shutdown()


# ==================== Main Entry Point ====================

def main():
    """Main function - parse arguments and run calibration."""
    
    parser = argparse.ArgumentParser(
        description="AprilTag Camera-to-Tag Calibration - Compute cam_T_tag transformation using AprilTag detection",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    # Camera parameters
    parser.add_argument(
        "--camera-topic",
        type=str,
        default="/oakd_bottom_view/color",
        help="Camera topic to subscribe to (default: /oakd_bottom_view/color)"
    )
    parser.add_argument(
        "--camera-name",
        type=str,
        default="oakd_bottom_view",
        help="Camera name for logging (default: oakd_bottom_view)"
    )
    
    # AprilTag parameters
    parser.add_argument(
        "--tag-family",
        type=str,
        default="tag36h11",
        choices=["tag36h11", "tag25h9", "tag16h5", "tagCircle21h7", "tagCircle49h12", "tagStandard41h12", "tagStandard52h13"],
        help="AprilTag family (default: tag36h11)"
    )
    parser.add_argument(
        "--tag-size",
        type=float,
        default=0.08,
        help="AprilTag size in meters (default: 0.08)"
    )
    parser.add_argument(
        "--tag-id",
        type=int,
        default=25,
        help="AprilTag ID to detect (default: 25)"
    )
    
    
    # Output and collection
    parser.add_argument(
        "-o", "--output",
        dest="output_file",
        type=str,
        default="camera_extrinsics.npy",
        help="Output file for camera-to-tag transformation (default: camera_extrinsics.npy)"
    )
    parser.add_argument(
        "--num-samples",
        type=int,
        default=30,
        help="Number of samples to average (default: 30)"
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Show visualization window"
    )
    
    args = parser.parse_args()
    
    print("🎯 AprilTag Camera-to-Tag Calibration")
    print("=" * 60)
    print("This script will:")
    print("1. Subscribe to camera images and intrinsics")
    print("2. Detect AprilTag markers in the camera view")
    print("3. Estimate camera pose relative to the tag")
    print("4. Average multiple samples for robustness")
    print("5. Save cam_T_tag transformation to file")
    print("=" * 60)
    
    # Initialize ROS
    rclpy.init()
    
    try:
        node = AprilTagCalibration(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⚠ Keyboard interrupt - exiting...")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            cv2.destroyAllWindows()
        except:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()

