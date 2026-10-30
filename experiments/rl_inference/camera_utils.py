#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Utilities - Shared camera listener for ROS2 nodes

Provides a thread-safe camera listener that subscribes to image and intrinsics topics.
Used by both apriltag_calibration.py and cube_repose_inference_node.py.
"""

import threading
from typing import Optional, Tuple
from copy import deepcopy

import numpy as np
from cv_bridge import CvBridge

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from ingress.msg import CameraIntrinsics

import cv2

class CameraListener:
    """
    Manages camera image and intrinsics subscription for a single camera.
    
    Subscribes to:
    - RGB image topic (e.g., /oakd_top_camera/color)
    - Camera intrinsics topic (e.g., /oakd_top_camera/intrinsics)
    
    Provides thread-safe access to the latest image and camera parameters.
    """
    
    def __init__(self, parent_node: Node, camera_topic: str, name: str):
        """
        Initialize camera listener.
        
        Args:
            parent_node: ROS2 node that owns this listener
            camera_topic: Camera image topic (e.g., '/oakd_top_camera/color')
            name: Camera name for logging
        """
        self.node = parent_node
        self.camera_topic = camera_topic
        self.name = name
        self.bridge = CvBridge()
        
        # Thread-safe storage for latest image
        self.lock = threading.Lock()
        self.image_bgr: Optional[np.ndarray] = None
        self.header = None

        # Camera intrinsic parameters (K matrix and distortion coefficients)
        self.mtx: Optional[np.ndarray] = None  # 3x3 camera matrix
        self.dist: Optional[np.ndarray] = None  # distortion coefficients
        self.fx, self.fy, self.cx, self.cy = None, None, None, None

        # Subscribe to camera image stream (BEST_EFFORT for low latency)
        self.sub = parent_node.create_subscription(
            Image, self.camera_topic, self.recv_im, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, 
                      history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        )
        
        # Subscribe to camera intrinsics (RELIABLE + TRANSIENT_LOCAL to get latest on startup)
        intrinsics_topic = self.camera_topic.replace("color", "intrinsics")
        self.intr_sub = parent_node.create_subscription(
            CameraIntrinsics, intrinsics_topic, self.recv_intr,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, 
                      history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        # Publishers for visualization (optional debug images)
        self.pub_cur = parent_node.create_publisher(Image, f"~/{name}/cur_img", 1)
        self.pub_crop = parent_node.create_publisher(Image, f"~/{name}/cropped_img", 1)
        
        parent_node.get_logger().info(f"📷 Camera listener created for {name} on {camera_topic}")

    def recv_im(self, msg: Image):
        """Callback for camera image messages - stores latest image thread-safely."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.node.get_logger().warn(f"[{self.name}] cv_bridge conversion failed: {e}")
            return
        with self.lock:
            self.image_bgr = img
            self.header = msg.header
    def recv_intr(self, msg: CameraIntrinsics):
        """
        Callback for camera intrinsics - builds K matrix and distortion coefficients.
        
        K matrix format:
        [[fx,  0, cx],
         [ 0, fy, cy],
         [ 0,  0,  1]]
        """
        if self.mtx is not None:
            return  # Already received, only need it once
            
        # Store individual parameters
        self.fx, self.fy = msg.fx, msg.fy
        self.cx, self.cy = msg.cx, msg.cy
        
        # Build camera intrinsic matrix
        self.mtx = np.eye(3)
        self.mtx[0, 0] = self.fx
        self.mtx[1, 1] = self.fy
        self.mtx[0, 2] = self.cx
        self.mtx[1, 2] = self.cy

        # Store distortion coefficients
        self.dist = np.array(msg.disto)
        
        self.node.get_logger().info(f"✓ [{self.name}] Received camera intrinsics:")
        self.node.get_logger().info(f"  fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")

    def get_frame(self) -> Tuple[Optional[np.ndarray], Optional[object]]:
        """Get the latest image frame and header (thread-safe)."""
        with self.lock:
            if self.image_bgr is None:
                return None, None
            return self.image_bgr.copy(), deepcopy(self.header)
    
    def is_ready(self) -> bool:
        """Check if camera is ready (has both image and intrinsics)."""
        with self.lock:
            return self.image_bgr is not None and self.mtx is not None

    def publish_images(self, cur_img: Optional[np.ndarray], cropped_img: Optional[np.ndarray], stamp):
        """Publish visualization images (optional debug output)."""
        if cur_img is not None:
            try:
                msg = self.bridge.cv2_to_imgmsg(cur_img, encoding="bgr8")
                msg.header.stamp = stamp
                msg.header.frame_id = "image"
                self.pub_cur.publish(msg)
            except Exception as e:
                self.node.get_logger().warn(f"[{self.name}] Failed publishing cur_img: {e}")
        if cropped_img is not None and cropped_img.size > 0:
            try:
                msg = self.bridge.cv2_to_imgmsg(cropped_img, encoding="bgr8")
                msg.header.stamp = stamp
                msg.header.frame_id = "image"
                self.pub_crop.publish(msg)
            except Exception as e:
                self.node.get_logger().warn(f"[{self.name}] Failed publishing cropped_img: {e}")

