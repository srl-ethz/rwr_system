#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from threading import RLock
from oakd_ingress import OakDDriver, OAK_CAMS_LIST
from copy import deepcopy
import time


class OakDPublisher(Node):
    def __init__(self, camera_dict=None):
        super().__init__("oakd_publisher")
        self.declare_parameter("visualize", False)
        self.declare_parameter("enable_front_camera", False)
        self.declare_parameter("enable_side_camera", True)
        self.declare_parameter("enable_wrist_camera", False)

        enable_front_camera = self.get_parameter("enable_front_camera").value
        enable_side_camera = self.get_parameter("enable_side_camera").value
        enable_wrist_camera = self.get_parameter("enable_wrist_camera").value

        self.bridge = CvBridge()
        camera_dict = {}
        if enable_front_camera:
            camera_dict["front_view"] = OAK_CAMS_LIST["FRONT_CAMERA"]
        if enable_side_camera:
            camera_dict["side_view"] = OAK_CAMS_LIST["SIDE_CAMERA"]
        if enable_wrist_camera:
            camera_dict["wrist_view"] = OAK_CAMS_LIST["WRIST_CAMERA"]
        self.camera_dict = camera_dict
        self.visualize = self.get_parameter("visualize").value

        self.init_cameras()

    def init_cameras(self):
        for camera_name, camera_id in self.camera_dict.items():
            self.camera_dict[camera_name] = {
                "lock": RLock(),
                "color": None,
                "depth": None,
                "driver": OakDDriver(
                    self.recv_oakd_images,
                    visualize=self.visualize,
                    device_mxid=camera_id,
                    camera_name=camera_name,
                ),
                "rgb_output_pub": self.create_publisher(
                    Image, f"/oakd_{camera_name}/color", 100
                ),
                "depth_output_pub": self.create_publisher(
                    Image, f"/oakd_{camera_name}/depth", 100
                ),
                "camera_info_pub": self.create_publisher(
                    CameraInfo, f"/oakd_{camera_name}/camera_info", 100
                ),
            }

    def recv_oakd_images(self, color, depth, camera_name):
        with self.camera_dict[camera_name]["lock"]:
            (
                self.camera_dict[camera_name]["color"],
                self.camera_dict[camera_name]["depth"],
            ) = (color, depth)

    def publish_images(self):
        for camera_name in self.camera_dict.keys():
            with self.camera_dict[camera_name]["lock"]:
                if (
                    self.camera_dict[camera_name]["color"] is None
                    or self.camera_dict[camera_name]["depth"] is None
                ):
                    continue

                color, depth = deepcopy(
                    self.camera_dict[camera_name]["color"]
                ), deepcopy(self.camera_dict[camera_name]["depth"])

                # 180 flip (need to do it for all oakd cameras for now)
                color = cv2.rotate(color, cv2.ROTATE_180)
                depth = cv2.rotate(depth, cv2.ROTATE_180)

                # publish normal images
                try:
                    header = Header()
                    header.stamp = self.get_clock().now().to_msg()
                    header.frame_id = "world"
                    output_img_rgb = self.bridge.cv2_to_imgmsg(
                        color, "bgr8", header=header
                    )
                    self.camera_dict[camera_name]["rgb_output_pub"].publish(
                        output_img_rgb
                    )
                    # print(f"Published image for {camera_name}")
                except CvBridgeError as e:
                    self.get_logger().error(f"Error publishing color image: {e}")

                try:
                    header = Header()
                    header.stamp = self.get_clock().now().to_msg()
                    header.frame_id = "world"
                    output_img_depth = self.bridge.cv2_to_imgmsg(
                        depth, "mono16", header=header
                    )
                    self.camera_dict[camera_name]["depth_output_pub"].publish(
                        output_img_depth
                    )
                except CvBridgeError as e:
                    self.get_logger().error(f"Error publishing depth image: {e}")

                # publish camera info
                try:
                    camera_info = CameraInfo()
                    camera_info.header.stamp = self.get_clock().now().to_msg()
                    camera_info.header.frame_id = "world"
                    camera_info.width = color.shape[1]
                    camera_info.height = color.shape[0]
                    camera_info.distortion_model = "plumb_bob"
                    camera_info.d = (
                        self.camera_dict[camera_name]["driver"]
                        .distortion_coeff.flatten()
                        .tolist()
                    )
                    intrinsics = (
                        np.asarray(self.camera_dict[camera_name]["driver"].intrinsics)
                        .flatten()
                        .tolist()
                    )
                    camera_info.k = intrinsics
                    self.camera_dict[camera_name]["camera_info_pub"].publish(
                        camera_info
                    )

                except Exception as e:
                    self.get_logger().error(f"Error publishing camera info: {e}")


def main():
    rclpy.init()
    print("Starting oakd_publisher")
    oakd_publisher = OakDPublisher()
    import threading

    spin_thread = threading.Thread(target=rclpy.spin, args=(oakd_publisher,))
    while rclpy.ok():
        oakd_publisher.publish_images()
        time.sleep(0.01)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
