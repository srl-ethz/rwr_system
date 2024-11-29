#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from threading import RLock
from faive_system.src.ingress.oakd.oakd_ingress import OakDDriver, OAK_CAMS_LIST
from copy import deepcopy
import time
from faive_system.src.common.utils import numpy_to_float32_multiarray


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
        self.calibrated = False

        self.init_cameras()

    def init_cameras(self):
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

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
                "intrinsics_pub": self.create_publisher(
                    Float32MultiArray, f"/oakd_{camera_name}/intrinsics", qos_profile
                ),
                "extrinsics_pub": self.create_publisher(
                    Float32MultiArray, f"/oakd_{camera_name}/extrinsics", qos_profile
                ),
                "projection_pub": self.create_publisher(
                    Float32MultiArray, f"/oakd_{camera_name}/projection", qos_profile
                ),
                "calibrated" : False
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

                if self.camera_dict[camera_name]["calibrated"]:
                    continue
                # publish camera info
                try:
                    projection_matrix = np.array(self.camera_dict[camera_name]["driver"].projection_matrix)
                    print(f"Projection matrix for {camera_name}: {projection_matrix}")
                    # flatten the matrix
                    projection_matrix_msg = Float32MultiArray()
                    projection_matrix_msg.data = projection_matrix.flatten().tolist()
                    self.camera_dict[camera_name]["projection_pub"].publish(
                        projection_matrix_msg
                    )

                    intrinsic_matrix = np.array(self.camera_dict[camera_name]["driver"].intrinsics)
                    print(f"intrinsic_matrix for {camera_name}: {intrinsic_matrix}")
                    intrinsic_matrix_msg = Float32MultiArray()
                    intrinsic_matrix_msg.data = intrinsic_matrix.flatten().tolist()
                    self.camera_dict[camera_name]["intrinsics_pub"].publish(
                        intrinsic_matrix_msg
                    )
                    
                    extrinsic_matrix = np.array(self.camera_dict[camera_name]["driver"].extrinsics)
                    print(f"Extrinsic matrix for {camera_name}: {extrinsic_matrix}")
                    extrinsic_matrix_msg = Float32MultiArray()
                    extrinsic_matrix_msg.data = extrinsic_matrix.flatten().tolist() 
                    self.camera_dict[camera_name]["extrinsics_pub"].publish(
                        extrinsic_matrix_msg
                    )
                    
                    self.camera_dict[camera_name]["calibrated"] = True
                    
                except Exception as e:
                    self.get_logger().error(f"Error publishing camera infos: {e}")
                    

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
