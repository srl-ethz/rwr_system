#!/usr/bin/env python3
"""
zed_multi_publisher.py
A ROS 2 component that can manage an arbitrary number of
ZED cameras, defined in an external YAML file that is passed in
as a node parameter `config_file`.
Example parameter YAML (to be passed to `ros2 run` or via a launch file):
zed_multi_publisher:
  ros__parameters:
    config_file: /path/to/cameras.yaml
    publish_depth: true
    publish_pointcloud: true
    depth_minimum_distance: 0.3
    depth_maximum_distance: 1.5

Each top-level key is the ZED camera serial number.  For every camera one (and
only one) set of topics will be created:
⇢  /zed_<name>/color        – `sensor_msgs/Image` (bgr8)
⇢  /zed_<name>/depth        – `sensor_msgs/Image` (32FC1) – optional, per-camera
⇢  /zed_<name>/intrinsics   – `ingress/CameraIntrinsics`

"""
import os
from dataclasses import dataclass
from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from ingress.msg import CameraIntrinsics
import yaml
import numpy as np
import pyzed.sl as sl
from cv_bridge import CvBridge
@dataclass
class CameraConfig:
    serial: int
    name: str
    depth: bool

class ZEDCamera:
    """Wraps a single physical ZED camera and its ROS 2 publishers."""
    def __init__(self, parent: Node, cfg: CameraConfig,
                 publish_depth_global: bool,
                 depth_min: float,
                 depth_max: float,
                 qos: int):
        self._parent = parent
        self._cfg = cfg
        self._publish_depth = publish_depth_global and cfg.depth
        self._bridge = CvBridge()
        self._camera = sl.Camera()
        self._frame_id = f"zed_{cfg.name}"
        self._color_pub = parent.create_publisher(
            Image, f"{self._frame_id}/color", qos
        )
        self._intrinsics_pub = parent.create_publisher(
            CameraIntrinsics, f"{self._frame_id}/intrinsics", qos
        )
        if self._publish_depth:
            self._depth_pub = parent.create_publisher(
                Image, f"{self._frame_id}/depth", qos
            )
        # ZED SDK initialisation parameters
        init_params = sl.InitParameters(
            camera_resolution=sl.RESOLUTION.HD720,
            camera_fps=20,
            depth_mode=sl.DEPTH_MODE.PERFORMANCE, #QUALITY,
            coordinate_units=sl.UNIT.METER,
            depth_minimum_distance=depth_min,
            depth_maximum_distance=depth_max,
        )
        init_params.set_from_serial_number(cfg.serial)
        status = self._camera.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(
                f"Failed to open ZED ({cfg.serial} – {cfg.name}): {status.name}"
            )
        self._image_mat = sl.Mat()
        self._depth_mat = sl.Mat()
    # ------------------------------------------------------------------ #
    def destroy(self):
        self._camera.close()
    # ------------------------------------------------------------------ #
    def grab_and_publish(self, runtime_params: sl.RuntimeParameters):
        if self._camera.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
            return
        # Colour image
        self._camera.retrieve_image(self._image_mat, sl.VIEW.LEFT)
        img_bgra = self._image_mat.get_data()
        img_bgr = img_bgra[:, :, :3]
        msg = self._bridge.cv2_to_imgmsg(img_bgr, "bgr8")
        msg.header = self._make_header()
        self._color_pub.publish(msg)
        # Depth (if enabled)
        if self._publish_depth:
            self._camera.retrieve_measure(self._depth_mat, sl.MEASURE.DEPTH)
            depth_np = np.nan_to_num(
                self._depth_mat.get_data(), nan=0.0, posinf=0.0, neginf=0.0
            )
            depth_msg = self._bridge.cv2_to_imgmsg(depth_np, encoding="32FC1")
            depth_msg.header = msg.header  # same timestamp/frame_id
            self._depth_pub.publish(depth_msg)
        # Intrinsics (published each grab; you may throttle if desired)
        cam_info = self._camera.get_camera_information()
        intr = cam_info.camera_configuration.calibration_parameters.left_cam
        self._intrinsics_pub.publish(
            CameraIntrinsics(
                fx=intr.fx, fy=intr.fy, cx=intr.cx, cy=intr.cy, disto=intr.disto
            )
        )
    # ------------------------------------------------------------------ #
    def _make_header(self) -> Header:
        hdr = Header()
        hdr.stamp = self._parent.get_clock().now().to_msg()
        hdr.frame_id = self._frame_id
        return hdr

class ZEDNode(Node):
    """ROS 2 node hosting any number of ZEDCamera instances."""
    def __init__(self):
        super().__init__("zed_node")
        # ------------------------- Parameters -------------------------- #
        self.declare_parameter("config_file", "")
        self.declare_parameter("publish_depth", True)
        self.declare_parameter("depth_minimum_distance", 0.3)
        self.declare_parameter("depth_maximum_distance", 1.5)
        self.declare_parameter("loop_rate_hz", 30.0)
        self.declare_parameter("pub_qos", 2)
        cfg_path: str = self.get_parameter("config_file").get_parameter_value().string_value
        if not cfg_path:
            cfg_path = os.path.join(os.path.dirname(__file__), "zed_cams.yaml")
        if not os.path.isfile(cfg_path):
            raise FileNotFoundError(
                "Parameter 'config_file' is empty or the file does not exist!"
            )
        # ----------------------- Load camera YAML ---------------------- #
        with open(cfg_path, "r") as f:
            yaml_cfg = yaml.safe_load(f)
        self._cameras: List[ZEDCamera] = []

        for serial_str, values in yaml_cfg.items():
            cfg = CameraConfig(
                serial=int(serial_str),
                name=values.get("name", f"cam_{serial_str}"),
                depth=bool(values.get("depth", False)),
            )
            try:
                self._cameras.append(
                    ZEDCamera(
                        parent=self,
                        cfg=cfg,
                        publish_depth_global=self.get_parameter("publish_depth").value,
                        depth_min=self.get_parameter("depth_minimum_distance").value,
                        depth_max=self.get_parameter("depth_maximum_distance").value,
                        qos=self.get_parameter("pub_qos").value,
                    )
                )
                self.get_logger().info(
                    f"Initialised ZED {cfg.serial} as '{cfg.name}' "
                    f"(depth {'ON' if cfg.depth else 'OFF'})"
                )
            except Exception as exc:  # noqa
                self.get_logger().error(str(exc))
        if not self._cameras:
            raise RuntimeError("No cameras were successfully initialised!")
        # ---------------------- Runtime & timer ------------------------ #
        self._runtime_params = sl.RuntimeParameters()
        hz = self.get_parameter("loop_rate_hz").value
        self.create_timer(1.0 / hz, self._timer_cb)
    # ----------------------------------------------------------------- #
    def _timer_cb(self):
        for cam in self._cameras:
            cam.grab_and_publish(self._runtime_params)
    # ----------------------------------------------------------------- #
    def destroy_node(self):
        for cam in self._cameras:
            cam.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZEDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
