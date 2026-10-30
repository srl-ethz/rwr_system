#!/bin/env python3

import numpy as np
import time
from copy import deepcopy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped, Vector3, Quaternion, PoseStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
from threading import Lock
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from ingress.msg import CameraIntrinsics
from cv_bridge import CvBridge
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pickle as pkl
try:
    from cam_calibration_box.calibration import CalibrationProject, find_latest_calibration_project
    from cam_calibration_box.calibration_boxes import get_calibration_box_cfg
    from cam_calibration_box.apriltag import apriltag
except ImportError as e:
    print("Failed to import cam_calibration_box")
    print("Please check https://github.com/srl-ethz/cam_calibration_box.git")

from faive_system.src.common.transformation_utils import rigid_transform, complete_rotation_matrix
from faive_system.src.common.enums import TeleopState, HandChirality
import cv2
from tqdm import tqdm

CHIRALITY = HandChirality.RIGHT
TARGET_CAMERA_DICT = {
    # "oakd_front_view": ("/oakd_front_view/color", "/oakd_front_view/intrinsics", False),
    # "oakd_side_view": ("/oakd_side_view/color", "/oakd_side_view/intrinsics", False)
    "zed_side_view": ("/zed_side_view/color", "/zed_side_view/intrinsics", False),
    "oakd_wrist_view": ("/oakd_wrist_view/color", "/oakd_wrist_view/intrinsics", True),
}

class CameraListener(Node):
    def __init__(self, name, camera_topic, intrinsics_topic, is_wrist_cam, node):
        super().__init__(f"camera_listener")

        self.camera_topic = camera_topic
        self.intrinsics_topic = intrinsics_topic
        self.lock = Lock()
        self.image = None
        self.name = name
        self._intrinsics = None
        self.is_wrist_cam = is_wrist_cam
        self.im_subscriber = node.create_subscription(
            Image, self.camera_topic, self.recv_im, 1
        )
        self.intr_subscriber = node.create_subscription(
            CameraIntrinsics, self.intrinsics_topic, self.recv_intrinsics, 1
        )

    def recv_im(self, msg: Image):
        with self.lock:
            self.image = msg

    def get_im(self):
        with self.lock:
            img = deepcopy(self.image)
            # self.image = None
            return img
        
    def recv_intrinsics(self, msg: CameraIntrinsics):
        self._intrinsics = dict(
            fx = msg.fx,
            fy = msg.fy,
            cx = msg.cx,
            cy = msg.cy,
            disto = msg.disto
        )
    
    @property
    def intrinsics(self):
        return self._intrinsics

class CameraCalibrationNode(Node):
    def __init__(self, chirality=HandChirality.RIGHT):
        super().__init__(f"{chirality}_camera_calibration")

        # publishes to arm/end_effector_pose_cmd
        self.bridge = CvBridge()
        self.arm_publisher = self.create_publisher(
            PoseStamped, f"/{chirality}/arm/end_effector_pose_cmd", 10
        )
        print("topic", f"/{chirality}/arm/end_effector_pose_cmd")

        self.arm_subscriber_topic = f"/{chirality}/arm/end_effector_pose"
        self.arm_subscriber = self.create_subscription(
            PoseStamped, self.arm_subscriber_topic, self.arm_pose_callback, 1
        )
        self.X_W_fEE = None

        self.camera_listeners = [
            CameraListener(k, img_topic, intrins_topic, is_wrist_cam, self) 
            for k, (img_topic, intrins_topic, is_wrist_cam) in TARGET_CAMERA_DICT.items()
        ]
        self.calibr_box_cfg = get_calibration_box_cfg("franka01")
    

    def calibrate_static_camera_pose(self):
        """
        Run this function with calibration box mounted on franka
        """
        waypoints = [
            (np.array([0.30, -0.1, 0.2]), complete_rotation_matrix(x=[1, 0, 0], z=[0, 1, 0])),
            (np.array([0.50, -0.2, 0.25]), complete_rotation_matrix(x=[1, 0, -1], z=[0, 1, 0])),
            (np.array([0.70, -0.1, 0.2]), complete_rotation_matrix(x=[1, 0, 0], z=[0, 0, -1])),
            (np.array([0.70,  0.1, 0.25]), complete_rotation_matrix(x=[1, 1, 0], z=[0, 0, -1])),
            (np.array([0.50,  0.1, 0.2]), complete_rotation_matrix(x=[0, 0, 1], z=[-1, 0, 0])),
            (np.array([0.50,  0.2, 0.25]), complete_rotation_matrix(x=[0, 1, 0], z=[-1, 0, -1])),
            (np.array([0.30,  0.1, 0.2]), complete_rotation_matrix(x=[1, 0, 1], z=[0, 1, 0])),
        ]
        print("#############\nCalibrating camera poses\nPlease make sure calibration box is mounted\n#############")
        calibration_project = CalibrationProject(None)
        
        for i, (pos, rot) in enumerate(waypoints):
            franka_target = rigid_transform(pos, Rotation.from_matrix(rot).as_quat())
            print(f"moving to target{i}")
            print(franka_target)
            self.publish_target_pose(franka_target)
            time.sleep(10)# wait for execution and wait for camera
            
            # get all images
            cameras = {}
            wait_count = 0
            num_static_cameras = sum([not cam.is_wrist_cam for cam in self.camera_listeners])
            while len(cameras) < num_static_cameras:
                for cam in self.camera_listeners:
                    if cam.name in cameras.keys():
                        continue
                    if cam.is_wrist_cam:
                        continue
                    img = cam.get_im()
                    if img is not None and cam.intrinsics is not None:
                        img = self.bridge.imgmsg_to_cv2(img, "bgr8")
                        cameras[cam.name] = {
                            "image": img,
                            "intrinsics":cam.intrinsics,
                            "mount_pose": None
                        }
                time.sleep(1)
                wait_count += 1
                if wait_count %10 == 0:
                    print("waiting for cameras", 
                          [cam.name for cam in self.camera_listeners if cam.name not in cameras.keys()])

            calibration_project.add_measurements(cameras, self.calibr_box_cfg, franka_target)

        calibration_project.solve_camera_poses()
        calibration_project.visualize_solution()
        return calibration_project.transformations


    def calibrate_wrist_cam_pose(self, calibration_project):
        """
        Run this function with calibration box placed on the table
        """
        waypoints = [
            (np.array([0.4, 0.1, 0.3]), complete_rotation_matrix(x=[-1,  -1, 0], z=[0, 0.1, -1])),
            (np.array([0.4, -0.1, 0.3]), complete_rotation_matrix(x=[1, -1, 0], z=[0, -0.1, -1])),
        ]
        print("#############\nCalibrating wrist cam poses\nPlease make sure calib box is placed on the table and wrist cam is mounted\n#############")
        for i, (pos, rot) in enumerate(waypoints):
            franka_target = rigid_transform(pos, Rotation.from_matrix(rot).as_quat())
            print(f"moving to target{i}")
            print(franka_target)
            self.publish_target_pose(franka_target)
            time.sleep(10)# wait for execution and wait for camera
            
            # get all images
            cameras = {}
            wait_count = 0
            while len(cameras) < len(self.camera_listeners):
                for cam in self.camera_listeners:
                    if cam.name in cameras.keys():
                        continue
                    img = cam.get_im()
                    if img is not None and cam.intrinsics is not None:
                        img = self.bridge.imgmsg_to_cv2(img, "bgr8")
                        if cam.is_wrist_cam:
                            mount_pose = franka_target
                        else:
                            mount_pose = None
                        cameras[cam.name] = {
                            "image": img,
                            "intrinsics":cam.intrinsics,
                            "mount_pose": mount_pose
                        }
                time.sleep(1)
                wait_count += 1
                if wait_count %10 == 0:
                    print("waiting for cameras", 
                          [cam.name for cam in self.camera_listeners if cam.name not in cameras.keys()])

            calibration_project.add_measurements(cameras, self.calibr_box_cfg, None)

        calibration_project.solve_camera_poses()
        calibration_project.visualize_solution()
        return calibration_project.transformations

    def detect(self, image_trajs, calibr_box_cfg, transformations):
        """
        image_trajs: a dict mapping from image name to list of images
        transformations: a dict mapping from image name to mnt_T_cam
        """
        out_trajs = {k:[] for k in image_trajs.keys()}
        detector = apriltag("tagStandard41h12")
        intr_dict = {
            cam.name: cam.intrinsics for cam in self.camera_listeners
        }
        for cam_name, img_trajs in image_trajs.items():
            intr = intr_dict[cam_name]
            camera_matrix = np.array([
                [intr['fx'], 0, intr['cx']],
                [0, intr['fy'], intr['cy']],
                [0, 0, 1]
            ])
            dist_coeffs = np.array(intr['disto'])
            for img in img_trajs:
                if img is None:
                    out_trajs[cam_name].append(None)
                    print(f"Warning: no image")
                    continue
                img = self.bridge.imgmsg_to_cv2(img, "bgr8")
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                detections = detector.detect(gray)
                if len(detections) == 0:
                    print(f"Warning: No detection found")
                    out_trajs[cam_name].append(None)
                    continue
                
                for detection in detections:
                    tag_cfg_id = f"tag_{detection['id']}"
                    if tag_cfg_id not in calibr_box_cfg:
                        print(f"Warning: {tag_cfg_id} not found in calibr_box_cfg")
                        out_trajs[cam_name].append(None)
                        continue

                    # Compute the tag's pose relative to the calibration box.
                    box_T_tag = np.eye(4)
                    tag_R = Rotation.from_quat(calibr_box_cfg[tag_cfg_id]["rotation"])
                    box_T_tag[:3, :3] = tag_R.as_matrix()
                    box_T_tag[:3, 3] = calibr_box_cfg[tag_cfg_id]["pos"]

                    # Estimate tag pose from the image.
                    corners = detection['lb-rb-rt-lt'][None, ::-1].astype(np.float32)
                    tag_width = calibr_box_cfg[tag_cfg_id]["tag_width"]
                    tag_rvecs, tag_tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, tag_width, camera_matrix, dist_coeffs)
                    cam_T_tag = np.eye(4)
                    cam_T_tag[:3, :3] = Rotation.from_rotvec(np.squeeze(tag_rvecs)).as_matrix()
                    cam_T_tag[:3, 3] = np.squeeze(tag_tvecs)
                    
                    mnt_T_cam = transformations[cam_name]
                    out_trajs[cam_name].append(mnt_T_cam @ cam_T_tag @ np.linalg.inv(box_T_tag))
                    break
        return out_trajs

    def analysing_delay(self, transformations):
        print("Calibration done, analysing delay")
        transformations = {k:v for k,v in transformations}
        def circ_traj_gen(center, R, n_cycles, N, rot):
            for i in np.linspace(0, 2*np.pi, N):
                pos = np.array(center) + np.array([
                    R * np.sin(i * n_cycles),
                    R * np.cos(i * n_cycles),
                    0
                ])
                yield rigid_transform(pos, Rotation.from_matrix(rot).as_quat())
        
        target_cmd_traj = []
        target_obs_traj = []
        time_traj = []
        images = {cam.name:[] for cam in self.camera_listeners}
        for target in tqdm(circ_traj_gen([0.50, 0.0, 0.3], 0.1, 2, 300, 
                        complete_rotation_matrix(x=[0, 0, 1], z=[0, 1, 0]))):
            self.publish_target_pose(target)
            time_traj.append(time.time())
            target_cmd_traj.append(target)
            target_obs_traj.append(self.X_W_fEE)
            for cam in self.camera_listeners:
                images[cam.name].append(cam.get_im())
            time.sleep(0.1)

        target_img_obs_trajs = self.detect(images, self.calibr_box_cfg, transformations)


        ## Plot
        time_traj = [(t-time_traj[0]) for t in time_traj]
        fig = plt.figure()
        plt.plot(time_traj, np.array(target_cmd_traj)[:,0,3], label = "cmd")
        plt.plot(time_traj, np.array(target_obs_traj)[:,0,3], label = "obs")
        for k,traj in target_img_obs_trajs.items():
            traj = np.array([a[0,3] if a is not None else np.nan for a in traj])
            # print(traj)
            plt.plot(time_traj, traj, label=k)
        plt.legend()
        with open("camera delay calibration", 'wb') as f:
            pkl.dump({
                "target_cmd_traj":target_cmd_traj,
                "target_obs_traj":target_obs_traj,
                "images":images,
                "time_traj": time_traj,
                "target_img_obs_trajs": target_img_obs_trajs
            }, f)

        plt.savefig("calibration.png")
        # plt.show()

    def run(self, resume_project, analyse_delay, calibrate_wrist_camera):
        wait_count = 0
        while self.X_W_fEE is None:
            wait_count += 1
            time.sleep(1)
            if wait_count %10 == 0:
                print("waiting for end effector pose", self.arm_subscriber_topic)

        # Calibrating camera poses
        if not resume_project or resume_project=="":
            transformations = self.calibrate_static_camera_pose()
        else:
            calibration_project = CalibrationProject(resume_project)
            transformations = calibration_project.transformations

        # Ask if analyse delay
        if analyse_delay:
            self.analysing_delay(transformations)

        # Calibrate wrist camera
        if calibrate_wrist_camera:
            assert resume_project, "wrist camera should be calibrated after static cameras are calibrated"
            transformations = self.calibrate_wrist_cam_pose(calibration_project)
        
    def arm_pose_callback(self, msg: PoseStamped):
        orientation = msg.pose.orientation
        position = msg.pose.position
        # Get the pose of the wrist
        self.X_W_fEE = rigid_transform(
            [position.x, position.y, position.z],
            [orientation.x, orientation.y, orientation.z, orientation.w],
        )

    def publish_target_pose(self, X_W_fEE_d):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        x, y, z = X_W_fEE_d[:3, 3]
        msg.pose.position = Point(x=x, y=y, z=z)
        quat = Rotation.from_matrix(X_W_fEE_d[:3,:3]).as_quat()# x, y, z, w
        msg.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )
        self.arm_publisher.publish(msg)

def main(args=None):
    import argparse
    parser = argparse.ArgumentParser(description="A runner that coordinates the nodes with keyboard input")

    parser.add_argument("--resume_project", default="", help="The directory where calibration project is resumed.")
    parser.add_argument("--analyse_delay", action="store_true", help="analyse camera delay with calibration box mounted")
    parser.add_argument("--calibrate_wrist_camera", action="store_true", help="Calibrate wrist camera pose")
    args = parser.parse_args()

    rclpy.init()
    rcd = CameraCalibrationNode(chirality=CHIRALITY) 

    import threading

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(rcd) 

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    rcd.run(
        resume_project = args.resume_project,
        analyse_delay = args.analyse_delay,
        calibrate_wrist_camera = args.calibrate_wrist_camera,
    )

    spin_thread.join()
    rclpy.shutdown()

if __name__ == "__main__":
    main()