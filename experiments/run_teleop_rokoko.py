#!/bin/env python3

import numpy as np
import time
from copy import deepcopy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped, Vector3, Quaternion, PoseStamped
from tf2_ros import TransformBroadcaster

from scipy.spatial.transform import Rotation as R


class RokokoCoilDemo(Node):
    def __init__(self):
        super().__init__("rokoko_coil_demo")

        self.X_bCP_G = None  # Transformation from coil pro to glove wrist
        self.X_bCP_G_init = None  # Initial transformation from coil pro to glove wrist

        # Transformation from map to coil pro
        self.X_W_bCP = self.create_transform(
            [1.0, 2.0, -1.3], R.from_euler("xyz", [90.0, 0.0, 0.0], degrees=True)
        )
        self.X_W_fEE_d = None
        self.X_W_fEE_init = None
        self.X_W_fEE = None

        self.rokoko_pose_sub = self.create_subscription(
            PoseStamped, "/ingress/wrist", self.rokoko_pose_callback, 10
        )

        self.arm_publisher = self.create_publisher(
            PoseStamped, "/franka/end_effector_pose_cmd", 10
        )

        self.tf_publisher = TransformBroadcaster(self)

        self.arm_subscriber = self.create_subscription(
            PoseStamped, "/franka/end_effector_pose", self.arm_pose_callback, 10
        )

    def create_transform(self, translation, rotation):
        return {"translation": np.array(translation), "rotation": rotation}

    def transform_to_pose(self, transform):
        return PoseStamped(
            pose={
                "position": Point(
                    x=transform["translation"][0],
                    y=transform["translation"][1],
                    z=transform["translation"][2],
                ),
                "orientation": Quaternion(
                    x=transform["rotation"].as_quat()[0],
                    y=transform["rotation"].as_quat()[1],
                    z=transform["rotation"].as_quat()[2],
                    w=transform["rotation"].as_quat()[3],
                ),
            }
        )

    def arm_pose_callback(self, msg: PoseStamped):
        orientation = msg.pose.orientation
        position = msg.pose.position
        self.X_W_fEE = self.create_transform(
            [position.x, position.y, position.z],
            R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]),
        )

    def rokoko_pose_callback(self, msg: PoseStamped):
        orientation = msg.pose.orientation
        position = msg.pose.position
        self.X_bCP_G = self.create_transform(
            [position.x, position.y, position.z],
            R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]),
        )
        self.publish_tf("coil_pro", self.X_W_bCP, "map")
        if self.X_W_fEE_init is not None:
            self.publish_tf("robot_init", self.X_W_fEE_init, "map")
        if self.X_bCP_G_init is not None:
            self.publish_tf("glove_init", self.X_bCP_G_init, "coil_pro")
        self.publish_tf("glove_wrist", self.X_bCP_G, "coil_pro")

    def get_hand_pose(self):
        return deepcopy(self.X_bCP_G)

    def compute_robot_target(self):
        if self.X_bCP_G_init is None or self.X_W_fEE_init is None:
            self.get_logger().warn("Initial poses not set yet")
            return self.X_W_fEE

        # Compute glove pose in world frame
        X_W_G_translation = self.X_W_bCP["translation"] + self.X_bCP_G["translation"]
        X_W_G_rotation = self.X_W_bCP["rotation"] * self.X_bCP_G["rotation"]

        # Transform delta and target computation
        X_W_fEE_d_translation = (
            self.X_W_fEE_init["translation"]
            + (X_W_G_translation - self.X_bCP_G_init["translation"])
        )
        X_W_fEE_d_rotation = self.X_W_fEE_init["rotation"] * X_W_G_rotation.inv()

        return self.create_transform(X_W_fEE_d_translation, X_W_fEE_d_rotation)

    def publish_target_pose(self, X_W_fEE_d):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position = Point(
            x=X_W_fEE_d["translation"][0],
            y=X_W_fEE_d["translation"][1],
            z=X_W_fEE_d["translation"][2],
        )
        quat = X_W_fEE_d["rotation"].as_quat()
        msg.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        self.arm_publisher.publish(msg)

    def calibrate(self):
        input("Press enter to move the robot to init pose")

        self.X_W_fEE_init = deepcopy(self.X_W_fEE)
        while self.X_W_fEE_init is None:
            print("Waiting for robot pose...")
            time.sleep(0.2)
            self.X_W_fEE_init = deepcopy(self.X_W_fEE)

        start_target = np.array(
            [[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0.5, -0.1, 0.25, 1]]
        ).T
        start_target = DrakeRigidTransform(start_target)
        self.publish_target_pose(start_target)

        input("Press enter to calibrate the robot")
        self.X_W_fEE_init = deepcopy(start_target)

        input("Press enter to calibrate the glove")
        self.X_bCP_G_init = self.get_hand_pose()
        while self.X_bCP_G_init is None:
            print("Waiting for glove pose...")
            time.sleep(0.2)
            self.X_bCP_G_init = self.get_hand_pose()

        self.get_logger().info("Calibration complete")

    def publish_tf(self, frame_id, transform, parent_frame_id):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent_frame_id
        msg.child_frame_id = frame_id
        msg.transform.translation = Vector3(
            x=transform["translation"][0],
            y=transform["translation"][1],
            z=transform["translation"][2],
        )
        quat = transform["rotation"].as_quat()
        msg.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        self.tf_publisher.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)

    rokoko_coil_demo = RokokoCoilDemo()

    import threading

    spin_thread = threading.Thread(target=rclpy.spin, args=(rokoko_coil_demo,))
    spin_thread.start()

    rokoko_coil_demo.calibrate()

    r = rokoko_coil_demo.create_rate(50)
    while rclpy.ok():
        try:
            X_W_fEE_d = rokoko_coil_demo.compute_robot_target()
            rokoko_coil_demo.publish_target_pose(X_W_fEE_d)
            r.sleep()
        except KeyboardInterrupt:
            break

    print("rokoko_coil_demo Done")
    spin_thread.join()
    rclpy.shutdown()


if __name__ == "__main__":
    main()