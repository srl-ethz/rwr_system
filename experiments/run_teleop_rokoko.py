#!/bin/env python3

import numpy as np
import time
from copy import deepcopy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped, Vector3, Quaternion, PoseStamped
from tf2_ros import TransformBroadcaster

from pydrake.math import RigidTransform as DrakeRigidTransform
from pydrake.math import RollPitchYaw as DrakeRollPitchYaw
from pydrake.common.eigen_geometry import Quaternion as DrakeQuaternion



class RokokoCoilDemo(Node):
    def __init__(self):
        super().__init__("rokoko_coil_demo")

        self.X_bCP_G = None  # DrakeRigidTransform from coil pro to glove wrist
        self.X_bCP_G_init = (
            None  # Initial DrakeRigidTransform from coil pro to glove wrist
        )

        self.X_W_bCP = DrakeRigidTransform(
            p=[1.0, 2.0, -1.3],
            R=DrakeRollPitchYaw(
                np.deg2rad(np.array([90.0, 0.0, 0.0]))
            ).ToRotationMatrix(),
        )
        self.X_W_fEE_d = None
        self.X_W_fEE_init = None
        self.X_W_fEE = None

        self.rokoko_pose_sub = self.create_subscription(
            PoseStamped, "/ingress/wrist", self.rokoko_pose_callback, 10
        )

        # publishes to franka/end_effector_pose_cmd
        self.arm_publisher = self.create_publisher(
            PoseStamped, "/franka/end_effector_pose_cmd", 10
        )

        self.tf_publisher = TransformBroadcaster(self)

        self.arm_subscriber = self.create_subscription(
            PoseStamped, "/franka/end_effector_pose", self.arm_pose_callback, 10
        )

    def arm_pose_callback(self, msg: PoseStamped):
        orientation = msg.pose.orientation
        position = msg.pose.position
        # Get the pose of the wrist
        self.X_W_fEE = DrakeRigidTransform(
            DrakeQuaternion(orientation.w, orientation.x, orientation.y, orientation.z),
            [position.x, position.y, position.z],
        )

    def rokoko_pose_callback(self, msg: PoseStamped):
        orientation = msg.pose.orientation
        position = msg.pose.position
        # Get the pose of the wrist
        self.X_bCP_G = DrakeRigidTransform(
            DrakeQuaternion(orientation.w, orientation.x, orientation.y, orientation.z),
            [position.x, position.y, position.z],
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

        # Compute the pose of the glove in world frame
        self.X_W_G = self.X_W_bCP @ self.X_bCP_G
        self.Xr_W_G = DrakeRigidTransform(self.X_W_G.rotation())

        self.Xt_W_G_delta = self.Xt_W_G_init.inverse() @ DrakeRigidTransform(
            p=self.X_W_G.translation()
        )

        # Compute the target pose for the robot
        X_W_fEE_d = (
            self.Xt_W_G_delta
            @ self.Xt_W_fEE_init
            @ self.Xr_W_G
            @ self.Xr_W_G_init_fEE_init
        )

        return X_W_fEE_d

    def publish_target_pose(self, X_W_fEE_d: DrakeRigidTransform):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position = Point(
            x=X_W_fEE_d.translation()[0],
            y=X_W_fEE_d.translation()[1],
            z=X_W_fEE_d.translation()[2],
        )
        quat = X_W_fEE_d.rotation().ToQuaternion()
        msg.pose.orientation = Quaternion(
            x=quat.x(), y=quat.y(), z=quat.z(), w=quat.w()
        )
        self.arm_publisher.publish(msg)

    def calibrate(self):
        input("Press enter to move the robot to init pose")

        # Assume robot is at home pose
        self.X_W_fEE_init = deepcopy(self.X_W_fEE)
        while self.X_W_fEE_init is None:
            print("Waiting for robot pose...")
            time.sleep(0.2)
            self.X_W_fEE_init = deepcopy(self.X_W_fEE)

        start_target = np.array(
            [[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0.45, -0.25, 0.25, 1]]
        ).T

        R_z_neg90 = np.array([
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Perform the matrix multiplication
        start_target = np.dot(start_target, R_z_neg90)


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

        print(f"Initial pose of the robot: {self.X_W_fEE_init}")
        print(f"Initial pose of the glove: {self.X_bCP_G_init}")

        self.Xr_W_fEE_init = DrakeRigidTransform(self.X_W_fEE_init.rotation())
        self.Xt_W_fEE_init = DrakeRigidTransform(self.X_W_fEE_init.translation())

        self.Xr_bCP_G_init = DrakeRigidTransform(self.X_bCP_G_init.rotation())

        self.X_W_G_init = self.X_W_bCP @ self.X_bCP_G_init

        self.Xt_W_G_init = DrakeRigidTransform(p=self.X_W_G_init.translation())
        self.Xr_W_G_init = DrakeRigidTransform(R=self.X_W_G_init.rotation())
        self.Xr_W_G_init_inv = self.Xr_W_G_init.inverse()

        self.Xr_W_G_init_fEE_init = self.Xr_W_G_init.inverse() @ self.Xr_W_fEE_init

        self.get_logger().info("Calibration complete")

    def publish_tf(self, frame_id, X: DrakeRigidTransform, parent_frame_id):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent_frame_id
        msg.child_frame_id = frame_id
        msg.transform.translation = Vector3(
            x=X.translation()[0], y=X.translation()[1], z=X.translation()[2]
        )
        quat = X.rotation().ToQuaternion()
        msg.transform.rotation = Quaternion(
            x=quat.x(), y=quat.y(), z=quat.z(), w=quat.w()
        )
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
