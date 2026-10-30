#!/bin/env python3

import numpy as np
import time
from copy import deepcopy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped, Vector3, Quaternion, PoseStamped
from tf2_ros import TransformBroadcaster

from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger
from faive_system.src.common.transformation_utils import rigid_transform, complete_rotation_matrix
from faive_system.src.common.enums import TeleopState, HandChirality

# Deprecated flags. Can be useful for running the node without launch file (toggle flow and fast mode)
LEFT_HAND=True
RIGHT_HAND=True

class RokokoCoilDemo(Node):
    def __init__(self, chirality=HandChirality.RIGHT):
        super().__init__(f"{chirality}_rokoko_coil_demo")
        self.declare_parameter("enable_left", False)      # global parameter for cross embodiment
        self.declare_parameter("enable_right", True)      # global parameter for cross embodiment
        self.declare_parameter("rokoko_source_frame_name", "wrist")      # global parameter for cross embodiment
        # interactive=False keeps the node purely service driven: no stdin prompts, so another
        # node (e.g. the HIL policy node) can own the terminal and call pause/resume itself.
        self.declare_parameter("interactive", True)

        self.enable_left = self.get_parameter("enable_left").value
        self.enable_right = self.get_parameter("enable_right").value
        self.rokoko_source_frame_name = self.get_parameter("rokoko_source_frame_name").value
        self.interactive = self.get_parameter("interactive").value

        enabled = False
        if chirality=="left" and self.enable_left:
            enabled = True
        elif chirality=="right" and self.enable_right:
            enabled = True
        if not enabled:
            return

        self.X_bCP_G = None  # rigid_transform from coil pro to glove wrist
        self.X_bCP_G_init = (
            None  # Initial rigid_transform from coil pro to glove wrist
        )

        self.X_W_bCP = rigid_transform( # from Coil pro in world frame. Translation doesn't matter
            [0.5, 0.0, 0.0],
            [0.7071, 0.0, 0.0, 0.7071]
        )

        self.chirality = chirality

        self.X_W_fEE_d = None
        self.X_W_fEE_init = None
        self.X_W_fEE = None

        self.rokoko_pose_sub = self.create_subscription(
            PoseStamped, f"/ingress/{self.chirality}/{self.rokoko_source_frame_name}", self.rokoko_pose_callback, 10
        )

        # publishes to arm/end_effector_pose_cmd
        self.arm_publisher = self.create_publisher(
            PoseStamped, f"/{self.chirality}/arm/end_effector_pose_cmd", 10
        )

        self.tf_publisher = TransformBroadcaster(self)

        self.arm_subscriber = self.create_subscription(
            PoseStamped, f"/{self.chirality}/arm/end_effector_pose", self.arm_pose_callback, 10
        )

        # the teleop is modeled as a state machine
        # 0: unready
        # 1: paused
        # 2: running
        # Transition from unready to paused is to move the robot to default position
        # Transition from paused to running requires the calibration of the hand
        self._state = TeleopState.UNREADY
        self.DT = 0.02
        self.timer = self.create_timer(self.DT, self.main_loop_cb)

        # Create Pause and Resume services
        self.move2init_service = self.create_service(
            Trigger, 
            f"/franka_teleop/{self.chirality}/move2init",   # service name
            self.move2init_callback
        )
        self.pause_service = self.create_service(
            Trigger, 
            f"/franka_teleop/{self.chirality}/pause",   # service name
            self.pause_callback
        )
        self.resume_service = self.create_service(
            Trigger,
            f"/franka_teleop/{self.chirality}/resume",  # service name
            self.resume_callback
        )

        self._calc_translation_func = self.calc_translation_ScaledPosMapping
        # self._calc_translation_func = self.calc_translation_ClampV
        # self._calc_translation_func = self.calc_translation_VJoyStick

    def move2init_callback(self, request, response):
        if self._state == TeleopState.UNREADY: 
            response.success = False
            print("Waiting for robot pose")
            return response
        if self._state == TeleopState.RUNNING: 
            response.success = False
            print("Cannot move to init pose while teleop is running")
            return response
        
        if self.chirality == HandChirality.LEFT:
            start_target_pos = np.array([0.40, 0.36, 0.2])
            start_target_rot = complete_rotation_matrix(x=[1, 0, 0], z=[0, 0, -1])
        else:
            start_target_pos = np.array([0.45, -0.36, 0.2])
            start_target_rot = complete_rotation_matrix(x=[1, 0, 0], z=[0, 0, -1])

        start_target = rigid_transform(start_target_pos, Rotation.from_matrix(start_target_rot).as_quat())
        self.publish_target_pose(start_target)
        self.X_W_fEE_init = deepcopy(start_target)
        self.Xr_W_fEE_init = np.eye(4)
        self.Xr_W_fEE_init[:3,:3] = self.X_W_fEE_init[:3,:3]
        self.Xt_W_fEE_init = rigid_transform(self.X_W_fEE_init[:3,3], [0,0,0,1])

        response.success = True
        return response
    
    def pause_callback(self, request, response):
        if self._state == TeleopState.UNREADY:
            response.success = False
            return response
        self._state = TeleopState.PAUSED
        response.success = True
        return response

    def resume_callback(self, request, response):
        if self._state == TeleopState.UNREADY:
            response.success = False
            return response

        if self.get_hand_pose() is None:
            # Resuming without a glove pose cannot be calibrated. Refuse instead of raising,
            # so a caller driving this service (HIL takeover) just stays paused.
            self.get_logger().warn("Cannot resume: no glove pose received yet")
            response.success = False
            return response

        self.init_glove_pose()
        self._state = TeleopState.RUNNING
        response.success = True
        return response

    def arm_pose_callback(self, msg: PoseStamped):
        orientation = msg.pose.orientation
        position = msg.pose.position
        # Get the pose of the wrist
        self.X_W_fEE = rigid_transform(
            [position.x, position.y, position.z],
            [orientation.x, orientation.y, orientation.z, orientation.w],
        )

    def rokoko_pose_callback(self, msg: PoseStamped):
        orientation = msg.pose.orientation
        position = msg.pose.position
        # Get the pose of the wrist
        self.X_bCP_G = rigid_transform(
            [position.x, position.y, position.z],
            [orientation.x, orientation.y, orientation.z, orientation.w],
        )

        if self.X_W_fEE_init is not None:
            self.publish_tf(f"{self.chirality}_robot_init", self.X_W_fEE_init, f"{self.chirality}_panda_link0")

        self.publish_tf(f"{self.chirality}_coil_pro", self.X_W_bCP, f"{self.chirality}_panda_link0")
        if self.X_bCP_G_init is not None:
            self.publish_tf(f"{self.chirality}_glove_init", self.X_bCP_G_init, "coil_pro")
        self.publish_tf(f"{self.chirality}_glove_wrist", self.X_bCP_G, "coil_pro")

    def get_hand_pose(self):
        return deepcopy(self.X_bCP_G)

    def main_loop_cb(self):
        """
        Main loop for the rokoko_coil_demo node. Modeled as a state machine
        """
        if self._state == TeleopState.UNREADY: # waiting robot pose
            self.X_W_fEE_init = deepcopy(self.X_W_fEE)
            if self.X_W_fEE_init is not None:
                self.Xr_W_fEE_init = np.eye(4)
                self.Xr_W_fEE_init[:3,:3] = self.X_W_fEE_init[:3,:3]
                self.Xt_W_fEE_init = rigid_transform(self.X_W_fEE_init[:3,3], [0,0,0,1])
                self._state = TeleopState.PAUSED
        elif self._state == TeleopState.PAUSED: # paused
            pass
        elif self._state == TeleopState.RUNNING:
            X_W_fEE_d = self.compute_robot_target()
            self.publish_target_pose(X_W_fEE_d)


    def compute_robot_target(self):
        if self.X_bCP_G_init is None or self.X_W_fEE_init is None:
            self.get_logger().warn("Initial poses not set yet")
            return self.X_W_fEE

        # Compute the pose of the glove in world frame
        X_W_G = self.X_W_bCP @ self.X_bCP_G
        Xr_W_G = np.eye(4)
        Xr_W_G[:3,:3] = X_W_G[:3,:3]

        Xt_W_fEE_d = self._calc_translation_func(X_W_G)
        # Compute the target pose for the robot
        X_W_fEE_d = (
            Xt_W_fEE_d
            @ Xr_W_G
            @ self.Xr_W_G_init_fEE_init
        )

        return X_W_fEE_d

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

    def init_glove_pose(self):

        self.X_W_fEE_init = deepcopy(self.X_W_fEE)
        if self.X_W_fEE_init is not None:
            self.Xr_W_fEE_init = np.eye(4)
            self.Xr_W_fEE_init[:3,:3] = self.X_W_fEE_init[:3,:3]
            self.Xt_W_fEE_init = rigid_transform(self.X_W_fEE_init[:3,3], [0,0,0,1])

        self.X_bCP_G_init = self.get_hand_pose()
        self.Xr_bCP_G_init = np.eye(4)
        self.Xr_bCP_G_init[:3,:3] = self.X_bCP_G_init[:3,:3]
        
        self.X_W_G_init = self.X_W_bCP @ self.X_bCP_G_init

        self.Xr_W_G_init = np.eye(4)
        self.Xr_W_G_init[:3,:3] = self.X_W_G_init[:3,:3]
        
        self.Xt_W_G_init = rigid_transform(self.X_W_G_init[:3,3], [0,0,0,1])
        self.Xr_W_G_init_inv = np.linalg.inv(self.Xr_W_G_init)

        self.Xr_W_G_init_fEE_init = np.linalg.inv(self.Xr_W_G_init) @ self.Xr_W_fEE_init

        self.get_logger().info("Calibration complete")

        
    def publish_tf(self, frame_id, X, parent_frame_id):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent_frame_id
        msg.child_frame_id = frame_id
        msg.transform.translation = Vector3(
            x=X[0,3], y=X[1,3], z=X[2,3]
        )
        quat = Rotation.from_matrix(X[:3,:3]).as_quat() # x, y, z, w
        msg.transform.rotation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )
        self.tf_publisher.sendTransform(msg)


    ### Implementations of different ways to calculate the translation cmd
    def calc_translation_VJoyStick(self, X_W_G):
        """
        assume the glove is a virtual joystick. Translate the position difference into velocity
        """
        if not hasattr(self, "_vjoy_gain"):
            self._vjoy_gain = 5.0
        if not hasattr(self, "_vjoy_deadzone"):
            self._vjoy_deadzone = 0.05
        
        t_W_G_delta = X_W_G[:3,3] - self.Xt_W_G_init[:3,3]
        if np.linalg.norm(t_W_G_delta) < self._vjoy_deadzone:
            t_W_G_delta *= 0
        t_W_G_delta *= self._vjoy_gain
        Xt_W_fEE_d = rigid_transform(t_W_G_delta* self.DT + self.X_W_fEE[:3,3],[0,0,0,1])
        return Xt_W_fEE_d

    ### Implementations of different ways to calculate the translation cmd
    def calc_translation_ClampV(self, X_W_G):
        """
        Calculate the difference between the previous step, clamp the velocity
        """
        if not hasattr(self, "_pdiff_max"):
            self._pdiff_max = 0.01
        if not hasattr(self, "X_W_G_prev"):
            self.X_W_G_prev = deepcopy(X_W_G)

        t_W_G_delta = X_W_G[:3,3] - self.X_W_G_prev[:3,3]
        self.X_W_G_prev = deepcopy(X_W_G)
        # print(np.linalg.norm(t_W_G_delta))
        # clamp the diff
        if np.linalg.norm(t_W_G_delta) > self._pdiff_max:
            t_W_G_delta = self._pdiff_max * t_W_G_delta / np.linalg.norm(t_W_G_delta)

        Xt_W_fEE_d = rigid_transform(t_W_G_delta + self.X_W_fEE[:3,3],[0,0,0,1])
        return Xt_W_fEE_d

    def calc_translation_ScaledPosMapping(self, X_W_G):
        """
        The robot follows the difference in the position of the glove, but the velocity is clamped
        """
        if not hasattr(self, "_pos_mapping_scale"):
            self._pos_mapping_scale = 1.0

        Xt_W_G_delta = np.linalg.inv(self.Xt_W_G_init) @ rigid_transform(
            X_W_G[:3,3],
            [0,0,0,1]
        )
        Xt_W_G_delta[:3,3] *= self._pos_mapping_scale
        Xt_W_fEE_d =  Xt_W_G_delta @ self.Xt_W_fEE_init
        return Xt_W_fEE_d


def main(args=None):
    rclpy.init(args=args)
    left_rcd = RokokoCoilDemo(chirality=HandChirality.LEFT) if LEFT_HAND else None
    right_rcd = RokokoCoilDemo(chirality=HandChirality.RIGHT) if RIGHT_HAND else None

    import threading

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(left_rcd) if LEFT_HAND else None
    executor.add_node(right_rcd) if RIGHT_HAND else None

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    while left_rcd and left_rcd.X_W_fEE is None:
        print("Waiting for left robot pose...")
        time.sleep(1)

    while right_rcd and right_rcd.X_W_fEE is None:
        print("Waiting for right robot pose...")
        time.sleep(1)

    interactive = next((rcd.interactive for rcd in [left_rcd, right_rcd] if rcd is not None), True)
    if not interactive:
        # Service driven mode (HIL): the robot is initialized by whoever owns it, and
        # pause/resume come from /franka_teleop/<chirality>/{pause,resume}. Just serve.
        print("Teleop running in non-interactive mode, waiting for pause/resume service calls")
        spin_thread.join()
        rclpy.shutdown()
        return

    input("Press enter to move the robot to init pose")
    result = left_rcd.move2init_callback(Trigger.Request(), Trigger.Response()) if LEFT_HAND else None
    result = right_rcd.move2init_callback(Trigger.Request(), Trigger.Response()) if RIGHT_HAND else None

    input("Press enter to calibrate the gloves")
    X_bCP_lG_init, X_bCP_rG_init = None, None

    for rcd, X_bCP_G_init in zip([left_rcd, right_rcd], [X_bCP_lG_init, X_bCP_rG_init]):
        if rcd is None:
            continue
        while X_bCP_G_init is None:
            print(f"Waiting for {rcd.chirality} glove pose...")
            time.sleep(0.2)
            X_bCP_G_init = rcd.get_hand_pose()
        result = rcd.resume_callback(Trigger.Request(), Trigger.Response())

    keep_running = True
    try:
        while keep_running:
            input("Press enter to pause")

            result = left_rcd.pause_callback(Trigger.Request(), Trigger.Response()) if LEFT_HAND else None
            result = right_rcd.pause_callback(Trigger.Request(), Trigger.Response()) if RIGHT_HAND else None

            input("Press enter to resume to accurate mode")
            for rcd in [left_rcd, right_rcd]:
                if rcd is None:
                    continue   
                # # use the v joy stick
                # rcd._calc_translation_func = rcd.calc_translation_VJoyStick
        

                # use the pose scale
                rcd._calc_translation_func = rcd.calc_translation_ScaledPosMapping 
                rcd._pos_mapping_scale = 0.6
        
            result = left_rcd.resume_callback(Trigger.Request(), Trigger.Response()) if LEFT_HAND else None
            result = right_rcd.resume_callback(Trigger.Request(), Trigger.Response())  if RIGHT_HAND else None

            input("Press enter to pause")
            result = left_rcd.pause_callback(Trigger.Request(), Trigger.Response()) if LEFT_HAND else None
            result = right_rcd.pause_callback(Trigger.Request(), Trigger.Response()) if RIGHT_HAND else None

            input("Press enter to resume to normal mode")
            for rcd in [left_rcd, right_rcd]:
                if rcd is None:
                    continue   
                rcd._calc_translation_func = rcd.calc_translation_ScaledPosMapping
                rcd._pos_mapping_scale = 1.0

            result = left_rcd.resume_callback(Trigger.Request(), Trigger.Response()) if LEFT_HAND else None
            result = right_rcd.resume_callback(Trigger.Request(), Trigger.Response())  if RIGHT_HAND else None


    except KeyboardInterrupt:
        keep_running = False

    spin_thread.join()
    rclpy.shutdown()

if __name__ == "__main__":
    main()