#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool
import os
from ament_index_python.packages import get_package_share_directory


class HandControllerNode(Node):
    def __init__(self, debug=False):
        super().__init__("hand_controller_node")

        # Determine which hand controller to use
        self.declare_parameter("hand_controller/hand_name", "orca_v2")
        self.declare_parameter("hand_controller/hand_chirality", "right")
        self.declare_parameter("hand_controller/calibrate", False)
        self.declare_parameter('hand_controller/compliant_test_mode', False)

        self.hand_name = self.get_parameter("hand_controller/hand_name").value
        self.chirality = self.get_parameter("hand_controller/hand_chirality").value
        self.calibrate = self.get_parameter("hand_controller/calibrate").value
        self.compliant_test_mode = self.get_parameter('hand_controller/compliant_test_mode').value
        self.get_logger().info(f"Hand Controller Node Started with hand: {self.hand_name}")
        self.get_logger().info(f"Hand Controller Node Started with chirality: {self.chirality}")

        # Franka gripper
        if self.hand_name == "franka_hand":
            from franka_gripper.hand_controller import HandController as FrankaGripperHandController
            self._hc = FrankaGripperHandController(hand_chirality = "right" if self.get_parameter("hand_controller/hand_chirality").value == "right" else "left")

        # Orca v2 hand, driven through the orca_core package
        elif self.hand_name == "orca_v2":
            self.declare_parameter('hand_controller/hand_cfg_name', 'orcahand_v2')
            hand_cfg_name = self.get_parameter('hand_controller/hand_cfg_name').value

            config_dir_path = os.path.join(
                get_package_share_directory("hand_control"),
                "orca",
                "configs",
                hand_cfg_name
            )
            # publisher for sensor values (in radians)
            self.joint_angle_pub = self.create_publisher(Float32MultiArray, f'/{self.chirality}/hand/joint_position_from_motor', 10)
            self.joint_velocity_pub = self.create_publisher(Float32MultiArray, f'/{self.chirality}/hand/joint_velocity_from_motor', 10)

            from faive_system.src.hand_control.orca.hand_controller_v2 import OrcaHandV2
            # orca_core requires the file name config.yaml; calibration.yaml is stored next to it
            self._hc = OrcaHandV2(os.path.join(config_dir_path, self.chirality, "config.yaml"))
            self._hc.init_joints(calibrate=self.calibrate, initialize=True, compliant_test_mode=self.compliant_test_mode)
            self._orca_timer = self.create_timer(
                0.01, # seconds
                self.orca_v2_control_cb,  # callback
            )

        else:
            raise NotImplementedError("Hand name not recognized")

        self.joint_angle_sub = self.create_subscription(
            Float32MultiArray, f"/{self.chirality}/hand/policy_output", self.joint_angle_cb, 0
        )
        
        # Add emergency stop service, kills itself when value is True
        self.estop_service = self.create_service(
            SetBool, f"/{self.chirality}/hand/estop", self.faive_estop_cb
        )

        # Service to enable/disable torque on the underlying hand controller.
        self.torque_service = self.create_service(
            SetBool,
            f"/{self.chirality}/hand/set_torque_enabled",
            self.set_torque_enabled_cb,
        )

    def joint_angle_cb(self, msg):
        num_motors = self._hc.num_motors
        if len(msg.data) != num_motors:
            self.get_logger().error(f"Expected {num_motors} joint angles, got {len(msg.data)}")
            return
        joint_angles = np.array(msg.data)
        joint_angles = np.rad2deg(joint_angles)
        if self.hand_name == "orca_v2":
            self._hc.set_joint_pos_array(joint_angles)
        else:
            self._hc.command_joint_angles(joint_angles)


    def orca_v2_control_cb(self):
        """
        publish joint angles and velocities from the orca_core hand to ROS
        """
        q_deg, qd_deg = self._hc.read_joint_state()
        self.joint_angle_pub.publish(Float32MultiArray(data=np.deg2rad(q_deg).astype(np.float32)))
        self.joint_velocity_pub.publish(Float32MultiArray(data=np.deg2rad(qd_deg).astype(np.float32)))


    def set_torque_enabled_cb(self, request, response):
        """
        Enable or disable torque on the underlying hand controller via a SetBool service.
        """
        try:
            if request.data:
                self._hc.enable_torque()
                self.get_logger().info("Torque enabled via service")
                response.success = True
                response.message = "Torque enabled"
            else:
                self._hc.disable_torque()
                self.get_logger().info("Torque disabled via service")
                response.success = True
                response.message = "Torque disabled"
        except Exception as e:
            self.get_logger().error(f"Failed to set torque: {e}")
            response.success = False
            response.message = str(e)
        return response

    def faive_estop_cb(self, request, response):
        if request.data:
            self.get_logger().warn("Emergency stop received, shutting down")
            if self.hand_name == "orca_v2":
                # stop reading and commanding before the bus goes away. Without this, an e-stop crashes the node
                if not self._orca_timer.is_canceled():
                    self._orca_timer.cancel()
                    self.destroy_subscription(self.joint_angle_sub)
                    self._hc.disconnect()
            else:
                self._hc.disable_torque()
                self._hc.disconnect_from_dynamixels()

            response.success = True
            response.message = "Emergency stop executed"
            return response
        else:
            response.success = False
            response.message = "contents not True, Emergency stop not executed"
            return response

def main(args=None):
    rclpy.init(args=args)
    node = HandControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
