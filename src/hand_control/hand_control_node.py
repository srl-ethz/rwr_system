#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import os
from faive_system.src.hand_control.hand_controller import HandController

class HandControllerNode(Node):
    def __init__(self, debug=False):
        super().__init__("hand_controller_node")
        self.get_logger().info("Hand Controller Node Started")

        # start tracker
        self.declare_parameter("hand_controller/port", "/dev/ttyUSB0")
        self.declare_parameter("hand_controller/baudrate", 3000000)

        port = self.get_parameter("hand_controller/port").value
        baudrate = self.get_parameter("hand_controller/baudrate").value
        # auto_calibrate = self.get_parameter("hand_controller/auto_calibrate").value

        # self._hc = HandController(port=port, baudrate=baudrate)

        self._hc = HandController(port=port, calibration=False, auto_calibrate= False)
        # self.get_logger().info("Hand Controller Before INIT =========================")

        self._hc.init_joints(calibrate=False)
        
        self.joint_angle_sub = self.create_subscription(
            Float32MultiArray, "/hand/policy_output", self.joint_angle_cb, 10
        )

    def joint_angle_cb(self, msg):
        assert len(msg.data) == 17, "Expected 17 joint angles, got {}".format(
            len(msg.data)
        )
        joint_angles = np.array(msg.data)
        joint_angles_deg = joint_angles * 180 / np.pi
        # self._hc.command_joint_angles(joint_angles_deg)

        # print(f"Node thumb angles: {joint_angles[1:5]}")
        self._hc.write_desired_joint_angles(joint_angles_deg)


def main(args=None):
    rclpy.init(args=args)
    node = HandControllerNode()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
