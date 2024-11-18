#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import sys
import os
import time
from mujoco_visualizer import MuJoCoVisualizer

class MuJoCoVisualizer(Node):
    def __init__(self):
        super().__init__("mujoco_visualizer_node")

        # Declare Parameters
        self.declare_parameter("viz/model_path", rclpy.Parameter.Type.STRING)

        # Initialize MuJoCo Visualizer
        self.mujoco_visualizer = MuJoCoVisualizer(sim_model=self.get_parameter("viz/model_path").value)

        # Subscribe to target joint angles
        self._subscription = self.create_subscription(
            Float32MultiArray,
            "/hand/policy_output",
            self._policy_output_callback,
            10
        )
        self.get_logger().info('Subscribing to "/hand/policy_output"')


        self.get_logger().info("MuJoCo Visualizer Started")

    def _policy_output_callback(self, msg):
        # Retrieve joints data
        joints = msg.data

        # Send commands to the MuJoCo Simulation
        self.mujoco_visualizer.command_joint_angles(joints)

        # time.sleep(0.1)

