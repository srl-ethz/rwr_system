#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import threading
import sys
import time


class PolicyPublisher(Node):
    def __init__(self):
        super().__init__('policy_publisher')

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/right/hand/policy_output',
            10
        )

        # -------------------------------
        # Keypoints (EDIT THESE)
        # Each must be length 17
        # -------------------------------
        self.keypoints = [
            np.array([0.89, -0.25, 0.96, 0.51, 0.0, 0.0, 0.0, 0.0, 2.0, 1.40, 0.0, 1.40, 1.42, 0.0, 1.36, 1.37, 0.0])
            for i in range(10*10+2)
        ]
        print(self.keypoints)
        for i in range(10):
            self.keypoints[10*i+1][6] = 0.12 * (i+1)
            self.keypoints[10*i+3][6] = 0.12 * (i+1)
            self.keypoints[10*i+5][6] = 0.12 * (i+1)
            self.keypoints[10*i+7][6] = 0.12 * (i+1)
            self.keypoints[10*i+9][6] = 0.12 * (i+1)

        self.current_idx = 0
        self.current_cmd = self.keypoints[0].copy()

        # Interpolation state
        self.in_transition = False
        self.t_start = 0.0
        self.duration = 1.0  # seconds
        self.start_cmd = None
        self.target_cmd = None

        # Publish loop
        self.dt = 0.01  # 100 Hz
        self.timer = self.create_timer(self.dt, self.update)

        # Keyboard listener (blocking input in another thread)
        self.input_thread = threading.Thread(
            target=self.wait_for_enter,
            daemon=True
        )
        self.input_thread.start()

        self.get_logger().info(
            "Policy publisher running. Press ENTER to move to next keypoint."
        )

    def wait_for_enter(self):
        while rclpy.ok():
            # input()
            time.sleep(1.75)
            self.start_transition()
            time.sleep(1.25)
            self.start_transition()

    def start_transition(self):
        if self.in_transition:
            return

        next_idx = (self.current_idx + 1) % len(self.keypoints)

        self.start_cmd = self.current_cmd.copy()
        self.target_cmd = self.keypoints[next_idx].copy()
        self.get_logger().info(f"target command {self.target_cmd}")
        self.t_start = time.time()
        self.in_transition = True
        self.current_idx = next_idx

        self.get_logger().info(
            f"Transitioning to keypoint {self.current_idx}"
        )

    def update(self):
        if self.in_transition:
            t = time.time() - self.t_start
            tau = np.clip(t / self.duration, 0.0, 1.0)

            # Sinusoidal interpolation (smooth)
            s = 0.5 * (1.0 - np.cos(np.pi * tau))
            self.current_cmd = (
                (1.0 - s) * self.start_cmd + s * self.target_cmd
            )

            if tau >= 1.0:
                self.current_cmd = self.target_cmd.copy()
                self.in_transition = False

        msg = Float32MultiArray()
        msg.data = self.current_cmd.astype(np.float32).tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PolicyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
