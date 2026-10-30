#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
import time


class GestureRunner(Node):
    def __init__(self):
        super().__init__('gesture_runner')

        # === Parameters ===
        self.declare_parameter('height_threshold', 0.10)     # [m]
        self.declare_parameter('frequency', 10.0)             # [Hz] check frequency
        self.declare_parameter('retargeter_set_enable_topic', '/right/retargeter/set_enabled')
        self.declare_parameter('enable_at_start', True)


        # === Internal state ===
        self.left_wrist_z = None
        self.right_wrist_z = None
        self.left_wrist_x = None
        self.right_wrist_x = None
        self.left_wrist_active = 0
        self.right_wrist_active = 0
        self.retargeter_enabled = self.get_parameter('enable_at_start').value

        self.last_condition_met = False
        self.condition_start_time = None

        # === Load parameters ===
        self.height_threshold = self.get_parameter('height_threshold').value
        self.frequency = self.get_parameter('frequency').value
        retargeter_set_enable_topic = self.get_parameter('retargeter_set_enable_topic').value

        # === Service client ===
        self.retargeter_set_enable_client = self.create_client(SetBool, retargeter_set_enable_topic)
        while not self.retargeter_set_enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for retargeter enable service...')

        # === Subscribers ===
        self.create_subscription(PoseStamped, '/ingress/right/wrist', self.right_wrist_callback, 10)
        self.create_subscription(PoseStamped, '/ingress/left/wrist', self.left_wrist_callback, 10)

        # === Timer to periodically check condition ===
        self.timer = self.create_timer(1.0 / self.frequency, self.evaluate_condition)
        self.get_logger().info(f'✅ GestureRunner started (freq={self.frequency} Hz, threshold={self.height_threshold} m).')

        self.toggle_retargeter(bool(self.retargeter_enabled))

    # -----------------------------------------------------
    def right_wrist_callback(self, msg: PoseStamped):
        self.right_wrist_z = msg.pose.position.z
        if self.right_wrist_x is None:
            self.right_wrist_x = msg.pose.position.x
        else:
            self.right_wrist_active = abs(msg.pose.position.x - self.right_wrist_x) * 0.5 + self.right_wrist_active * 0.5
            self.right_wrist_x = msg.pose.position.x

    def left_wrist_callback(self, msg: PoseStamped):
        self.left_wrist_z = msg.pose.position.z
        if self.left_wrist_x is None:
            self.left_wrist_x = msg.pose.position.x
        else:
            self.left_wrist_active = abs(msg.pose.position.x - self.left_wrist_x) * 0.5 + self.left_wrist_active * 0.5
            self.left_wrist_x = msg.pose.position.x

    # -----------------------------------------------------
    def evaluate_condition(self):
        if self.left_wrist_z is None or self.right_wrist_z is None:
            return  # wait until both wrists are received
        
        # self.get_logger().info(f"Left wrist z: {self.left_wrist_z:.3f} (active: {self.left_wrist_active>1e-7}), Right wrist z: {self.right_wrist_z:.3f} (active: {self.right_wrist_active>1e-7})")

        height_diff = abs(self.right_wrist_z - self.left_wrist_z)
        condition_met = (height_diff > self.height_threshold) and self.left_wrist_active > 1e-7 and self.right_wrist_active > 1e-7

        current_time = time.time()

        # Track how long condition stays True/False
        if condition_met != self.last_condition_met:
            # reset timer when condition flips
            self.condition_start_time = current_time
            self.last_condition_met = condition_met
            return

        # If condition has been stable for >= 1 second, act if necessary
        if self.condition_start_time is not None and (current_time - self.condition_start_time) >= 1.0:
            if condition_met and not self.retargeter_enabled:
                self.toggle_retargeter(True)
            elif not condition_met and self.retargeter_enabled:
                self.toggle_retargeter(False)

    # -----------------------------------------------------
    def toggle_retargeter(self, enable: bool):
        req = SetBool.Request()
        req.data = enable

        future = self.retargeter_set_enable_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_response(f, enable))

    def _handle_response(self, future, enable):
        try:
            response = future.result()
            if response.success:
                self.retargeter_enabled = enable
                state = 'ENABLED' if enable else 'DISABLED'
                self.get_logger().info(f'Retargeter successfully {state}.')
            else:
                self.get_logger().warn(f'Retargeter service call failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GestureRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
