#!/usr/bin/env python3

import json
import threading
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool


class RetargeterCalibrationCollector(Node):
    """
    Minimal helper node to collect calibration pairs of:
        - MANO keypoints from the ingress
        - Ground-truth joint positions from the real hand (after manual adjustment)

    Usage (conceptual):
        - Start ingress, hand_control_node, retargeter, and this node.
        - In the terminal running this node:
            * Press Enter to pause/resume the retargeter and toggle compliant mode on the hand.
            * Press 's' + Enter to save the current pair of (MANO, joints).
            * Press 'q' + Enter to stop and write all collected samples to a JSON file.
    """

    def __init__(self) -> None:
        super().__init__("retargeter_calibration_collector")

        # Only keep chirality as a parameter; topics and behavior are fixed for simplicity.
        self.declare_parameter("retarget/hand_chirality", "right")
        self.hand_chirality = self.get_parameter("retarget/hand_chirality").value

        self.mano_topic = f"/ingress/{self.hand_chirality}/mano"
        self.joints_topic = f"/{self.hand_chirality}/hand/joint_position_from_motor"

        # Internal state
        self.latest_mano = None
        self.latest_joints = None
        self.samples = []
        self.paused = False  # False: retargeter on, torque on
        self.running = True

        # Subscriptions
        self.create_subscription(
            Float32MultiArray, self.mano_topic, self.mano_cb, 10
        )
        self.create_subscription(
            Float32MultiArray, self.joints_topic, self.joints_cb, 10
        )

        # Service client to enable/disable the retargeter.
        self._retargeter_enable_client = self.create_client(
            SetBool, f"/{self.hand_chirality}/retargeter/set_enabled"
        )

        # Service client to enable/disable torque on the hand controller.
        self._torque_client = self.create_client(
            SetBool,
            f"/{self.hand_chirality}/hand/set_torque_enabled",
        )

        self.get_logger().info(
            f"RetargeterCalibrationCollector started.\n"
            f"  MANO topic:   {self.mano_topic}\n"
            f"  Joints topic: {self.joints_topic}"
        )

    def mano_cb(self, msg: Float32MultiArray) -> None:
        try:
            data = np.array(msg.data, dtype=np.float32).reshape(-1, 3)
        except ValueError:
            self.get_logger().warn(
                f"Received MANO data of unexpected size ({len(msg.data)}), "
                "expected a multiple of 3."
            )
            return

        self.latest_mano = data

    def joints_cb(self, msg: Float32MultiArray) -> None:
        data = np.array(msg.data, dtype=np.float32)
        self.latest_joints = data

    def _set_retargeter_enabled(self, enabled: bool) -> None:
        # Best-effort: if the service is not available, just log and keep going.
        if not self._retargeter_enable_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                f"/{self.hand_chirality}/retargeter/set_enabled service not available; "
                "cannot toggle retargeter. "
                "Please ensure the retargeter node is running."
            )
            return

        req = SetBool.Request()
        req.data = enabled
        future = self._retargeter_enable_client.call_async(req)

        def _done_cb(fut):
            exc = fut.exception()
            if exc is not None:
                self.get_logger().error(
                    f"Failed to call /{self.hand_chirality}/retargeter/set_enabled: {exc}"
                )

        future.add_done_callback(_done_cb)

    def _set_torque_enabled(self, enabled: bool) -> None:
        # Best-effort: if the service is not available, just log and keep going.
        if not self._torque_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                "set_torque_enabled service not available; "
                "cannot change torque. "
                "Please ensure hand_control is running."
            )
            return

        req = SetBool.Request()
        req.data = enabled
        future = self._torque_client.call_async(req)

        def _done_cb(fut):
            exc = fut.exception()
            if exc is not None:
                self.get_logger().error(f"Failed to call set_torque_enabled: {exc}")

        future.add_done_callback(_done_cb)

    def toggle_pause(self) -> None:
        """
        Pause/resume retargeter and enable/disable torque on the hand.

        When paused:
            - retargeter is disabled
            - torque is disabled (hand compliant)
        When resumed:
            - torque is enabled
            - retargeter is re-enabled
        """
        if not self.paused:
            self.get_logger().info("Pausing retargeter and disabling torque...")
            self._set_retargeter_enabled(False)
            self._set_torque_enabled(False)
            self.paused = True
        else:
            self.get_logger().info("Resuming retargeter and enabling torque...")
            self._set_torque_enabled(True)
            self._set_retargeter_enabled(True)
            self.paused = False

    def save_current_sample(self) -> None:
        if self.latest_mano is None or self.latest_joints is None:
            self.get_logger().warn(
                "Cannot save sample: missing MANO keypoints or joint positions."
            )
            return

        sample = {
            "mano_keypoints": self.latest_mano.tolist(),
            "joint_angles": self.latest_joints.tolist(),
        }
        self.samples.append(sample)
        self.get_logger().info(f"Saved sample #{len(self.samples)}")

    def build_output_path(self) -> Path:
        timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        filename = f"retargeter_calibration_{self.hand_chirality}_{timestamp}.json"
        return Path(__file__).resolve().parent / filename

    def save_dataset(self) -> None:
        if not self.samples:
            self.get_logger().warn("No samples collected, nothing to save.")
            return

        output_path = self.build_output_path()
        output_path.parent.mkdir(parents=True, exist_ok=True)

        dataset = {
            "hand_chirality": self.hand_chirality,
            "created_at": datetime.utcnow().isoformat() + "Z",
            "samples": self.samples,
        }

        try:
            with output_path.open("w") as f:
                json.dump(dataset, f, indent=2)
            self.get_logger().info(
                f"Saved {len(self.samples)} samples to {output_path}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to write dataset to {output_path}: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RetargeterCalibrationCollector()

    # Spin the ROS node in a background thread so that we can read keyboard
    # input in the main thread.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(
        "\nRetargeter calibration collector controls:\n"
        "  <Enter> : pause/resume retargeter and toggle hand torque (compliant mode)\n"
        "  s       : save current (MANO, joints) sample\n"
        "  q       : quit and write dataset to JSON\n"
    )

    try:
        while node.running:
            try:
                cmd = input().strip().lower()
            except EOFError:
                # End of input stream: behave as if 'q' was pressed.
                cmd = "q"

            if cmd == "":
                node.toggle_pause()
            elif cmd == "s":
                node.save_current_sample()
            elif cmd == "q":
                node.running = False
                break
            else:
                print(
                    "Unknown command. Use <Enter> to toggle retargeter/compliant mode, "
                    "'s' to save a sample, 'q' to quit."
                )
    finally:
        # Try to save dataset before shutting down.
        node.save_dataset()
        node.get_logger().info("Shutting down calibration collector.")
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
