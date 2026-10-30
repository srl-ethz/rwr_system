#!/bin/env python3

import argparse
import re
import select
import sys
import termios
import threading
import time
import tty

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster

from faive_system.src.common.transformation_utils import complete_rotation_matrix


PROGRAM_COMMANDS = ("x_sin", "y_sin", "z_sin", "xy_circ", "yz_circ", "xz_circ")


class KeyboardFrankaTeleop(Node):
    def __init__(self, chirality: str, pos_step: float, rot_step_deg: float, publish_hz: float):
        super().__init__(f"{chirality}_franka_keyboard_teleop")
        self.chirality = chirality
        self.pos_step = float(pos_step)
        self.rot_step_rad = np.deg2rad(float(rot_step_deg))

        self.arm_pub = self.create_publisher(
            PoseStamped, f"/{self.chirality}/arm/end_effector_pose_cmd", 10
        )
        self.arm_sub = self.create_subscription(
            PoseStamped, f"/{self.chirality}/arm/end_effector_pose", self.arm_pose_callback, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.target_frame_name = f"{self.chirality}_ee_target"

        self._lock = threading.Lock()
        self.current_pose = None
        self.target_pos = None
        self.target_rot = None
        self.running = True
        self.pending_command = None
        self.program_active = False
        self.program_name = None
        self.program_amplitude = 0.0
        self.program_frequency_hz = 0.5
        self.program_origin_pos = None
        self.program_start_time = 0.0
        self.recording_active = False
        self.recording_duration = 0.0
        self.recording_start_time = 0.0
        self.recording_samples = []
        self.completed_recordings = []

        self.timer = self.create_timer(1.0 / publish_hz, self.publish_target_pose)

    def arm_pose_callback(self, msg: PoseStamped):
        with self._lock:
            self.current_pose = msg

    def wait_for_current_pose(self):
        while rclpy.ok():
            with self._lock:
                if self.current_pose is not None:
                    return
            self.get_logger().info("Waiting for current arm pose...")
            time.sleep(0.5)

    def move_to_default_pose(self):
        if self.chirality == "left":
            default_pos = np.array([0.40, 0.36, 0.3], dtype=np.float64)
            default_rot = complete_rotation_matrix(x=[1, 0, 0], z=[0, 0, -1])
        elif self.chirality == "right":
            default_pos = np.array([0.45, -0.36, 0.2], dtype=np.float64)
            default_rot = complete_rotation_matrix(x=[1, 0, 0], z=[0, 0, -1])
        else:
            default_pos = np.array([0.45, 0, 0.2], dtype=np.float64)
            default_rot = complete_rotation_matrix(x=[1, 0, 0], z=[0, 0, -1])
        with self._lock:
            self.target_pos = default_pos
            self.target_rot = Rotation.from_matrix(default_rot)

        self.get_logger().info("Moving to default pose...")
        end_time = time.time() + 4.0
        while rclpy.ok() and time.time() < end_time:
            self.publish_target_pose()
            time.sleep(0.05)

    def _apply_position_delta(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0):
        self.target_pos += np.array([dx, dy, dz], dtype=np.float64) * self.pos_step

    def _apply_rotation_delta(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0):
        inc = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False)
        self.target_rot = self.target_rot * inc

    def handle_key(self, key: str):
        with self._lock:
            if self.target_pos is None or self.target_rot is None:
                return
            if self.program_active:
                return

            # Position control
            if key == "w":
                self._apply_position_delta(dx=1.0)
            elif key == "s":
                self._apply_position_delta(dx=-1.0)
            elif key == "a":
                self._apply_position_delta(dy=1.0)
            elif key == "d":
                self._apply_position_delta(dy=-1.0)
            elif key == "q":
                self._apply_position_delta(dz=1.0)
            elif key == "e":
                self._apply_position_delta(dz=-1.0)

            # Orientation control (numpad with NumLock on):
            # 7/9: roll +/- , 8/2: pitch +/- , 4/6: yaw +/-
            elif key == "7":
                self._apply_rotation_delta(roll=self.rot_step_rad)
            elif key == "9":
                self._apply_rotation_delta(roll=-self.rot_step_rad)
            elif key == "8":
                self._apply_rotation_delta(pitch=self.rot_step_rad)
            elif key == "2":
                self._apply_rotation_delta(pitch=-self.rot_step_rad)
            elif key == "4":
                self._apply_rotation_delta(yaw=self.rot_step_rad)
            elif key == "6":
                self._apply_rotation_delta(yaw=-self.rot_step_rad)

    def prepare_preprogrammed_command(self, command: str):
        cmd = command.strip()
        if cmd == "list":
            return "list", self.list_preprogrammed_commands()

        record_match = re.fullmatch(r"record\(\s*([0-9]*\.?[0-9]+)\s*\)", cmd)
        if record_match is not None:
            duration_text = record_match.group(1)
            duration = float(duration_text)
            if duration <= 0.0:
                return "error", "record duration must be > 0"
            return self.start_recording(duration, duration_text)

        match = re.fullmatch(
            r"(x_sin|y_sin|z_sin|xy_circ|yz_circ|xz_circ)\(\s*([0-9]*\.?[0-9]+)\s*\)", cmd
        )
        if match is None:
            return (
                "error",
                'Unsupported command. Use `list` to see commands, or try: x_sin(0.1), xy_circ(0.1), record(5)',
            )

        command_name = match.group(1)
        amplitude_text = match.group(2)
        amplitude = float(match.group(2))
        if amplitude <= 0.0:
            return "error", f"{command_name} length must be > 0"

        with self._lock:
            self.pending_command = (command_name, amplitude)
            self.program_active = False

        if command_name.endswith("_sin"):
            axis = command_name[0]
            description = f"sine wave in {axis} direction with length {amplitude_text}m"
        else:
            plane = command_name[:2]
            description = f"circular motion in {plane} plane with radius {amplitude_text}m"

        return (
            "ready",
            f"arm is going to do {description}, press enter to proceed",
        )

    def list_preprogrammed_commands(self):
        lines = [
            "Implemented commands:",
            "  list",
            "  x_sin(length)",
            "  y_sin(length)",
            "  z_sin(length)",
            "  xy_circ(radius)",
            "  yz_circ(radius)",
            "  xz_circ(radius)",
            "  record(duration_seconds)",
        ]
        return "\n".join(lines)

    def start_recording(self, duration: float, duration_text: str):
        with self._lock:
            if self.recording_active:
                return "error", "Recording is already running"
            if self.target_pos is None or self.target_rot is None:
                return "error", "Cannot record before the target pose is initialized"

            self.recording_active = True
            self.recording_duration = duration
            self.recording_start_time = time.monotonic()
            self.recording_samples = []

        return (
            "recording",
            f"Recording arm command and state for {duration_text}s. You can teleop or start a preprogrammed command now.",
        )

    def start_preprogrammed_command(self):
        with self._lock:
            if self.pending_command is None or self.target_pos is None:
                return False

            name, amplitude = self.pending_command
            self.program_name = name
            self.program_amplitude = amplitude
            self.program_origin_pos = self.target_pos.copy()
            self.program_start_time = time.time()
            self.program_active = True
            self.pending_command = None
        return True

    def stop_preprogrammed_command(self):
        with self._lock:
            self.program_active = False
            self.program_name = None
            self.program_origin_pos = None

    def is_preprogrammed_active(self):
        with self._lock:
            return self.program_active

    def pop_completed_recording(self):
        with self._lock:
            if not self.completed_recordings:
                return None
            return self.completed_recordings.pop(0)

    @staticmethod
    def _pose_msg_to_6d(msg: PoseStamped):
        quat = msg.pose.orientation
        euler = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler("xyz")
        pos = msg.pose.position
        return np.array([pos.x, pos.y, pos.z, euler[0], euler[1], euler[2]], dtype=np.float64)

    def _target_to_6d_locked(self):
        euler = self.target_rot.as_euler("xyz")
        return np.concatenate((self.target_pos.copy(), euler)).astype(np.float64)

    def _record_sample_locked(self, now: float):
        if not self.recording_active:
            return

        elapsed = now - self.recording_start_time
        if self.current_pose is not None:
            try:
                self.recording_samples.append(
                    {
                        "t": elapsed,
                        "command": self._target_to_6d_locked(),
                        "state": self._pose_msg_to_6d(self.current_pose),
                    }
                )
            except ValueError:
                self.get_logger().warn("Skipping recording sample with invalid arm-state orientation")

        if elapsed >= self.recording_duration:
            stopped_preprogram = self.program_active
            self.recording_active = False
            self.recording_duration = 0.0
            self.recording_start_time = 0.0
            self.program_active = False
            self.program_name = None
            self.program_origin_pos = None
            self.completed_recordings.append(
                {
                    "samples": self.recording_samples,
                    "duration": elapsed,
                    "stopped_preprogram": stopped_preprogram,
                }
            )
            self.recording_samples = []

    def _update_preprogrammed_target_locked(self):
        if not self.program_active or self.program_name not in PROGRAM_COMMANDS or self.program_origin_pos is None:
            return

        t = time.time() - self.program_start_time
        omega_t = 2.0 * np.pi * self.program_frequency_hz * t
        s = np.sin(omega_t)
        c_minus_one = np.cos(omega_t) - 1.0

        if self.program_name == "x_sin":
            self.target_pos[0] = self.program_origin_pos[0] + self.program_amplitude * s
        elif self.program_name == "y_sin":
            self.target_pos[1] = self.program_origin_pos[1] + self.program_amplitude * s
        elif self.program_name == "z_sin":
            self.target_pos[2] = self.program_origin_pos[2] + self.program_amplitude * s
        elif self.program_name == "xy_circ":
            self.target_pos[0] = self.program_origin_pos[0] + self.program_amplitude * c_minus_one
            self.target_pos[1] = self.program_origin_pos[1] + self.program_amplitude * s
        elif self.program_name == "yz_circ":
            self.target_pos[1] = self.program_origin_pos[1] + self.program_amplitude * c_minus_one
            self.target_pos[2] = self.program_origin_pos[2] + self.program_amplitude * s
        elif self.program_name == "xz_circ":
            self.target_pos[0] = self.program_origin_pos[0] + self.program_amplitude * c_minus_one
            self.target_pos[2] = self.program_origin_pos[2] + self.program_amplitude * s

    def publish_target_pose(self):
        with self._lock:
            if self.target_pos is None or self.target_rot is None:
                return
            self._update_preprogrammed_target_locked()
            self._record_sample_locked(time.monotonic())

            stamp = self.get_clock().now().to_msg()
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = "world"

            msg.pose.position = Point(
                x=float(self.target_pos[0]),
                y=float(self.target_pos[1]),
                z=float(self.target_pos[2]),
            )
            quat = self.target_rot.as_quat()  # x, y, z, w
            msg.pose.orientation = Quaternion(
                x=float(quat[0]),
                y=float(quat[1]),
                z=float(quat[2]),
                w=float(quat[3]),
            )

            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = "world"
            tf_msg.child_frame_id = self.target_frame_name
            tf_msg.transform.translation.x = float(self.target_pos[0])
            tf_msg.transform.translation.y = float(self.target_pos[1])
            tf_msg.transform.translation.z = float(self.target_pos[2])
            tf_msg.transform.rotation.x = float(quat[0])
            tf_msg.transform.rotation.y = float(quat[1])
            tf_msg.transform.rotation.z = float(quat[2])
            tf_msg.transform.rotation.w = float(quat[3])

        self.arm_pub.publish(msg)
        self.tf_broadcaster.sendTransform(tf_msg)


def _redraw_prompt_line(prompt: str, buffer: list, cursor: int):
    content = "".join(buffer)
    sys.stdout.write("\r")
    sys.stdout.write(prompt + content)
    sys.stdout.write("\x1b[K")
    move_left = len(content) - cursor
    if move_left > 0:
        sys.stdout.write(f"\x1b[{move_left}D")
    sys.stdout.flush()


def refresh_matplotlib_windows():
    plt = sys.modules.get("matplotlib.pyplot")
    if plt is None:
        return

    try:
        if plt.get_fignums():
            plt.pause(0.001)
    except Exception as exc:
        print(f"Matplotlib window update failed: {exc}")


def read_line_with_editor(prompt: str):
    buffer = []
    cursor = 0
    sys.stdout.write(prompt)
    sys.stdout.flush()
    while True:
        if not select.select([sys.stdin], [], [], 0.05)[0]:
            refresh_matplotlib_windows()
            continue
        ch = sys.stdin.read(1)
        if ch in ("\n", "\r"):
            sys.stdout.write("\n")
            sys.stdout.flush()
            return "".join(buffer)
        if ch == "\x03":
            raise KeyboardInterrupt
        if ch in ("\x7f", "\b"):
            if cursor > 0:
                del buffer[cursor - 1]
                cursor -= 1
                _redraw_prompt_line(prompt, buffer, cursor)
            continue
        if ch == "\x1b":
            seq = sys.stdin.read(1)
            if seq != "[":
                continue
            code = sys.stdin.read(1)
            if code == "D" and cursor > 0:  # Left arrow
                cursor -= 1
                sys.stdout.write("\x1b[D")
                sys.stdout.flush()
            elif code == "C" and cursor < len(buffer):  # Right arrow
                cursor += 1
                sys.stdout.write("\x1b[C")
                sys.stdout.flush()
            elif code in ("H", "1", "7"):  # Home
                if code in ("1", "7"):
                    tail = sys.stdin.read(1)
                    if tail != "~":
                        continue
                cursor = 0
                _redraw_prompt_line(prompt, buffer, cursor)
            elif code in ("F", "4", "8"):  # End
                if code in ("4", "8"):
                    tail = sys.stdin.read(1)
                    if tail != "~":
                        continue
                cursor = len(buffer)
                _redraw_prompt_line(prompt, buffer, cursor)
            elif code == "3":  # Delete key: ESC [ 3 ~
                tail = sys.stdin.read(1)
                if tail == "~" and cursor < len(buffer):
                    del buffer[cursor]
                    _redraw_prompt_line(prompt, buffer, cursor)
            continue
        if ch.isprintable():
            buffer.insert(cursor, ch)
            cursor += 1
            _redraw_prompt_line(prompt, buffer, cursor)


def plot_recording(recording):
    samples = recording["samples"]
    if not samples:
        print("Recording finished, but no samples were captured.")
        return

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("Recording finished, but matplotlib is not installed.")
        return

    plt.ion()

    t = np.array([sample["t"] for sample in samples], dtype=np.float64)
    command = np.vstack([sample["command"] for sample in samples])
    state = np.vstack([sample["state"] for sample in samples])

    command_plot = command.copy()
    state_plot = state.copy()
    command_plot[:, 3:] = np.rad2deg(np.unwrap(command_plot[:, 3:], axis=0))
    state_plot[:, 3:] = np.rad2deg(np.unwrap(state_plot[:, 3:], axis=0))

    labels = ("x", "y", "z", "roll", "pitch", "yaw")
    units = ("m", "m", "m", "deg", "deg", "deg")
    fig, axes = plt.subplots(3, 2, sharex=True, num="Franka command tracking")
    axes = axes.ravel()
    for idx, ax in enumerate(axes):
        ax.plot(t, command_plot[:, idx], label="command", linewidth=1.8)
        ax.plot(t, state_plot[:, idx], label="state", linewidth=1.4)
        ax.set_ylabel(f"{labels[idx]} [{units[idx]}]")
        ax.grid(True, alpha=0.3)
        if idx >= 4:
            ax.set_xlabel("time [s]")
    axes[0].legend(loc="best")
    fig.suptitle("End-effector command tracking")
    fig.tight_layout()
    plt.show(block=False)
    plt.pause(0.001)


def show_completed_recordings(node: KeyboardFrankaTeleop):
    while True:
        recording = node.pop_completed_recording()
        if recording is None:
            return

        samples = len(recording["samples"])
        message = f"Recording finished with {samples} samples"
        if recording["stopped_preprogram"]:
            message += "; preprogrammed command stopped."
        else:
            message += "."
        print(message)
        plot_recording(recording)


def keyboard_listener(node: KeyboardFrankaTeleop):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if not sys.stdin.isatty():
            raise RuntimeError("Keyboard teleop requires a TTY")

        print("Keyboard teleop active")
        print("Position: w/s (+/-x), a/d (+/-y), q/e (+/-z)")
        print("Orientation (NumLock ON): 7/9 roll, 8/2 pitch, 4/6 yaw")
        print("Press / to enter command mode (`list` to see implemented commands)")
        print("Use record(duration_seconds) in command mode to plot command tracking")
        print("While command mode is running, press Enter to stop")
        print("Press x to exit")

        while rclpy.ok() and node.running:
            show_completed_recordings(node)
            refresh_matplotlib_windows()
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                if key in ("x", "X"):
                    node.running = False
                    break
                if key == "/":
                    command = read_line_with_editor("command> ")
                    status, message = node.prepare_preprogrammed_command(command)
                    print(message)
                    if status == "ready":
                        read_line_with_editor("")
                        if node.start_preprogrammed_command():
                            print("Preprogrammed command started. Press Enter to stop.")
                    continue
                if key in ("\n", "\r"):
                    if node.is_preprogrammed_active():
                        node.stop_preprogrammed_command()
                        print("Preprogrammed command stopped.")
                    continue
                node.handle_key(key)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    parser = argparse.ArgumentParser(description="Basic keyboard teleop for Franka end-effector pose")
    parser.add_argument("--chirality", choices=["panda", "left", "right", "mj_left", "mj_right"], default="right")
    parser.add_argument("--pos-step", type=float, default=0.01, help="Position increment in meters per key press")
    parser.add_argument("--rot-step-deg", type=float, default=5.0, help="Rotation increment in degrees per key press")
    parser.add_argument("--publish-hz", type=float, default=20.0, help="Pose command publish rate")
    parsed, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = KeyboardFrankaTeleop(
        chirality=parsed.chirality,
        pos_step=parsed.pos_step,
        rot_step_deg=parsed.rot_step_deg,
        publish_hz=parsed.publish_hz,
    )

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.wait_for_current_pose()
        node.move_to_default_pose()
        keyboard_listener(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
