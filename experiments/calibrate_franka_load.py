#!/usr/bin/env python3
"""
ROS 2 quasi-static payload calibration (mass + CoM) for Franka.

Subscribes to a FrankaState-like message with fields:
  - dq                (7,)
  - k_f_ext_hat_k     (6,)  external wrench at stiffness frame K, expressed in K
  - o_t_ee            (16,) ^O T_EE  (column-major)  transforms EE -> O
  - ee_t_k            (16,) ^EE T_K  (column-major)  transforms K  -> EE
  - f_t_ee            (16,) ^F T_EE  (column-major)  transforms EE -> F   (optional output)

Workflow:
  - User moves robot to multiple poses and holds still
  - Press Enter to capture an averaged sample
  - After enough samples, compute payload mass and CoM in EE frame

Notes:
  - This estimates the "unmodeled load" that shows up in k_f_ext_hat_k. If your
    configured load/end-effector parameters already compensate the payload, the
    external wrench may be near zero, and you'll estimate ~0 mass.
  - Gravity is assumed normal in base frame O: [0, 0, -9.81]
"""

import sys
import time
import threading
from dataclasses import dataclass
from typing import Optional, List, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

# Adjust this import if your FrankaState is in a different package.
from franka_msgs.msg import FrankaState


def mat44_from_col_major(a16) -> np.ndarray:
    """Franka 4x4 transforms are column-major."""
    return np.array(a16, dtype=float).reshape((4, 4), order="F")


def skew(v: np.ndarray) -> np.ndarray:
    """skew(v) @ x = v x x"""
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    return np.array([[0.0, -z,  y],
                     [z,  0.0, -x],
                     [-y,  x,  0.0]], dtype=float)


def mad_inlier_mask(x: np.ndarray, z: float = 3.5) -> np.ndarray:
    """Median absolute deviation inlier mask."""
    x = np.array(x, dtype=float).reshape((-1,))
    med = np.median(x)
    mad = np.median(np.abs(x - med))
    if mad < 1e-12:
        return np.ones_like(x, dtype=bool)
    return np.abs(x - med) <= (z * mad)


@dataclass
class Sample:
    g_EE: np.ndarray     # (3,)
    F_EE: np.ndarray     # (3,)
    tau_EE: np.ndarray   # (3,)
    stamp_s: float
    dq_max: float
    T_F_EE: np.ndarray   # (4,4) for optional flange output


class FrankaStateBuffer(Node):
    def __init__(self):
        super().__init__("calibrate_franka_load")

        self.declare_parameter("franka_state_topic", "/paco/franka_robot_state_broadcaster/robot_state")
        self.declare_parameter("dq_thresh", 0.05)          # rad/s
        self.declare_parameter("avg_window_s", 0.25)       # seconds
        self.declare_parameter("sample_rate_hz", 200.0)    # Hz
        self.declare_parameter("g_mag", 9.81)

        topic = self.get_parameter("franka_state_topic").get_parameter_value().string_value

        self._lock = threading.Lock()
        self._last_msg: Optional[FrankaState] = None
        self._last_rx_walltime: float = 0.0

        self._sub = self.create_subscription(
            FrankaState,
            topic,
            self._cb,
            qos_profile=qos_profile_sensor_data,
        )

        self.samples: List[Sample] = []

        self.get_logger().info(f"Subscribed to {topic}")

    def _cb(self, msg: FrankaState):
        with self._lock:
            self._last_msg = msg
            self._last_rx_walltime = time.time()

    def get_last_msg(self) -> Optional[FrankaState]:
        with self._lock:
            return self._last_msg

    def wait_for_first_msg(self, timeout_s: float = 8.0) -> bool:
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_s:
            if self.get_last_msg() is not None:
                return True
            time.sleep(0.05)
        return False

    def _gravity_in_EE(self, msg: FrankaState) -> np.ndarray:
        """Assume gravity in base O is [0,0,-g]. Convert to EE: g_EE = R_O_EE^T g_O."""
        g_mag = float(self.get_parameter("g_mag").value)
        T_O_EE = mat44_from_col_major(msg.o_t_ee)  # ^O T_EE
        R_O_EE = T_O_EE[:3, :3]
        g_O = np.array([0.0, 0.0, -g_mag], dtype=float)
        return R_O_EE.T @ g_O

    def _wrench_K_to_EE_about_EE(self, msg: FrankaState) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convert wrench from K (expressed in K, moment about K) to:
          - expressed in EE
          - moment about EE origin

        Using ^EE T_K = msg.ee_t_k (column-major).
        """
        T_EE_K = mat44_from_col_major(msg.ee_t_k)  # ^EE T_K
        R_EE_K = T_EE_K[:3, :3]
        p_EE_K = T_EE_K[:3, 3]  # position of K origin expressed in EE coords

        wK = np.array(msg.k_f_ext_hat_k, dtype=float).reshape((6,))
        F_K = wK[:3]
        tau_K = wK[3:]

        # Rotate
        F_EE = R_EE_K @ F_K
        tau_EE_about_K = R_EE_K @ tau_K

        # Shift moment reference point from K-origin to EE-origin:
        # tau_about_EE = tau_about_K + p_EE_K x F
        tau_EE = tau_EE_about_K + np.cross(p_EE_K, F_EE)

        return F_EE, tau_EE

    def capture_static_sample(self) -> Optional[Sample]:
        """
        Capture one sample averaged over avg_window_s while requiring max(|dq|) < dq_thresh.
        """
        dq_thresh = float(self.get_parameter("dq_thresh").value)
        avg_window_s = float(self.get_parameter("avg_window_s").value)
        sample_rate_hz = float(self.get_parameter("sample_rate_hz").value)

        dt = 1.0 / max(1.0, sample_rate_hz)
        end_t = time.time() + max(0.05, avg_window_s)

        g_list, F_list, tau_list, dqmax_list, stamp_list = [], [], [], [], []
        last_msg = None

        while rclpy.ok() and time.time() < end_t:
            msg = self.get_last_msg()
            if msg is None:
                time.sleep(dt)
                continue

            dq = np.array(msg.dq, dtype=float)
            dq_max = float(np.max(np.abs(dq)))
            if dq_max > dq_thresh:
                return None  # not static enough

            g_EE = self._gravity_in_EE(msg)
            F_EE, tau_EE = self._wrench_K_to_EE_about_EE(msg)

            g_list.append(g_EE)
            F_list.append(F_EE)
            tau_list.append(tau_EE)
            dqmax_list.append(dq_max)

            # Prefer header stamp if available; fall back to msg.time; else wall time.
            stamp_s = None
            try:
                stamp_s = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
            except Exception:
                stamp_s = None
            if stamp_s is None or stamp_s <= 0.0:
                stamp_s = float(getattr(msg, "time", 0.0)) or time.time()
            stamp_list.append(stamp_s)

            last_msg = msg
            time.sleep(dt)

        if last_msg is None or len(g_list) < 3:
            return None

        g_mean = np.mean(np.stack(g_list, axis=0), axis=0)
        F_mean = np.mean(np.stack(F_list, axis=0), axis=0)
        tau_mean = np.mean(np.stack(tau_list, axis=0), axis=0)
        dq_max_mean = float(np.mean(np.array(dqmax_list)))
        stamp_mean = float(np.mean(np.array(stamp_list)))

        T_F_EE = mat44_from_col_major(last_msg.f_t_ee)

        return Sample(
            g_EE=g_mean, F_EE=F_mean, tau_EE=tau_mean,
            stamp_s=stamp_mean, dq_max=dq_max_mean,
            T_F_EE=T_F_EE
        )

    def fit_mass_and_com(self) -> Tuple[float, np.ndarray, np.ndarray, dict]:
        """
        Returns:
          m     [kg]
          r_EE  [m]  CoM in EE frame
          r_F   [m]  CoM in flange frame (optional convenience output)
          diagnostics dict
        """
        if len(self.samples) < 3:
            raise RuntimeError("Need at least 3 samples; use ~8–20 with varied orientations for good results.")

        g = np.stack([s.g_EE for s in self.samples], axis=0)      # (N,3)
        F = np.stack([s.F_EE for s in self.samples], axis=0)      # (N,3)
        tau = np.stack([s.tau_EE for s in self.samples], axis=0)  # (N,3)

        g2 = np.sum(g * g, axis=1)  # should be ~9.81^2
        m_i = np.sum(g * F, axis=1) / g2

        mask = mad_inlier_mask(m_i, z=3.5)
        if np.count_nonzero(mask) < 3:
            mask = np.ones_like(mask, dtype=bool)

        # Fix sign convention: prefer positive mass
        m_med = float(np.median(m_i[mask]))
        sign_flip = False
        if m_med < 0.0:
            F = -F
            tau = -tau
            m_i = -m_i
            sign_flip = True

        mask = mad_inlier_mask(m_i, z=3.5)
        if np.count_nonzero(mask) < 3:
            mask = np.ones_like(mask, dtype=bool)

        m = float(np.mean(m_i[mask]))
        if m <= 1e-9:
            raise RuntimeError(
                "Estimated mass is ~0. Either the payload is already compensated (configured load matches), "
                "or samples had contact forces/noise dominating."
            )

        # Solve for p = m*r_EE using torque equation: tau = -skew(g) p
        A_blocks = []
        b_blocks = []
        for i in range(g.shape[0]):
            if not mask[i]:
                continue
            A_blocks.append(-skew(g[i, :]))   # (3,3)
            b_blocks.append(tau[i, :])        # (3,)

        A = np.vstack(A_blocks)                 # (3M, 3)
        b = np.concatenate(b_blocks, axis=0)    # (3M,)

        p, residuals, rank, svals = np.linalg.lstsq(A, b, rcond=None)  # (3,)
        p = p.reshape((3,))

        r_EE = p / m

        # Optional: CoM in flange frame using ^F T_EE
        T_F_EE = self.samples[-1].T_F_EE
        R_F_EE = T_F_EE[:3, :3]
        p_F_EE = T_F_EE[:3, 3]
        r_F = p_F_EE + (R_F_EE @ r_EE)

        # diagnostics
        F_pred = m * g
        force_res = (F - F_pred)
        force_rms = float(np.sqrt(np.mean(np.sum(force_res[mask, :] ** 2, axis=1))))

        tau_pred = np.vstack([(-skew(g[i, :]) @ p).reshape((1, 3)) for i in range(g.shape[0])])
        tau_res = (tau - tau_pred)
        tau_rms = float(np.sqrt(np.mean(np.sum(tau_res[mask, :] ** 2, axis=1))))

        diag = {
            "num_samples_total": int(g.shape[0]),
            "num_samples_used": int(np.count_nonzero(mask)),
            "mass_mean_all": float(np.mean(m_i)),
            "mass_median_all": float(np.median(m_i)),
            "mass_std_all": float(np.std(m_i)),
            "A_rank": int(rank),
            "A_singular_values": [float(x) for x in svals],
            "force_rms_N": force_rms,
            "torque_rms_Nm": tau_rms,
            "sign_flip_applied": bool(sign_flip),
        }

        return m, r_EE, r_F, diag


def main():
    rclpy.init()

    node = FrankaStateBuffer()

    # Spin node in background so we can use blocking input() in main thread.
    exec_ = SingleThreadedExecutor()
    exec_.add_node(node)

    spin_thread = threading.Thread(target=exec_.spin, daemon=True)
    spin_thread.start()

    print("\n=== Franka ROS 2 quasi-static load calibration (mass + CoM) ===")
    print("Instructions:")
    print("  1) Move robot to a pose and HOLD STILL (no contact).")
    print("  2) Press Enter to capture a sample (averaged over a short window).")
    print("  3) Repeat for many orientations (8–20 is a good start).")
    print("\nCommands:")
    print("  Enter : capture sample")
    print("  undo  : remove last sample")
    print("  fit   : fit and exit")
    print("  q     : fit and exit\n")

    if not node.wait_for_first_msg(timeout_s=8.0):
        print("ERROR: No FrankaState messages received. Check your topic/type and that the publisher is running.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    while rclpy.ok():
        try:
            cmd = input(f"[samples={len(node.samples)}] > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            cmd = "fit"

        if cmd in ("fit", "q", "quit", "exit", "done"):
            break

        if cmd == "undo":
            if node.samples:
                node.samples.pop()
                print("Removed last sample.")
            else:
                print("No samples to remove.")
            continue

        if cmd != "":
            print("Unknown command. Use Enter / undo / fit / q.")
            continue

        sample = node.capture_static_sample()
        if sample is None:
            dq_thresh = float(node.get_parameter("dq_thresh").value)
            print(f"Sample rejected: robot not static enough (try holding still, or increase dq_thresh from {dq_thresh}).")
            continue

        node.samples.append(sample)
        print("Captured sample:")
        print(f"  dq_max(avg) = {sample.dq_max:.4f} rad/s")
        print(f"  g_EE        = {sample.g_EE}  [m/s^2]")
        print(f"  F_EE        = {sample.F_EE}  [N]")
        print(f"  tau_EE      = {sample.tau_EE} [Nm]")

    if len(node.samples) < 3:
        print("Not enough samples to fit (need >= 3; use 8–20).")
        exec_.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    try:
        m, r_EE, r_F, diag = node.fit_mass_and_com()
    except Exception as e:
        print(f"Fit failed: {e}")
        exec_.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(3)

    print("\n=== Fit result ===")
    print(f"Mass m = {m:.6f} kg")
    print(f"CoM in EE frame r_EE = {r_EE}  [m]")
    print(f"CoM in flange frame r_F = {r_F}  [m]  (convenience output)")

    print("\nDiagnostics:")
    for k, v in diag.items():
        print(f"  {k}: {v}")

    print("\nTips if results look wrong:")
    print("  - Ensure NO contact forces while sampling (table, cable drag, etc.).")
    print("  - Use many diverse orientations; if A_rank < 3 you didn’t rotate enough.")
    print("  - If mass ~0, your configured load/EE parameters might already compensate the payload.")
    print("  - If mass flips sign often, your wrench sign convention differs; this script auto-flips to positive mass.")

    exec_.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()