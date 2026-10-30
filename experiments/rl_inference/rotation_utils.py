import os
import pickle
import threading
from copy import deepcopy
from typing import Dict, List, Optional, Tuple
from scipy.spatial.transform import Rotation as R

import numpy as np
import cv2
import torch

def create_transform_matrix(R,t):
    """
    Create a 4x4 transform matrix from a 3x3 rotation matrix and a 3x1 translation vector.
    """
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    """
    Convert 3x3 rotation matrix to (x,y,z,w) quaternion (ROS convention: x,y,z,w).
    """
    m00, m01, m02 = R[0, :]
    m10, m11, m12 = R[1, :]
    m20, m21, m22 = R[2, :]
    tr = m00 + m11 + m22
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (m21 - m12) / S
        y = (m02 - m20) / S
        z = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / S
        x = 0.25 * S
        y = (m01 + m10) / S
        z = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / S
        x = (m01 + m10) / S
        y = 0.25 * S
        z = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / S
        x = (m02 + m20) / S
        y = (m12 + m21) / S
        z = 0.25 * S
    return float(x), float(y), float(z), float(w)

def quaternion_to_rot_matrix(q: np.ndarray) -> np.ndarray:
    """
    Convert (x,y,z,w) quaternion to 3x3 rotation matrix.
    """
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = (q / n).tolist()
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    R = np.array([
        [1 - 2 * (yy + zz),     2 * (xy - wz),         2 * (xz + wy)],
        [2 * (xy + wz),         1 - 2 * (xx + zz),     2 * (yz - wx)],
        [2 * (xz - wy),         2 * (yz + wx),         1 - 2 * (xx + yy)]
    ], dtype=np.float64)
    return R

# Quaternion helpers
def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).copy()
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / n

def quat_conjugate(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product, ROS convention (x,y,z,w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return np.array([x, y, z, w], dtype=np.float64)

def quat_to_angle_axis(q: np.ndarray) -> Tuple[float, np.ndarray]:
    """Return (angle [rad], axis[3]) for unit quaternion q (x,y,z,w)."""
    qn = quat_normalize(q)
    w = float(np.clip(qn[3], -1.0, 1.0))
    v = qn[:3]
    angle = 2.0 * np.arccos(w)
    s = np.sqrt(max(1e-16, 1.0 - w * w))
    if s < 1e-8:
        axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        axis = (v / s).astype(np.float64)
    # wrap to [-pi, pi]
    angle = float(((angle + np.pi) % (2 * np.pi)) - np.pi)
    return angle, axis


def euler_angles_to_rotation_matrix(pitch, yaw, roll, convention='xyz'):
    """
    Create a 3x3 rotation matrix from euler angles.
    """
    rot = R.from_euler(convention, [pitch, yaw, roll]).as_matrix()
    return rot

