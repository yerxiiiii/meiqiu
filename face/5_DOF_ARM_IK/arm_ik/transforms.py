#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SE(3) 工具：与 URDF 一致的 RPY(x-y-z 内旋) 约定。"""

from __future__ import annotations

import math
from typing import Iterable, Tuple

import numpy as np

Vec3 = np.ndarray


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF rpy = 固定轴 X·Y·Z（与 ROS 一致）。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def origin_matrix(xyz: Iterable[float], rpy: Iterable[float]) -> np.ndarray:
    t = np.eye(4, dtype=float)
    t[:3, :3] = rpy_matrix(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    t[:3, 3] = np.asarray(xyz, dtype=float).reshape(3)
    return t


def axis_angle_matrix(axis: Vec3, angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float).reshape(3)
    n = np.linalg.norm(a)
    if n < 1e-12:
        return np.eye(4, dtype=float)
    a = a / n
    x, y, z = a
    c, s = math.cos(angle), math.sin(angle)
    c1 = 1.0 - c
    r = np.array(
        [
            [c + x * x * c1, x * y * c1 - z * s, x * z * c1 + y * s],
            [y * x * c1 + z * s, c + y * y * c1, y * z * c1 - x * s],
            [z * x * c1 - y * s, z * y * c1 + x * s, c + z * z * c1],
        ],
        dtype=float,
    )
    t = np.eye(4, dtype=float)
    t[:3, :3] = r
    return t


def rotation_error(R_current: np.ndarray, R_target: np.ndarray) -> Vec3:
    """so(3) 旋转向量误差 R_target ≈ exp([w]×) R_current。"""
    r_err = R_target @ R_current.T
    trace = float(np.trace(r_err))
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    angle = math.acos(cos_angle)
    if angle < 1e-9:
        return np.zeros(3, dtype=float)
    w = np.array(
        [
            r_err[2, 1] - r_err[1, 2],
            r_err[0, 2] - r_err[2, 0],
            r_err[1, 0] - r_err[0, 1],
        ],
        dtype=float,
    )
    w_norm = np.linalg.norm(w)
    if w_norm < 1e-12:
        return np.zeros(3, dtype=float)
    return (w / w_norm) * angle


def tool_z_error(R_current: np.ndarray, R_target: np.ndarray) -> Vec3:
    """仅约束末端 z 轴方向（5 自由度常用）。"""
    z_c = R_current[:, 2]
    z_t = R_target[:, 2]
    return np.cross(z_c, z_t)


def clamp_vector(v: Vec3, lo: Vec3, hi: Vec3) -> Vec3:
    return np.minimum(np.maximum(v, lo), hi)


def split_pose(T: np.ndarray) -> Tuple[Vec3, np.ndarray]:
    return T[:3, 3].copy(), T[:3, :3].copy()
