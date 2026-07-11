#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""base_link → torso_link：腰 yaw + 固定躯干（目标在 base 系时转换到臂基座系）。"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

from .transforms import axis_angle_matrix, origin_matrix
from .urdf_parser import RevoluteJoint, load_revolute_joints


def _parse_fixed_origin(urdf_path: str, joint_name: str) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    root = ET.parse(str(urdf_path)).getroot()
    j = root.find(f".//joint[@name='{joint_name}']")
    if j is None:
        raise KeyError(f"未找到关节 {joint_name}")
    origin = j.find("origin")
    xyz = tuple(float(x) for x in origin.get("xyz", "0 0 0").split())
    rpy = tuple(float(x) for x in origin.get("rpy", "0 0 0").split())
    return xyz, rpy


class TorsoMountFK:
    """T_base_torso(q_waist)。"""

    def __init__(self, waist: RevoluteJoint, torso_xyz, torso_rpy):
        self._waist = waist
        self._torso_static = origin_matrix(torso_xyz, torso_rpy)

    def compute(self, q_waist: float = 0.0) -> np.ndarray:
        t = origin_matrix(self._waist.origin_xyz, self._waist.origin_rpy)
        t = t @ axis_angle_matrix(self._waist.axis, q_waist)
        return t @ self._torso_static

    def target_in_torso_frame(
        self, target_base: np.ndarray, q_waist: float = 0.0
    ) -> np.ndarray:
        return np.linalg.inv(self.compute(q_waist)) @ target_base


def load_torso_mount(urdf_path: str) -> Optional[TorsoMountFK]:
    path = Path(urdf_path)
    joints = load_revolute_joints(path)
    if "waist_yaw_joint" not in joints:
        return None
    try:
        torso_xyz, torso_rpy = _parse_fixed_origin(path, "torso_joint")
    except KeyError:
        return None
    return TorsoMountFK(joints["waist_yaw_joint"], torso_xyz, torso_rpy)
