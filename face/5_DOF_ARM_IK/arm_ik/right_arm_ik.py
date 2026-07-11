#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""右臂 5-DOF 逆解对外接口。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Union

import numpy as np
import yaml

from .forward_kinematics import ForwardKinematics
from .inverse_kinematics import IKConfig, IKResult, IKTaskMode, NumericalIK
from .transforms import origin_matrix
from .mount_chain import TorsoMountFK, load_torso_mount
from .urdf_package import default_urdf_file, resolve_urdf_path
from .urdf_parser import (
    ArmChain,
    auto_guess_right_arm,
    find_chain,
    load_revolute_joints,
)


def _load_yaml(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


_TELEOP_AXIS_NAMES = ("forward", "back", "right", "left", "up", "down")
_DEFAULT_TELEOP_AXES = {
    "forward": [0, 0, -1],
    "back": [0, -1, 0],
    "right": [1, 0, 0],
    "left": [0, 0, 1],
    "up": [0, 1, 0],
    "down": [-1, 0, 0],
}


def load_teleop_axes(config_path: Union[str, Path]) -> dict[str, np.ndarray]:
    """键盘平移单位方向（base_link），见 config teleop 段。"""
    data = _load_yaml(Path(config_path))
    block = data.get("teleop") or {}
    out: dict[str, np.ndarray] = {}
    for name in _TELEOP_AXIS_NAMES:
        raw = block.get(name, _DEFAULT_TELEOP_AXES[name])
        v = np.asarray(raw, dtype=float).reshape(3)
        n = float(np.linalg.norm(v))
        if n < 1e-9:
            raise ValueError(f"teleop.{name} 方向向量不能为零")
        out[name] = v / n
    return out


def load_standing_home_q(config_path: Union[str, Path]) -> np.ndarray:
    """
    站立默认关节角，作为 IK/键盘末端的参考原点。

    非 URDF q=0（手臂伸直）构型。配置键 standing_home_q，兼容 default_arm_q。
    """
    data = _load_yaml(Path(config_path))
    raw = data.get("standing_home_q") or data.get("default_arm_q")
    if raw is None:
        raise KeyError(
            f"{config_path} 缺少 standing_home_q（站立 IK 原点，非 q=0 伸直姿）",
        )
    q = np.asarray(raw, dtype=float).reshape(-1)
    if q.size != 5:
        raise ValueError(f"standing_home_q 需 5 维，收到 {q.size}")
    return q


def pose_from_xyz_rpy(
    xyz: Union[List[float], np.ndarray],
    rpy: Union[List[float], np.ndarray],
) -> np.ndarray:
    return origin_matrix(xyz, rpy)


def pose_from_xyz_quat(
    xyz: Union[List[float], np.ndarray],
    quat_xyzw: Union[List[float], np.ndarray],
) -> np.ndarray:
    """四元数顺序 x,y,z,w。"""
    x, y, z, w = [float(v) for v in quat_xyzw]
    r = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    t = np.eye(4, dtype=float)
    t[:3, :3] = r
    t[:3, 3] = np.asarray(xyz, dtype=float).reshape(3)
    return t


@dataclass
class RightArmIKSolver:
    """
    右手 5 自由度逆运动学求解器（不含夹爪）。

    用法:
        solver = RightArmIKSolver.from_urdf("urdf/PiPlusPro_....urdf")
        result = solver.ik_position_orientation(target_T, q_seed=q0)
    """

    chain: ArmChain
    fk_solver: ForwardKinematics
    ik_solver: NumericalIK
    urdf_path: Path
    torso_mount: Optional[TorsoMountFK] = None

    @classmethod
    def from_urdf(
        cls,
        urdf_path: Union[str, Path],
        config_path: Optional[Union[str, Path]] = None,
        joint_names: Optional[List[str]] = None,
        base_link: Optional[str] = None,
        ee_link: Optional[str] = None,
    ) -> "RightArmIKSolver":
        urdf_file = resolve_urdf_path(urdf_path)

        cfg = {}
        cfg = {}
        if config_path is not None:
            cfg = _load_yaml(Path(config_path))
            if cfg.get("urdf_file") and Path(urdf_path).is_dir():
                urdf_file = Path(urdf_path) / cfg["urdf_file"]
            elif cfg.get("urdf_file") and not Path(urdf_path).is_file():
                alt = Path(urdf_path).parent / cfg["urdf_file"]
                if alt.is_file():
                    urdf_file = alt

        joints = load_revolute_joints(urdf_file)
        names = joint_names or cfg.get("joint_names")
        base = base_link or cfg.get("base_link")
        ee = ee_link or cfg.get("ee_link")

        if names is None:
            names, base_auto, ee_auto = auto_guess_right_arm(joints, side="r")
            base = base or base_auto
            ee = ee or ee_auto

        chain = find_chain(joints, list(names), base_link=base, ee_link=ee)
        ik_cfg = IKConfig()
        ik_section = cfg.get("ik") or {}
        for key in IKConfig.__dataclass_fields__:
            if key in ik_section:
                setattr(ik_cfg, key, ik_section[key])

        fk_solver = ForwardKinematics(chain)
        ik_solver = NumericalIK(fk_solver, chain, ik_cfg)
        mount = load_torso_mount(urdf_file)
        return cls(
            chain=chain,
            fk_solver=fk_solver,
            ik_solver=ik_solver,
            urdf_path=Path(urdf_file),
            torso_mount=mount,
        )

    @classmethod
    def from_config(
        cls,
        config_path: Union[str, Path],
        urdf_path: Optional[Union[str, Path]] = None,
    ) -> "RightArmIKSolver":
        config_path = Path(config_path)
        if urdf_path is None:
            urdf_path = default_urdf_file(config_path.parent.parent)
        return cls.from_urdf(urdf_path, config_path=config_path)

    @property
    def joint_names(self) -> List[str]:
        return self.chain.joint_names

    @property
    def dof(self) -> int:
        return self.chain.dof

    def fk(self, q: np.ndarray) -> np.ndarray:
        return self.fk_solver.compute(q)

    def ik(
        self,
        target_T: np.ndarray,
        q_seed: Optional[np.ndarray] = None,
        mode: IKTaskMode = IKTaskMode.POSITION_TOOL_Z,
    ) -> IKResult:
        return self.ik_solver.solve(target_T, q_seed=q_seed, mode=mode)

    def ik_position(
        self,
        target_xyz: Union[List[float], np.ndarray],
        q_seed: Optional[np.ndarray] = None,
    ) -> IKResult:
        t = np.eye(4, dtype=float)
        t[:3, 3] = np.asarray(target_xyz, dtype=float).reshape(3)
        return self.ik(t, q_seed=q_seed, mode=IKTaskMode.POSITION)

    def ik_position_orientation(
        self,
        target_T: np.ndarray,
        q_seed: Optional[np.ndarray] = None,
        *,
        tool_z_only: bool = True,
    ) -> IKResult:
        mode = (
            IKTaskMode.POSITION_TOOL_Z
            if tool_z_only
            else IKTaskMode.POSITION_ORIENTATION
        )
        return self.ik(target_T, q_seed=q_seed, mode=mode)

    def ik_in_base_frame(
        self,
        target_base: np.ndarray,
        q_seed: Optional[np.ndarray] = None,
        q_waist: float = 0.0,
        *,
        tool_z_only: bool = True,
    ) -> IKResult:
        """目标位姿在 base_link 系；内部转换到 torso_link 后求 IK。"""
        if self.torso_mount is None:
            raise RuntimeError("URDF 无 waist/torso 链，请直接用 torso_link 系目标")
        target_torso = self.torso_mount.target_in_torso_frame(target_base, q_waist)
        return self.ik_position_orientation(
            target_torso, q_seed=q_seed, tool_z_only=tool_z_only,
        )

    def reachable_distance_bounds(self, q_seed: Optional[np.ndarray] = None) -> tuple[float, float]:
        """粗略工作空间半径（几何求和，不含碰撞）。"""
        q = (
            np.zeros(self.dof)
            if q_seed is None
            else np.asarray(q_seed, dtype=float)
        )
        pts = self.fk_solver.joint_positions(q)
        reach = 0.0
        prev = np.zeros(3)
        for p, _ in pts:
            reach += float(np.linalg.norm(p - prev))
            prev = p
        return 0.0, reach
