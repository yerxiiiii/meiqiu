#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 URDF 提取右臂 5 关节串联链（revolute）。"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from .transforms import origin_matrix


@dataclass
class RevoluteJoint:
    name: str
    parent: str
    child: str
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]
    axis: Tuple[float, float, float]
    lower: float
    upper: float


@dataclass
class ArmChain:
    base_link: str
    ee_link: str
    joints: List[RevoluteJoint]

    @property
    def dof(self) -> int:
        return len(self.joints)

    @property
    def joint_names(self) -> List[str]:
        return [j.name for j in self.joints]

    def limits(self) -> Tuple[np.ndarray, np.ndarray]:
        lo = np.array([j.lower for j in self.joints], dtype=float)
        hi = np.array([j.upper for j in self.joints], dtype=float)
        return lo, hi


def _parse_floats(text: str, n: int) -> Tuple[float, ...]:
    parts = [float(x) for x in text.split()]
    if len(parts) != n:
        raise ValueError(f"期望 {n} 个数，得到: {text!r}")
    return tuple(parts)  # type: ignore


def _read_origin(joint_elem: ET.Element) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    origin = joint_elem.find("origin")
    if origin is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = origin.get("xyz", "0 0 0")
    rpy = origin.get("rpy", "0 0 0")
    return _parse_floats(xyz, 3), _parse_floats(rpy, 3)


def _read_axis(joint_elem: ET.Element) -> Tuple[float, float, float]:
    axis = joint_elem.find("axis")
    if axis is None:
        return (0.0, 0.0, 1.0)
    return _parse_floats(axis.get("xyz", "0 0 1"), 3)


def _read_limits(joint_elem: ET.Element) -> Tuple[float, float]:
    limit = joint_elem.find("limit")
    if limit is None:
        return -math.pi, math.pi
    return float(limit.get("lower", "-3.14159")), float(limit.get("upper", "3.14159"))


def load_revolute_joints(urdf_path: str | Path) -> Dict[str, RevoluteJoint]:
    tree = ET.parse(str(urdf_path))
    root = tree.getroot()
    out: Dict[str, RevoluteJoint] = {}
    for j in root.findall("joint"):
        if j.get("type") != "revolute":
            continue
        name = j.get("name")
        parent = j.find("parent").get("link")  # type: ignore
        child = j.find("child").get("link")  # type: ignore
        xyz, rpy = _read_origin(j)
        axis = _read_axis(j)
        lo, hi = _read_limits(j)
        out[name] = RevoluteJoint(
            name=name,
            parent=parent,
            child=child,
            origin_xyz=xyz,
            origin_rpy=rpy,
            axis=axis,
            lower=lo,
            upper=hi,
        )
    return out


def _child_map(joints: Dict[str, RevoluteJoint]) -> Dict[str, List[RevoluteJoint]]:
    m: Dict[str, List[RevoluteJoint]] = {}
    for j in joints.values():
        m.setdefault(j.parent, []).append(j)
    return m


def find_chain(
    joints: Dict[str, RevoluteJoint],
    joint_names: List[str],
    base_link: Optional[str] = None,
    ee_link: Optional[str] = None,
) -> ArmChain:
    """按给定关节名顺序组装链，并校验 parent-child 连续。"""
    chain: List[RevoluteJoint] = []
    for i, jn in enumerate(joint_names):
        if jn not in joints:
            raise KeyError(f"URDF 中未找到关节: {jn}")
        chain.append(joints[jn])
    if base_link is None:
        base_link = chain[0].parent
    if ee_link is None:
        ee_link = chain[-1].child
    if chain[0].parent != base_link:
        raise ValueError(
            f"首关节 {chain[0].name} 父连杆为 {chain[0].parent}，"
            f"与 base_link={base_link} 不一致",
        )
    for a, b in zip(chain, chain[1:]):
        if a.child != b.parent:
            raise ValueError(
                f"链路断裂: {a.name}.child={a.child} != {b.name}.parent={b.parent}",
            )
    if chain[-1].child != ee_link:
        # 允许 ee_link 为腕关节子连杆：沿树向下走一段
        children = _child_map(joints)
        cur = chain[-1].child
        steps = 0
        while cur != ee_link and steps < 8:
            nxt = children.get(cur)
            if not nxt:
                break
            if len(nxt) != 1:
                break
            cur = nxt[0].child
            steps += 1
        if cur != ee_link:
            raise ValueError(
                f"末端连杆期望 {ee_link}，由链推算为 {chain[-1].child}",
            )
    return ArmChain(base_link=base_link, ee_link=ee_link, joints=chain)


def auto_guess_right_arm(
    joints: Dict[str, RevoluteJoint],
    side: str = "r",
) -> Tuple[List[str], str, str]:
    """根据命名猜测右臂 5 关节与基座/末端。"""
    patterns = [
        f"{side}_shoulder_pitch_joint",
        f"{side}_shoulder_roll_joint",
        f"{side}_upper_arm_joint",
        f"{side}_elbow_joint",
        f"{side}_wrist_joint",
    ]
    missing = [p for p in patterns if p not in joints]
    if missing:
        raise KeyError(f"无法自动匹配右臂关节，缺少: {missing}")
    j0 = joints[patterns[0]]
    j4 = joints[patterns[4]]
    return patterns, j0.parent, j4.child


def static_origin_transform(joint: RevoluteJoint) -> np.ndarray:
    return origin_matrix(joint.origin_xyz, joint.origin_rpy)
