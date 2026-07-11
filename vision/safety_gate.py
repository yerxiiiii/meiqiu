#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
安全门控：纯函数，无 ROS / ZED 依赖。

输入：跟随（或遥控）算出的期望摇杆 + ObstacleState
输出：裁剪后的摇杆
"""

from __future__ import annotations

import math
from typing import Tuple

from obstacle_state import ObstacleState, nearest_valid

# 距离阈值 (m)
STOP_DIST = 0.50          # 近于此：前进归零
SLOW_DIST = 1.00          # STOP~SLOW：线性限速
CLEAR_DIST = 1.50         # 大于此：不干预前进

# 绕障：中间堵、一侧更通时叠加转向
ENABLE_SIDESTEP = True
SIDESTEP_CENTER = 0.80    # 中区近于此才考虑偏转
SIDESTEP_GAIN = 0.45      # 叠加到 rot 的幅度上限
SIDESTEP_CLEAR_MARGIN = 0.25  # 侧区至少比中区远这么多才偏

# 视觉丢失策略（由调用方结合 OBSTACLE_REQUIRED 使用）
STALE_FORWARD_CAP = 0.0   # 强制安全时：无有效障碍则禁止前进


def _cap_from_distance(d: float) -> float:
    """单区距离 → forward_cap 0~1。"""
    if not math.isfinite(d) or d <= 0:
        return 1.0
    if d <= STOP_DIST:
        return 0.0
    if d >= CLEAR_DIST:
        return 1.0
    if d <= SLOW_DIST:
        # STOP → SLOW：0 → ~0.5
        return 0.5 * (d - STOP_DIST) / max(1e-6, SLOW_DIST - STOP_DIST)
    # SLOW → CLEAR：0.5 → 1
    return 0.5 + 0.5 * (d - SLOW_DIST) / max(1e-6, CLEAR_DIST - SLOW_DIST)


def compute_caps_from_zones(
    left_m: float,
    center_m: float,
    right_m: float,
) -> Tuple[float, float]:
    """
    由三区深度算 forward_cap / rotate_bias。
    感知节点可调用；门控也可在只有距离时重算。
    """
    # 前进以中区为主，左右作辅助（取更近者略收紧）
    mid_cap = _cap_from_distance(center_m) if math.isfinite(center_m) else 1.0
    side_near = nearest_valid(left_m, right_m)
    if side_near is not None and side_near < SLOW_DIST:
        side_cap = _cap_from_distance(side_near)
        forward_cap = min(mid_cap, max(side_cap, mid_cap * 0.7))
    else:
        forward_cap = mid_cap

    rotate_bias = 0.0
    if ENABLE_SIDESTEP and math.isfinite(center_m) and center_m < SIDESTEP_CENTER:
        left_ok = math.isfinite(left_m)
        right_ok = math.isfinite(right_m)
        if left_ok and right_ok:
            # 往更远的一侧偏（rot>0 约定与 uwb_follow 的 r_horizontal 一致：正=右转）
            if right_m > left_m + SIDESTEP_CLEAR_MARGIN:
                rotate_bias = SIDESTEP_GAIN * (1.0 - center_m / SIDESTEP_CENTER)
            elif left_m > right_m + SIDESTEP_CLEAR_MARGIN:
                rotate_bias = -SIDESTEP_GAIN * (1.0 - center_m / SIDESTEP_CENTER)
        elif right_ok and (not left_ok or right_m > center_m + SIDESTEP_CLEAR_MARGIN):
            rotate_bias = SIDESTEP_GAIN * 0.5
        elif left_ok and (not right_ok or left_m > center_m + SIDESTEP_CLEAR_MARGIN):
            rotate_bias = -SIDESTEP_GAIN * 0.5

    return float(max(0.0, min(1.0, forward_cap))), float(max(-1.0, min(1.0, rotate_bias)))


def apply_safety_gate(
    fwd: float,
    rot: float,
    obs: ObstacleState,
    *,
    use_rotate_bias: bool = True,
    stale: bool = False,
    required: bool = False,
) -> Tuple[float, float, str]:
    """
    裁剪期望摇杆。

    stale: 障碍话题超时/未收到
    required: True 且 stale → 禁止前进（fail-safe）
    返回: (fwd, rot, reason)
    """
    if stale or not obs.valid:
        if required:
            return 0.0, 0.0, "NO_VISION"
        return fwd, rot, "VISION_OPTIONAL"

    cap = float(max(0.0, min(1.0, obs.forward_cap)))
    # 只限制前进；后退（fwd<0）暂不因前方障碍放大（可按需改）
    if fwd > 0.0:
        fwd = fwd * cap

    reason = "CLEAR"
    if cap <= 0.0 and fwd >= 0.0:
        fwd = 0.0
        reason = "STOP"
    elif cap < 1.0:
        reason = "SLOW"

    if use_rotate_bias and abs(obs.rotate_bias) > 1e-3:
        # 叠加偏置，不覆盖 UWB 主转向；限幅到 [-1,1]
        rot = max(-1.0, min(1.0, rot + obs.rotate_bias))
        if reason == "CLEAR":
            reason = "SIDESTEP"
        else:
            reason = reason + "+SIDE"

    return fwd, rot, reason
