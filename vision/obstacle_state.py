#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""障碍状态：感知与跟随之间的唯一契约（无硬件依赖）。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

# /moon/obstacle 布局（std_msgs/Float32MultiArray.data）
# [left_m, center_m, right_m, forward_cap, rotate_bias, valid]
LAYOUT_LEN = 6


@dataclass
class ObstacleState:
    left_m: float = float("nan")
    center_m: float = float("nan")
    right_m: float = float("nan")
    forward_cap: float = 1.0   # 0~1，乘到前进摇杆
    rotate_bias: float = 0.0   # -1~1，正=建议右转（与 joy r_horizontal 同号约定）
    valid: bool = False
    stamp: float = 0.0         # time.time()，由订阅端填写亦可

    def to_list(self) -> list:
        return [
            float(self.left_m),
            float(self.center_m),
            float(self.right_m),
            float(self.forward_cap),
            float(self.rotate_bias),
            1.0 if self.valid else 0.0,
        ]

    @classmethod
    def from_list(cls, data: Sequence[float], stamp: float = 0.0) -> "ObstacleState":
        if len(data) < LAYOUT_LEN:
            return cls(valid=False, stamp=stamp)
        return cls(
            left_m=float(data[0]),
            center_m=float(data[1]),
            right_m=float(data[2]),
            forward_cap=float(data[3]),
            rotate_bias=float(data[4]),
            valid=bool(data[5] >= 0.5),
            stamp=stamp,
        )

    @staticmethod
    def is_finite_dist(d: float) -> bool:
        return d is not None and math.isfinite(d) and d > 0.0


def nearest_valid(*dists: float) -> Optional[float]:
    vals = [d for d in dists if ObstacleState.is_finite_dist(d)]
    return min(vals) if vals else None
