# -*- coding: utf-8 -*-
"""脸偏差 → 脖子目标（控制律，不碰相机）。"""

from __future__ import annotations

import math
from typing import Dict, Tuple

# 与 face locate_face / face_tracker 对齐
DEAD_BAND_X = 0.02
DEAD_BAND_Y = 0.05
K_YAW_DEG = 30.0
K_PITCH_DEG = 15.0
MAX_STEP_YAW_DEG = 10.0
MAX_STEP_PITCH_DEG = 6.5
TARGET_EMA_ALPHA = 0.38
YAW_DX_SIGN = 1.0
YAW_LIMIT_DEG = 80.0
PITCH_UP_DEG = -40.0
PITCH_DOWN_DEG = 60.0
RETURN_HOME_RATE_DEG_PER_SEC = 45.0
NO_FACE_RETURN_HOME_SEC = 1.0

HEAD_YAW_JOINT = "head_yaw_joint"
HEAD_PITCH_JOINT = "head_pitch_joint"
ABSOLUTE_TOPIC = "/pi_plus_absolute"


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


class NeckServo:
    """根据 /moon/face 的 dx_n, dy_n 更新 yaw/pitch 目标。"""

    def __init__(self):
        self._state: Dict[str, float] = {"yaw_rad": 0.0, "pitch_rad": 0.0}
        self._last_face_t = 0.0
        self.yaw = 0.0
        self.pitch = 0.0

    def reset_home(self) -> None:
        self._state = {"yaw_rad": 0.0, "pitch_rad": 0.0}
        self.yaw = 0.0
        self.pitch = 0.0

    def update_from_face(
        self,
        dx_n: float,
        dy_n: float,
        has_face: bool,
        now: float,
        dt: float,
    ) -> Tuple[float, float]:
        if has_face:
            self._last_face_t = now
            if abs(dx_n) < DEAD_BAND_X:
                dx_n = 0.0
            if abs(dy_n) < DEAD_BAND_Y:
                dy_n = 0.0
            dx_ctrl = YAW_DX_SIGN * dx_n
            delta_yaw_deg = _clamp(
                -K_YAW_DEG * dx_ctrl, -MAX_STEP_YAW_DEG, MAX_STEP_YAW_DEG
            )
            delta_pitch_deg = _clamp(
                K_PITCH_DEG * dy_n, -MAX_STEP_PITCH_DEG, MAX_STEP_PITCH_DEG
            )
            base_yaw = self._state["yaw_rad"]
            base_pitch = self._state["pitch_rad"]
            raw_yaw = base_yaw + math.radians(delta_yaw_deg)
            raw_pitch = base_pitch + math.radians(delta_pitch_deg)
            a = TARGET_EMA_ALPHA
            yaw_new = base_yaw * (1 - a) + raw_yaw * a
            pitch_new = base_pitch * (1 - a) + raw_pitch * a
            yaw_new = _clamp(
                yaw_new,
                -math.radians(YAW_LIMIT_DEG),
                math.radians(YAW_LIMIT_DEG),
            )
            pitch_new = _clamp(
                pitch_new,
                math.radians(PITCH_UP_DEG),
                math.radians(PITCH_DOWN_DEG),
            )
            self._state["yaw_rad"] = yaw_new
            self._state["pitch_rad"] = pitch_new
            self.yaw, self.pitch = yaw_new, pitch_new
            return self.yaw, self.pitch

        # 无人脸：超时后回中
        if self._last_face_t > 0 and (
            now - self._last_face_t
        ) > NO_FACE_RETURN_HOME_SEC:
            step = math.radians(RETURN_HOME_RATE_DEG_PER_SEC * max(dt, 1e-3))
            y, p = self._state["yaw_rad"], self._state["pitch_rad"]
            y = 0.0 if abs(y) <= step else y - math.copysign(step, y)
            p = 0.0 if abs(p) <= step else p - math.copysign(step, p)
            self._state["yaw_rad"] = y
            self._state["pitch_rad"] = p
            self.yaw, self.pitch = y, p
        return self.yaw, self.pitch
