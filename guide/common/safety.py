#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""订 /moon/obstacle，为路线执行器提供暂停/慢行判定。"""

from __future__ import annotations

import os
import sys
import time
from typing import Optional, Tuple

import rospy
from std_msgs.msg import Float32MultiArray

_VISION = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "vision")
_VISION = os.path.normpath(_VISION)
if _VISION not in sys.path:
    sys.path.insert(0, _VISION)

from obstacle_state import ObstacleState  # noqa: E402

OBSTACLE_TOPIC = "/moon/obstacle"
OBSTACLE_TIMEOUT = 0.8


class ObstacleMonitor:
    """订阅障碍状态；forward_cap==0 → 应停车暂停路线。"""

    def __init__(self, enabled: bool = True, required: bool = False):
        self.enabled = enabled
        self.required = required
        self._obs = ObstacleState(valid=False)
        self._stamp = 0.0
        self._sub: Optional[rospy.Subscriber] = None

    def start(self) -> None:
        if not self.enabled:
            print("\033[90m[OBS]\033[0m 障碍门控关闭")
            return
        self._sub = rospy.Subscriber(OBSTACLE_TOPIC, Float32MultiArray, self._cb, queue_size=5)
        print(f"\033[92m[OBS]\033[0m 订阅 {OBSTACLE_TOPIC}")

    def _cb(self, msg: Float32MultiArray) -> None:
        self._obs = ObstacleState.from_list(msg.data, stamp=time.time())
        self._stamp = time.time()

    def stale(self) -> bool:
        if not self.enabled:
            return False
        if self._stamp <= 0:
            return True
        return (time.time() - self._stamp) > OBSTACLE_TIMEOUT

    def status(self) -> Tuple[str, float]:
        """
        返回 (reason, forward_cap)。
        reason: CLEAR | SLOW | STOP | NO_VISION | OFF
        """
        if not self.enabled:
            return "OFF", 1.0
        if self.stale() or not self._obs.valid:
            if self.required:
                return "NO_VISION", 0.0
            return "VISION_OPTIONAL", 1.0
        cap = float(max(0.0, min(1.0, self._obs.forward_cap)))
        if cap <= 0.0:
            return "STOP", 0.0
        if cap < 1.0:
            return "SLOW", cap
        return "CLEAR", 1.0

    def should_pause(self) -> bool:
        reason, cap = self.status()
        return reason in ("STOP", "NO_VISION") or cap <= 0.0

    @property
    def center_m(self) -> float:
        return float(self._obs.center_m)
