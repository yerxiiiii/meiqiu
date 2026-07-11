# -*- coding: utf-8 -*-
"""简化手柄仲裁：有输入时让路。"""

from __future__ import annotations

import threading
import time

import rospy
from sensor_msgs.msg import Joy

# humanoid_driver 转发 /joy_input → /joy；joy.yaml: button[10]=R（语音开麦）
JOY_TOPIC = "/joy"
MIC_ARM_BUTTON_INDEX = 10
# 右摇杆垂直轴：按下 R 时必然有位移，不能当作“抢遥控”
IGNORE_AXES = {4}
ACTIVE_THRESH = 0.15
IDLE_SEC = 3.0
TRIGGER_AXES = (2, 5)
TRIGGER_REST = 1.0
TRIGGER_MARGIN = 0.35


class JoyMonitor:
    def __init__(self, idle_sec: float = IDLE_SEC):
        self._lock = threading.Lock()
        self._idle_sec = idle_sec
        self._last_active_t = 0.0
        self._sub = rospy.Subscriber(JOY_TOPIC, Joy, self._cb, queue_size=5)

    def _axis_active(self, idx: int, val: float) -> bool:
        if idx in IGNORE_AXES:
            return False
        v = float(val)
        if idx in TRIGGER_AXES:
            return v < (TRIGGER_REST - TRIGGER_MARGIN)
        return abs(v) > ACTIVE_THRESH

    def _cb(self, msg: Joy):
        for i, ax in enumerate(msg.axes):
            if self._axis_active(i, ax):
                with self._lock:
                    self._last_active_t = time.time()
                return
        for i, btn in enumerate(msg.buttons):
            if i == MIC_ARM_BUTTON_INDEX:
                continue
            if int(btn) != 0:
                with self._lock:
                    self._last_active_t = time.time()
                return

    def blocks(self) -> bool:
        with self._lock:
            if self._last_active_t <= 0:
                return False
            return (time.time() - self._last_active_t) < self._idle_sec
