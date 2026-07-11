# -*- coding: utf-8 -*-
"""运控 FSM 监护：订阅 /fsm_state，解析 RUNNING 日志，摔倒急停判定。"""

from __future__ import annotations

import threading
import time
from typing import Optional, Set

import rospy
from rosgraph_msgs.msg import Log
from std_msgs.msg import Int32

FSM_TOPIC = "/fsm_state"
ROSOUT_TOPIC = "/rosout"

# 与 sim2real fsm.h FsmNodeType 一致
FSM_INIT = 0
FSM_ERROR = 1
FSM_EXEC_DEFAULT = 5  # STANDBY / RUNNING 都在此主态
FSM_PROTECTION_SHUTDOWN = 8

# 允许下发跟随速度的主态
MOTION_OK_FSM: Set[int] = {FSM_EXEC_DEFAULT}

# 必须立即零速并退出跟随
FATAL_FSM: Set[int] = {
    FSM_ERROR,
    FSM_PROTECTION_SHUTDOWN,
}

_RUNNING_RE = "sim2real switch to running state"


class FsmGuard:
    """线程安全的 FSM / RUNNING 观测。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.fsm: Optional[int] = None
        self.updated_at: float = 0.0
        self.running_hint: bool = False  # 见过 switch to running
        self.running_hint_at: float = 0.0
        self.standby_hint_at: float = 0.0
        self._fsm_sub = rospy.Subscriber(FSM_TOPIC, Int32, self._on_fsm, queue_size=5)
        self._log_sub = rospy.Subscriber(ROSOUT_TOPIC, Log, self._on_log, queue_size=50)

    def _on_fsm(self, msg: Int32) -> None:
        with self._lock:
            self.fsm = int(msg.data)
            self.updated_at = time.time()

    def _on_log(self, msg: Log) -> None:
        text = msg.msg or ""
        if _RUNNING_RE in text:
            with self._lock:
                self.running_hint = True
                self.running_hint_at = time.time()
        # 精确匹配运控日志，避免误清
        if "sim2real switch to standby state" in text:
            with self._lock:
                self.running_hint = False
                self.standby_hint_at = time.time()

    def snapshot(self) -> tuple:
        with self._lock:
            return self.fsm, self.running_hint, self.updated_at

    def is_fatal(self) -> bool:
        fsm, _, _ = self.snapshot()
        return fsm is not None and fsm in FATAL_FSM

    def allows_motion(self) -> bool:
        """非致命且处于 EXEC_DEFAULT 才允许发非零速度。"""
        fsm, _, _ = self.snapshot()
        if fsm is None:
            return True  # 尚未收到时不堵死（启动窗口）
        if fsm in FATAL_FSM:
            return False
        return fsm in MOTION_OK_FSM

    def is_running(self) -> bool:
        _, run_h, _ = self.snapshot()
        return bool(run_h)

    def clear_running_hint(self) -> None:
        with self._lock:
            self.running_hint = False
            self.running_hint_at = 0.0

    def wait_running_hint(self, since: float, timeout: float = 2.5) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            with self._lock:
                if self.running_hint and self.running_hint_at >= since:
                    return True
            time.sleep(0.05)
        with self._lock:
            return self.running_hint and self.running_hint_at >= since

    def saw_standby_since(self, since: float) -> bool:
        with self._lock:
            return self.standby_hint_at >= since
    def unregister(self) -> None:
        for sub in (self._fsm_sub, self._log_sub):
            try:
                sub.unregister()
            except Exception:
                pass
