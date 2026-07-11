# -*- coding: utf-8 -*-
"""运控 FSM 监护：订阅 /fsm_state，解析 RUNNING 日志，摔倒急停判定。"""

from __future__ import annotations

import re
import threading
import time
from pathlib import Path
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
_STANDBY_RE = "sim2real switch to standby state"
_TAIL_BYTES = 400_000


def _last_run_standby_from_rosout_log() -> Optional[str]:
    """读 rosout 尾部，返回最后一次是 'running' 还是 'standby'。"""
    candidates = [
        Path.home() / ".ros" / "log" / "latest" / "rosout.log",
        Path("/home/nvidia/.ros/log/latest/rosout.log"),
    ]
    for path in candidates:
        try:
            if not path.is_file():
                continue
            text = path.read_bytes()[-_TAIL_BYTES:].decode("utf-8", errors="ignore")
            last = None
            for m in re.finditer(
                r"sim2real switch to (running|standby) state", text
            ):
                last = m.group(1)
            return last
        except OSError:
            continue
    return None


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
        # arbiter 重启后订阅无历史：从日志恢复，避免误点 LB 把 RUNNING 拨回 STANDBY
        self.recover_running_from_log()

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
        if _STANDBY_RE in text:
            with self._lock:
                self.running_hint = False
                self.standby_hint_at = time.time()

    def recover_running_from_log(self) -> bool:
        """
        用 ~/.ros/log/latest/rosout.log 尾部恢复 running_hint。

        返回当前是否判定为 RUNNING。已在 RUNNING 时切勿再发 LB。
        """
        last = _last_run_standby_from_rosout_log()
        now = time.time()
        with self._lock:
            if last == "running":
                self.running_hint = True
                # 用略早时间戳，避免 wait_running_hint(since=now) 误判
                self.running_hint_at = now - 1.0
                print(
                    "\033[92m[FSM]\033[0m 日志恢复: 运控仍在 RUNNING"
                    "（跳过 LB，避免误拨回 STANDBY）"
                )
                return True
            if last == "standby":
                self.running_hint = False
                self.standby_hint_at = now - 1.0
                print("\033[90m[FSM]\033[0m 日志恢复: 运控在 STANDBY")
                return False
        print("\033[90m[FSM]\033[0m 日志无 running/standby 记录，保持未知")
        return False

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
