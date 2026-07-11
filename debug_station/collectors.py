#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
调试台数据采集（只读）
====================
- 订 ROS 话题：/moon/obstacle, /cmd_vel, /fsm_state, /joy_msg
- 尾读 uwb_follow 会话日志（不占串口，不改跟随代码）
- 不发布任何控制指令
"""

from __future__ import annotations

import glob
import os
import re
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional

# ---------------------------------------------------------------------------
# 路径（相对本包，不硬编码进功能模块）
# ---------------------------------------------------------------------------
MOON_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
LOG_DIR = os.path.join(MOON_ROOT, "logs")
UWB_LOG_GLOB = os.path.join(LOG_DIR, "uwb_follow_*.log")

OBS_STALE_SEC = 0.8
UWB_STALE_SEC = 1.5
CMD_STALE_SEC = 1.0


@dataclass
class Snapshot:
    t: float = field(default_factory=time.time)
    # perception
    obstacle: Dict[str, Any] = field(default_factory=dict)
    obstacle_age: float = 999.0
    uwb: Dict[str, Any] = field(default_factory=dict)
    uwb_age: float = 999.0
    uwb_log_path: str = ""
    # decision (from log MOTION)
    decision: Dict[str, Any] = field(default_factory=dict)
    # execution
    fsm_state: Optional[int] = None
    fsm_age: float = 999.0
    cmd_vel: Dict[str, float] = field(default_factory=dict)
    cmd_age: float = 999.0
    cmd_hz: float = 0.0
    joy_msg: Dict[str, float] = field(default_factory=dict)
    joy_age: float = 999.0
    # meta
    ros_ok: bool = False
    layers: Dict[str, Any] = field(default_factory=dict)
    events: List[str] = field(default_factory=list)


class Collectors:
    def __init__(self):
        self._lock = threading.Lock()
        self._obs: Dict[str, Any] = {}
        self._obs_t = 0.0
        self._fsm: Optional[int] = None
        self._fsm_t = 0.0
        self._cmd = {"vx": 0.0, "vy": 0.0, "vz": 0.0, "wz": 0.0}
        self._cmd_t = 0.0
        self._cmd_stamps: Deque[float] = deque(maxlen=50)
        self._joy: Dict[str, float] = {}
        self._joy_t = 0.0
        self._uwb: Dict[str, Any] = {}
        self._uwb_t = 0.0
        self._decision: Dict[str, Any] = {}
        self._events: Deque[str] = deque(maxlen=80)
        self._log_path = ""
        self._log_pos = 0
        self._ros_ok = False
        self._stop = False

    # ---- ROS callbacks ----
    def _on_obstacle(self, msg) -> None:
        d = list(msg.data) if msg.data else []
        now = time.time()
        with self._lock:
            self._obs = {
                "left_m": float(d[0]) if len(d) > 0 else float("nan"),
                "center_m": float(d[1]) if len(d) > 1 else float("nan"),
                "right_m": float(d[2]) if len(d) > 2 else float("nan"),
                "forward_cap": float(d[3]) if len(d) > 3 else 1.0,
                "rotate_bias": float(d[4]) if len(d) > 4 else 0.0,
                "valid": bool(len(d) > 5 and d[5] >= 0.5),
            }
            self._obs_t = now

    def _on_fsm(self, msg) -> None:
        with self._lock:
            self._fsm = int(msg.data)
            self._fsm_t = time.time()

    def _on_cmd(self, msg) -> None:
        now = time.time()
        with self._lock:
            self._cmd = {
                "vx": float(msg.linear.x),
                "vy": float(msg.linear.y),
                "vz": float(msg.linear.z),
                "wz": float(msg.angular.z),
            }
            self._cmd_t = now
            self._cmd_stamps.append(now)

    def _on_joy(self, msg) -> None:
        with self._lock:
            self._joy = {
                "l_vertical": float(getattr(msg, "l_vertical", 0.0)),
                "r_horizontal": float(getattr(msg, "r_horizontal", 0.0)),
                "lt": float(getattr(msg, "lt", 0.0)),
                "rt": float(getattr(msg, "rt", 0.0)),
                "lb": float(getattr(msg, "lb", 0.0)),
            }
            self._joy_t = time.time()

    def start_ros(self) -> bool:
        try:
            import rospy
            from geometry_msgs.msg import Twist
            from std_msgs.msg import Float32MultiArray, Int32
            from sim2real_msg.msg import Joy as SimJoy
        except Exception as e:
            self._push_event(f"ROS import failed: {e}")
            return False

        if not rospy.core.is_initialized():
            rospy.init_node("moon_debug_station", anonymous=True, disable_signals=True)

        rospy.Subscriber("/moon/obstacle", Float32MultiArray, self._on_obstacle, queue_size=2)
        rospy.Subscriber("/fsm_state", Int32, self._on_fsm, queue_size=2)
        rospy.Subscriber("/cmd_vel", Twist, self._on_cmd, queue_size=5)
        rospy.Subscriber("/joy_msg", SimJoy, self._on_joy, queue_size=5)
        self._ros_ok = True
        self._push_event("ROS subscribers ready (read-only)")
        return True

    def start_log_tail(self) -> None:
        t = threading.Thread(target=self._log_loop, name="uwb-log-tail", daemon=True)
        t.start()

    def _push_event(self, line: str) -> None:
        with self._lock:
            self._events.appendleft(f"{time.strftime('%H:%M:%S')} {line}")

    def _pick_log(self) -> str:
        files = sorted(glob.glob(UWB_LOG_GLOB), key=os.path.getmtime, reverse=True)
        return files[0] if files else ""

    def _log_loop(self) -> None:
        motion_re = re.compile(
            r"MOTION\s+fsm=(?P<fsm>\S+)\s+dist=(?P<dist>\S+)\s+ang=(?P<ang>\S+)\s+"
            r"ctrl=(?P<ctrl>\S+)\s+fwd=(?P<fwd>\S+)\s+rot=(?P<rot>\S+)\s+"
            r"cmd_vel=\((?P<vx>[^,]+),0,(?P<wz>[^)]+)\)\s+joy\(lt=(?P<lt>[^,]+),rt=(?P<rt>[^,]+),lb=(?P<lb>[^)]+)\)\s+"
            r"soft=(?P<soft>\S+)\s+gate=(?P<gate>\S+)"
        )
        while not self._stop:
            path = self._pick_log()
            if not path:
                time.sleep(0.5)
                continue
            if path != self._log_path:
                self._log_path = path
                self._log_pos = 0
                self._push_event(f"tail log: {os.path.basename(path)}")
            try:
                with open(path, "r", encoding="utf-8", errors="replace") as f:
                    f.seek(0, os.SEEK_END)
                    size = f.tell()
                    if self._log_pos > size:
                        self._log_pos = 0
                    f.seek(self._log_pos)
                    chunk = f.read()
                    self._log_pos = f.tell()
                if chunk:
                    for line in chunk.splitlines():
                        if "EVENT" in line:
                            self._push_event(line.split("] ", 1)[-1] if "] " in line else line)
                        m = motion_re.search(line)
                        if m:
                            g = m.groupdict()
                            now = time.time()
                            with self._lock:
                                self._uwb = {
                                    "dist_cm": _f(g["dist"]),
                                    "ang_deg": _f(g["ang"]),
                                    "source": "uwb_follow_log",
                                }
                                self._uwb_t = now
                                self._decision = {
                                    "ctrl": g["ctrl"],
                                    "fwd": _f(g["fwd"]),
                                    "rot": _f(g["rot"]),
                                    "soft": _f(g["soft"]),
                                    "gate": g["gate"],
                                    "cmd_vx": _f(g["vx"]),
                                    "cmd_wz": _f(g["wz"]),
                                    "fsm_in_log": g["fsm"],
                                }
            except Exception as e:
                self._push_event(f"log read err: {e}")
            time.sleep(0.15)

    def snapshot(self) -> Snapshot:
        now = time.time()
        with self._lock:
            obs_age = (now - self._obs_t) if self._obs_t else 999.0
            uwb_age = (now - self._uwb_t) if self._uwb_t else 999.0
            fsm_age = (now - self._fsm_t) if self._fsm_t else 999.0
            cmd_age = (now - self._cmd_t) if self._cmd_t else 999.0
            joy_age = (now - self._joy_t) if self._joy_t else 999.0
            stamps = list(self._cmd_stamps)
            hz = 0.0
            if len(stamps) >= 2:
                dt = stamps[-1] - stamps[0]
                if dt > 1e-3:
                    hz = (len(stamps) - 1) / dt
            snap = Snapshot(
                t=now,
                obstacle=dict(self._obs),
                obstacle_age=obs_age,
                uwb=dict(self._uwb),
                uwb_age=uwb_age,
                uwb_log_path=self._log_path,
                decision=dict(self._decision),
                fsm_state=self._fsm,
                fsm_age=fsm_age,
                cmd_vel=dict(self._cmd),
                cmd_age=cmd_age,
                cmd_hz=hz,
                joy_msg=dict(self._joy),
                joy_age=joy_age,
                ros_ok=self._ros_ok,
                events=list(self._events)[:40],
            )

        snap.layers = _eval_layers(snap)
        return snap

    def stop(self) -> None:
        self._stop = True


def _f(s: str) -> float:
    try:
        if s in ("nan", "None"):
            return float("nan")
        return float(s)
    except Exception:
        return float("nan")


def _eval_layers(s: Snapshot) -> Dict[str, Any]:
    """感知 / 决策 / 执行 健康灯。"""
    # perception
    uwb_ok = s.uwb_age < UWB_STALE_SEC and bool(s.uwb)
    obs_ok = s.obstacle_age < OBS_STALE_SEC and s.obstacle.get("valid", False)
    if uwb_ok and obs_ok:
        perc = {"status": "green", "detail": "UWB log fresh + obstacle valid"}
    elif uwb_ok or obs_ok:
        which = []
        if uwb_ok:
            which.append("UWB")
        else:
            which.append("UWB stale/missing")
        if obs_ok:
            which.append("ZED obstacle")
        else:
            which.append("obstacle stale/off")
        perc = {"status": "yellow", "detail": " / ".join(which)}
    else:
        perc = {
            "status": "red",
            "detail": f"UWB age={s.uwb_age:.1f}s obstacle age={s.obstacle_age:.1f}s",
        }

    # decision
    dec = s.decision
    if dec and s.uwb_age < UWB_STALE_SEC:
        gate = str(dec.get("gate", ""))
        ctrl = str(dec.get("ctrl", ""))
        if gate == "STOP" or (abs(float(dec.get("fwd", 0) or 0)) < 1e-3 and ctrl == "FOLLOW"):
            decision = {"status": "yellow", "detail": f"ctrl={ctrl} gate={gate} (限速/停)"}
        else:
            decision = {"status": "green", "detail": f"ctrl={ctrl} gate={gate} fwd={dec.get('fwd')}"}
    elif s.uwb_age < UWB_STALE_SEC:
        decision = {"status": "yellow", "detail": "有 UWB 日志但尚无 MOTION"}
    else:
        decision = {"status": "red", "detail": "无跟随决策输出（检查 uwb_follow 是否在跑）"}

    # execution
    fsm = s.fsm_state
    cmd_live = s.cmd_age < CMD_STALE_SEC and s.cmd_hz > 1.0
    if fsm == 8:
        exec_ = {"status": "red", "detail": "fsm=8 PROTECTION_SHUTDOWN"}
    elif cmd_live and fsm == 5:
        vx = abs(s.cmd_vel.get("vx", 0.0))
        if vx < 0.02 and abs(s.cmd_vel.get("wz", 0.0)) < 0.02:
            exec_ = {"status": "yellow", "detail": f"fsm=5 cmd hz={s.cmd_hz:.0f} 但速度≈0"}
        else:
            exec_ = {"status": "green", "detail": f"fsm=5 cmd hz={s.cmd_hz:.0f} vx={s.cmd_vel.get('vx', 0):.2f}"}
    elif cmd_live:
        exec_ = {"status": "yellow", "detail": f"有 cmd_vel 但 fsm={fsm}"}
    else:
        exec_ = {"status": "red", "detail": f"无有效 cmd_vel (age={s.cmd_age:.1f}s hz={s.cmd_hz:.1f})"}

    return {"perception": perc, "decision": decision, "execution": exec_}
