#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
调试台数据采集（只读 + 语音链路状态）
====================================
- ROS: /moon/obstacle, /cmd_vel, /fsm_state, /joy_msg, /imu/data
       /moon/mode, /guide/state, /moon/voice_cmd, /guide/voice_command
- 尾读 uwb_follow 日志
- 进程健康 + 麦克风电平代理
"""

from __future__ import annotations

import glob
import math
import os
import re
import subprocess
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional

MOON_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
LOG_DIR = os.path.join(MOON_ROOT, "logs")
UWB_LOG_GLOB = os.path.join(LOG_DIR, "uwb_follow_*.log")
MIC_METER_URL = "http://127.0.0.1:8091/api/rms"

OBS_STALE_SEC = 0.8
UWB_STALE_SEC = 1.5
CMD_STALE_SEC = 1.0
IMU_STALE_SEC = 0.5

PROC_PATTERNS = {
    "kws_trigger": "kws_trigger_node.py",
    "kws_brain": "kws_node.py",
    "mode_arbiter": "mode_arbiter.py",
    "guide_demo": "guide_demo_node.py",
    "uwb_follow": "uwb_follow.py",
    "zed_obstacle": "zed_obstacle_node.py",
}


@dataclass
class Snapshot:
    t: float = field(default_factory=time.time)
    obstacle: Dict[str, Any] = field(default_factory=dict)
    obstacle_age: float = 999.0
    uwb: Dict[str, Any] = field(default_factory=dict)
    uwb_age: float = 999.0
    uwb_log_path: str = ""
    decision: Dict[str, Any] = field(default_factory=dict)
    fsm_state: Optional[int] = None
    fsm_age: float = 999.0
    cmd_vel: Dict[str, float] = field(default_factory=dict)
    cmd_age: float = 999.0
    cmd_hz: float = 0.0
    joy_msg: Dict[str, float] = field(default_factory=dict)
    joy_age: float = 999.0
    imu: Dict[str, Any] = field(default_factory=dict)
    imu_age: float = 999.0
    moon_mode: str = ""
    moon_mode_age: float = 999.0
    guide_state: str = ""
    guide_state_age: float = 999.0
    last_voice_cmd: str = ""
    last_voice_cmd_t: float = 0.0
    last_guide_cmd: str = ""
    last_guide_cmd_t: float = 0.0
    mic: Dict[str, Any] = field(default_factory=dict)
    processes: Dict[str, Any] = field(default_factory=dict)
    voice_chain: Dict[str, Any] = field(default_factory=dict)
    ros_ok: bool = False
    layers: Dict[str, Any] = field(default_factory=dict)
    events: List[str] = field(default_factory=list)


def _quat_to_euler(x: float, y: float, z: float, w: float) -> Dict[str, float]:
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return {
        "roll_deg": math.degrees(roll),
        "pitch_deg": math.degrees(pitch),
        "yaw_deg": math.degrees(yaw),
    }


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
        self._imu: Dict[str, Any] = {}
        self._imu_t = 0.0
        self._moon_mode = ""
        self._moon_mode_t = 0.0
        self._guide_state = ""
        self._guide_state_t = 0.0
        self._last_voice_cmd = ""
        self._last_voice_cmd_t = 0.0
        self._last_guide_cmd = ""
        self._last_guide_cmd_t = 0.0
        self._uwb: Dict[str, Any] = {}
        self._uwb_t = 0.0
        self._decision: Dict[str, Any] = {}
        self._events: Deque[str] = deque(maxlen=80)
        self._log_path = ""
        self._log_pos = 0
        self._ros_ok = False
        self._stop = False
        self._mic_cache: Dict[str, Any] = {}
        self._mic_cache_t = 0.0
        self._process_cache: Dict[str, Any] = _scan_processes()

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

    def _on_imu(self, msg) -> None:
        now = time.time()
        o = msg.orientation
        euler = _quat_to_euler(o.x, o.y, o.z, o.w)
        la = msg.linear_acceleration
        av = msg.angular_velocity
        with self._lock:
            self._imu = {
                **euler,
                "ax": float(la.x),
                "ay": float(la.y),
                "az": float(la.z),
                "gx": float(av.x),
                "gy": float(av.y),
                "gz": float(av.z),
            }
            self._imu_t = now

    def _on_moon_mode(self, msg) -> None:
        with self._lock:
            self._moon_mode = str(msg.data)
            self._moon_mode_t = time.time()

    def _on_guide_state(self, msg) -> None:
        with self._lock:
            self._guide_state = str(msg.data)
            self._guide_state_t = time.time()

    def _on_voice_cmd(self, msg) -> None:
        now = time.time()
        with self._lock:
            self._last_voice_cmd = str(msg.data)
            self._last_voice_cmd_t = now
        self._push_event(f"voice_cmd: {msg.data}")

    def _on_guide_cmd(self, msg) -> None:
        now = time.time()
        with self._lock:
            self._last_guide_cmd = str(msg.data)
            self._last_guide_cmd_t = now
        self._push_event(f"guide_cmd: {msg.data}")

    def start_ros(self) -> bool:
        try:
            import rospy
            from geometry_msgs.msg import Twist
            from sensor_msgs.msg import Imu
            from std_msgs.msg import Float32MultiArray, Int32, String
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
        rospy.Subscriber("/imu/data", Imu, self._on_imu, queue_size=10)
        rospy.Subscriber("/moon/mode", String, self._on_moon_mode, queue_size=2)
        rospy.Subscriber("/guide/state", String, self._on_guide_state, queue_size=2)
        rospy.Subscriber("/moon/voice_cmd", String, self._on_voice_cmd, queue_size=10)
        rospy.Subscriber("/guide/voice_command", String, self._on_guide_cmd, queue_size=10)
        self._ros_ok = True
        self._push_event("ROS subscribers ready (read-only + voice chain)")
        return True

    def start_log_tail(self) -> None:
        threading.Thread(target=self._log_loop, name="uwb-log-tail", daemon=True).start()
        threading.Thread(target=self._mic_poll_loop, name="mic-proxy", daemon=True).start()
        threading.Thread(target=self._proc_poll_loop, name="proc-poll", daemon=True).start()

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

    def _mic_poll_loop(self) -> None:
        while not self._stop:
            try:
                req = urllib.request.Request(MIC_METER_URL, method="GET")
                with urllib.request.urlopen(req, timeout=1.2) as resp:
                    data = resp.read().decode("utf-8")
                mic = __import__("json").loads(data)
                now = time.time()
                with self._lock:
                    self._mic_cache = mic
                    self._mic_cache_t = now
            except Exception:
                pass
            time.sleep(0.25)

    def _proc_poll_loop(self) -> None:
        while not self._stop:
            procs = _scan_processes()
            with self._lock:
                self._process_cache = procs
            time.sleep(2.0)

    def snapshot(self) -> Snapshot:
        now = time.time()
        with self._lock:
            obs_age = (now - self._obs_t) if self._obs_t else 999.0
            uwb_age = (now - self._uwb_t) if self._uwb_t else 999.0
            fsm_age = (now - self._fsm_t) if self._fsm_t else 999.0
            cmd_age = (now - self._cmd_t) if self._cmd_t else 999.0
            joy_age = (now - self._joy_t) if self._joy_t else 999.0
            imu_age = (now - self._imu_t) if self._imu_t else 999.0
            mode_age = (now - self._moon_mode_t) if self._moon_mode_t else 999.0
            guide_age = (now - self._guide_state_t) if self._guide_state_t else 999.0
            stamps = list(self._cmd_stamps)
            hz = 0.0
            if len(stamps) >= 2:
                dt = stamps[-1] - stamps[0]
                if dt > 1e-3:
                    hz = (len(stamps) - 1) / dt
            mic = dict(self._mic_cache) if self._mic_cache else {}
            mic_age = (now - self._mic_cache_t) if self._mic_cache_t else 999.0
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
                imu=dict(self._imu),
                imu_age=imu_age,
                moon_mode=self._moon_mode,
                moon_mode_age=mode_age,
                guide_state=self._guide_state,
                guide_state_age=guide_age,
                last_voice_cmd=self._last_voice_cmd,
                last_voice_cmd_t=self._last_voice_cmd_t,
                last_guide_cmd=self._last_guide_cmd,
                last_guide_cmd_t=self._last_guide_cmd_t,
                mic={**mic, "proxy_age": mic_age},
                processes=dict(self._process_cache),
                ros_ok=self._ros_ok,
                events=list(self._events)[:40],
            )

        snap.layers = _eval_layers(snap)
        snap.voice_chain = _eval_voice_chain(snap)
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


def _scan_processes() -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for key, pattern in PROC_PATTERNS.items():
        try:
            r = subprocess.run(
                ["pgrep", "-af", pattern],
                capture_output=True,
                text=True,
                timeout=2,
            )
            lines = [ln.strip() for ln in (r.stdout or "").splitlines() if ln.strip()]
            out[key] = {"running": bool(lines), "lines": lines[:3]}
        except Exception as e:
            out[key] = {"running": False, "error": str(e)}
    return out


def _eval_voice_chain(s: Snapshot) -> Dict[str, Any]:
    """语音 → 模式 → 多模态 链路四段状态。"""
    mic = s.mic or {}
    mic_t = float(mic.get("t") or 0)
    mic_age = s.t - mic_t if mic_t else 999.0
    listening = bool(mic.get("listening"))
    rms = float(mic.get("rms") or 0)
    live_audio = listening and mic_age < 1.5 and rms >= 0.0

    if not s.processes.get("kws_trigger", {}).get("running") and not s.processes.get("kws_brain", {}).get("running"):
        mic_status = "red"
        mic_detail = "KWS 未运行"
    elif live_audio:
        mic_status = "green"
        mic_detail = f"开麦 · RMS={rms:.4f}"
    elif listening and mic_age < 1.5:
        mic_status = "yellow"
        mic_detail = "KWS 在跑但电平停滞"
    else:
        mic_status = "yellow"
        mic_detail = "关麦 / 待机"

    hit = mic.get("last_hit") or s.last_guide_cmd or s.last_voice_cmd or ""
    hit_age = min(
        s.t - float(mic.get("hit_t") or 0) if mic.get("hit_t") else 999,
        s.t - s.last_guide_cmd_t if s.last_guide_cmd_t else 999,
        s.t - s.last_voice_cmd_t if s.last_voice_cmd_t else 999,
    )
    if hit and hit_age < 30:
        kws_status = "green"
        kws_detail = f"命中: {hit} ({hit_age:.1f}s前)"
    elif s.processes.get("kws_trigger", {}).get("running") or s.processes.get("kws_brain", {}).get("running"):
        kws_status = "yellow"
        kws_detail = "监听中，尚无命中"
    else:
        kws_status = "red"
        kws_detail = "KWS 离线"

    mode = s.moon_mode or "—"
    if s.moon_mode_age < 5.0 and mode:
        if mode == "IDLE":
            mode_status = "yellow"
            mode_detail = f"模式 IDLE ({s.moon_mode_age:.1f}s)"
        else:
            mode_status = "green"
            mode_detail = f"模式 {mode} ({s.moon_mode_age:.1f}s)"
    elif s.processes.get("mode_arbiter", {}).get("running"):
        mode_status = "yellow"
        mode_detail = "arbiter 在跑，/moon/mode stale"
    else:
        mode_status = "red"
        mode_detail = "mode_arbiter 未运行"

    follow_ok = s.uwb_age < UWB_STALE_SEC or (s.cmd_age < CMD_STALE_SEC and abs(s.cmd_vel.get("vx", 0)) > 0.01)
    imu_ok = s.imu_age < IMU_STALE_SEC
    if mode in ("UWB_FOLLOW",) and follow_ok:
        react_status = "green"
        react_detail = f"UWB/cmd 活跃 · IMU {'ok' if imu_ok else 'stale'}"
    elif mode == "FACE_LOOK":
        react_status = "green"
        react_detail = "FACE_LOOK 模式"
    elif s.guide_state and s.guide_state_age < 3.0:
        react_status = "yellow"
        react_detail = f"guide: {s.guide_state}"
    elif follow_ok:
        react_status = "yellow"
        react_detail = "有运动输出，模式未确认"
    else:
        react_status = "red"
        react_detail = "无跟随/反应输出"

    return {
        "mic": {"status": mic_status, "detail": mic_detail, "open": live_audio, "rms": rms},
        "kws": {"status": kws_status, "detail": kws_detail, "last_hit": hit},
        "mode": {"status": mode_status, "detail": mode_detail, "value": mode},
        "react": {"status": react_status, "detail": react_detail},
        "guide_state": s.guide_state,
        "guide_state_age": s.guide_state_age,
    }


def _eval_layers(s: Snapshot) -> Dict[str, Any]:
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
        decision = {"status": "red", "detail": "无跟随决策输出"}

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
