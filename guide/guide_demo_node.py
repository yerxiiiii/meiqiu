#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
固定场地语音带路 — 状态机 + 路线执行器（唯一 /cmd_vel 出口）

用法:
  source .../sim2real_master-.../install/setup.bash
  python3 guide_demo_node.py --dry-run --text "小派带我去炳胜餐厅"
  sudo systemctl stop uwb-follow.service
  python3 guide_demo_node.py --text "小派带我去炳胜餐厅"
  python3 guide_demo_node.py --enter-running --enable-obstacle --enable-uwb
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from enum import Enum
from typing import Any, Dict, List, Optional

import rospy
import yaml
from std_msgs.msg import String

_GUIDE_DIR = os.path.dirname(os.path.abspath(__file__))
if _GUIDE_DIR not in sys.path:
    sys.path.insert(0, _GUIDE_DIR)

from common.motion_io import MotionIO  # noqa: E402
from common.safety import ObstacleMonitor  # noqa: E402
from common.uwb_distance import UWBDistanceMonitor  # noqa: E402

VOICE_CMD_TOPIC = "/guide/voice_command"
DEFAULT_DEST_CFG = os.path.join(_GUIDE_DIR, "config", "destinations.yaml")
DEFAULT_VOICE_CFG = os.path.join(_GUIDE_DIR, "config", "voice_keywords.yaml")
AUDIO_DIR = os.path.join(_GUIDE_DIR, "audio")


class State(Enum):
    IDLE = "IDLE"
    ACK = "ACK"
    LEAD_TO_DEST = "LEAD_TO_DEST"
    ARRIVED = "ARRIVED"
    PAUSED = "PAUSED"


def load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def match_text_to_command(text: str, voice_cfg: dict) -> Optional[str]:
    """文本 → go_to:<id> | stop | None。需唤醒词+目的地（stop 除外）。"""
    t = (text or "").strip().replace(" ", "")
    if not t:
        return None
    for w in voice_cfg.get("stop_words") or []:
        if w in t:
            return "stop"
    wake_ok = any(w in t for w in (voice_cfg.get("wake_words") or []))
    if not wake_ok:
        # 允许纯目的地短语在 --text 调试时也匹配（若含「带我去」）
        if "带我去" not in t and "去" not in t:
            return None
    for dest in voice_cfg.get("destinations") or []:
        for phrase in dest.get("phrases") or []:
            if phrase in t:
                return f"go_to:{dest['id']}"
    return None


def play_audio_or_log(text: str, audio_name: Optional[str] = None) -> None:
    path = None
    if audio_name:
        path = os.path.join(AUDIO_DIR, audio_name)
        if not os.path.isfile(path):
            path = None
    if path:
        try:
            subprocess.Popen(
                ["aplay", "-q", path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            print(f"[SPEAK] audio={audio_name} text={text}")
            return
        except Exception as e:
            print(f"[SPEAK] aplay failed: {e}")
    print(f"[SPEAK] {text}")


class RouteExecutor:
    """按时间片执行 segments；支持暂停（剩余时间续跑）。"""

    def __init__(self, segments: List[dict], max_vx: float, max_wz: float):
        self.segments = segments
        self.max_vx = max_vx
        self.max_wz = max_wz
        self.idx = 0
        self.seg_elapsed = 0.0
        self.done = False
        self._seg_started = False

    def reset(self) -> None:
        self.idx = 0
        self.seg_elapsed = 0.0
        self.done = False
        self._seg_started = False

    def current(self) -> Optional[dict]:
        if self.done or self.idx >= len(self.segments):
            return None
        return self.segments[self.idx]

    def remaining(self) -> float:
        seg = self.current()
        if not seg:
            return 0.0
        dur = float(seg.get("duration", 0.0) or 0.0)
        return max(0.0, dur - self.seg_elapsed)

    def tick(self, dt: float, motion: MotionIO, paused: bool) -> str:
        """
        推进一段。paused=True 时发零速且不累计时间。
        返回事件: running | segment_end | speak | arrived | idle
        """
        if self.done:
            motion.zero(note="route_done")
            return "arrived"

        seg = self.current()
        if seg is None:
            self.done = True
            motion.zero(note="no_segment")
            return "arrived"

        stype = seg.get("type", "wait")

        if not self._seg_started:
            self._seg_started = True
            self._on_segment_start(seg, motion)

        if stype == "speak":
            # speak 瞬时完成
            play_audio_or_log(seg.get("text", ""), seg.get("audio"))
            return self._advance(motion)

        if paused:
            motion.zero(note="paused")
            return "paused"

        dur = float(seg.get("duration", 0.0) or 0.0)
        vx, wz = 0.0, 0.0
        if stype == "forward":
            vx = float(seg.get("vx", 0.0))
            vx = max(-self.max_vx, min(self.max_vx, vx))
        elif stype == "turn":
            wz = float(seg.get("wz", 0.0))
            wz = max(-self.max_wz, min(self.max_wz, wz))
        elif stype == "wait":
            vx, wz = 0.0, 0.0
        else:
            print(f"[ROUTE] unknown type={stype}, skip")
            return self._advance(motion)

        motion.publish(vx, wz, note=f"seg[{self.idx}] {stype}")
        self.seg_elapsed += dt
        if self.seg_elapsed >= dur:
            motion.zero(note=f"seg[{self.idx}] end")
            return self._advance(motion)
        return "running"

    def _on_segment_start(self, seg: dict, motion: MotionIO) -> None:
        stype = seg.get("type")
        rem = float(seg.get("duration", 0) or 0)
        if self.seg_elapsed > 0:
            rem = max(0.0, rem - self.seg_elapsed)
        print(
            f"[ROUTE] start seg[{self.idx}] type={stype} "
            f"vx={seg.get('vx', '-')} wz={seg.get('wz', '-')} "
            f"duration={seg.get('duration', '-')} remaining≈{rem:.2f}s"
        )
        if stype in ("forward", "turn", "wait"):
            motion.zero(note="seg_boundary")

    def _advance(self, motion: MotionIO) -> str:
        motion.zero(note="advance")
        self.idx += 1
        self.seg_elapsed = 0.0
        self._seg_started = False
        if self.idx >= len(self.segments):
            self.done = True
            print("[ROUTE] all segments done")
            return "arrived"
        return "segment_end"


class GuideNode:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.dest_cfg = load_yaml(args.dest_config)
        self.voice_cfg = load_yaml(args.voice_config)
        self.defaults = self.dest_cfg.get("defaults") or {}
        self.max_vx = float(self.defaults.get("max_vx", 0.12))
        self.max_wz = float(self.defaults.get("max_wz", 0.25))
        self.hz = float(self.defaults.get("control_hz", 50))

        self.state = State.IDLE
        self.dest_id: Optional[str] = None
        self.executor: Optional[RouteExecutor] = None
        self._stop_requested = False
        self._pause_reason = ""

        self.motion = MotionIO(dry_run=args.dry_run)
        self.obstacle = ObstacleMonitor(
            enabled=args.enable_obstacle and not args.dry_run,
            required=args.obstacle_required,
        )
        self.uwb = UWBDistanceMonitor(enabled=args.enable_uwb and not args.dry_run)

    def start(self) -> None:
        rospy.init_node("guide_demo_node", anonymous=True)
        self.motion.setup()
        self.obstacle.start()
        self.uwb.start()
        rospy.Subscriber(VOICE_CMD_TOPIC, String, self._on_voice, queue_size=10)
        rospy.on_shutdown(self._on_shutdown)
        signal.signal(signal.SIGINT, self._sigint)

        if self.args.enter_running and not self.args.dry_run:
            self.motion.enter_running()

        if self.args.text:
            cmd = match_text_to_command(self.args.text, self.voice_cfg)
            print(f"[CMD] --text → {cmd!r}")
            if cmd:
                self._handle_command(cmd)
            else:
                print("[CMD] 未匹配到有效命令")

        print(f"[STATE] {self.state.value} | dry_run={self.args.dry_run}")
        print(f"[HINT] 订阅 {VOICE_CMD_TOPIC}；Ctrl+C 急停")

        rate = rospy.Rate(self.hz)
        last = time.time()
        while not rospy.is_shutdown() and not self._stop_requested:
            now = time.time()
            dt = now - last
            last = now
            self._tick(dt)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

        self._cleanup()

    def _sigint(self, *_args) -> None:
        print("\n[ESTOP] Ctrl+C → 零速")
        self._stop_requested = True
        try:
            self.motion.zero(note="SIGINT")
        except Exception:
            pass

    def _on_shutdown(self) -> None:
        self._cleanup()

    def _cleanup(self) -> None:
        self.state = State.IDLE
        self.executor = None
        try:
            self.uwb.close()
        except Exception:
            pass
        self.motion.shutdown()
        print("[EXIT] guide_demo_node 已退出")

    def _on_voice(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        print(f"[VOICE] recv {raw!r}")
        # 允许直接传 go_to:xxx / stop，或自然语言
        if raw.startswith("go_to:") or raw == "stop":
            self._handle_command(raw)
            return
        cmd = match_text_to_command(raw, self.voice_cfg)
        if cmd:
            self._handle_command(cmd)

    def _handle_command(self, cmd: str) -> None:
        if cmd == "stop":
            print("[CMD] stop")
            play_audio_or_log("好的，已停止", "stop.wav")
            self._abort_to_idle()
            return
        if cmd.startswith("go_to:"):
            dest_id = cmd.split(":", 1)[1].strip()
            self._start_route(dest_id)

    def _abort_to_idle(self) -> None:
        self.motion.zero(note="abort")
        self.executor = None
        self.dest_id = None
        self.state = State.IDLE
        print(f"[STATE] → {self.state.value}")

    def _start_route(self, dest_id: str) -> None:
        dests = (self.dest_cfg.get("destinations") or {})
        dest = dests.get(dest_id)
        if not dest:
            print(f"[ERR] 未知目的地 id={dest_id}")
            return
        segments = dest.get("segments") or []
        if not segments:
            print(f"[ERR] 目的地无 segments: {dest_id}")
            return
        self.dest_id = dest_id
        self.executor = RouteExecutor(segments, self.max_vx, self.max_wz)
        self.state = State.ACK
        name = dest.get("name", dest_id)
        print(f"[STATE] → ACK / LEAD_TO_DEST dest={name} ({dest_id}) segs={len(segments)}")
        # ACK 后立即进入带路
        self.state = State.LEAD_TO_DEST
        print(f"[STATE] → {self.state.value}")

    def _tick(self, dt: float) -> None:
        self.uwb.poll()

        if self.state not in (State.LEAD_TO_DEST, State.PAUSED):
            if self.state == State.IDLE:
                # 空闲保持零速心跳（非 dry-run 时低频）
                pass
            return

        if self.executor is None:
            self._abort_to_idle()
            return

        paused = False
        reasons = []
        if self.obstacle.should_pause():
            paused = True
            reasons.append(f"obstacle:{self.obstacle.status()[0]}")
        if self.uwb.should_wait():
            paused = True
            reasons.append(f"uwb:{self.uwb.distance_cm:.0f}cm")

        if paused:
            if self.state != State.PAUSED:
                self.state = State.PAUSED
                self._pause_reason = ",".join(reasons)
                print(f"[STATE] → PAUSED ({self._pause_reason})")
        else:
            if self.state == State.PAUSED:
                self.state = State.LEAD_TO_DEST
                print(f"[STATE] → LEAD_TO_DEST (resume remaining={self.executor.remaining():.2f}s)")

        # SLOW：缩减前进（仅在非完全 pause 时）
        event = self.executor.tick(dt, self.motion, paused=paused)
        if not paused:
            reason, cap = self.obstacle.status()
            if reason == "SLOW" and cap < 1.0 and self.executor.current():
                # 已在 publish 后；下一帧会再发。这里对当前帧补一次限速重发
                seg = self.executor.current()
                if seg and seg.get("type") == "forward":
                    vx = float(seg.get("vx", 0.0)) * cap
                    self.motion.publish(vx, 0.0, note=f"slow cap={cap:.2f}")

        if event == "arrived":
            self.state = State.ARRIVED
            print(f"[STATE] → {self.state.value}")
            play_audio_or_log("带路结束", None)
            self.motion.zero(note="arrived")
            time.sleep(0.3)
            self._abort_to_idle()
            if self.args.once:
                self._stop_requested = True


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Mini Pi Plus 固定场地语音带路")
    p.add_argument("--dry-run", action="store_true", help="只打印路线状态，不发 /cmd_vel")
    p.add_argument("--text", type=str, default="", help="直接注入文本命令（跳过麦克风）")
    p.add_argument("--once", action="store_true", help="跑完一条路线后退出")
    p.add_argument("--enter-running", action="store_true", help="启动时发 LT+RT+LB 进 RUNNING")
    p.add_argument("--enable-obstacle", action="store_true", help="订 /moon/obstacle 停障")
    p.add_argument("--obstacle-required", action="store_true", help="无视觉则禁止前进")
    p.add_argument("--enable-uwb", action="store_true", help="UWB 掉队等待")
    p.add_argument("--dest-config", default=DEFAULT_DEST_CFG)
    p.add_argument("--voice-config", default=DEFAULT_VOICE_CFG)
    return p


def main() -> None:
    args = build_argparser().parse_args()
    # dry-run + text 默认 once，方便冒烟
    if args.dry_run and args.text and not any(
        a == "--once" or a.startswith("--once=") for a in sys.argv
    ):
        args.once = True
    node = GuideNode(args)
    node.start()


if __name__ == "__main__":
    main()
