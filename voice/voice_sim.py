#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
语音口令模拟：键盘输入 → /moon/voice_cmd（不控电机）。

  python3 /home/nvidia/moon/voice/voice_sim.py
  # 输入 1 / 2 / 0 或完整口令「小派看我」

  python3 /home/nvidia/moon/voice/voice_sim.py --once face_look
"""

from __future__ import annotations

import argparse
import os
import sys

import rospy
from std_msgs.msg import String

try:
    import yaml
except ImportError:
    yaml = None

TOPIC = "/moon/voice_cmd"
DEFAULT_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "keywords.yaml")

FALLBACK = {
    "小派看我": "face_look",
    "小派我们走": "uwb_follow",
    "小派停止": "stop",
    "小派停下": "stop",
    "1": "face_look",
    "2": "uwb_follow",
    "0": "stop",
}


def load_maps(path: str):
    phrase_to_cmd = dict(FALLBACK)
    shortcuts = {"1": "face_look", "2": "uwb_follow", "0": "stop", "q": "quit"}
    if yaml and os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        for item in data.get("commands") or []:
            phrase_to_cmd[str(item["phrase"]).strip()] = str(item["cmd"]).strip()
        for k, v in (data.get("shortcuts") or {}).items():
            shortcuts[str(k)] = str(v)
            phrase_to_cmd[str(k)] = str(v)
    return phrase_to_cmd, shortcuts


def resolve(text: str, phrase_to_cmd: dict) -> str:
    t = text.strip()
    if t in phrase_to_cmd:
        return phrase_to_cmd[t]
    low = t.lower()
    for k, v in phrase_to_cmd.items():
        if k.lower() == low:
            return v
    # 直接传 cmd
    if t in ("face_look", "uwb_follow", "stop", "idle", "look", "follow"):
        return t
    return ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--once", default="", help="发一条命令后退出")
    args = ap.parse_args()

    phrase_to_cmd, shortcuts = load_maps(args.yaml)
    rospy.init_node("moon_voice_sim", anonymous=True)
    pub = rospy.Publisher(TOPIC, String, queue_size=5)
    time_mod = __import__("time")
    time_mod.sleep(0.3)

    def publish(cmd: str):
        if not cmd or cmd == "quit":
            return False
        pub.publish(String(data=cmd))
        print(f"\033[92m[SIM]\033[0m → {TOPIC}  data={cmd!r}")
        return True

    if args.once:
        cmd = resolve(args.once, phrase_to_cmd)
        if not cmd:
            print(f"无法解析: {args.once!r}")
            sys.exit(1)
        publish(cmd)
        time_mod.sleep(0.2)
        return

    print()
    print("语音模拟 →", TOPIC)
    print("  1 = 小派看我 (face_look)")
    print("  2 = 小派我们走 (uwb_follow)")
    print("  0 = 小派停止 (stop)")
    print("  或直接输入口令 / cmd，q 退出")
    print()

    while not rospy.is_shutdown():
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        if line in shortcuts and shortcuts[line] == "quit":
            break
        if line.lower() in ("q", "quit", "exit"):
            break
        cmd = resolve(line, phrase_to_cmd)
        if not cmd:
            print(f"  未识别: {line!r}")
            continue
        publish(cmd)


if __name__ == "__main__":
    main()
