#!/usr/bin/env python3
"""
离线关键词 / 文本命令 demo

当前支持:
  --text  直接匹配关键词并发布 /guide/voice_command
  后续接入 Vosk 麦克风流

用法:
  python3 voice_keyword_demo.py --text "小派带我去炳胜餐厅"
  python3 voice_keyword_demo.py --text "小派停下"
  python3 voice_keyword_demo.py --dry-run --text "小派带我去炳胜餐厅"
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Any, Dict, Optional

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_DEST = os.path.join(ROOT, "config", "destinations.yaml")


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def match_destination(text: str, destinations: Dict[str, Any]) -> Optional[str]:
    for dest_id, cfg in destinations.items():
        for kw in cfg.get("keywords", []) + [cfg.get("name", "")]:
            if kw and kw in text:
                return dest_id
    return None


def parse_command(text: str, cfg: Dict[str, Any]) -> Optional[str]:
    text = (text or "").strip()
    if not text:
        return None

    keywords = cfg.get("keywords", {})
    stop_words = keywords.get("stop", ["停下", "停止", "停车"])
    wake_words = keywords.get("wake", ["小派"])

    for w in stop_words:
        if w in text:
            # 有唤醒词或单独停止指令都接受
            return "stop"

    dest_id = match_destination(text, cfg.get("destinations", {}))
    has_wake = any(w in text for w in wake_words)
    go_to_prefix = keywords.get("go_to_prefix", ["带我去", "去", "带路"])
    if dest_id and (has_wake or any(p in text for p in go_to_prefix)):
        return f"go_to:{dest_id}"
    return None


def publish_command(cmd: str, dry_run: bool, topic: str) -> None:
    if dry_run:
        print(f"[DRY-RUN] would publish {topic}: {cmd}", flush=True)
        return
    try:
        import rospy
        from std_msgs.msg import String
    except ImportError as exc:
        print("需要 ROS1 rospy", file=sys.stderr)
        raise SystemExit(1) from exc

    rospy.init_node("voice_keyword_demo", anonymous=True)
    pub = rospy.Publisher(topic, String, queue_size=1)
    # 等待订阅者
    import time

    for _ in range(20):
        if pub.get_num_connections() > 0:
            break
        time.sleep(0.05)
    pub.publish(String(data=cmd))
    print(f"published {topic}: {cmd}", flush=True)
    time.sleep(0.2)


def main() -> None:
    parser = argparse.ArgumentParser(description="Voice keyword demo")
    parser.add_argument("--text", required=True, help="输入文本（模拟 ASR 结果）")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--dest-config", default=DEFAULT_DEST)
    parser.add_argument("--topic", default="/guide/voice_command")
    args = parser.parse_args()

    cfg = load_yaml(args.dest_config)
    cmd = parse_command(args.text, cfg)
    print(f"text={args.text!r} -> command={cmd!r}", flush=True)
    if not cmd:
        print("no keyword matched", flush=True)
        raise SystemExit(2)
    publish_command(cmd, args.dry_run, args.topic)


if __name__ == "__main__":
    main()
