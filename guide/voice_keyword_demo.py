#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线关键词 ASR → /guide/voice_command

用法:
  # 无麦克风：文本注入
  python3 voice_keyword_demo.py --text "小派带我去炳胜餐厅"
  python3 voice_keyword_demo.py --text "小派停下"

  # 真实麦克风（需 vosk + 中文模型）
  pip3 install vosk sounddevice
  # 下载模型到 models/vosk-model-small-cn-0.22
  python3 voice_keyword_demo.py --model models/vosk-model-small-cn-0.22
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Optional

import rospy
import yaml
from std_msgs.msg import String

_GUIDE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_VOICE_CFG = os.path.join(_GUIDE_DIR, "config", "voice_keywords.yaml")
VOICE_CMD_TOPIC = "/guide/voice_command"
DEFAULT_MODEL = os.path.join(_GUIDE_DIR, "models", "vosk-model-small-cn-0.22")


def load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def match_command(text: str, cfg: dict) -> Optional[str]:
    t = (text or "").strip().replace(" ", "")
    if not t:
        return None
    for w in cfg.get("stop_words") or []:
        if w in t:
            # 停止建议带唤醒词，降低误触发；若含唤醒或单独「停下」也接受
            wakes = cfg.get("wake_words") or []
            if any(w0 in t for w0 in wakes) or w in ("停下", "停止"):
                return "stop"
    wake_ok = any(w in t for w in (cfg.get("wake_words") or []))
    if not wake_ok:
        return None
    for dest in cfg.get("destinations") or []:
        for phrase in dest.get("phrases") or []:
            if phrase in t:
                return f"go_to:{dest['id']}"
    return None


def publish_command(pub: rospy.Publisher, cmd: str, dry: bool) -> None:
    if dry:
        print(f"[DRY-RUN] would publish {VOICE_CMD_TOPIC}: {cmd}")
        return
    pub.publish(String(data=cmd))
    print(f"[PUB] {VOICE_CMD_TOPIC} ← {cmd}")


def run_text_mode(args: argparse.Namespace, cfg: dict) -> None:
    rospy.init_node("voice_keyword_demo", anonymous=True)
    pub = rospy.Publisher(VOICE_CMD_TOPIC, String, queue_size=10)
    time.sleep(0.3)
    cmd = match_command(args.text, cfg)
    print(f"[ASR] text={args.text!r} → cmd={cmd!r}")
    if cmd:
        publish_command(pub, cmd, args.dry_run)
    else:
        print("[ASR] 未匹配关键词（需唤醒词+目的地，或停止词）")
    # 给订阅者一点时间
    time.sleep(0.2)


def run_mic_mode(args: argparse.Namespace, cfg: dict) -> None:
    try:
        from vosk import Model, KaldiRecognizer
    except ImportError:
        print("[ERR] 未安装 vosk。请: pip3 install vosk sounddevice")
        print("      或使用 --text 做无麦联调")
        sys.exit(1)
    try:
        import sounddevice as sd
    except ImportError:
        print("[ERR] 未安装 sounddevice。请: pip3 install sounddevice")
        sys.exit(1)

    model_path = args.model
    if not os.path.isdir(model_path):
        print(f"[ERR] 模型目录不存在: {model_path}")
        print("下载示例:")
        print("  cd /home/nvidia/moon/guide/models")
        print("  wget https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip")
        print("  unzip vosk-model-small-cn-0.22.zip")
        sys.exit(1)

    rospy.init_node("voice_keyword_demo", anonymous=True)
    pub = rospy.Publisher(VOICE_CMD_TOPIC, String, queue_size=10)
    time.sleep(0.3)

    sample_rate = 16000
    model = Model(model_path)
    rec = KaldiRecognizer(model, sample_rate)
    rec.SetWords(True)

    print(f"[ASR] Vosk 已启动 model={model_path}")
    print("[ASR] 请说：小派带我去炳胜餐厅 / 小派停下  （Ctrl+C 退出）")

    last_cmd = ""
    last_t = 0.0
    debounce = 2.0

    def audio_cb(indata, frames, time_info, status):
        nonlocal last_cmd, last_t
        if status:
            print(f"[ASR] status={status}", file=sys.stderr)
        data = bytes(indata)
        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            text = (result.get("text") or "").strip()
            if not text:
                return
            print(f"[ASR] heard: {text}")
            cmd = match_command(text, cfg)
            if not cmd:
                return
            now = time.time()
            if cmd == last_cmd and (now - last_t) < debounce:
                return
            last_cmd, last_t = cmd, now
            publish_command(pub, cmd, args.dry_run)

    with sd.RawInputStream(
        samplerate=sample_rate,
        blocksize=8000,
        dtype="int16",
        channels=1,
        callback=audio_cb,
    ):
        rospy.spin()


def main() -> None:
    p = argparse.ArgumentParser(description="离线关键词 → /guide/voice_command")
    p.add_argument("--text", type=str, default="", help="跳过麦克风，直接匹配文本")
    p.add_argument("--dry-run", action="store_true", help="只打印不发布")
    p.add_argument("--model", type=str, default=DEFAULT_MODEL, help="Vosk 中文模型目录")
    p.add_argument("--voice-config", default=DEFAULT_VOICE_CFG)
    args = p.parse_args()

    cfg = load_yaml(args.voice_config)
    if args.text:
        run_text_mode(args, cfg)
    else:
        run_mic_mode(args, cfg)


if __name__ == "__main__":
    main()
