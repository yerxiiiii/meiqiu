#!/usr/bin/env python3
"""
麦克风 -> Vosk -> /guide/voice_command

持续监听麦克风，识别到语音后把原始文本发布到 voice_command_topic
（默认 /guide/voice_command），由 guide_demo_node.py 自己的
parse_voice_command() 做唤醒词/目的地匹配，这里不重复解析逻辑。

注: vosk-model-cn-0.22 不支持 runtime graph（语法约束），
    实测会打印 "Runtime graphs are not supported by this model"
    并静默忽略约束，因此这里用无约束识别。

用法:
  python3 voice_listener_node.py --duration 20
  python3 voice_listener_node.py --duration 20 --device 25
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import time

import numpy as np
import scipy.signal as sig
import sounddevice as sd
import yaml

try:
    import rospy
    from std_msgs.msg import String
except ImportError as exc:  # pragma: no cover
    print("需要 ROS1 rospy: source /opt/ros/noetic/setup.bash", flush=True)
    raise SystemExit(1) from exc

from vosk import KaldiRecognizer, Model

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_DEST = os.path.join(ROOT, "config", "destinations.yaml")
DEFAULT_SAFETY = os.path.join(ROOT, "config", "safety.yaml")
DEFAULT_MODEL = os.path.join(ROOT, "models", "vosk-model-current")

SAMPLE_RATE_IN = 44100
SAMPLE_RATE_OUT = 16000


def load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


class VoiceListener:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.dest_cfg = load_yaml(args.dest_config)
        self.safety = load_yaml(args.safety_config)

        self.model = Model(args.model)
        self.rec = KaldiRecognizer(self.model, SAMPLE_RATE_OUT)
        self.rec.SetWords(True)

        self.q: "queue.Queue[bytes]" = queue.Queue()

        rospy.init_node("voice_listener_node", anonymous=False)
        topic = self.safety.get("voice_command_topic", "/guide/voice_command")
        self.pub = rospy.Publisher(topic, String, queue_size=1)
        self.topic = topic
        print(f"发布话题: {topic}", flush=True)

    def _audio_callback(self, indata, frames, time_info, status) -> None:
        if status:
            print("audio status:", status, flush=True)
        self.q.put(bytes(indata))

    def _resample_chunk(self, raw: bytes) -> bytes:
        arr = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
        out = sig.resample_poly(arr, 160, 441)
        out_i = np.clip(out, -32768, 32767).astype(np.int16)
        return out_i.tobytes()

    def run(self, duration: float) -> None:
        end_time = time.time() + duration
        print(f"开始监听 {duration:.0f}s ...", flush=True)
        with sd.RawInputStream(
            samplerate=SAMPLE_RATE_IN,
            blocksize=4096,
            device=self.args.device,
            dtype="int16",
            channels=1,
            callback=self._audio_callback,
        ):
            while time.time() < end_time and not rospy.is_shutdown():
                try:
                    raw = self.q.get(timeout=0.5)
                except queue.Empty:
                    continue
                chunk16 = self._resample_chunk(raw)
                if self.rec.AcceptWaveform(chunk16):
                    result = json.loads(self.rec.Result())
                    text = (result.get("text") or "").replace(" ", "")
                    if text:
                        print(f"[识别] {text}", flush=True)
                        self.pub.publish(String(data=text))
                else:
                    partial = json.loads(self.rec.PartialResult())
                    ptext = (partial.get("partial") or "").replace(" ", "")
                    if ptext:
                        print(f"[中间] {ptext}", flush=True)

        final = json.loads(self.rec.FinalResult())
        ftext = (final.get("text") or "").replace(" ", "")
        if ftext:
            print(f"[识别-末尾] {ftext}", flush=True)
            self.pub.publish(String(data=ftext))
        print("监听结束", flush=True)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Mic -> Vosk -> /guide/voice_command")
    p.add_argument("--duration", type=float, default=20.0, help="监听时长(秒)")
    p.add_argument("--device", type=int, default=None, help="sounddevice 设备编号")
    p.add_argument("--model", default=DEFAULT_MODEL)
    p.add_argument("--dest-config", default=DEFAULT_DEST)
    p.add_argument("--safety-config", default=DEFAULT_SAFETY)
    return p


def main() -> None:
    args = build_parser().parse_args()
    VoiceListener(args).run(args.duration)


if __name__ == "__main__":
    main()
