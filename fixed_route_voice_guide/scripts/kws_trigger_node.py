#!/usr/bin/env python3
"""
Sherpa-ONNX keyword-spotting -> /guide/voice_command

Default arming: right-stick click (Joy.R / /joy_input buttons[10]).
Follow 期间 joy_teleop 会被停，/joy_msg.R 不再更新；开/关麦改读 /joy_input
（及 humanoid 转发的 /joy）上的物理按键。

On R (open): mic + guide_demo + mode_arbiter.
On R again (close): stop follow/route and restore amp policy.
"""

from __future__ import annotations

import argparse
import os
import queue
import subprocess
import sys
import threading
import time
from typing import Optional

import numpy as np
import rospy
import scipy.signal as sig
import sounddevice as sd
from std_msgs.msg import String

import sherpa_onnx

try:
    from sensor_msgs.msg import Joy as SensorJoy
except ImportError:
    SensorJoy = None  # type: ignore

try:
    from sim2real_msg.msg import Joy as SimJoy
except ImportError:
    SimJoy = None  # type: ignore

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MOON_ROOT = os.path.abspath(os.path.join(ROOT, ".."))
DEFAULT_MODEL_DIR = os.path.join(
    ROOT, "models", "sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01"
)
DEFAULT_KEYWORDS = os.path.join(ROOT, "config", "kws_keywords.txt")
GUIDE_SCRIPT = os.path.join(ROOT, "scripts", "guide_demo_node.py")
ARBITER_SCRIPT = os.path.join(MOON_ROOT, "brain", "mode_arbiter.py")

SAMPLE_RATE_IN = 44100
SAMPLE_RATE_OUT = 16000
DEFAULT_LISTEN_SEC = 30 * 60  # 30 minutes
BUTTON_PRESS_THRESH = 0.5
DEBOUNCE_SEC = 0.3
# sim2real_master/joy.yaml: button 10 -> Joy.R（右摇杆按下）
DEFAULT_MIC_BUTTON_INDEX = 10
DEFAULT_MIC_JOY_TOPICS = ("/joy_input", "/joy")


def is_button_pressed(v: float) -> bool:
    return v > BUTTON_PRESS_THRESH


def build_spotter(model_dir: str, keywords_file: str) -> "sherpa_onnx.KeywordSpotter":
    return sherpa_onnx.KeywordSpotter(
        tokens=os.path.join(model_dir, "tokens.txt"),
        encoder=os.path.join(model_dir, "encoder-epoch-12-avg-2-chunk-16-left-64.onnx"),
        decoder=os.path.join(model_dir, "decoder-epoch-12-avg-2-chunk-16-left-64.onnx"),
        joiner=os.path.join(model_dir, "joiner-epoch-12-avg-2-chunk-16-left-64.onnx"),
        keywords_file=keywords_file,
        num_threads=2,
        max_active_paths=8,
        keywords_score=2.0,
        keywords_threshold=0.12,
    )


class KwsTrigger:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.spotter = build_spotter(args.model_dir, args.keywords_file)
        self.stream = self.spotter.create_stream()
        self.q: "queue.Queue[bytes]" = queue.Queue()

        self._listen_until = 0.0
        self._listening = False
        self._stop_listen = False
        self._arm_event = threading.Event()
        self._mic_from_sensor = False
        self._mic_from_sim = False
        self._prev_mic_pressed = False
        self._last_edge_at = 0.0
        self._lock = threading.Lock()
        self._guide_proc: Optional[subprocess.Popen] = None
        self._arbiter_proc: Optional[subprocess.Popen] = None

        rospy.init_node("kws_trigger_node", anonymous=False)
        self.pub = rospy.Publisher(args.topic, String, queue_size=1)
        self.brain_pub = rospy.Publisher(
            args.brain_topic, String, queue_size=1, latch=False
        )
        rospy.on_shutdown(self._on_shutdown)

        if not args.no_wait_joy:
            if SensorJoy is None and SimJoy is None:
                raise SystemExit(
                    "需要 sensor_msgs/Joy 或 sim2real_msg (source setup.bash) "
                    "才能用摇杆 R 开麦；或加 --no-wait-joy"
                )
            if SensorJoy is not None:
                for topic in args.mic_joy_topics:
                    rospy.Subscriber(
                        topic, SensorJoy, self._on_sensor_joy, queue_size=5
                    )
            if SimJoy is not None:
                rospy.Subscriber("/joy_msg", SimJoy, self._on_sim_joy, queue_size=5)
            print(
                f"等待摇杆 R（/joy_input[{args.mic_button_index}]）："
                f"开麦+guide+arbiter / 再按 R 关麦停跟随"
                f"（最长 {args.duration / 60.0:.0f} min）",
                flush=True,
            )

    def _on_shutdown(self) -> None:
        self._stop_guide_demo()
        # arbiter 不随 KWS 退出而杀：跟随时可能仍要用

    def _rosnode_has(self, needle: str) -> bool:
        try:
            out = subprocess.check_output(
                ["rosnode", "list"],
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=2,
            )
            return any(needle in line for line in out.splitlines())
        except (subprocess.SubprocessError, FileNotFoundError):
            return False

    def _guide_alive(self) -> bool:
        if self._guide_proc is not None and self._guide_proc.poll() is None:
            return True
        return self._rosnode_has("guide_demo")

    def _arbiter_alive(self) -> bool:
        if self._arbiter_proc is not None and self._arbiter_proc.poll() is None:
            return True
        return self._rosnode_has("moon_mode_arbiter") or self._rosnode_has(
            "mode_arbiter"
        )

    def _ensure_guide_demo(self) -> None:
        """R 开麦时：保证 guide_demo 在跑，能接 /guide/voice_command。"""
        if self.args.no_guide:
            return
        if self._guide_alive():
            print("[GUIDE] guide_demo 已在运行", flush=True)
            return
        if not os.path.isfile(GUIDE_SCRIPT):
            print(f"[GUIDE] 找不到 {GUIDE_SCRIPT}", flush=True)
            return
        log_path = os.path.join(ROOT, "logs")
        os.makedirs(log_path, exist_ok=True)
        log_f = open(
            os.path.join(log_path, "guide_demo_from_kws.log"), "a", buffering=1
        )
        self._guide_proc = subprocess.Popen(
            [sys.executable, "-u", GUIDE_SCRIPT],
            cwd=ROOT,
            stdout=log_f,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        print(
            f"[GUIDE] 已随 R 启动 guide_demo (pid={self._guide_proc.pid})，"
            f"日志 logs/guide_demo_from_kws.log",
            flush=True,
        )

    def _ensure_mode_arbiter(self) -> None:
        """R 开麦时：保证 mode_arbiter 在跑，才能执行 uwb_follow。"""
        if self.args.no_arbiter:
            return
        if self._arbiter_alive():
            print("[ARBITER] mode_arbiter 已在运行", flush=True)
            return
        if not os.path.isfile(ARBITER_SCRIPT):
            print(f"[ARBITER] 找不到 {ARBITER_SCRIPT}", flush=True)
            return
        # 避免与独立 uwb_follow 双写 /cmd_vel
        try:
            subprocess.call(
                ["pkill", "-f", "uwb_follow.py"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3,
            )
        except (subprocess.SubprocessError, FileNotFoundError):
            pass
        log_path = os.path.join(ROOT, "logs")
        os.makedirs(log_path, exist_ok=True)
        log_f = open(
            os.path.join(log_path, "mode_arbiter_from_kws.log"), "a", buffering=1
        )
        self._arbiter_proc = subprocess.Popen(
            [sys.executable, "-u", ARBITER_SCRIPT, "--ignore-mutex"],
            cwd=os.path.dirname(ARBITER_SCRIPT),
            stdout=log_f,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=os.environ.copy(),
        )
        print(
            f"[ARBITER] 已随 R 启动 mode_arbiter (pid={self._arbiter_proc.pid})，"
            f"日志 logs/mode_arbiter_from_kws.log",
            flush=True,
        )

    def _ensure_voice_stack(self) -> None:
        """R 开麦：只拉起 arbiter + guide，不切策略（等口令再切）。"""
        self._ensure_mode_arbiter()
        time.sleep(0.3)
        self._ensure_guide_demo()

    def _stop_voice_motion(self) -> None:
        """关麦：停跟随/带路，并恢复默认 amp 策略。"""
        self.pub.publish(String(data="停下"))
        self.brain_pub.publish(String(data="stop"))
        print("[MIC] 关麦：停跟随/带路，恢复 amp", flush=True)
        rospy.sleep(0.2)

    def _stop_guide_demo(self) -> None:
        """R 关麦时：停掉由本节点拉起的 guide_demo。"""
        if self.args.no_guide:
            return
        proc = self._guide_proc
        self._guide_proc = None
        if proc is None or proc.poll() is not None:
            return
        print(f"[GUIDE] 随 R 关麦，停止 guide_demo (pid={proc.pid})", flush=True)
        try:
            proc.terminate()
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        except OSError:
            pass
        try:
            subprocess.call(
                ["rosnode", "kill", "/guide_demo_node"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3,
            )
        except (subprocess.SubprocessError, FileNotFoundError):
            pass

    def _sensor_mic_pressed(self, msg) -> bool:
        idx = self.args.mic_button_index
        if len(msg.buttons) <= idx:
            return False
        return int(msg.buttons[idx]) != 0

    def _mic_pressed_now(self) -> bool:
        return self._mic_from_sensor or self._mic_from_sim

    def _sync_mic_pressed(self) -> None:
        pressed = self._mic_pressed_now()
        rising = pressed and not self._prev_mic_pressed
        self._prev_mic_pressed = pressed
        if not rising:
            return
        now = time.time()
        if now - self._last_edge_at < DEBOUNCE_SEC:
            return
        self._last_edge_at = now
        self._on_mic_rising_edge(now)

    def _on_sensor_joy(self, msg) -> None:
        with self._lock:
            self._mic_from_sensor = self._sensor_mic_pressed(msg)
            self._sync_mic_pressed()

    def _on_sim_joy(self, msg) -> None:
        with self._lock:
            self._mic_from_sim = is_button_pressed(getattr(msg, "R", 0.0))
            self._sync_mic_pressed()

    def _on_mic_rising_edge(self, now: float) -> None:
        if self._listening:
            self._stop_listen = True
            self._listen_until = 0.0
            print("[MIC] 再次按下 R：关麦，回待机", flush=True)
            stop_guide = True
        else:
            self._stop_listen = False
            self._listen_until = now + self.args.duration
            self._listening = True
            stop_guide = False
            self._arm_event.set()
            print(
                f"[MIC] 摇杆 R 按下：开麦，最长 {self.args.duration / 60.0:.0f} min "
                f"（再按 R 可提前关麦）",
                flush=True,
            )

        if stop_guide:
            threading.Thread(target=self._close_mic_session, daemon=True).start()
        else:
            threading.Thread(target=self._ensure_voice_stack, daemon=True).start()

    def _close_mic_session(self) -> None:
        """R 关麦：先停运动，再停 guide_demo。"""
        self._stop_voice_motion()
        self._stop_guide_demo()

    def _audio_callback(self, indata, frames, time_info, status) -> None:
        if status:
            print("audio status:", status, flush=True)
        self.q.put(bytes(indata))

    def _resample_chunk(self, raw: bytes) -> np.ndarray:
        arr = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
        out = sig.resample_poly(arr, 160, 441)
        return (out / 32768.0).astype(np.float32)

    def _process_audio(self) -> Optional[float]:
        try:
            raw = self.q.get(timeout=0.5)
        except queue.Empty:
            return None
        samples = self._resample_chunk(raw)
        peak = float(np.max(np.abs(samples))) if samples.size else 0.0
        self.stream.accept_waveform(SAMPLE_RATE_OUT, samples)
        while self.spotter.is_ready(self.stream):
            self.spotter.decode_stream(self.stream)
        result = self.spotter.get_result(self.stream)
        if result:
            print(f"[KWS 命中] {result}", flush=True)
            self.pub.publish(String(data=result))
            self.spotter.reset_stream(self.stream)
        return peak

    def _listen_window(self, end_time: float) -> None:
        while not self.q.empty():
            try:
                self.q.get_nowait()
            except queue.Empty:
                break
        self.spotter.reset_stream(self.stream)

        print(
            f"KWS 监听中 ... 截止 {time.strftime('%H:%M:%S', time.localtime(end_time))} "
            f"(keywords: {self.args.keywords_file})",
            flush=True,
        )
        stopped_by_r = False
        last_level_log = 0.0
        peak_window = 0.0
        with sd.RawInputStream(
            samplerate=SAMPLE_RATE_IN,
            blocksize=4096,
            device=self.args.device,
            dtype="int16",
            channels=1,
            callback=self._audio_callback,
        ):
            while not rospy.is_shutdown():
                with self._lock:
                    until = self._listen_until
                    stop = self._stop_listen
                if stop or time.time() >= until:
                    stopped_by_r = stop
                    break
                peak = self._process_audio()
                if peak is not None:
                    peak_window = max(peak_window, peak)
                now = time.time()
                if now - last_level_log >= 2.0:
                    status = (
                        "有声"
                        if peak_window > 0.02
                        else ("微弱" if peak_window > 0.002 else "静音")
                    )
                    print(
                        f"[MIC level] peak={peak_window:.3f} {status}",
                        flush=True,
                    )
                    peak_window = 0.0
                    last_level_log = now
        reason = "再按 R 关麦" if stopped_by_r else "超时关麦"
        print(f"监听结束（{reason}）", flush=True)
        with self._lock:
            self._listening = False
            self._stop_listen = False
        self._arm_event.clear()
        if not stopped_by_r:
            self._close_mic_session()

    def run_once(self, duration: float) -> None:
        with self._lock:
            self._listen_until = time.time() + duration
            self._listening = True
            self._stop_listen = False
        self._listen_window(self._listen_until)

    def run_joy_armed(self) -> None:
        while not rospy.is_shutdown():
            print(
                f"待机：按摇杆 R = 开麦+guide_demo+mode_arbiter"
                f"（最长 {self.args.duration / 60.0:.0f} min，再按 R 关麦停跟随）",
                flush=True,
            )
            self._arm_event.clear()
            while not rospy.is_shutdown() and not self._arm_event.wait(timeout=0.5):
                pass
            if rospy.is_shutdown():
                break
            with self._lock:
                until = self._listen_until
            self._listen_window(until)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Sherpa-ONNX KWS -> /guide/voice_command")
    p.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_LISTEN_SEC,
        help="开麦后监听时长（秒），默认 1800=30min",
    )
    p.add_argument(
        "--no-wait-joy",
        action="store_true",
        help="不等摇杆，立刻开麦听 --duration 秒后退出",
    )
    p.add_argument(
        "--no-guide",
        action="store_true",
        help="R 开麦时不自动启动 guide_demo",
    )
    p.add_argument(
        "--no-arbiter",
        action="store_true",
        help="R 开麦时不自动启动 mode_arbiter",
    )
    p.add_argument("--device", type=int, default=None)
    p.add_argument("--model-dir", default=DEFAULT_MODEL_DIR)
    p.add_argument("--keywords-file", default=DEFAULT_KEYWORDS)
    p.add_argument("--topic", default="/guide/voice_command")
    p.add_argument("--brain-topic", default="/moon/voice_cmd")
    p.add_argument(
        "--mic-button-index",
        type=int,
        default=DEFAULT_MIC_BUTTON_INDEX,
        help="sensor_msgs/Joy buttons[] 下标，joy.yaml 里 R=10",
    )
    p.add_argument(
        "--mic-joy-topics",
        nargs="+",
        default=list(DEFAULT_MIC_JOY_TOPICS),
        help="物理手柄话题（跟随中仍有效）",
    )
    return p


def main() -> None:
    args = build_parser().parse_args()
    node = KwsTrigger(args)
    if args.no_wait_joy:
        node.run_once(args.duration)
    else:
        node.run_joy_armed()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
