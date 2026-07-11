#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线关键词 → /moon/voice_cmd（只发命令，不控电机）。

优先 Sherpa-ONNX KeywordSpotting；若未安装则提示并用 --text 回退。

安装:
  pip3 install --user sherpa-onnx sounddevice pypinyin sentencepiece

默认模型（已下载）:
  moon/voice/models/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01

启动:
  source .../install/setup.bash
  python3 /home/nvidia/moon/voice/kws_node.py
  # 或无麦冒烟:
  python3 /home/nvidia/moon/voice/kws_node.py --text "小派我们走"
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import time

import rospy
from std_msgs.msg import String

VOICE_DIR = os.path.dirname(os.path.abspath(__file__))
if VOICE_DIR not in sys.path:
    sys.path.insert(0, VOICE_DIR)
TOPIC = "/moon/voice_cmd"
DEFAULT_YAML = os.path.join(VOICE_DIR, "keywords.yaml")
DEFAULT_KEYWORDS = os.path.join(VOICE_DIR, "sherpa_keywords.txt")
DEFAULT_MODEL_DIR = os.path.join(
    VOICE_DIR,
    "models",
    "sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01",
)

PHRASE_DEFAULTS = {
    "小派看我": "face_look",
    "小派我们走": "uwb_follow",
    "小派跟我走": "uwb_follow",
    "跟我走": "uwb_follow",
    "小派停止": "stop",
    "小派停下": "stop",
}


def load_phrase_map(path: str) -> dict:
    m = dict(PHRASE_DEFAULTS)
    try:
        import yaml

        if os.path.isfile(path):
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            for item in data.get("commands") or []:
                m[str(item["phrase"]).strip()] = str(item["cmd"]).strip()
    except Exception:
        pass
    return m


def phrase_to_cmd(text: str, mapping: dict) -> str:
    t = text.strip()
    if t in mapping:
        return mapping[t]
    for k, v in mapping.items():
        if k in t:
            return v
    return ""


def publish_cmd(pub, cmd: str) -> None:
    pub.publish(String(data=cmd))
    print(f"\033[92m[KWS]\033[0m → {TOPIC}  {cmd}")


def run_text_once(pub, mapping, text: str) -> None:
    cmd = phrase_to_cmd(text, mapping)
    if not cmd:
        print(f"未映射口令: {text!r}")
        sys.exit(1)
    publish_cmd(pub, cmd)
    time.sleep(0.2)


def _pick(model_dir: str, patterns) -> str:
    for pat in patterns:
        hits = sorted(glob.glob(os.path.join(model_dir, pat)))
        if hits:
            # prefer int8 encoder/joiner when available
            int8 = [h for h in hits if ".int8." in os.path.basename(h)]
            return int8[0] if int8 else hits[0]
    return ""


def resolve_sherpa_paths(args) -> None:
    """Fill tokens/encoder/decoder/joiner from --sherpa-dir if missing."""
    model_dir = args.sherpa_dir or ""
    if not model_dir and not (args.tokens and args.encoder and args.decoder and args.joiner):
        if os.path.isdir(DEFAULT_MODEL_DIR):
            model_dir = DEFAULT_MODEL_DIR

    if model_dir:
        if not args.tokens:
            args.tokens = os.path.join(model_dir, "tokens.txt")
        if not args.encoder:
            args.encoder = _pick(
                model_dir,
                [
                    "encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx",
                    "encoder-epoch-12-avg-2-chunk-16-left-64.onnx",
                    "encoder*.int8.onnx",
                    "encoder*.onnx",
                ],
            )
        if not args.decoder:
            # decoder: fp32 preferred (int8 decoder less useful per sherpa docs)
            args.decoder = _pick(
                model_dir,
                [
                    "decoder-epoch-12-avg-2-chunk-16-left-64.onnx",
                    "decoder*.onnx",
                ],
            )
        if not args.joiner:
            args.joiner = _pick(
                model_dir,
                [
                    "joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx",
                    "joiner-epoch-12-avg-2-chunk-16-left-64.onnx",
                    "joiner*.int8.onnx",
                    "joiner*.onnx",
                ],
            )

    if not args.keywords_file:
        if os.path.isfile(DEFAULT_KEYWORDS):
            args.keywords_file = DEFAULT_KEYWORDS


def run_sherpa(pub, mapping, args) -> None:
    try:
        import sherpa_onnx
        import sounddevice as sd
    except ImportError as e:
        raise SystemExit(
            "缺少 sherpa-onnx 或 sounddevice。请 pip 安装，或用 --text / voice_sim.py\n"
            f"ImportError: {e}"
        )

    resolve_sherpa_paths(args)

    missing = [
        n
        for n, p in (
            ("tokens", args.tokens),
            ("encoder", args.encoder),
            ("decoder", args.decoder),
            ("joiner", args.joiner),
            ("keywords-file", args.keywords_file),
        )
        if not p or not os.path.isfile(p)
    ]
    if missing:
        raise SystemExit(
            "Sherpa KWS 缺少文件: "
            + ", ".join(missing)
            + "\n请放好模型到 moon/voice/models/… 或显式传 --tokens/--encoder/--decoder/--joiner\n"
            "无麦冒烟: python3 kws_node.py --text '小派我们走'"
        )

    kws = sherpa_onnx.KeywordSpotter(
        tokens=args.tokens,
        encoder=args.encoder,
        decoder=args.decoder,
        joiner=args.joiner,
        keywords_file=args.keywords_file,
        num_threads=args.num_threads,
        keywords_score=args.keywords_score,
        keywords_threshold=args.keywords_threshold,
    )
    print(f"\033[92m[KWS]\033[0m Sherpa 已加载")
    print(f"  encoder={args.encoder}")
    print(f"  keywords={args.keywords_file}")
    print(f"  sample_rate={args.sample_rate}")
    stream = kws.create_stream()
    cooldown = 0.0

    try:
        from mic_meter_server import (  # noqa: WPS433
            set_device,
            set_hit,
            start_meter_server,
            update_levels,
        )

        start_meter_server(port=args.meter_port)
    except Exception as e:
        print(f"\033[93m[MIC]\033[0m meter 不可用: {e}")
        set_device = set_hit = update_levels = None  # type: ignore

    def audio_cb(indata, frames, time_info, status):
        nonlocal cooldown
        if status:
            print(status, file=sys.stderr)
        samples = indata[:, 0].copy() if indata.ndim > 1 else indata.copy()
        if update_levels is not None:
            # float32 [-1,1]
            peak = float(abs(samples).max()) if samples.size else 0.0
            rms = float((samples.astype("float64") ** 2).mean() ** 0.5) if samples.size else 0.0
            update_levels(rms, peak)
        stream.accept_waveform(args.sample_rate, samples)
        while kws.is_ready(stream):
            kws.decode_stream(stream)
        result = kws.get_result(stream)
        if not result:
            return
        now = time.time()
        if now < cooldown:
            return
        text = result if isinstance(result, str) else str(result)
        print(f"\033[90m[KWS hit]\033[0m {text!r}")
        if set_hit is not None:
            set_hit(text)
        cmd = phrase_to_cmd(text, mapping)
        if cmd:
            publish_cmd(pub, cmd)
            cooldown = now + args.cooldown
            kws.reset_stream(stream)

    device = _resolve_input_device(sd, args.device)
    try:
        dinfo = sd.query_devices(device)
        dname = dinfo.get("name", str(device))
    except Exception:
        dname = str(device)
    print(f"\033[92m[KWS]\033[0m 录音设备: {device} ({dname})")
    if set_device is not None:
        set_device(device, dname)
    if update_levels is not None:
        print(f"\033[92m[MIC]\033[0m 电平页 http://<机器人IP>:{args.meter_port}/")

    # 上电早期声卡可能尚未就绪，短重试
    last_err = None
    for attempt in range(max(1, args.audio_retries)):
        try:
            with sd.InputStream(
                device=device,
                samplerate=args.sample_rate,
                channels=1,
                dtype="float32",
                blocksize=int(args.sample_rate * 0.1),
                callback=audio_cb,
            ):
                print("麦克风监听中… Ctrl+C 退出")
                while not rospy.is_shutdown():
                    time.sleep(0.2)
            return
        except Exception as e:
            last_err = e
            print(f"\033[93m[KWS]\033[0m 开麦失败 ({attempt + 1}/{args.audio_retries}): {e}")
            time.sleep(args.audio_retry_sec)
    raise SystemExit(f"无法打开录音设备: {last_err}")


def _resolve_input_device(sd, device_arg):
    """解析 --device；空则用默认输入，并确认有输入通道。"""
    devices = sd.query_devices()
    if not devices:
        raise SystemExit(
            "未检测到录音设备（sounddevice 设备列表为空）。\n"
            "请先接上 USB 麦 / 确认 ALSA，或用: python3 kws_node.py --text '小派我们走'"
        )

    if device_arg is None or device_arg == "":
        try:
            idx = sd.default.device[0]
        except Exception:
            idx = None
        if idx is None or idx < 0:
            # 找第一个有输入的设备
            for i, d in enumerate(devices):
                if d.get("max_input_channels", 0) > 0:
                    idx = i
                    break
        if idx is None:
            raise SystemExit("没有可用的录音输入设备")
        return idx

    # 数字索引或设备名子串
    try:
        return int(device_arg)
    except ValueError:
        pass
    key = str(device_arg).lower()
    for i, d in enumerate(devices):
        if key in str(d.get("name", "")).lower() and d.get("max_input_channels", 0) > 0:
            return i
    raise SystemExit(f"找不到录音设备: {device_arg!r}")


def main():
    ap = argparse.ArgumentParser(description="Moon KWS → /moon/voice_cmd")
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--text", default="", help="不启麦，发一条映射后的命令")
    ap.add_argument(
        "--sherpa-dir",
        default="",
        help=f"KWS 模型目录（默认 {DEFAULT_MODEL_DIR}）",
    )
    ap.add_argument("--tokens", default="")
    ap.add_argument("--encoder", default="")
    ap.add_argument("--decoder", default="")
    ap.add_argument("--joiner", default="")
    ap.add_argument("--keywords-file", default="")
    ap.add_argument("--sample-rate", type=int, default=16000)
    ap.add_argument("--num-threads", type=int, default=2)
    ap.add_argument("--cooldown", type=float, default=1.5)
    ap.add_argument("--keywords-score", type=float, default=1.0)
    ap.add_argument("--keywords-threshold", type=float, default=0.25)
    ap.add_argument(
        "--device",
        default="",
        help="录音设备索引或名称子串（默认系统输入设备）",
    )
    ap.add_argument("--audio-retries", type=int, default=30, help="开麦失败重试次数")
    ap.add_argument("--audio-retry-sec", type=float, default=2.0, help="开麦重试间隔秒")
    ap.add_argument("--meter-port", type=int, default=8091, help="麦克风 RMS 监视页端口")
    args = ap.parse_args()

    mapping = load_phrase_map(args.yaml)
    rospy.init_node("moon_kws", anonymous=False)
    pub = rospy.Publisher(TOPIC, String, queue_size=5)
    time.sleep(0.3)

    if args.text:
        run_text_once(pub, mapping, args.text)
        return

    run_sherpa(pub, mapping, args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nKWS 退出")
    except rospy.ROSInterruptException:
        pass
