#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Moon 语音链路测试上位机
======================
  python3 /home/nvidia/moon/debug_station/server.py

默认 http://0.0.0.0:8090
- 命令终端（ROS 发布 + 白名单 shell）
- IMU /imu/data
- 语音链路状态（开麦/关麦、模式、跟随）
- 常用命令快捷框
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import threading
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, "static")
sys.path.insert(0, HERE)

from collectors import Collectors, LOG_DIR  # noqa: E402
from command_runner import CommandRunner  # noqa: E402
from zed_process import ZedProcessManager  # noqa: E402

FPV_UPSTREAM = "http://127.0.0.1:8080"


def _json(obj: Any) -> bytes:
    return json.dumps(obj, ensure_ascii=False, allow_nan=True).encode("utf-8")


class DebugServer:
    def __init__(
        self,
        host: str,
        port: int,
        collectors: Collectors,
        zed: ZedProcessManager,
        cmd_runner: CommandRunner,
    ):
        self.host = host
        self.port = port
        self.collectors = collectors
        self.zed = zed
        self.cmd_runner = cmd_runner
        self._httpd = None

    def serve_forever(self) -> None:
        coll = self.collectors
        zed = self.zed
        runner = self.cmd_runner

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                return

            def _send(self, code: int, body: bytes, ctype: str) -> None:
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(body)

            def _read_json(self) -> Dict:
                n = int(self.headers.get("Content-Length", 0) or 0)
                if n <= 0:
                    return {}
                try:
                    return json.loads(self.rfile.read(n).decode("utf-8"))
                except Exception:
                    return {}

            def do_OPTIONS(self):
                self.send_response(204)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.end_headers()

            def do_POST(self):
                parsed = urlparse(self.path)
                path = parsed.path
                if path == "/api/zed/start":
                    return self._send(200, _json(zed.start()), "application/json; charset=utf-8")
                if path == "/api/zed/stop":
                    return self._send(200, _json(zed.stop()), "application/json; charset=utf-8")
                if path == "/api/cmd":
                    body = self._read_json()
                    line = body.get("cmd") or body.get("line") or ""
                    return self._send(200, _json(runner.run(line)), "application/json; charset=utf-8")
                if path == "/api/cmd/action":
                    body = self._read_json()
                    return self._send(200, _json(runner.run_action(body)), "application/json; charset=utf-8")
                self.send_error(404)

            def do_GET(self):
                parsed = urlparse(self.path)
                path = parsed.path
                qs = parse_qs(parsed.query)

                if path in ("/", "/index.html"):
                    return self._file("index.html", "text/html; charset=utf-8")
                if path == "/app.js":
                    return self._file("app.js", "application/javascript; charset=utf-8")
                if path == "/style.css":
                    return self._file("style.css", "text/css; charset=utf-8")

                if path == "/api/zed/status":
                    return self._send(200, _json(zed.status()), "application/json; charset=utf-8")

                if path in ("/fpv/stream.mjpg", "/fpv/stream.mjpg/"):
                    return self._proxy_fpv_stream()
                if path in ("/fpv", "/fpv/"):
                    body = (
                        b"<!DOCTYPE html><html><body style='margin:0;background:#000'>"
                        b"<img src='/fpv/stream.mjpg' style='width:100%;max-height:100vh;object-fit:contain'/>"
                        b"</body></html>"
                    )
                    return self._send(200, body, "text/html; charset=utf-8")

                if path == "/api/snapshot":
                    snap = coll.snapshot()
                    host = (self.headers.get("Host") or "localhost").split(":")[0] or "localhost"
                    payload = {
                        "t": snap.t,
                        "ros_ok": snap.ros_ok,
                        "layers": snap.layers,
                        "voice_chain": snap.voice_chain,
                        "moon_mode": snap.moon_mode,
                        "moon_mode_age": snap.moon_mode_age,
                        "guide_state": snap.guide_state,
                        "guide_state_age": snap.guide_state_age,
                        "last_voice_cmd": snap.last_voice_cmd,
                        "last_guide_cmd": snap.last_guide_cmd,
                        "mic": snap.mic,
                        "processes": snap.processes,
                        "imu": snap.imu,
                        "imu_age": snap.imu_age,
                        "uwb": snap.uwb,
                        "uwb_age": snap.uwb_age,
                        "uwb_log_path": snap.uwb_log_path,
                        "obstacle": snap.obstacle,
                        "obstacle_age": snap.obstacle_age,
                        "decision": snap.decision,
                        "fsm_state": snap.fsm_state,
                        "fsm_age": snap.fsm_age,
                        "cmd_vel": snap.cmd_vel,
                        "cmd_age": snap.cmd_age,
                        "cmd_hz": snap.cmd_hz,
                        "joy_msg": snap.joy_msg,
                        "joy_age": snap.joy_age,
                        "events": snap.events,
                        "zed": zed.status(),
                        "quick_buttons": runner.quick_buttons(),
                        "links": {
                            "fpv": f"http://{host}:8080/stream.mjpg",
                            "mic_meter": f"http://{host}:8091/",
                        },
                    }
                    return self._send(200, _json(payload), "application/json; charset=utf-8")

                if path == "/api/terminal":
                    n = 100
                    try:
                        n = min(300, max(20, int(qs.get("n", ["100"])[0])))
                    except Exception:
                        pass
                    return self._send(
                        200,
                        _json({"lines": runner.history(n)}),
                        "application/json; charset=utf-8",
                    )

                if path == "/api/logs":
                    n = 120
                    try:
                        n = min(500, max(20, int(qs.get("n", ["120"])[0])))
                    except Exception:
                        pass
                    lines = _tail_latest_log(n)
                    return self._send(
                        200,
                        _json({"path": lines[0] if lines else "", "lines": lines[1:]}),
                        "application/json; charset=utf-8",
                    )

                if path == "/api/health":
                    snap = coll.snapshot()
                    return self._send(
                        200,
                        _json({
                            "ros_ok": snap.ros_ok,
                            "layers": snap.layers,
                            "voice_chain": snap.voice_chain,
                            "zed": zed.status(),
                        }),
                        "application/json; charset=utf-8",
                    )

                self.send_error(404)

            def _file(self, name: str, ctype: str) -> None:
                fp = os.path.join(STATIC, name)
                if not os.path.isfile(fp):
                    self.send_error(404)
                    return
                with open(fp, "rb") as f:
                    body = f.read()
                self._send(200, body, ctype)

            def _proxy_fpv_stream(self) -> None:
                url = FPV_UPSTREAM + "/stream.mjpg"
                try:
                    req = urllib.request.Request(url, method="GET")
                    upstream = urllib.request.urlopen(req, timeout=3)
                except Exception as e:
                    msg = (
                        f"FPV upstream unavailable ({url}): {e}. "
                        "先点「开启 ZED」或确认 zed_obstacle_node 在跑。"
                    ).encode("utf-8")
                    self._send(502, msg, "text/plain; charset=utf-8")
                    return

                try:
                    self.send_response(200)
                    ctype = upstream.headers.get(
                        "Content-Type",
                        "multipart/x-mixed-replace; boundary=frame",
                    )
                    self.send_header("Content-Type", ctype)
                    self.send_header("Cache-Control", "no-cache, private")
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    while True:
                        chunk = upstream.read(16 * 1024)
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                except Exception:
                    pass
                finally:
                    try:
                        upstream.close()
                    except Exception:
                        pass

        self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        print(f"[debug_station] http://{self.host}:{self.port}/")
        print(f"[debug_station] 语音链路测试 · 终端 /api/cmd · IMU /imu/data")
        print(f"[debug_station] FPV proxy: /fpv/stream.mjpg -> {FPV_UPSTREAM}/stream.mjpg")
        print(f"[debug_station] logs dir: {LOG_DIR}")
        self._httpd.serve_forever()


def _tail_latest_log(n: int):
    files = sorted(glob.glob(os.path.join(LOG_DIR, "uwb_follow_*.log")), key=os.path.getmtime, reverse=True)
    if not files:
        return ["", "(no uwb_follow_*.log yet)"]
    path = files[0]
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            lines = f.readlines()
        return [path] + [ln.rstrip("\n") for ln in lines[-n:]]
    except Exception as e:
        return [path, f"(read error: {e})"]


def main():
    ap = argparse.ArgumentParser(description="Moon voice-chain test station")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--no-ros", action="store_true", help="only tail logs (no rospy)")
    args = ap.parse_args()

    coll = Collectors()
    zed = ZedProcessManager()
    runner = CommandRunner()
    coll.start_log_tail()
    if not args.no_ros:
        ok = coll.start_ros()
        if ok:
            runner.init_ros()
        else:
            print("[debug_station] WARN: ROS not available, log-only mode")
    else:
        print("[debug_station] --no-ros: log tail only")

    srv = DebugServer(args.host, args.port, coll, zed, runner)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        coll.stop()
        print("\n[debug_station] bye")


if __name__ == "__main__":
    main()
