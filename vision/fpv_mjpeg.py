#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轻量 MJPEG HTTP 第一视角流（给 SSH 远程浏览器看）。"""

from __future__ import annotations

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import cv2
import numpy as np


class MjpegStream:
    def __init__(self, host: str = "0.0.0.0", port: int = 8080, jpeg_quality: int = 70):
        self.host = host
        self.port = port
        self.jpeg_quality = jpeg_quality
        self._lock = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._server: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    def update_bgr(self, frame_bgr: np.ndarray) -> None:
        ok, buf = cv2.imencode(
            ".jpg",
            frame_bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            return
        with self._lock:
            self._jpeg = buf.tobytes()

    def _get_jpeg(self) -> Optional[bytes]:
        with self._lock:
            return self._jpeg

    def start(self) -> None:
        stream = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                return

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    body = (
                        b"<!DOCTYPE html><html><head><meta charset=utf-8>"
                        b"<title>Moon FPV</title>"
                        b"<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif}"
                        b"img{width:100%;max-width:960px;display:block;margin:0 auto}"
                        b"p{text-align:center;opacity:.7}</style></head><body>"
                        b"<p>ZED First Person View</p>"
                        b"<img src='/stream.mjpg' alt='fpv'/>"
                        b"</body></html>"
                    )
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path != "/stream.mjpg":
                    self.send_error(404)
                    return

                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.end_headers()
                try:
                    while True:
                        jpeg = stream._get_jpeg()
                        if jpeg is None:
                            threading.Event().wait(0.05)
                            continue
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        threading.Event().wait(0.03)
                except (BrokenPipeError, ConnectionResetError):
                    return

        self._server = ThreadingHTTPServer((self.host, self.port), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server = None
