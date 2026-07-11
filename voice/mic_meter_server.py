#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""麦克风电平监视：HTTP 页实时显示 RMS（与 KWS 共用同一路回调数据）。"""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional

_lock = threading.Lock()
_state: Dict[str, Any] = {
    "t": 0.0,
    "rms": 0.0,
    "peak": 0.0,
    "device": None,
    "device_name": "",
    "listening": False,
    "last_hit": "",
    "hit_t": 0.0,
}


def update_levels(rms: float, peak: float = 0.0) -> None:
    with _lock:
        _state["t"] = time.time()
        _state["rms"] = float(rms)
        _state["peak"] = float(peak)
        _state["listening"] = True


def set_device(device: Any, name: str = "") -> None:
    with _lock:
        _state["device"] = device
        _state["device_name"] = name or str(device)


def set_hit(text: str) -> None:
    with _lock:
        _state["last_hit"] = text
        _state["hit_t"] = time.time()


def snapshot() -> Dict[str, Any]:
    with _lock:
        return dict(_state)


_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>Moon Mic Meter</title>
<style>
  :root {
    --bg: #12141a;
    --fg: #e8eaed;
    --muted: #8b919a;
    --bar: #3d8bfd;
    --ok: #3dd68c;
    --warn: #f5a524;
    --bad: #f31260;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; min-height: 100vh;
    font-family: "IBM Plex Sans", "Noto Sans SC", sans-serif;
    background: var(--bg); color: var(--fg);
    display: flex; flex-direction: column; align-items: stretch;
    padding: 28px 32px;
  }
  h1 { font-size: 1.35rem; font-weight: 600; margin: 0 0 4px; letter-spacing: -0.02em; }
  .sub { color: var(--muted); font-size: 0.9rem; margin-bottom: 28px; }
  .meter-wrap {
    flex: 1; display: flex; flex-direction: column; justify-content: center;
    gap: 18px; max-width: 720px;
  }
  .label-row { display: flex; justify-content: space-between; align-items: baseline; }
  .rms-num { font-variant-numeric: tabular-nums; font-size: 2.4rem; font-weight: 600; }
  .unit { color: var(--muted); font-size: 0.95rem; margin-left: 6px; }
  .track {
    height: 28px; background: #1e222b; border-radius: 4px; overflow: hidden;
    border: 1px solid #2a303c;
  }
  .fill {
    height: 100%; width: 0%; background: var(--bar);
    transition: width 80ms linear, background 120ms;
  }
  .kv { display: grid; grid-template-columns: 120px 1fr; gap: 8px 16px;
        color: var(--muted); font-size: 0.92rem; margin-top: 24px; }
  .kv b { color: var(--fg); font-weight: 500; }
  .dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%;
         margin-right: 8px; background: var(--bad); vertical-align: middle; }
  .dot.on { background: var(--ok); }
  .hint { margin-top: auto; padding-top: 32px; color: var(--muted); font-size: 0.85rem; }
</style>
</head>
<body>
  <h1>麦克风电平</h1>
  <p class="sub"><span class="dot" id="dot"></span><span id="status">connecting…</span></p>
  <div class="meter-wrap">
    <div class="label-row">
      <div><span class="rms-num" id="rms">0.0000</span><span class="unit">RMS</span></div>
      <div style="color:var(--muted)">peak <b id="peak" style="color:var(--fg)">0.0000</b></div>
    </div>
    <div class="track"><div class="fill" id="fill"></div></div>
    <div class="kv">
      <span>设备</span><b id="dev">—</b>
      <span>最近命中</span><b id="hit">—</b>
      <span>刷新</span><b>10 Hz · 同源 KWS 回调</b>
    </div>
  </div>
  <p class="hint">对着麦说话：绿色条应跳动。长期接近 0 = 没收音。口令命中会出现在「最近命中」。</p>
<script>
async function tick() {
  try {
    const r = await fetch('/api/rms?t=' + Date.now(), { cache: 'no-store' });
    const s = await r.json();
    const rms = Number(s.rms) || 0;
    const peak = Number(s.peak) || 0;
    document.getElementById('rms').textContent = rms.toFixed(4);
    document.getElementById('peak').textContent = peak.toFixed(4);
    // 视觉：rms 0.02 左右已较响；映射到条宽
    const pct = Math.min(100, rms * 500);
    const fill = document.getElementById('fill');
    fill.style.width = pct + '%';
    fill.style.background = rms < 0.001 ? 'var(--bad)' : (rms < 0.01 ? 'var(--warn)' : 'var(--ok)');
    const age = s.t ? (Date.now()/1000 - s.t) : 999;
    const live = !!s.listening && age < 1.5;
    document.getElementById('dot').className = 'dot' + (live ? ' on' : '');
    document.getElementById('status').textContent = live
      ? '收音中 · age ' + age.toFixed(2) + 's'
      : (s.listening ? '电平停滞 / 无回调' : 'KWS 未开麦');
    document.getElementById('dev').textContent =
      (s.device_name || s.device || '—') + '';
    const hitAge = s.hit_t ? (Date.now()/1000 - s.hit_t) : 999;
    document.getElementById('hit').textContent =
      s.last_hit ? (s.last_hit + ' · ' + hitAge.toFixed(1) + 's 前') : '—';
  } catch (e) {
    document.getElementById('status').textContent = 'API 断开: ' + e;
    document.getElementById('dot').className = 'dot';
  }
}
tick();
setInterval(tick, 100);
</script>
</body>
</html>
"""


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args) -> None:  # noqa: A003
        return

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            body = _HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/api/rms":
            body = json.dumps(snapshot()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)


_httpd: Optional[ThreadingHTTPServer] = None


def start_meter_server(host: str = "0.0.0.0", port: int = 8091) -> None:
    global _httpd
    if _httpd is not None:
        return

    def _run() -> None:
        global _httpd
        try:
            httpd = ThreadingHTTPServer((host, port), _Handler)
            _httpd = httpd
            print(f"\033[92m[MIC]\033[0m meter http://{host}:{port}/")
            httpd.serve_forever()
        except OSError as e:
            print(f"\033[93m[MIC]\033[0m meter 未启动 ({e}) — 端口 {port} 可能被占用")

    t = threading.Thread(target=_run, name="mic-meter", daemon=True)
    t.start()
    time.sleep(0.15)
