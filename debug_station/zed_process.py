#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZED 视觉节点进程管理（仅调试台使用）
====================================
启停 moon/vision/zed_obstacle_node.py，不改视觉功能代码本身。
"""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from typing import Any, Dict, List, Optional

MOON_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
ZED_SCRIPT = os.path.join(MOON_ROOT, "vision", "zed_obstacle_node.py")
SETUP_BASH = None
try:
    import sys
    from pathlib import Path

    _MOON = str(Path(__file__).resolve().parents[1])
    if _MOON not in sys.path:
        sys.path.insert(0, _MOON)
    from common.sim2real_env import resolve_setup_bash

    SETUP_BASH = str(resolve_setup_bash())
except Exception:
    SETUP_BASH = "/home/nvidia/sim2real/install/setup.bash"

LOG_DIR = os.path.join(MOON_ROOT, "logs")
ZED_OUT_LOG = os.path.join(LOG_DIR, "zed_obstacle_debug_station.log")


class ZedProcessManager:
    def __init__(self):
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._started_by_us = False
        self._last_error = ""

    def status(self) -> Dict[str, Any]:
        running = self._is_running()
        pids = self._find_pids()
        with self._lock:
            owned = self._started_by_us and self._proc is not None and self._proc.poll() is None
            err = self._last_error
        return {
            "running": running,
            "owned_by_station": owned,
            "pids": pids,
            "script": ZED_SCRIPT,
            "fpv_port": 8080,
            "last_error": err,
        }

    def start(self) -> Dict[str, Any]:
        with self._lock:
            if self._is_running():
                return {"ok": True, "msg": "ZED 节点已在运行", **self.status()}
            if not os.path.isfile(ZED_SCRIPT):
                self._last_error = f"script missing: {ZED_SCRIPT}"
                return {"ok": False, "msg": self._last_error, **self.status()}
            if not os.path.isfile(SETUP_BASH):
                self._last_error = f"setup.bash missing: {SETUP_BASH}"
                return {"ok": False, "msg": self._last_error, **self.status()}

            os.makedirs(LOG_DIR, exist_ok=True)
            cmd = (
                f"source {SETUP_BASH} && "
                f"exec python3 -u {ZED_SCRIPT}"
            )
            try:
                logf = open(ZED_OUT_LOG, "a", buffering=1)
                logf.write(f"\n===== start {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n")
                self._proc = subprocess.Popen(
                    ["bash", "-lc", cmd],
                    stdout=logf,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid,
                    cwd=MOON_ROOT,
                )
                self._started_by_us = True
                self._last_error = ""
            except Exception as e:
                self._last_error = str(e)
                return {"ok": False, "msg": f"启动失败: {e}", **self.status()}

        # 等一下看是否秒退
        time.sleep(1.2)
        if not self._is_running():
            self._last_error = f"进程已退出，见日志 {ZED_OUT_LOG}"
            return {"ok": False, "msg": self._last_error, **self.status()}
        return {"ok": True, "msg": "ZED Mini / zed_obstacle_node 已启动", **self.status()}

    def stop(self) -> Dict[str, Any]:
        """停止本台启动的进程；若有外部启动的同名节点也一并 rosnode/kill。"""
        msgs: List[str] = []
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
                    msgs.append(f"SIGTERM pgid of {self._proc.pid}")
                except Exception as e:
                    msgs.append(f"killpg: {e}")
                try:
                    self._proc.wait(timeout=5)
                except Exception:
                    try:
                        os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                        msgs.append("SIGKILL")
                    except Exception as e:
                        msgs.append(f"SIGKILL fail: {e}")
                self._proc = None
                self._started_by_us = False

        # 清理可能残留的同脚本进程
        for pid in self._find_pids():
            try:
                os.kill(pid, signal.SIGTERM)
                msgs.append(f"SIGTERM pid={pid}")
            except Exception as e:
                msgs.append(f"kill {pid}: {e}")

        # 尝试 rosnode kill（若 ROS 在）
        try:
            subprocess.run(
                ["bash", "-lc", f"source {SETUP_BASH} && rosnode kill /zed_obstacle 2>/dev/null"],
                timeout=5,
                capture_output=True,
            )
        except Exception:
            pass

        time.sleep(0.5)
        still = self._is_running()
        ok = not still
        msg = "ZED 已关闭" if ok else ("关闭未完成: " + "; ".join(msgs))
        if not ok:
            self._last_error = msg
        return {"ok": ok, "msg": msg, "detail": msgs, **self.status()}

    def _is_running(self) -> bool:
        if self._proc is not None and self._proc.poll() is None:
            return True
        return len(self._find_pids()) > 0

    @staticmethod
    def _find_pids() -> List[int]:
        pids: List[int] = []
        try:
            out = subprocess.check_output(["pgrep", "-f", "zed_obstacle_node.py"], text=True)
            for line in out.strip().splitlines():
                try:
                    pids.append(int(line.strip()))
                except ValueError:
                    pass
        except subprocess.CalledProcessError:
            pass
        except Exception:
            pass
        # 排除 debug_station 自己若误匹配（一般不会）
        me = os.getpid()
        return [p for p in pids if p != me]
