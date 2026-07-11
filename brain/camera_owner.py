# -*- coding: utf-8 -*-
"""ZED 占用方启停：FACE_LOOK ↔ 障碍节点互斥。"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from typing import Optional

from modes import Mode

MOON_DIR = "/home/nvidia/moon"
ROS_SETUP = (
    "source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash"
)

FACE_OBS_SCRIPT = os.path.join(MOON_DIR, "vision", "face_obs_node.py")
ZED_OBS_SCRIPT = os.path.join(MOON_DIR, "vision", "zed_obstacle_node.py")


class CameraOwner:
    """按模式启停感知子进程（同一时刻只占一个 ZED 消费者）。"""

    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        self._proc: Optional[subprocess.Popen] = None
        self._owner: Optional[str] = None  # "face" | "obstacle" | None

    @property
    def owner(self) -> Optional[str]:
        return self._owner

    def _wanted(self, mode: Mode) -> Optional[str]:
        if mode == Mode.FACE_LOOK:
            return "face"
        if mode == Mode.UWB_FOLLOW:
            return "obstacle"
        return None

    def _alive(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _kill_current(self) -> None:
        if self._proc is None:
            self._owner = None
            return
        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
        except Exception:
            try:
                self._proc.send_signal(signal.SIGINT)
            except Exception:
                pass
        try:
            self._proc.wait(timeout=4.0)
        except Exception:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass
        self._proc = None
        self._owner = None
        time.sleep(0.6)

    def _start(self, script: str, owner: str) -> None:
        if not self.enabled:
            print(f"[CAM] disabled: pretend owner={owner}")
            self._owner = owner
            return
        if not os.path.isfile(script):
            print(f"\033[91m[CAM]\033[0m 脚本不存在: {script}")
            return
        cmd = f"{ROS_SETUP} && python3 -u {script}"
        self._proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        self._owner = owner
        print(f"\033[92m[CAM]\033[0m 已启动 {owner} pid={self._proc.pid}")

    def apply_mode(self, mode: Mode) -> None:
        want = self._wanted(mode)
        if want == self._owner and (not self.enabled or self._alive()):
            return
        if want is None and self._owner is None:
            return

        self._kill_current()
        if want == "face":
            self._start(FACE_OBS_SCRIPT, "face")
        elif want == "obstacle":
            self._start(ZED_OBS_SCRIPT, "obstacle")
        else:
            print("\033[90m[CAM]\033[0m IDLE：已释放 ZED")

    def shutdown(self) -> None:
        self._kill_current()
        print("\033[93m[CAM]\033[0m camera owner 已关闭")
