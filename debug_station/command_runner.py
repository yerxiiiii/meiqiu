#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试上位机命令执行：ROS 发布 + 白名单 shell。"""

from __future__ import annotations

import os
import re
import subprocess
import threading
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CMD_CFG = os.path.join(HERE, "commands.yaml")
SETUP_BASH = "/home/nvidia/sim2real/install/setup.bash"

try:
    from common.sim2real_env import resolve_setup_bash

    SETUP_BASH = str(resolve_setup_bash())
except Exception:
    pass


def _load_cfg() -> Dict[str, Any]:
    if not os.path.isfile(CMD_CFG):
        return {}
    with open(CMD_CFG, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _ros_master_up(timeout: float = 0.35) -> bool:
    import socket

    uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    host = "localhost"
    if "//" in uri:
        host = uri.split("//", 1)[1].split(":")[0] or "localhost"
    try:
        s = socket.create_connection((host, 11311), timeout=timeout)
        s.close()
        return True
    except Exception:
        return False


class CommandRunner:
    def __init__(self, max_history: int = 200):
        self._cfg = _load_cfg()
        self._lock = threading.Lock()
        self._history: Deque[str] = deque(maxlen=max_history)
        self._ros_pub = None
        self._ros_ok = False

    def _log(self, line: str) -> None:
        ts = time.strftime("%H:%M:%S")
        with self._lock:
            self._history.append(f"[{ts}] {line}")

    def history(self, n: int = 80) -> List[str]:
        with self._lock:
            return list(self._history)[-n:]

    def init_ros(self) -> bool:
        try:
            import rospy
            from std_msgs.msg import String
        except Exception as e:
            self._log(f"ROS 不可用: {e}")
            return False
        if not rospy.core.is_initialized():
            rospy.init_node("moon_debug_cmd", anonymous=True, disable_signals=True)
        topics = self._cfg.get("ros_topics") or {}
        self._ros_pub = {}
        for topic in topics:
            self._ros_pub[topic] = rospy.Publisher(topic, String, queue_size=3, latch=False)
        self._ros_ok = True
        self._log("命令终端 ROS publisher 就绪")
        return True

    def quick_buttons(self) -> List[Dict[str, Any]]:
        return self._cfg.get("quick_buttons") or []

    def run(self, raw: str) -> Dict[str, Any]:
        line = (raw or "").strip()
        if not line:
            return {"ok": False, "msg": "空命令"}
        self._log(f"> {line}")

        if line.startswith("voice "):
            cmd = line[6:].strip()
            return self._ros_publish("/moon/voice_cmd", cmd)
        if line.startswith("guide "):
            text = line[6:].strip()
            return self._ros_publish("/guide/voice_command", text)
        if line.startswith("ros "):
            topic, data = self._parse_ros_line(line[4:].strip())
            if topic:
                return self._ros_publish(topic, data)
            return {"ok": False, "msg": "用法: ros /topic payload 或 ros pub /topic std_msgs/String \"data: xxx\""}

        allowed, timeout = self._shell_allowed(line)
        if allowed:
            return self._run_shell(line, timeout, background=False)
        if line.startswith("bg "):
            inner = line[3:].strip()
            ok, _ = self._shell_allowed(inner)
            if ok:
                return self._run_shell(inner, 0, background=True)
        return {
            "ok": False,
            "msg": "未在白名单。可用: voice <cmd> | guide <text> | ros /topic data | 白名单 shell",
        }

    def run_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        typ = action.get("type", "")
        if typ == "ros":
            return self._ros_publish(action.get("topic", ""), action.get("data", ""))
        if typ == "snippet":
            return {"ok": True, "msg": "snippet", "snippet": action.get("data", "")}
        if typ == "shell":
            cmd = action.get("data") or action.get("cmd") or ""
            allowed, timeout = self._shell_allowed(cmd)
            if not allowed:
                return {"ok": False, "msg": f"shell 不在白名单: {cmd}"}
            return self._run_shell(cmd, timeout, background=False)
        if typ == "bg_shell":
            cmd = action.get("data") or action.get("cmd") or ""
            allowed, _ = self._shell_allowed(cmd)
            if not allowed:
                # bg 启动脚本：仅允许固定路径前缀
                if self._bg_shell_allowed(cmd):
                    return self._run_shell(cmd, 0, background=True)
                return {"ok": False, "msg": f"后台命令未授权: {cmd}"}
            return self._run_shell(cmd, 0, background=True)
        return {"ok": False, "msg": f"未知 action type: {typ}"}

    def _bg_shell_allowed(self, cmd: str) -> bool:
        prefixes = (
            "bash /home/nvidia/moon/fixed_route_voice_guide/scripts/start_voice_stack.sh",
            "python3 /home/nvidia/moon/voice/voice_sim.py",
            "python3 /home/nvidia/moon/brain/mode_arbiter.py",
        )
        return any(cmd.startswith(p) for p in prefixes)

    def _parse_ros_line(self, rest: str) -> Tuple[str, str]:
        # ros /moon/voice_cmd uwb_follow
        parts = rest.split(None, 2)
        if len(parts) >= 2 and parts[0].startswith("/"):
            return parts[0], parts[1] if len(parts) == 2 else parts[2]
        m = re.search(r'(/[\w/]+).*["\']?data:\s*([^"\']+)', rest)
        if m:
            return m.group(1), m.group(2).strip()
        return "", ""

    def _ros_publish(self, topic: str, data: str) -> Dict[str, Any]:
        if not topic or data is None:
            return {"ok": False, "msg": "topic/data 为空"}
        if not _ros_master_up():
            self._log("ROS master 不可达")
            return {"ok": False, "msg": "ROS master 不可达（请先启动 sim2real / roscore）"}
        allowed = (self._cfg.get("ros_topics") or {}).get(topic, [])
        if allowed and data not in allowed:
            # 仍允许 guide 自由文本（列表为常用项）
            if topic != "/guide/voice_command":
                return {"ok": False, "msg": f"载荷不在白名单: {data} (允许: {allowed})"}
        try:
            import rospy
            from std_msgs.msg import String

            if not rospy.core.is_initialized():
                rospy.init_node("moon_debug_cmd", anonymous=True, disable_signals=True)
            pub = self._ros_pub.get(topic) if self._ros_pub else None
            if pub is None:
                pub = rospy.Publisher(topic, String, queue_size=3)
            pub.publish(String(data=str(data)))
            rospy.sleep(0.05)
            self._log(f"ROS publish {topic} := {data!r}")
            return {"ok": True, "msg": f"已发布 {topic} = {data}"}
        except Exception as e:
            self._log(f"ROS 发布失败: {e}")
            return {"ok": False, "msg": str(e)}

    def _shell_allowed(self, cmd: str) -> Tuple[bool, float]:
        for item in self._cfg.get("shell_whitelist") or []:
            pat = item.get("pattern", "")
            if pat and re.match(pat, cmd.strip()):
                return True, float(item.get("timeout", 8))
        return False, 0.0

    def _run_shell(self, cmd: str, timeout: float, background: bool) -> Dict[str, Any]:
        full = f"source {SETUP_BASH} && {cmd}"
        if background:
            try:
                subprocess.Popen(
                    ["bash", "-lc", full],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
                self._log(f"后台启动: {cmd}")
                return {"ok": True, "msg": f"已在后台启动: {cmd}"}
            except Exception as e:
                self._log(f"后台启动失败: {e}")
                return {"ok": False, "msg": str(e)}

        try:
            out = subprocess.run(
                ["bash", "-lc", full],
                capture_output=True,
                text=True,
                timeout=max(1.0, timeout),
            )
            text = (out.stdout or "") + (out.stderr or "")
            text = text.strip() or "(无输出)"
            for ln in text.splitlines()[-30:]:
                self._log(ln)
            ok = out.returncode == 0
            return {"ok": ok, "msg": text, "code": out.returncode}
        except subprocess.TimeoutExpired:
            self._log(f"超时 ({timeout}s): {cmd}")
            return {"ok": False, "msg": f"命令超时 {timeout}s"}
        except Exception as e:
            self._log(f"shell 错误: {e}")
            return {"ok": False, "msg": str(e)}
