#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UWB 仅读距离：用户掉队时暂停路线。复用 uwb_follow 的串口探测/解析逻辑（精简版）。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None  # type: ignore

SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.005
UWB_PROBE_SECONDS = 0.8
UWB_TIMEOUT = 1.2

# 掉队阈值（cm，与 uwb_follow 一致用厘米）
WAIT_DIST_CM = 180.0   # 超过则等待
OK_DIST_CM = 120.0     # 回到此距离内继续


@dataclass
class UWBSample:
    distance_cm: float
    stamp: float


class UWBDistanceMonitor:
    """可选：无串口/无 pyserial 时 disabled，不阻塞带路。"""

    def __init__(
        self,
        enabled: bool = False,
        wait_cm: float = WAIT_DIST_CM,
        ok_cm: float = OK_DIST_CM,
    ):
        self.enabled = enabled and serial is not None
        self.wait_cm = wait_cm
        self.ok_cm = ok_cm
        self._ser = None
        self._buffer = ""
        self._latest: Optional[UWBSample] = None
        self._waiting = False
        self._port: Optional[str] = None

    def start(self) -> None:
        if not self.enabled:
            print("\033[90m[UWB]\033[0m 掉队监测关闭")
            return
        port = self._find_port()
        if not port:
            print("\033[93m[UWB]\033[0m 未找到串口，掉队监测降级为关闭")
            self.enabled = False
            return
        try:
            self._ser = serial.Serial(port, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
            self._port = port
            print(f"\033[92m[UWB]\033[0m 掉队监测已连接 {port} (wait>{self.wait_cm}cm ok<{self.ok_cm}cm)")
        except Exception as e:
            print(f"\033[93m[UWB]\033[0m 打开失败: {e}，监测关闭")
            self.enabled = False

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def poll(self) -> None:
        if not self.enabled or self._ser is None:
            return
        try:
            n = self._ser.in_waiting
            if n <= 0:
                return
            chunk = self._ser.read(n).decode("utf-8", errors="ignore")
            self._buffer += chunk
            if len(self._buffer) > 2000:
                self._buffer = self._buffer[-2000:]
            idx = self._buffer.rfind("###1.9")
            if idx == -1:
                return
            part = self._buffer[idx:]
            nl = part.find("\n")
            if nl == -1:
                return
            packet = part[:nl].strip()
            self._buffer = part[nl + 1 :]
            dist = self._parse_dist(packet)
            if dist is not None:
                self._latest = UWBSample(distance_cm=dist, stamp=time.time())
        except Exception:
            pass

    def should_wait(self) -> bool:
        """滞回：过远进入等待，回到 ok 才解除。"""
        if not self.enabled:
            return False
        if self._latest is None:
            return False
        if (time.time() - self._latest.stamp) > UWB_TIMEOUT:
            return False  # 信号丢失不阻塞带路（与跟随不同）
        d = self._latest.distance_cm
        if not math.isfinite(d) or d <= 0:
            return False
        if self._waiting:
            if d <= self.ok_cm:
                self._waiting = False
                print(f"\033[92m[UWB]\033[0m 用户跟上 dist={d:.0f}cm，继续路线")
            return self._waiting
        if d > self.wait_cm:
            self._waiting = True
            print(f"\033[93m[UWB]\033[0m 用户掉队 dist={d:.0f}cm，等待...")
            return True
        return False

    @property
    def distance_cm(self) -> float:
        if self._latest is None:
            return float("nan")
        return self._latest.distance_cm

    @staticmethod
    def _parse_dist(packet: str) -> Optional[float]:
        parts = packet.split(",")
        if len(parts) < 9:
            return None
        try:
            return float(parts[5].strip())
        except ValueError:
            return None

    def _find_port(self) -> Optional[str]:
        ports = []
        for p in serial.tools.list_ports.comports():
            if "ttyUSB" not in p.device:
                continue
            is_cp210x = p.vid == 0x10C4
            ports.append((0 if is_cp210x else 1, p.device))
        ports.sort(key=lambda x: (x[0], -int(x[1].replace("/dev/ttyUSB", "") or 0)))
        for _, port in ports:
            if self._port_has_data(port):
                return port
        return ports[0][1] if ports else None

    @staticmethod
    def _port_has_data(port: str) -> bool:
        ser = None
        try:
            ser = serial.Serial(port, SERIAL_BAUDRATE, timeout=0.1)
            time.sleep(UWB_PROBE_SECONDS)
            n = ser.in_waiting
            raw = ser.read(n) if n else b""
            return b"###1.9" in raw or "###1.9" in raw.decode("utf-8", errors="ignore")
        except Exception:
            return False
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
