# -*- coding: utf-8 -*-
"""UWB 意图库：解析 + 跟随摇杆计算（不发布话题）。供 mode_arbiter 调用。"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import serial
import serial.tools.list_ports

# ----- 跟随参数（保守：日志显示摔倒主因是边冲边拧）-----
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.005
UWB_PROBE_SECONDS = 0.8

TARGET_DISTANCE = 70.0
DISTANCE_DEADZONE = 10.0
FORWARD_MIN = 0.10
FORWARD_MAX = 0.22
ROTATE_MIN = 0.10
ROTATE_MAX = 0.16
ANGLE_DEADZONE = 15.0
ANGLE_SCALE = 70.0
DISTANCE_SCALE = 130.0
ANGLE_EMA_ALPHA = 0.22
ANGLE_JUMP_REJECT_DEG = 30.0
ANGLE_HOLD_ON_JUMP = True
TURN_FORWARD_COUPLING = 0.88
# |ang| 大于此值时优先转向、前进再压一档（先转后走）
TURN_FIRST_ANGLE_DEG = 25.0
TURN_FIRST_FWD_SCALE = 0.35
SINGLE_PAIR = True
UWB_TIMEOUT = 1.2

CMD_VEL_X_SCALE = 1.5
CMD_VEL_YAW_SCALE = 1.57

# 软启动：yaw 比前进更慢爬升，避免起步拧腰
SOFT_YAW_POWER = 1.6  # soft**power 作用于 rot


@dataclass
class UWBData:
    distance: float
    x: float
    y: float
    z: float
    angle: float
    is_anomaly: bool


class UWBParser:
    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.buffer = ""

    def read_latest(self) -> Optional[UWBData]:
        try:
            n = self.ser.in_waiting
            if n <= 0:
                return None
            chunk = self.ser.read(n).decode("utf-8", errors="ignore")
            self.buffer += chunk
            if len(self.buffer) > 2000:
                self.buffer = self.buffer[-2000:]
            idx = self.buffer.rfind("###1.9")
            if idx == -1:
                return None
            packet_part = self.buffer[idx:]
            nl_idx = packet_part.find("\n")
            if nl_idx == -1:
                return None
            packet = packet_part[:nl_idx].strip()
            self.buffer = packet_part[nl_idx + 1 :]
            return self._parse(packet)
        except (OSError, serial.SerialException):
            raise
        except Exception:
            return None

    @staticmethod
    def _parse(packet: str) -> Optional[UWBData]:
        parts = packet.split(",")
        if len(parts) < 9:
            return None
        try:
            dist = float(parts[5].strip())
            x = float(parts[6].strip())
            y = float(parts[7].strip())
            z = -float(parts[8].strip())
        except ValueError:
            return None
        is_anomaly = (
            abs(x) < 5.0 and abs(z) < 5.0 and abs(abs(y) - dist) < 2.0
        )
        angle = math.atan2(x, y) * 180.0 / math.pi
        return UWBData(
            distance=dist, x=x, y=y, z=z, angle=angle, is_anomaly=is_anomaly
        )


class AngleFilter:
    def __init__(self):
        self._ang: Optional[float] = None
        self.last_rejected = False

    def update(self, raw_ang: float, is_anomaly: bool) -> float:
        self.last_rejected = False
        if is_anomaly:
            return 0.0 if self._ang is None else self._ang
        if self._ang is None:
            self._ang = raw_ang
            return self._ang
        if abs(raw_ang - self._ang) > ANGLE_JUMP_REJECT_DEG:
            self.last_rejected = True
            if ANGLE_HOLD_ON_JUMP:
                return self._ang
            raw_ang = self._ang + math.copysign(
                ANGLE_JUMP_REJECT_DEG * 0.3, raw_ang - self._ang
            )
        self._ang = (1.0 - ANGLE_EMA_ALPHA) * self._ang + ANGLE_EMA_ALPHA * raw_ang
        return self._ang


class FollowController:
    def __init__(self):
        self.angle_filter = AngleFilter()

    def calculate(self, data: UWBData) -> Tuple[float, float, str, float]:
        filt_ang = self.angle_filter.update(data.angle, data.is_anomaly)
        err = data.distance - TARGET_DISTANCE
        if abs(err) < DISTANCE_DEADZONE:
            fwd = 0.0
            state = "KEEPING"
        else:
            raw = err / DISTANCE_SCALE
            fwd = max(-FORWARD_MAX, min(FORWARD_MAX, raw))
            if abs(fwd) < FORWARD_MIN:
                fwd = math.copysign(FORWARD_MIN, fwd)
            state = "FOLLOW" if err > 0 else "BACK"

        use_angle = (not data.is_anomaly) and (not self.angle_filter.last_rejected)
        if (not use_angle) or abs(filt_ang) < ANGLE_DEADZONE:
            rot = 0.0
            if data.is_anomaly and SINGLE_PAIR:
                state = "DIST_ONLY"
            elif self.angle_filter.last_rejected:
                state = "ANG_HOLD"
            elif state == "KEEPING":
                state = "ALIGNED"
        else:
            raw_r = filt_ang / ANGLE_SCALE
            rot = max(-ROTATE_MAX, min(ROTATE_MAX, raw_r))
            if abs(rot) < ROTATE_MIN:
                rot = math.copysign(ROTATE_MIN, rot)

        if data.is_anomaly and not SINGLE_PAIR:
            rot = 0.0
            fwd = max(-0.1, min(0.1, fwd))
            state = "ANOMALY"

        if abs(rot) > 1e-3 and ROTATE_MAX > 1e-6:
            scale = 1.0 - TURN_FORWARD_COUPLING * (abs(rot) / ROTATE_MAX)
            fwd *= max(0.10, scale)

        # 大偏角：先转后走
        if abs(filt_ang) >= TURN_FIRST_ANGLE_DEG and abs(rot) > 1e-3:
            fwd *= TURN_FIRST_FWD_SCALE
            if state in ("FOLLOW", "BACK", "KEEPING", "ALIGNED"):
                state = "TURN_FIRST"

        return fwd, rot, state, filt_ang


def apply_soft_scales(fwd: float, rot: float, soft: float) -> Tuple[float, float]:
    """软启动：前进线性，转向用更高次幂更慢爬升。"""
    s = max(0.0, min(1.0, soft))
    return fwd * s, rot * (s ** SOFT_YAW_POWER)


def _list_ttyusb_ports() -> list:
    ports = []
    for p in serial.tools.list_ports.comports():
        if "ttyUSB" not in p.device:
            continue
        is_cp210x = p.vid == 0x10C4
        ports.append((0 if is_cp210x else 1, p.device, p))
    ports.sort(
        key=lambda x: (x[0], -int(x[1].replace("/dev/ttyUSB", "") or 0))
    )
    return [dev for _, dev, _ in ports]


def _port_has_uwb_data(port: str) -> bool:
    ser = None
    try:
        ser = serial.Serial(port, SERIAL_BAUDRATE, timeout=0.1)
        time.sleep(UWB_PROBE_SECONDS)
        n = ser.in_waiting
        raw = ser.read(n) if n else b""
        return "###1.9" in raw.decode("utf-8", errors="ignore")
    except Exception:
        return False
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


def try_open_serial(port: str) -> Optional[serial.Serial]:
    try:
        return serial.Serial(port, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
    except Exception:
        return None


def find_uwb_port() -> Optional[str]:
    candidates = _list_ttyusb_ports()
    if not candidates:
        return None
    for port in candidates:
        if _port_has_uwb_data(port):
            return port
    for port in candidates:
        ser = try_open_serial(port)
        if ser is not None:
            ser.close()
            return port
    return None
