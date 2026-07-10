#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HighTorque Pi Plus UWB 自主跟随系统 - 最终版 (支持热插拔 + 开机自启)
====================================================================

系统架构：
  UWB 串口 (ttyUSB0) → 本脚本解析 → /joy_input (sensor_msgs/Joy) → humanoid_driver (IMU 补偿)
  → joy_teleop (格式转换) → sim2real_master_node (RL 步态策略 → 电机驱动)

三层控制状态：
  1. UWB 设备未接入 (USB 未插)  → 脚本待机轮询，手柄完全控制
  2. UWB 设备已接入但信号丢失   → 脚本停止发布，手柄自动接管
  3. UWB 设备已接入且信号正常   → 脚本 50Hz 发布跟随指令，手柄被覆盖

运行方式：
  手动运行：
    source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash
    python3 /home/nvidia/moon/uwb_follow.py

  开机自启 (systemd 服务)：
    sudo systemctl enable uwb-follow.service
    sudo systemctl start uwb-follow.service

手柄兼容性：
  - UWB 跟随激活时：手柄信号被覆盖
  - UWB 信号丢失 / USB 拔出 / 脚本退出：手柄立即恢复控制
"""

import rospy
from sensor_msgs.msg import Joy
import serial
import serial.tools.list_ports
import math
import time
import os
from dataclasses import dataclass
from typing import Optional, Tuple

# ============================================================
# 配置参数
# ============================================================

# UWB 串口
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.005
SERIAL_RETRY_INTERVAL = 2.0  # 串口未找到时的重试间隔 (秒)

# 跟随距离 (cm)
TARGET_DISTANCE = 70.0
DISTANCE_DEADZONE = 10.0

# 摇杆输出限制 (-1.0 ~ 1.0)
FORWARD_MIN = 0.15
FORWARD_MAX = 0.6
ROTATE_MIN = 0.30
ROTATE_MAX = 0.8

# 角度与距离控制比例
ANGLE_DEADZONE = 6.0   # 度
ANGLE_SCALE = 45.0     # 度/满量程
DISTANCE_SCALE = 100.0 # cm/满量程

# 信号超时
UWB_TIMEOUT = 1.2      # 秒

# 控制频率
CONTROL_RATE = 50      # Hz

# ============================================================
# 数据结构
# ============================================================

@dataclass
class UWBData:
    distance: float
    x: float
    y: float
    z: float
    angle: float
    is_anomaly: bool

# ============================================================
# UWB 串口解析器
# ============================================================

class UWBParser:
    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.buffer = ""

    def read_latest(self) -> Optional[UWBData]:
        try:
            n = self.ser.in_waiting
            if n <= 0:
                return None
            chunk = self.ser.read(n).decode('utf-8', errors='ignore')
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
            self.buffer = packet_part[nl_idx + 1:]
            return self._parse(packet)
        except (OSError, serial.SerialException):
            raise  # 串口断开，抛出让外层处理
        except Exception:
            return None

    @staticmethod
    def _parse(packet: str) -> Optional[UWBData]:
        parts = packet.split(',')
        if len(parts) < 9:
            return None
        try:
            dist = float(parts[5].strip())
            x = float(parts[6].strip())
            y = float(parts[7].strip())
            z = -float(parts[8].strip())
        except ValueError:
            return None

        is_anomaly = (abs(x) < 5.0) and (abs(z) < 5.0) and (abs(abs(y) - dist) < 2.0)
        angle = math.atan2(x, y) * 180.0 / math.pi
        return UWBData(distance=dist, x=x, y=y, z=z, angle=angle, is_anomaly=is_anomaly)

# ============================================================
# 跟随控制器
# ============================================================

class FollowController:
    @staticmethod
    def calculate(data: UWBData) -> Tuple[float, float, str]:
        # 前后
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

        # 旋转
        if abs(data.angle) < ANGLE_DEADZONE:
            rot = 0.0
            if state == "KEEPING":
                state = "ALIGNED"
        else:
            raw_r = data.angle / ANGLE_SCALE
            rot = max(-ROTATE_MAX, min(ROTATE_MAX, raw_r))
            if abs(rot) < ROTATE_MIN:
                rot = math.copysign(ROTATE_MIN, rot)

        # 异常保护
        if data.is_anomaly:
            rot = 0.0
            fwd = max(-0.1, min(0.1, fwd))
            state = "ANOMALY"

        return fwd, rot, state

# ============================================================
# 串口探测
# ============================================================
# 现场两台 UWB：671 / 672，其中 672 为主站（跟随用）
# 不要写死 ttyUSB0：本机上无数据的口常占 USB0，主站可能在 USB1

UWB_PROBE_SECONDS = 0.8


def _list_ttyusb_ports() -> list:
    """列出候选串口：优先 CP210x，再按设备名排序（USB1 先于 USB0）"""
    ports = []
    for p in serial.tools.list_ports.comports():
        if 'ttyUSB' not in p.device:
            continue
        is_cp210x = (p.vid == 0x10C4)
        ports.append((0 if is_cp210x else 1, p.device, p))
    # 设备号倒序：ttyUSB1 优先于 ttyUSB0（避免误连空口）
    ports.sort(key=lambda x: (x[0], -int(x[1].replace('/dev/ttyUSB', '') or 0)))
    return [dev for _, dev, _ in ports]


def _port_has_uwb_data(port: str) -> bool:
    """短时探测该口是否输出 ###1.9 协议"""
    ser = None
    try:
        ser = serial.Serial(port, SERIAL_BAUDRATE, timeout=0.1)
        time.sleep(UWB_PROBE_SECONDS)
        n = ser.in_waiting
        raw = ser.read(n) if n else b''
        text = raw.decode('utf-8', errors='ignore')
        return '###1.9' in text
    except Exception:
        return False
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


def find_uwb_port() -> Optional[str]:
    """探测 UWB 主站串口：优先有 ###1.9 数据的口，否则返回可打开的候选口"""
    candidates = _list_ttyusb_ports()
    if not candidates:
        return None

    for port in candidates:
        if _port_has_uwb_data(port):
            print(f"\033[92m[DETECT]\033[0m 找到 UWB 主站数据口: {port} (672)")
            return port

    # 有口但暂时无包：仍返回优先候选，交给后续超时/热插拔逻辑
    for port in candidates:
        ser = try_open_serial(port)
        if ser is not None:
            ser.close()
            print(f"\033[93m[DETECT]\033[0m 未读到 ###1.9，暂用候选口: {port}")
            return port
    return None


def try_open_serial(port: str) -> Optional[serial.Serial]:
    """尝试打开串口，失败返回 None"""
    try:
        return serial.Serial(port, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
    except Exception:
        return None

# ============================================================
# 主程序
# ============================================================

def main():
    rospy.init_node('uwb_follow', anonymous=True)
    joy_input_pub = rospy.Publisher('/joy_input', Joy, queue_size=10)
    rate = rospy.Rate(CONTROL_RATE)

    controller = FollowController()

    print()
    print("╔══════════════════════════════════════════════════╗")
    print("║       UWB 自主跟随系统 (热插拔版)                ║")
    print("║                                                  ║")
    print("║  • UWB 插入 → 自动进入跟随模式                   ║")
    print("║  • UWB 拔出 → 手柄自动接管控制                   ║")
    print("║  • 按 Ctrl+C 退出                                ║")
    print("╚══════════════════════════════════════════════════╝")
    print()

    while not rospy.is_shutdown():
        # ========== 外层循环：等待 UWB 设备接入 ==========
        port = find_uwb_port()

        if port is None:
            # 设备未接入，安静等待
            print(f"\033[90m[WAITING]\033[0m UWB 设备未检测到 (/dev/ttyUSB*)，手柄控制中... ({time.strftime('%H:%M:%S')})")
            time.sleep(SERIAL_RETRY_INTERVAL)
            continue

        ser = try_open_serial(port)
        if ser is None:
            print(f"\033[91m[ERROR]\033[0m 无法打开串口 {port}，{SERIAL_RETRY_INTERVAL}s 后重试...")
            time.sleep(SERIAL_RETRY_INTERVAL)
            continue

        print(f"\033[92m[CONNECTED]\033[0m UWB 串口已连接: {port}")
        print(f"\033[92m[CONNECTED]\033[0m 进入 UWB 跟随模式！手柄信号将被接管。")

        parser = UWBParser(ser)
        latest_data: Optional[UWBData] = None
        last_update_time = time.time()
        print_ticker = 0
        is_publishing = False

        try:
            # ========== 内层循环：正常 UWB 跟随控制 ==========
            while not rospy.is_shutdown():
                data = parser.read_latest()
                if data:
                    latest_data = data
                    last_update_time = time.time()

                elapsed = time.time() - last_update_time

                if elapsed > UWB_TIMEOUT:
                    # 信号丢失 → 停止发布，手柄接管
                    if is_publishing:
                        stop_msg = Joy()
                        stop_msg.header.stamp = rospy.Time.now()
                        stop_msg.axes = [0.0] * 8
                        stop_msg.buttons = [0] * 11
                        joy_input_pub.publish(stop_msg)
                        is_publishing = False

                    print_ticker += 1
                    if print_ticker >= 5:
                        print_ticker = 0
                        print(f"\033[90m[STANDBY]\033[0m UWB 信号丢失 ({elapsed:.1f}s) | 手柄已接管")

                elif latest_data:
                    # 信号正常 → 计算并发布
                    fwd, rot, state = controller.calculate(latest_data)

                    joy_msg = Joy()
                    joy_msg.header.stamp = rospy.Time.now()
                    joy_msg.axes = [0.0] * 8
                    joy_msg.buttons = [0] * 11
                    joy_msg.axes[1] = fwd
                    joy_msg.axes[3] = rot

                    joy_input_pub.publish(joy_msg)
                    is_publishing = True

                    print_ticker += 1
                    if print_ticker >= 5:
                        print_ticker = 0
                        prefix = "\033[93m[ANOMALY]\033[0m" if latest_data.is_anomaly else "\033[92m[FOLLOW ]\033[0m"
                        print(f"{prefix} Dist:{latest_data.distance:5.1f}cm Ang:{latest_data.angle:5.1f}° | {state:<8} | fwd:{fwd:5.2f} rot:{rot:5.2f}")

                rate.sleep()

        except (OSError, serial.SerialException):
            # ========== USB 被拔出 ==========
            print(f"\033[91m[DISCONNECTED]\033[0m UWB 串口断开 ({port})！手柄已恢复控制。")
            # 发送全零确保平滑过渡
            stop_msg = Joy()
            stop_msg.axes = [0.0] * 8
            stop_msg.buttons = [0] * 11
            for _ in range(5):
                stop_msg.header.stamp = rospy.Time.now()
                joy_input_pub.publish(stop_msg)
                time.sleep(0.02)

        except KeyboardInterrupt:
            raise  # 让外层捕获

        finally:
            try:
                ser.close()
            except Exception:
                pass

        # USB 拔出后短暂等待再重新探测
        time.sleep(SERIAL_RETRY_INTERVAL)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n✅ UWB 跟随系统已退出，手柄控制已恢复。")
    except rospy.ROSInterruptException:
        pass
