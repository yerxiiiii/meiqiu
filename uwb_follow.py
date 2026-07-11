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
    source ~/sim2real/install/setup.bash   # or: export SIM2REAL_WS=...
    python3 /home/nvidia/moon/uwb_follow.py

  开机自启 (systemd 服务)：
    sudo systemctl enable uwb-follow.service
    sudo systemctl start uwb-follow.service

  语音切模式（推荐）：用中央决策，勿与本脚本同时发 /cmd_vel：
    python3 /home/nvidia/moon/brain/mode_arbiter.py
    # 「小派我们走」→ UWB_FOLLOW；意图库见 brain/uwb_intent.py
    sudo systemctl stop uwb-follow.service   # 避免双写

手柄兼容性：
  - UWB 跟随激活时：手柄信号被覆盖
  - UWB 信号丢失 / USB 拔出 / 脚本退出：手柄立即恢复控制
"""

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32
from sim2real_msg.msg import Joy as SimJoy
import serial
import serial.tools.list_ports
import math
import time
import os
import sys
import subprocess
import logging
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, Tuple

# moon repo root on sys.path for common.sim2real_env
_MOON = os.path.dirname(os.path.abspath(__file__))
if _MOON not in sys.path:
    sys.path.insert(0, _MOON)

# 视觉安全门控（与 ZED 节点解耦：只订 /moon/obstacle）
_VISION_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vision")
if _VISION_DIR not in sys.path:
    sys.path.insert(0, _VISION_DIR)
from obstacle_state import ObstacleState  # noqa: E402
from safety_gate import SIDESTEP_BLEND, apply_safety_gate  # noqa: E402
from person_mask import estimate_person_center_m  # noqa: E402

# ============================================================
# 配置参数
# ============================================================

# UWB 串口
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.005
SERIAL_RETRY_INTERVAL = 2.0  # 串口未找到时的重试间隔 (秒)

# 跟随距离 (cm) —— 与 brain/uwb_intent 对齐
TARGET_DISTANCE = 50.0
DISTANCE_DEADZONE = 12.0

# 摇杆输出限制 —— 与 brain/uwb_intent 对齐（日志摔倒主因：边冲边拧）
FORWARD_MIN = 0.08
FORWARD_MAX = 0.18
ROTATE_MIN = 0.10
ROTATE_MAX = 0.14

# 角度与距离控制比例
ANGLE_DEADZONE = 15.0
ANGLE_SCALE = 70.0
DISTANCE_SCALE = 120.0
DISTANCE_ERR_CAP_CM = 35.0

# 角度滤波 / 跳变抑制（日志里 ang 曾在 0.1s 内 -34→0→63）
ANGLE_EMA_ALPHA = 0.22
ANGLE_JUMP_REJECT_DEG = 30.0
ANGLE_HOLD_ON_JUMP = True

# 大转向时压低前进（避免边转边冲）
TURN_FORWARD_COUPLING = 0.88
TURN_FIRST_ANGLE_DEG = 25.0
TURN_FIRST_FWD_SCALE = 0.35
# False=只用距离前后（rot=0）
USE_ANGLE = False

# 信号超时
UWB_TIMEOUT = 1.2      # 秒

# 控制频率
CONTROL_RATE = 50      # Hz

# 进入跟随后自动发一次 LB 边沿，把策略从 STANDBY 切到 RUNNING
# （sim2real 仅在 RUNNING 时接收 cmd_vel，否则速度指令会被丢弃）
AUTO_ENTER_RUNNING = True

# 单对模式：现场只有 672(主站) + 671(标签) 一对
# 单对时常出现 x=z=0、y≈距离（无可靠 AOA），只能做前后跟随，不转
# 两对齐全、角度正常时可改 False，恢复“异常则限速”保护
SINGLE_PAIR = True

# ------------------------------------------------------------
# 视觉避障门控（感知在 vision/zed_obstacle_node.py，本脚本只消费）
# UWB 跟随时目标人常被 ZED 当成障碍 → gate=SLOW 把前进压没；
# 先测跟随请保持 False；避障联调时再改 True。
# ------------------------------------------------------------
ENABLE_OBSTACLE_GATE = False      # False：完全忽略视觉，只跟 UWB
OBSTACLE_TOPIC = "/moon/obstacle"
OBSTACLE_TIMEOUT = 0.8            # 秒：超时视为 stale
OBSTACLE_REQUIRED = False         # True：无视觉则禁止前进（fail-safe）
USE_OBSTACLE_SIDESTEP = True      # 使用感知给出的 rotate_bias（经 SIDESTEP_BLEND 软叠）

# 进 RUNNING 后速度软启动（秒），避免瞬间满速导致摔倒
SOFT_START_SEC = 5.0
SOFT_YAW_POWER = 1.6              # rot *= soft**power，转向更慢爬升

# fsm.h: 8 = PROTECTION_SHUTDOWN → 立即零速，禁止继续发跟随
FSM_PROTECTION_SHUTDOWN = 8
FSM_ERROR = 1

# 会话日志目录（每次启动一个新文件）
LOG_DIR = "/home/nvidia/moon/logs"

# ============================================================
# 会话日志
# ============================================================

class SessionLogger:
    """把关键事件与控制指令落到文件，方便摔倒后复盘。"""

    def __init__(self, log_dir: str = LOG_DIR):
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(log_dir, f"uwb_follow_{stamp}.log")
        self._logger = logging.getLogger(f"uwb_follow_{stamp}")
        self._logger.setLevel(logging.DEBUG)
        self._logger.handlers.clear()
        self._logger.propagate = False
        fh = logging.FileHandler(self.path, encoding="utf-8")
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(logging.Formatter("%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s",
                                         datefmt="%Y-%m-%d %H:%M:%S"))
        self._logger.addHandler(fh)
        self._motion_i = 0
        self.fsm_state: Optional[int] = None
        self.info(f"=== session start === log={self.path}")
        self.info(
            f"config SINGLE_PAIR={SINGLE_PAIR} FWD_MAX={FORWARD_MAX} ROT_MAX={ROTATE_MAX} "
            f"SOFT_START={SOFT_START_SEC}s ANGLE_DZ={ANGLE_DEADZONE} "
            f"AUTO_RUNNING={AUTO_ENTER_RUNNING} OBSTACLE_GATE={ENABLE_OBSTACLE_GATE}"
        )

    def info(self, msg: str) -> None:
        self._logger.info(msg)

    def warn(self, msg: str) -> None:
        self._logger.warning(msg)

    def error(self, msg: str) -> None:
        self._logger.error(msg)

    def event(self, name: str, **kwargs) -> None:
        extra = " ".join(f"{k}={v}" for k, v in kwargs.items())
        self.info(f"EVENT {name} {extra}".rstrip())

    def motion(
        self,
        *,
        dist: float,
        ang: float,
        state: str,
        fwd: float,
        rot: float,
        vx: float,
        wz: float,
        lt: float,
        rt: float,
        lb: float,
        gate: str,
        soft: float,
        force: bool = False,
    ) -> None:
        # 50Hz 全记太大：默认每 5 帧一条；关键变化 force=True
        self._motion_i += 1
        if not force and (self._motion_i % 5) != 0:
            return
        self.info(
            f"MOTION fsm={self.fsm_state} dist={dist:.1f} ang={ang:.1f} "
            f"ctrl={state} fwd={fwd:.3f} rot={rot:.3f} "
            f"cmd_vel=({vx:.3f},0,{wz:.3f}) joy(lt={lt:.1f},rt={rt:.1f},lb={lb:.0f}) "
            f"soft={soft:.2f} gate={gate}"
        )

    def close(self, reason: str = "exit") -> None:
        self.info(f"=== session end ({reason}) ===")
        for h in list(self._logger.handlers):
            h.close()
            self._logger.removeHandler(h)

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

class AngleFilter:
    """抑制 UWB AOA 跳变：EMA + 单帧跳变拒绝。"""

    def __init__(self):
        self._ang: Optional[float] = None
        self.last_rejected = False

    def update(self, raw_ang: float, is_anomaly: bool) -> float:
        self.last_rejected = False
        if is_anomaly:
            # 无方位：保持上次滤波角，但不强制清零（由 controller 决定）
            return 0.0 if self._ang is None else self._ang

        if self._ang is None:
            self._ang = raw_ang
            return self._ang

        if abs(raw_ang - self._ang) > ANGLE_JUMP_REJECT_DEG:
            self.last_rejected = True
            if ANGLE_HOLD_ON_JUMP:
                return self._ang
            # 否则仍缓慢靠拢
            raw_ang = self._ang + math.copysign(ANGLE_JUMP_REJECT_DEG * 0.3, raw_ang - self._ang)

        self._ang = (1.0 - ANGLE_EMA_ALPHA) * self._ang + ANGLE_EMA_ALPHA * raw_ang
        return self._ang


class FollowController:
    def __init__(self):
        self.angle_filter = AngleFilter()

    def calculate(self, data: UWBData) -> Tuple[float, float, str, float]:
        """返回 (fwd, rot, state, filtered_angle)"""
        filt_ang = self.angle_filter.update(data.angle, data.is_anomaly)

        # 前后
        err = data.distance - TARGET_DISTANCE
        if err > DISTANCE_ERR_CAP_CM:
            err = DISTANCE_ERR_CAP_CM
        elif err < -DISTANCE_ERR_CAP_CM:
            err = -DISTANCE_ERR_CAP_CM
        if abs(data.distance - TARGET_DISTANCE) < DISTANCE_DEADZONE:
            fwd = 0.0
            state = "KEEPING"
        else:
            raw = err / DISTANCE_SCALE
            fwd = max(-FORWARD_MAX, min(FORWARD_MAX, raw))
            if abs(fwd) < FORWARD_MIN:
                fwd = math.copysign(FORWARD_MIN, fwd)
            state = "FOLLOW" if (data.distance - TARGET_DISTANCE) > 0 else "BACK"

        if not USE_ANGLE:
            if state == "KEEPING":
                state = "ALIGNED"
            else:
                state = "DIST_ONLY"
            return fwd, 0.0, state, filt_ang

        # 旋转
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

        # 无可靠方位：双对限速
        if data.is_anomaly and not SINGLE_PAIR:
            rot = 0.0
            fwd = max(-0.1, min(0.1, fwd))
            state = "ANOMALY"

        # 大转向时压低前进，防止边冲边拧摔倒
        if abs(rot) > 1e-3 and ROTATE_MAX > 1e-6:
            scale = 1.0 - TURN_FORWARD_COUPLING * (abs(rot) / ROTATE_MAX)
            fwd *= max(0.10, scale)

        if abs(filt_ang) >= TURN_FIRST_ANGLE_DEG and abs(rot) > 1e-3:
            fwd *= TURN_FIRST_FWD_SCALE
            if state in ("FOLLOW", "BACK", "KEEPING", "ALIGNED"):
                state = "TURN_FIRST"

        return fwd, rot, state, filt_ang

# ============================================================
# 串口探测
# ============================================================
# 现场 UWB：672 主站（机器人）+ 671 标签（跟随目标）；当前按单对配置

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
    """探测 UWB 主站串口：只返回有 ###1.9 的口（不回退到 IMU 等空口）"""
    candidates = _list_ttyusb_ports()
    if not candidates:
        return None

    for port in candidates:
        if _port_has_uwb_data(port):
            print(f"\033[92m[DETECT]\033[0m 找到 UWB 主站数据口: {port} (672)")
            return port
    return None


def try_open_serial(port: str) -> Optional[serial.Serial]:
    """尝试打开串口，失败返回 None"""
    try:
        return serial.Serial(port, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
    except Exception:
        return None


# joy.yaml 速度映射（与 joy_teleop 一致）
CMD_VEL_X_SCALE = 1.5
CMD_VEL_YAW_SCALE = 1.57

def _joy_teleop_setup() -> str:
    try:
        from common.sim2real_env import joy_teleop_restore_cmd
        return joy_teleop_restore_cmd()
    except Exception:
        return (
            "source /home/nvidia/sim2real/install/setup.bash && "
            "roslaunch sim2real_master joy_teleop.launch use_filter:=true &"
        )


JOY_TELEOP_SETUP = _joy_teleop_setup()


def kill_joy_teleop() -> None:
    """停掉 joy_teleop，避免与 UWB 抢 /joy_msg、/cmd_vel（同 keyboard_teleop）"""
    subprocess.run(['rosnode', 'kill', '/joy_teleop'], capture_output=True)
    time.sleep(0.8)
    print("\033[93m[CTRL]\033[0m 已停止 /joy_teleop，改由 UWB 直连 /joy_msg + /cmd_vel")


def restore_joy_teleop() -> None:
    """恢复手柄链路"""
    subprocess.Popen(
        ['bash', '-c', JOY_TELEOP_SETUP],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    print("\033[93m[CTRL]\033[0m 正在恢复 /joy_teleop 手柄控制...")


def publish_motion(
    joy_msg_pub,
    cmd_pub,
    fwd: float,
    rot: float,
    lb: float = 0.0,
    lt: float = 0.0,
    rt: float = 0.0,
    slog: Optional[SessionLogger] = None,
    *,
    dist: float = float("nan"),
    ang: float = float("nan"),
    state: str = "",
    gate: str = "",
    soft: float = 1.0,
    force_log: bool = False,
) -> None:
    """直接发布策略所需的 /joy_msg 与 /cmd_vel"""
    joy = SimJoy()
    joy.lt = float(lt)
    joy.rt = float(rt)
    joy.l_vertical = float(fwd)
    joy.r_horizontal = float(rot)
    joy.lb = float(lb)
    joy_msg_pub.publish(joy)

    twist = Twist()
    twist.linear.x = float(fwd) * CMD_VEL_X_SCALE
    twist.linear.y = 0.0
    twist.linear.z = 1.0  # 键盘侧习惯；C++ 运控不读 linear.z
    twist.angular.z = float(rot) * CMD_VEL_YAW_SCALE
    cmd_pub.publish(twist)

    if slog is not None:
        slog.motion(
            dist=dist, ang=ang, state=state or "pub",
            fwd=fwd, rot=rot,
            vx=twist.linear.x, wz=twist.angular.z,
            lt=lt, rt=rt, lb=lb, gate=gate or "-", soft=soft,
            force=force_log,
        )


def enter_running_via_joy_msg(joy_msg_pub, cmd_pub, slog: Optional[SessionLogger] = None) -> None:
    """
    DefaultController 切 RUNNING 条件（已核对源码）：
      lt < -0.5 且 rt < -0.5（按住扳机）时，lb 上升沿 → STANDBY↔RUNNING
    """
    msg = "按住 LT+RT(=-1) 并点 LB，切入 RUNNING..."
    print(f"\033[93m[RUNNING]\033[0m {msg}")
    if slog:
        slog.event("enter_running_begin")

    for _ in range(10):
        publish_motion(joy_msg_pub, cmd_pub, 0.0, 0.0, lb=0.0, lt=-1.0, rt=-1.0,
                       slog=slog, state="LT_RT_HOLD", force_log=True)
        time.sleep(0.02)
    publish_motion(joy_msg_pub, cmd_pub, 0.0, 0.0, lb=1.0, lt=-1.0, rt=-1.0,
                   slog=slog, state="LB_PULSE", force_log=True)
    time.sleep(0.05)
    for _ in range(5):
        publish_motion(joy_msg_pub, cmd_pub, 0.0, 0.0, lb=0.0, lt=-1.0, rt=-1.0,
                       slog=slog, state="LT_RT_HOLD", force_log=True)
        time.sleep(0.02)
    time.sleep(1.6)
    for _ in range(5):
        publish_motion(joy_msg_pub, cmd_pub, 0.0, 0.0, lb=0.0, lt=0.0, rt=0.0,
                       slog=slog, state="RELEASE", force_log=True)
        time.sleep(0.02)

    done = "LT+RT+LB 已发送；请确认踏步。复盘看日志与 rosout 'switch to running'"
    print(f"\033[93m[RUNNING]\033[0m {done}")
    if slog:
        slog.event("enter_running_done")

# ============================================================
# 主程序
# ============================================================

def main():
    slog = SessionLogger()
    rospy.init_node('uwb_follow', anonymous=True)
    joy_msg_pub = rospy.Publisher('/joy_msg', SimJoy, queue_size=10)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def _on_fsm(msg: Int32):
        if slog.fsm_state != msg.data:
            slog.event("fsm_state_change", old=slog.fsm_state, new=msg.data)
        slog.fsm_state = msg.data

    rospy.Subscriber('/fsm_state', Int32, _on_fsm, queue_size=1)

    # 障碍状态：由 vision/zed_obstacle_node 发布；本节点只订阅
    obstacle_lock_state = {"obs": ObstacleState(valid=False), "t": 0.0}

    def _on_obstacle(msg: Float32MultiArray):
        obstacle_lock_state["obs"] = ObstacleState.from_list(msg.data, stamp=time.time())
        obstacle_lock_state["t"] = time.time()

    if ENABLE_OBSTACLE_GATE:
        rospy.Subscriber(OBSTACLE_TOPIC, Float32MultiArray, _on_obstacle, queue_size=1)

    time.sleep(0.5)
    rate = rospy.Rate(CONTROL_RATE)

    controller = FollowController()
    teleop_taken_over = False
    soft_start_t0: Optional[float] = None

    print()
    print("╔══════════════════════════════════════════════════╗")
    print("║       UWB 自主跟随系统 (直连 cmd_vel 版)         ║")
    print("║                                                  ║")
    print("║  • 单对 672主站+671标签 → 前后跟随               ║")
    print("║  • 接管 joy_teleop，直发 /joy_msg + /cmd_vel     ║")
    print("║  • 软启动 + 会话日志 → /home/nvidia/moon/logs    ║")
    print("║  • 按 Ctrl+C 退出并恢复手柄                      ║")
    print("╚══════════════════════════════════════════════════╝")
    print()
    print(f"\033[93m[LOG]\033[0m 本次日志: {slog.path}")
    print("\033[93m[NOTE]\033[0m 请先让机器人站立完成（STANDBY），再插入/等待 UWB 信号")
    if ENABLE_OBSTACLE_GATE:
        req = "强制刹停" if OBSTACLE_REQUIRED else "可选（无视觉仍跟随）"
        print(f"\033[93m[NOTE]\033[0m 障碍门控已开 → {OBSTACLE_TOPIC} | 无视觉时: {req}")

    try:
        while not rospy.is_shutdown():
            port = find_uwb_port()

            if port is None:
                if teleop_taken_over:
                    restore_joy_teleop()
                    teleop_taken_over = False
                    slog.event("restore_joy_teleop", reason="no_uwb")
                print(f"\033[90m[WAITING]\033[0m UWB 未检测到，手柄控制中... ({time.strftime('%H:%M:%S')})")
                time.sleep(SERIAL_RETRY_INTERVAL)
                continue

            ser = try_open_serial(port)
            if ser is None:
                slog.warn(f"open serial failed: {port}")
                print(f"\033[91m[ERROR]\033[0m 无法打开串口 {port}，{SERIAL_RETRY_INTERVAL}s 后重试...")
                time.sleep(SERIAL_RETRY_INTERVAL)
                continue

            if not teleop_taken_over:
                kill_joy_teleop()
                teleop_taken_over = True
                slog.event("kill_joy_teleop")

            print(f"\033[92m[CONNECTED]\033[0m UWB 串口已连接: {port}")
            print(f"\033[92m[CONNECTED]\033[0m 直连发布 /joy_msg + /cmd_vel")
            slog.event("uwb_connected", port=port)

            parser = UWBParser(ser)
            latest_data: Optional[UWBData] = None
            last_update_time = time.time()
            print_ticker = 0
            is_publishing = False
            running_enter_sent = False
            soft_start_t0 = None

            try:
                while not rospy.is_shutdown():
                    data = parser.read_latest()
                    if data:
                        latest_data = data
                        last_update_time = time.time()

                    elapsed = time.time() - last_update_time

                    if elapsed > UWB_TIMEOUT:
                        if is_publishing:
                            publish_motion(joy_msg_pub, cmd_pub, 0.0, 0.0, slog=slog,
                                           state="SIGNAL_LOST", force_log=True)
                            is_publishing = False
                            slog.event("uwb_signal_lost", elapsed=f"{elapsed:.2f}")
                        print_ticker += 1
                        if print_ticker >= 5:
                            print_ticker = 0
                            print(f"\033[90m[STANDBY]\033[0m UWB 信号丢失 ({elapsed:.1f}s) | 已发零速")

                    elif latest_data:
                        # 保护关机 / ERROR：立即零速，不再发跟随
                        if slog.fsm_state in (FSM_PROTECTION_SHUTDOWN, FSM_ERROR):
                            publish_motion(
                                joy_msg_pub, cmd_pub, 0.0, 0.0, slog=slog,
                                state="FSM_FATAL", force_log=True,
                            )
                            if is_publishing:
                                slog.event(
                                    "fsm_fatal_stop",
                                    fsm=slog.fsm_state,
                                    hint="PROTECTION/ERROR → zero cmd_vel",
                                )
                            is_publishing = False
                            print_ticker += 1
                            if print_ticker >= 5:
                                print_ticker = 0
                                print(
                                    f"\033[91m[SAFE]\033[0m fsm={slog.fsm_state} "
                                    "保护态，已停发跟随（请扶起后重启跟随）"
                                )
                            rate.sleep()
                            continue

                        fwd, rot, state, filt_ang = controller.calculate(latest_data)
                        gate_reason = "OFF"

                        if ENABLE_OBSTACLE_GATE:
                            obs = obstacle_lock_state["obs"]
                            age = time.time() - obstacle_lock_state["t"]
                            stale = (obstacle_lock_state["t"] <= 0.0) or (age > OBSTACLE_TIMEOUT)
                            person_m = estimate_person_center_m(
                                uwb_distance_cm=latest_data.distance
                            )
                            fwd, rot, gate_reason = apply_safety_gate(
                                fwd, rot, obs,
                                use_rotate_bias=USE_OBSTACLE_SIDESTEP,
                                stale=stale,
                                required=OBSTACLE_REQUIRED,
                                sidestep_blend=SIDESTEP_BLEND,
                                person_center_m=person_m,
                            )

                        if AUTO_ENTER_RUNNING and not running_enter_sent:
                            running_enter_sent = True
                            enter_running_via_joy_msg(joy_msg_pub, cmd_pub, slog)
                            soft_start_t0 = time.time()
                            slog.event("soft_start_begin", sec=SOFT_START_SEC)

                        soft = 1.0
                        if soft_start_t0 is not None and SOFT_START_SEC > 0:
                            soft = min(1.0, (time.time() - soft_start_t0) / SOFT_START_SEC)
                            fwd *= soft
                            rot *= soft ** SOFT_YAW_POWER

                        publish_motion(
                            joy_msg_pub, cmd_pub, fwd, rot, slog=slog,
                            dist=latest_data.distance, ang=filt_ang,
                            state=state, gate=gate_reason, soft=soft,
                        )
                        is_publishing = True

                        print_ticker += 1
                        if print_ticker >= 5:
                            print_ticker = 0
                            prefix = (
                                "\033[93m[DIST  ]\033[0m" if latest_data.is_anomaly and SINGLE_PAIR
                                else "\033[93m[ANG   ]\033[0m" if state == "ANG_HOLD"
                                else "\033[93m[ANOMALY]\033[0m" if latest_data.is_anomaly
                                else "\033[92m[FOLLOW ]\033[0m"
                            )
                            vx = fwd * CMD_VEL_X_SCALE
                            print(
                                f"{prefix} Dist:{latest_data.distance:5.1f}cm "
                                f"Ang:{latest_data.angle:5.1f}°→{filt_ang:5.1f}° "
                                f"| {state:<8} | fwd:{fwd:5.2f} rot:{rot:5.2f} | cmd_vx:{vx:5.2f} "
                                f"| soft:{soft:.2f} fsm:{slog.fsm_state} gate:{gate_reason}"
                            )

                    rate.sleep()

            except (OSError, serial.SerialException) as e:
                print(f"\033[91m[DISCONNECTED]\033[0m UWB 串口断开 ({port})")
                slog.event("uwb_disconnected", port=port, err=str(e))
                publish_motion(joy_msg_pub, cmd_pub, 0.0, 0.0, slog=slog,
                               state="DISCONNECT", force_log=True)

            except KeyboardInterrupt:
                raise

            finally:
                try:
                    ser.close()
                except Exception:
                    pass

            time.sleep(SERIAL_RETRY_INTERVAL)

    finally:
        if teleop_taken_over:
            restore_joy_teleop()
            slog.event("restore_joy_teleop", reason="shutdown")
        slog.close("shutdown")
        print(f"\033[93m[LOG]\033[0m 日志已保存: {slog.path}")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n✅ UWB 跟随系统已退出，手柄控制已恢复。")
    except rospy.ROSInterruptException:
        pass
