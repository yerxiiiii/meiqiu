#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一运动 IO：kill/restore joy_teleop、双发 /joy_msg + /cmd_vel、RUNNING 脉冲、零速。
抽自 moon/uwb_follow.py / keyboard_teleop.py，供 guide_demo_node 唯一控制出口使用。
"""

from __future__ import annotations

import subprocess
import time
from typing import Optional

import rospy
from geometry_msgs.msg import Twist
from sim2real_msg.msg import Joy as SimJoy

# 与 joy.yaml / uwb_follow 一致
CMD_VEL_X_SCALE = 1.5
CMD_VEL_YAW_SCALE = 1.57

# 带路保守上限（m/s, rad/s）
MAX_LINEAR_X = 0.12
MAX_ANGULAR_Z = 0.25

JOY_TELEOP_SETUP = (
    "source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash && "
    "roslaunch sim2real_master joy_teleop.launch use_filter:=true &"
)


def kill_joy_teleop() -> None:
    """停掉 joy_teleop，避免与 guide 抢 /joy_msg、/cmd_vel。"""
    subprocess.run(["rosnode", "kill", "/joy_teleop"], capture_output=True)
    time.sleep(0.8)
    print("\033[93m[CTRL]\033[0m 已停止 /joy_teleop，改由 guide 直连 /joy_msg + /cmd_vel")


def restore_joy_teleop() -> None:
    """恢复手柄链路。"""
    subprocess.Popen(
        ["bash", "-c", JOY_TELEOP_SETUP],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    print("\033[93m[CTRL]\033[0m 正在恢复 /joy_teleop 手柄控制...")


def warn_if_uwb_follow_running() -> None:
    """提示停掉 uwb-follow，避免抢控制。"""
    try:
        r = subprocess.run(
            ["systemctl", "is-active", "uwb-follow.service"],
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        if r.stdout.strip() == "active":
            print(
                "\033[91m[WARN]\033[0m uwb-follow.service 仍在运行，会抢 /cmd_vel。"
                "请先: sudo systemctl stop uwb-follow.service"
            )
    except Exception:
        pass


def clamp_cmd(vx: float, wz: float) -> tuple:
    """限幅到带路保守速度。"""
    vx = max(-MAX_LINEAR_X, min(MAX_LINEAR_X, float(vx)))
    wz = max(-MAX_ANGULAR_Z, min(MAX_ANGULAR_Z, float(wz)))
    return vx, wz


def vx_wz_to_sticks(vx: float, wz: float) -> tuple:
    """cmd_vel → 摇杆 fwd/rot（与 publish_motion 互逆）。"""
    vx, wz = clamp_cmd(vx, wz)
    fwd = vx / CMD_VEL_X_SCALE if CMD_VEL_X_SCALE else 0.0
    rot = wz / CMD_VEL_YAW_SCALE if CMD_VEL_YAW_SCALE else 0.0
    return fwd, rot


class MotionIO:
    """唯一运动发布器。dry_run=True 时只打印不发速。"""

    def __init__(self, dry_run: bool = False):
        self.dry_run = dry_run
        self._joy_pub: Optional[rospy.Publisher] = None
        self._cmd_pub: Optional[rospy.Publisher] = None
        self._took_control = False
        self._last_log = 0.0

    def setup(self) -> None:
        if self.dry_run:
            print("\033[93m[DRY-RUN]\033[0m MotionIO：不发布 /joy_msg /cmd_vel")
            return
        warn_if_uwb_follow_running()
        kill_joy_teleop()
        self._joy_pub = rospy.Publisher("/joy_msg", SimJoy, queue_size=10)
        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        time.sleep(0.3)
        self._took_control = True
        print("\033[92m[CTRL]\033[0m MotionIO 已接管 /joy_msg + /cmd_vel")

    def publish(
        self,
        vx: float = 0.0,
        wz: float = 0.0,
        *,
        lb: float = 0.0,
        lt: float = 0.0,
        rt: float = 0.0,
        note: str = "",
    ) -> None:
        vx, wz = clamp_cmd(vx, wz)
        fwd, rot = vx_wz_to_sticks(vx, wz)

        if self.dry_run:
            now = time.time()
            if now - self._last_log > 0.4 or note:
                print(
                    f"[DRY-RUN] motion vx={vx:.3f} wz={wz:.3f} "
                    f"fwd={fwd:.3f} rot={rot:.3f} {note}".rstrip()
                )
                self._last_log = now
            return

        if self._joy_pub is None or self._cmd_pub is None:
            return

        joy = SimJoy()
        joy.lt = float(lt)
        joy.rt = float(rt)
        joy.l_vertical = float(fwd)
        joy.r_horizontal = float(rot)
        joy.lb = float(lb)
        self._joy_pub.publish(joy)

        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = 0.0
        twist.linear.z = 1.0
        twist.angular.z = float(wz)
        self._cmd_pub.publish(twist)

    def zero(self, note: str = "zero") -> None:
        self.publish(0.0, 0.0, note=note)

    def enter_running(self) -> None:
        """
        DefaultController：lt<-0.5 且 rt<-0.5 时 lb 上升沿 → STANDBY↔RUNNING。
        """
        if self.dry_run:
            print("[DRY-RUN] enter_running (LT+RT+LB pulse)")
            return
        print("\033[93m[RUNNING]\033[0m 发送 LT+RT+LB 切入 RUNNING...")
        for _ in range(10):
            self.publish(0.0, 0.0, lb=0.0, lt=-1.0, rt=-1.0, note="LT_RT_HOLD")
            time.sleep(0.02)
        self.publish(0.0, 0.0, lb=1.0, lt=-1.0, rt=-1.0, note="LB_PULSE")
        time.sleep(0.05)
        for _ in range(5):
            self.publish(0.0, 0.0, lb=0.0, lt=-1.0, rt=-1.0, note="LT_RT_HOLD")
            time.sleep(0.02)
        time.sleep(1.6)
        for _ in range(5):
            self.publish(0.0, 0.0, note="RELEASE")
            time.sleep(0.02)
        print("\033[93m[RUNNING]\033[0m 已发送；请确认踏步 / rosout 'switch to running'")

    def shutdown(self) -> None:
        """急停 + 恢复手柄。"""
        try:
            self.zero(note="shutdown")
            # 多发几次确保底层收到
            if not self.dry_run:
                for _ in range(5):
                    self.zero()
                    time.sleep(0.02)
        except Exception:
            pass
        if self._took_control and not self.dry_run:
            restore_joy_teleop()
            self._took_control = False
