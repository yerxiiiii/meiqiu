#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pi Plus 上半身实时关节调节（腿由 lr / footstep 等 lowerBody 策略负责）。

前置（很重要）:
  1. 已启动 joy_control_pi_plus*.launch（全身 22dof）
  2. /fsm_state == 5 (EXEC_DEFAULT)
  3. 走路策略必须是 lowerBody（lr 或 footstep），不能是 amp
     —— amp 会每周期覆盖手臂，本工具无效

切到 lr（STANDBY 下）:
  手柄十字键左右切换算法，直到 OLED/日志出现 lr、ctrl group:(lowerBody)
  或另开终端: python3 moon/switch_to_lr.py

键位:
  1-0 / - =   选关节（见启动时列表）
  w / s       当前关节 +/- step
  a / d       step 变小 / 变大
  r           从 /sim2real_master_node/rbt_state 重新采样当前位置
  c           当前关节回采样中心
  z           全部回采样中心
  p           打印全部目标
  q           退出
"""

import select
import sys
import termios
import threading
import time
import tty
from typing import Dict, Optional

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

ABSOLUTE_TOPIC = "/pi_plus_absolute"
STATE_TOPIC = "/sim2real_master_node/rbt_state"
FSM_TOPIC = "/fsm_state"
FSM_EXEC_DEFAULT = 5

# index >= 12 才会被 sim2real absolute 回调接受
UPPER_JOINTS = [
    "l_shoulder_pitch_joint",
    "l_shoulder_roll_joint",
    "l_upper_arm_joint",
    "l_elbow_joint",
    "r_shoulder_pitch_joint",
    "r_shoulder_roll_joint",
    "r_upper_arm_joint",
    "r_elbow_joint",
    "head_yaw_joint",
    "head_pitch_joint",
]

# 相对采样中心的软限位（rad）；头更紧一点
DEFAULT_RANGE = {
    "head_yaw_joint": 0.8,
    "head_pitch_joint": 0.5,
}
DEFAULT_RANGE_FALLBACK = 0.35

KEY_TO_IDX = {
    "1": 0, "2": 1, "3": 2, "4": 3,
    "5": 4, "6": 5, "7": 6, "8": 7,
    "9": 8, "0": 9,
}


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class UpperBodyTeleop:
    def __init__(self):
        rospy.init_node("upper_body_teleop", anonymous=False)

        self.pub_topic = rospy.get_param("~pub_topic", ABSOLUTE_TOPIC)
        self.state_topic = rospy.get_param("~state_topic", STATE_TOPIC)
        self.rate_hz = float(rospy.get_param("~rate", 30.0))
        self.step = float(rospy.get_param("~step", 0.03))
        self.require_fsm = bool(rospy.get_param("~require_fsm", True))

        self._lock = threading.Lock()
        self._measured = {}  # type: Dict[str, float]
        self._center = {n: 0.0 for n in UPPER_JOINTS}  # type: Dict[str, float]
        self._target = {n: 0.0 for n in UPPER_JOINTS}  # type: Dict[str, float]
        self._fsm = None  # type: Optional[int]
        self._sel = 0
        self._got_state = False

        self._pub = rospy.Publisher(self.pub_topic, JointState, queue_size=1)
        rospy.Subscriber(self.state_topic, JointState, self._on_state, queue_size=5)
        rospy.Subscriber(FSM_TOPIC, Int32, self._on_fsm, queue_size=5)

        self._old_term = termios.tcgetattr(sys.stdin)

    def _on_fsm(self, msg: Int32) -> None:
        self._fsm = int(msg.data)

    def _on_state(self, msg: JointState) -> None:
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._target:
                    self._measured[name] = float(pos)
            if not self._got_state and all(n in self._measured for n in UPPER_JOINTS):
                for n in UPPER_JOINTS:
                    self._center[n] = self._measured[n]
                    self._target[n] = self._measured[n]
                self._got_state = True
                rospy.loginfo("sampled upper-body pose from %s", self.state_topic)

    def _range_of(self, name: str) -> float:
        return float(DEFAULT_RANGE.get(name, DEFAULT_RANGE_FALLBACK))

    def _wait_ready(self, timeout: float = 8.0) -> bool:
        t0 = time.time()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.time() - t0 < timeout:
            if self._got_state:
                if self.require_fsm and self._fsm is not None and self._fsm != FSM_EXEC_DEFAULT:
                    rospy.logwarn_throttle(
                        2.0,
                        "fsm_state=%s (want %d EXEC_DEFAULT); absolute may be ignored",
                        self._fsm,
                        FSM_EXEC_DEFAULT,
                    )
                return True
            rate.sleep()
        rospy.logwarn("no rbt_state yet; start targets at 0")
        self._got_state = True
        return True

    def _publish(self) -> None:
        with self._lock:
            names = list(UPPER_JOINTS)
            pos = [self._target[n] for n in names]
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = names
        msg.position = pos
        self._pub.publish(msg)

    def _print_help(self) -> None:
        print("\n=== upper_body_teleop →", self.pub_topic, "===")
        print("选关节: 1..0   调节: w/s   step: a/d   重采样:r  单关节回中:c  全回中:z  打印:p  退出:q")
        for i, n in enumerate(UPPER_JOINTS):
            mark = "<<" if i == self._sel else "  "
            print(f"  {mark}[{(i + 1) % 10}] {n}")
        print(f"step={self.step:.3f} rad  fsm={self._fsm}\n")

    def _print_targets(self) -> None:
        with self._lock:
            cur = self._sel
            name = UPPER_JOINTS[cur]
            print(
                f"sel={name}  target={self._target[name]:+.3f}  "
                f"center={self._center[name]:+.3f}  "
                f"meas={self._measured.get(name, float('nan')):+.3f}  "
                f"step={self.step:.3f}"
            )

    def _get_key(self) -> str:
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if rlist:
            return sys.stdin.read(1)
        return ""

    def run(self) -> None:
        if not self._wait_ready():
            return

        n_sub = self._pub.get_num_connections()
        rospy.loginfo("publish %s (subscribers=%d)", self.pub_topic, n_sub)
        if n_sub == 0:
            rospy.logwarn("no subscriber on %s yet — is sim2real_master_node up?", self.pub_topic)

        self._print_help()
        self._print_targets()

        tty.setcbreak(sys.stdin.fileno())
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                key = self._get_key()
                if key:
                    k = key.lower()
                    if k in KEY_TO_IDX:
                        self._sel = KEY_TO_IDX[k]
                        print(f"selected [{self._sel + 1}] {UPPER_JOINTS[self._sel]}")
                    elif k == "w":
                        self._nudge(+self.step)
                    elif k == "s":
                        self._nudge(-self.step)
                    elif k == "a":
                        self.step = max(0.005, self.step * 0.7)
                        print(f"step={self.step:.3f}")
                    elif k == "d":
                        self.step = min(0.2, self.step * 1.4)
                        print(f"step={self.step:.3f}")
                    elif k == "r":
                        self._resample()
                    elif k == "c":
                        self._recenter_one()
                    elif k == "z":
                        self._recenter_all()
                    elif k == "p":
                        with self._lock:
                            for n in UPPER_JOINTS:
                                print(f"  {n}: {self._target[n]:+.4f}")
                    elif k == "h":
                        self._print_help()
                    elif k == "q" or key == "\x03":
                        print("quit")
                        break
                    self._print_targets()

                if self.require_fsm and self._fsm is not None and self._fsm != FSM_EXEC_DEFAULT:
                    rate.sleep()
                    continue

                self._publish()
                rate.sleep()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term)

    def _nudge(self, delta: float) -> None:
        name = UPPER_JOINTS[self._sel]
        with self._lock:
            center = self._center[name]
            lim = self._range_of(name)
            self._target[name] = _clamp(self._target[name] + delta, center - lim, center + lim)

    def _resample(self) -> None:
        with self._lock:
            for n in UPPER_JOINTS:
                if n in self._measured:
                    self._center[n] = self._measured[n]
                    self._target[n] = self._measured[n]
        print("resampled centers from rbt_state")

    def _recenter_one(self) -> None:
        name = UPPER_JOINTS[self._sel]
        with self._lock:
            self._target[name] = self._center[name]
        print(f"recenter {name}")

    def _recenter_all(self) -> None:
        with self._lock:
            for n in UPPER_JOINTS:
                self._target[n] = self._center[n]
        print("recenter all")


if __name__ == "__main__":
    try:
        UpperBodyTeleop().run()
    except rospy.ROSInterruptException:
        pass
