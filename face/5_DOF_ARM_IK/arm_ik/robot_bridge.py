# -*- coding: utf-8 -*-
"""
实机桥接：仅右臂 IK（5 关节 + 夹爪）→ /pi_plus_absolute。

默认在 Start 站立后 FSM=EXEC_DEFAULT(5) 下发，不控左臂。
可选 arm_backend: lowlevel（需 EXEC_DEVELOP，双臂 12 关节）。
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np
import rospy
import yaml
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

from arm_ik.lowlevel_arm import (
    ARM12_JOINT_NAMES,
    RIGHT_ARM_DOF,
    RIGHT_ARM_START,
    RIGHT_CLAW_IDX,
    LowlevelArmClient,
    discover_lowlevel_namespace,
    try_connect_lowlevel,
)

FSM_STATE_TOPIC = "/fsm_state"
FSM_EXEC_DEFAULT = 5
FSM_EXEC_DEVELOP = 16
ABSOLUTE_TOPIC = "/pi_plus_absolute"

_FSM_NAMES = {
    0: "INIT",
    5: "EXEC_DEFAULT",
    6: "EXEC_CUSTOM",
    8: "PROTECTION_SHUTDOWN",
    15: "CANDIDATE_DEVELOP",
    16: "EXEC_DEVELOP",
}


@dataclass
class RobotBridgeConfig:
    arm_backend: str = "absolute"
    absolute_topic: str = ABSOLUTE_TOPIC
    claw_joint_name: str = "r_claw_joint"
    right_only: bool = True
    lowlevel_namespace: str = "/sim2real_master_node"
    lowlevel_group: str = "arm_joint"
    lowlevel_server_timeout: float = 3.0
    joint_state_prefix: str = "/livelybot_real_real"
    claw_open_rad: float = 0.0
    claw_close_rad: float = 1.04
    publish_rate_hz: float = 50.0
    joint_ramp_rad_per_sec: float = 4.0
    goal_max_step_rad: float = 0.12
    lowlevel_transition_type: int = 4
    lowlevel_duration: float = 0.05
    require_fsm_exec_default: bool = True
    left_arm_hold_rad: Optional[List[float]] = None


class FsmStateMonitor:
    def __init__(self, topic: str = FSM_STATE_TOPIC):
        self._lock = threading.Lock()
        self._state: Optional[int] = None
        self._sub = rospy.Subscriber(topic, Int32, self._cb, queue_size=10)

    def _cb(self, msg: Int32) -> None:
        with self._lock:
            self._state = int(msg.data)

    @property
    def state(self) -> Optional[int]:
        with self._lock:
            return self._state

    @classmethod
    def state_name(cls, v: Optional[int]) -> str:
        if v is None:
            return "未知(未收到 /fsm_state)"
        return _FSM_NAMES.get(v, f"UNKNOWN({v})")

    def is_exec_default(self) -> bool:
        return self.state == FSM_EXEC_DEFAULT

    def is_exec_develop(self) -> bool:
        return self.state == FSM_EXEC_DEVELOP

    def wait_for_exec_default(self, timeout: float = 60.0) -> bool:
        return self._wait_state(FSM_EXEC_DEFAULT, timeout)

    def wait_for_exec_develop(self, timeout: float = 120.0) -> bool:
        return self._wait_state(FSM_EXEC_DEVELOP, timeout)

    def _wait_state(self, target: int, timeout: float) -> bool:
        t0 = time.time()
        while not rospy.is_shutdown():
            if self.state == target:
                return True
            if timeout > 0 and time.time() - t0 > timeout:
                return False
            time.sleep(0.1)
        return False


def load_robot_config(path: str | Path) -> RobotBridgeConfig:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    block = data.get("robot") or {}
    hold = block.get("left_arm_hold_rad")
    return RobotBridgeConfig(
        arm_backend=str(block.get("arm_backend", "absolute")),
        absolute_topic=str(block.get("absolute_topic", ABSOLUTE_TOPIC)),
        claw_joint_name=str(block.get("claw_joint_name", "r_claw_joint")),
        right_only=bool(block.get("right_only", True)),
        lowlevel_namespace=str(
            block.get("lowlevel_namespace", "/sim2real_master_node"),
        ),
        lowlevel_group=str(block.get("lowlevel_group", "arm_joint")),
        lowlevel_server_timeout=float(block.get("lowlevel_server_timeout", 3.0)),
        joint_state_prefix=str(
            block.get("joint_state_prefix", "/livelybot_real_real"),
        ),
        claw_open_rad=float(block.get("claw_open_rad", 0.0)),
        claw_close_rad=float(block.get("claw_close_rad", 1.04)),
        publish_rate_hz=float(block.get("publish_rate_hz", 50.0)),
        joint_ramp_rad_per_sec=float(block.get("joint_ramp_rad_per_sec", 4.0)),
        goal_max_step_rad=float(block.get("goal_max_step_rad", 0.12)),
        lowlevel_transition_type=int(block.get("lowlevel_transition_type", 4)),
        lowlevel_duration=float(block.get("lowlevel_duration", 0.05)),
        require_fsm_exec_default=bool(block.get("require_fsm_exec_default", True)),
        left_arm_hold_rad=list(hold) if hold is not None else None,
    )


def _step_toward(cur: float, goal: float, max_step: float) -> float:
    if abs(goal - cur) <= max_step:
        return goal
    return cur + max_step if goal > cur else cur - max_step


def _read_joint_positions(
    joint_names: List[str],
    prefix: str,
    timeout: float = 0.3,
) -> List[float]:
    try:
        from livelybot_serial.msg import MotorState
    except ImportError:
        return [0.0] * len(joint_names)

    out: List[float] = []
    for name in joint_names:
        topic = f"{prefix.rstrip('/')}/{name}_controller/state"
        try:
            msg = rospy.wait_for_message(topic, MotorState, timeout=timeout)
            out.append(float(msg.pos))
        except (rospy.ROSException, rospy.ROSInterruptException):
            out.append(0.0)
    return out


class RightArmRobotBridge:
    """仅右臂：默认 absolute + EXEC_DEFAULT(Start 站立)。"""

    def __init__(
        self,
        ik_joint_names: list[str],
        cfg: RobotBridgeConfig,
        *,
        dry_run: bool = False,
        check_fsm: Optional[bool] = None,
        prefer_develop: bool = False,
    ):
        if len(ik_joint_names) != RIGHT_ARM_DOF:
            raise ValueError(f"IK 需 {RIGHT_ARM_DOF} 关节，收到 {len(ik_joint_names)}")
        self._ik_joint_names = list(ik_joint_names)
        self._cfg = cfg
        self._dry_run = dry_run
        self._check_fsm = (
            cfg.require_fsm_exec_default
            if check_fsm is None
            else check_fsm
        )
        self._fsm = FsmStateMonitor() if self._check_fsm else None
        self._lock = threading.Lock()
        self._q_goal = np.zeros(RIGHT_ARM_DOF)
        self._q_cur = np.zeros(RIGHT_ARM_DOF)
        self._claw_open = True
        self._claw_pos = float(cfg.claw_open_rad)

        self._pub_names = list(ik_joint_names) + [cfg.claw_joint_name]
        self._lowlevel: Optional[LowlevelArmClient] = None
        self._abs_pub: Optional[rospy.Publisher] = None
        self._effective_backend = "none"
        self._arm12_cur: Optional[np.ndarray] = None
        self._arm12_goal: Optional[np.ndarray] = None

        if not dry_run:
            right_q = _read_joint_positions(
                self._pub_names, cfg.joint_state_prefix,
            )
            self._q_cur[:] = right_q[:RIGHT_ARM_DOF]
            self._q_goal[:] = right_q[:RIGHT_ARM_DOF]
            if len(right_q) > RIGHT_ARM_DOF:
                self._claw_pos = float(right_q[RIGHT_ARM_DOF])
                self._claw_open = abs(self._claw_pos - cfg.claw_open_rad) < abs(
                    self._claw_pos - cfg.claw_close_rad,
                )
            rospy.loginfo(
                "[arm_ik] 右臂初值: %s claw=%.2f",
                ", ".join(f"{v:+.2f}" for v in self._q_cur),
                self._claw_pos,
            )
            self._setup_backends(prefer_develop)

        hz = max(float(cfg.publish_rate_hz), 1.0)
        self._dt = 1.0 / hz
        ramp = max(float(cfg.joint_ramp_rad_per_sec), 0.0)
        self._q_step = ramp * self._dt if ramp > 0 else float("inf")
        self._stop = threading.Event()
        self._goal_dirty = False
        self._last_pub_time = 0.0
        self._hold_pub_interval = 0.5
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _setup_backends(self, prefer_develop: bool) -> None:
        want = self._cfg.arm_backend
        if want == "lowlevel" or (want == "auto" and prefer_develop):
            if prefer_develop and self._fsm is not None:
                rospy.loginfo("[arm_ik] 等待 EXEC_DEVELOP(16)…")
                self._fsm.wait_for_exec_develop(timeout=120.0)
            self._init_lowlevel_arm12()
            if self._lowlevel is not None:
                self._effective_backend = "lowlevel"
                return

        self._abs_pub = rospy.Publisher(
            self._cfg.absolute_topic, JointState, queue_size=10,
        )
        self._effective_backend = "absolute"
        rospy.loginfo(
            "[arm_ik] 仅右臂 → %s（需 Start 后 FSM=EXEC_DEFAULT/5）",
            self._cfg.absolute_topic,
        )

    def _init_lowlevel_arm12(self) -> None:
        ns = discover_lowlevel_namespace(self._cfg.lowlevel_namespace)
        client = try_connect_lowlevel(
            ns,
            self._cfg.lowlevel_group,
            list(ARM12_JOINT_NAMES),
            timeout=self._cfg.lowlevel_server_timeout,
        )
        if client is None:
            return
        self._lowlevel = client
        n12 = len(ARM12_JOINT_NAMES)
        self._arm12_cur = np.zeros(n12)
        self._arm12_goal = np.zeros(n12)
        left_names = ARM12_JOINT_NAMES[:RIGHT_ARM_START]
        if self._cfg.left_arm_hold_rad is not None:
            hold = list(self._cfg.left_arm_hold_rad)
            self._arm12_goal[:RIGHT_ARM_START] = hold
            self._arm12_cur[:RIGHT_ARM_START] = hold
        else:
            left_q = _read_joint_positions(
                left_names, self._cfg.joint_state_prefix,
            )
            self._arm12_goal[:RIGHT_ARM_START] = left_q
            self._arm12_cur[:RIGHT_ARM_START] = left_q
        self._arm12_goal[RIGHT_ARM_START: RIGHT_ARM_START + RIGHT_ARM_DOF] = (
            self._q_goal
        )
        self._arm12_cur[RIGHT_ARM_START: RIGHT_ARM_START + RIGHT_ARM_DOF] = (
            self._q_cur
        )
        self._arm12_goal[RIGHT_CLAW_IDX] = self._claw_pos
        self._arm12_cur[RIGHT_CLAW_IDX] = self._claw_pos

    @property
    def effective_backend(self) -> str:
        return self._effective_backend

    def wait_for_ready(self, fsm_wait_sec: float = 60.0) -> bool:
        if self._dry_run:
            return True
        if self._check_fsm and self._fsm is not None:
            if self._effective_backend == "lowlevel":
                if not self._fsm.is_exec_develop():
                    rospy.logwarn(
                        "[arm_ik] FSM=%s，lowlevel 需 EXEC_DEVELOP(16)",
                        FsmStateMonitor.state_name(self._fsm.state),
                    )
                    return False
            elif not self._fsm.wait_for_exec_default(fsm_wait_sec):
                rospy.logwarn(
                    "[arm_ik] 等待 Start 站立 FSM=5，当前 %s",
                    FsmStateMonitor.state_name(self._fsm.state),
                )
                return False
        if not self._dry_run and self._abs_pub is not None:
            t0 = time.time()
            while (
                self._abs_pub.get_num_connections() == 0
                and not rospy.is_shutdown()
                and time.time() - t0 < 5.0
            ):
                time.sleep(0.05)
            n = self._abs_pub.get_num_connections()
            if n == 0:
                rospy.logwarn("[arm_ik] %s 无订阅者", self._cfg.absolute_topic)
            else:
                rospy.loginfo(
                    "[arm_ik] %s 订阅者=%d，仅右臂调试",
                    self._cfg.absolute_topic,
                    n,
                )
        return True

    @property
    def claw_is_open(self) -> bool:
        with self._lock:
            return self._claw_open

    def get_arm_q(self) -> np.ndarray:
        """当前下发中的右臂关节角（与实机读数对齐后的 ramp 状态）。"""
        with self._lock:
            return self._q_cur.copy()

    def set_arm_goal(self, q: np.ndarray, *, smooth: bool = True) -> None:
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.size != RIGHT_ARM_DOF:
            raise ValueError(f"期望 {RIGHT_ARM_DOF} 个关节角，收到 {q.size}")
        step = (
            float(self._cfg.goal_max_step_rad)
            if smooth
            else float("inf")
        )
        with self._lock:
            if math.isinf(step):
                self._q_goal = q.copy()
            else:
                for i in range(RIGHT_ARM_DOF):
                    self._q_goal[i] = _step_toward(
                        self._q_goal[i], q[i], step,
                    )
            self._goal_dirty = True

    def hold_current_pose(self) -> None:
        """以当前实机姿态为 IK/下发起点，避免启动时向零位猛拉。"""
        with self._lock:
            self._q_goal = self._q_cur.copy()
            self._goal_dirty = True

    def toggle_claw(self) -> bool:
        with self._lock:
            self._claw_open = not self._claw_open
            self._claw_pos = (
                self._cfg.claw_open_rad
                if self._claw_open
                else self._cfg.claw_close_rad
            )
            return self._claw_open

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _can_publish(self) -> bool:
        if self._fsm is None:
            return True
        if self._effective_backend == "lowlevel":
            return self._fsm.is_exec_develop()
        return self._fsm.is_exec_default()

    def _right_positions(self) -> List[float]:
        with self._lock:
            goal = self._q_goal.copy()
            claw = self._claw_pos
        for i in range(RIGHT_ARM_DOF):
            self._q_cur[i] = _step_toward(
                self._q_cur[i], goal[i], self._q_step,
            )
        return [float(v) for v in self._q_cur] + [float(claw)]

    def _loop(self) -> None:
        warned_fsm = False
        while not self._stop.is_set() and not rospy.is_shutdown():
            if not self._can_publish():
                if not warned_fsm:
                    need = (
                        "EXEC_DEVELOP(16)"
                        if self._effective_backend == "lowlevel"
                        else "EXEC_DEFAULT(5) Start站立"
                    )
                    rospy.logwarn_throttle(
                        2.0,
                        "[arm_ik] FSM=%s，需要 %s",
                        self._fsm.state if self._fsm else None,
                        need,
                    )
                    warned_fsm = True
                time.sleep(self._dt)
                continue
            warned_fsm = False

            if self._dry_run:
                time.sleep(self._dt)
                continue

            if self._lowlevel is not None and self._arm12_cur is not None:
                with self._lock:
                    self._arm12_goal[RIGHT_ARM_START: RIGHT_ARM_START + RIGHT_ARM_DOF] = (
                        self._q_goal.copy()
                    )
                    self._arm12_goal[RIGHT_CLAW_IDX] = self._claw_pos
                for i in range(len(self._arm12_cur)):
                    self._arm12_cur[i] = _step_toward(
                        self._arm12_cur[i],
                        self._arm12_goal[i],
                        self._q_step,
                    )
                self._lowlevel.send_positions(
                    [float(v) for v in self._arm12_cur],
                    transition_type=self._cfg.lowlevel_transition_type,
                    duration=self._cfg.lowlevel_duration,
                    cancel_previous=True,
                )
            elif self._abs_pub is not None:
                pos = self._right_positions()
                now = time.time()
                with self._lock:
                    dirty = self._goal_dirty
                    settled = np.allclose(
                        self._q_cur, self._q_goal, atol=1e-3,
                    )
                if (
                    not dirty
                    and settled
                    and (now - self._last_pub_time) < self._hold_pub_interval
                ):
                    time.sleep(self._dt)
                    continue
                msg = JointState()
                msg.name = self._pub_names
                msg.position = pos
                msg.velocity = []
                msg.effort = []
                msg.header.stamp = rospy.Time.now()
                self._abs_pub.publish(msg)
                self._last_pub_time = now
                with self._lock:
                    if dirty and settled:
                        self._goal_dirty = False
            time.sleep(self._dt)
