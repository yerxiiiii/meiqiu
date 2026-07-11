# -*- coding: utf-8 -*-
"""sim2real lowlevel_controller（arm_joint），仅在 EXEC_DEVELOP 下通常可用。"""

from __future__ import annotations

import os
import sys
from typing import List, Optional

import actionlib
import rospy
from sensor_msgs.msg import JointState

ARM12_JOINT_NAMES: List[str] = [
    "l_shoulder_pitch_joint",
    "l_shoulder_roll_joint",
    "l_upper_arm_joint",
    "l_elbow_joint",
    "l_wrist_joint",
    "l_claw_joint",
    "r_shoulder_pitch_joint",
    "r_shoulder_roll_joint",
    "r_upper_arm_joint",
    "r_elbow_joint",
    "r_wrist_joint",
    "r_claw_joint",
]

RIGHT_ARM_START = 6
RIGHT_ARM_DOF = 5
RIGHT_CLAW_IDX = 11


def _ensure_sim2real_msg() -> None:
    for rel in (
        "~/sim2real/devel/lib/python3/dist-packages",
        "~/sim2real/install/lib/python3/dist-packages",
    ):
        p = os.path.expanduser(rel)
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)
    import sim2real_msg  # noqa: F401


def discover_lowlevel_namespace(
    default: str = "/sim2real_master_node",
) -> str:
    """从已发布话题推断 lowlevel action 所在命名空间。"""
    suffix = "/lowlevel_controller/status"
    try:
        for topic, _ in rospy.get_published_topics():
            if topic.endswith(suffix):
                return topic[: -len(suffix)]
    except Exception:
        pass
    return default.rstrip("/")


def try_connect_lowlevel(
    namespace: str,
    group: str = "arm_joint",
    joint_names: Optional[List[str]] = None,
    *,
    timeout: float = 3.0,
) -> Optional["LowlevelArmClient"]:
    try:
        client = LowlevelArmClient(
            namespace,
            group,
            joint_names,
            wait_server=True,
            server_timeout=timeout,
        )
        return client
    except RuntimeError:
        return None


class LowlevelArmClient:
    def __init__(
        self,
        namespace: str = "/sim2real_master_node",
        group: str = "arm_joint",
        joint_names: Optional[List[str]] = None,
        *,
        wait_server: bool = True,
        server_timeout: float = 10.0,
    ):
        _ensure_sim2real_msg()
        from sim2real_msg.msg import lowlevel_controllerAction, lowlevel_controllerGoal

        self._Goal = lowlevel_controllerGoal
        self.group = group
        self.joint_names = list(joint_names or ARM12_JOINT_NAMES)
        if len(self.joint_names) != 12:
            raise ValueError(f"arm_joint 需要 12 个关节，收到 {len(self.joint_names)}")
        ns = namespace.rstrip("/")
        self._action_name = f"{ns}/lowlevel_controller"
        self._client = actionlib.SimpleActionClient(
            self._action_name,
            lowlevel_controllerAction,
        )
        if wait_server:
            if not self._client.wait_for_server(rospy.Duration(server_timeout)):
                raise RuntimeError(
                    f"lowlevel_controller 未就绪: {self._action_name}",
                )
            rospy.loginfo("[arm_ik] lowlevel 已连接: %s", self._action_name)

    def send_positions(
        self,
        positions: List[float],
        *,
        transition_type: int = 4,
        duration: float = 0.05,
        cancel_previous: bool = True,
    ) -> None:
        if len(positions) != len(self.joint_names):
            raise ValueError(
                f"位置数量 {len(positions)} != 关节数 {len(self.joint_names)}",
            )
        state = self._client.get_state()
        if cancel_previous and state in (
            actionlib.GoalStatus.ACTIVE,
            actionlib.GoalStatus.PENDING,
        ):
            self._client.cancel_goal()
        goal = self._Goal()
        goal.group = self.group
        goal.type = int(transition_type)
        goal.duration = float(duration)
        js = JointState()
        js.name = self.joint_names
        js.position = [float(p) for p in positions]
        goal.robotOutput.append(js)
        self._client.send_goal(goal)
