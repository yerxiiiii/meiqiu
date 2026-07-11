# -*- coding: utf-8 -*-
"""
决策层 → 运控 walk 策略切换。

通过 /joy_msg（LT+RT + 十字键）在 STANDBY 下切换，并用 /rosout（及日志）
校验 `policy name: [xxx]`。标准跟随策略名为 amp_right_hold。
"""

from __future__ import annotations

import re
import time
from pathlib import Path
from typing import Optional

import rospy
from rosgraph_msgs.msg import Log
from std_msgs.msg import Int32

try:
    from sim2real_msg.msg import Joy as SimJoy
except ImportError:
    SimJoy = None

# 与 docs/AMP_RIGHT_HOLD.md / rl_config.yaml 一致
WALK_POLICY_FOLLOW = "amp_right_hold"

JOY_TOPIC = "/joy_msg"
FSM_TOPIC = "/fsm_state"
ROSOUT_TOPIC = "/rosout"

FSM_EXEC_DEFAULT = 5

_POLICY_RE = re.compile(r"policy name:\s*\[([^\]]+)\]")
_CTRL_RE = re.compile(r"ctrl group:\(([^)\]]+)\)")


def _read_last_policy_from_log() -> Optional[str]:
    """从 ~/.ros/log/latest/rosout.log 读最后一次 policy name（订阅无历史）。"""
    candidates = [
        Path.home() / ".ros" / "log" / "latest" / "rosout.log",
        Path("/home/nvidia/.ros/log/latest/rosout.log"),
    ]
    for path in candidates:
        try:
            if not path.is_file():
                continue
            # 只扫尾部，避免大文件
            data = path.read_bytes()
            text = data[-400_000:].decode("utf-8", errors="ignore")
            matches = _POLICY_RE.findall(text)
            if matches:
                return matches[-1].strip()
        except OSError:
            continue
    return None


class PolicyTracker:
    """从 /rosout 解析当前 policy name。"""

    def __init__(self) -> None:
        self.policy: Optional[str] = _read_last_policy_from_log()
        self.ctrl_group: Optional[str] = None
        self.updated_at: float = time.time() if self.policy else 0.0
        self._sub = rospy.Subscriber(ROSOUT_TOPIC, Log, self._on_log, queue_size=50)

    def _on_log(self, msg: Log) -> None:
        text = msg.msg or ""
        m = _POLICY_RE.search(text)
        if m:
            self.policy = m.group(1).strip()
            self.updated_at = time.time()
        g = _CTRL_RE.search(text)
        if g:
            self.ctrl_group = g.group(1).strip()
            self.updated_at = time.time()

    def wait_update(self, since: float, timeout: float = 2.5) -> Optional[str]:
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            if self.updated_at >= since and self.policy:
                return self.policy
            time.sleep(0.05)
        # 日志兜底
        logged = _read_last_policy_from_log()
        if logged:
            self.policy = logged
        return self.policy


def _lt_rt_held(**fields) -> "SimJoy":
    """LT+RT 按下（< -0.5），供切策略边沿。"""
    msg = SimJoy()
    msg.lt = -1.0
    msg.rt = -1.0
    for k, v in fields.items():
        setattr(msg, k, float(v))
    return msg


def _pulse(pub, settle: float = 1.7, **fields) -> None:
    pub.publish(_lt_rt_held(**fields))
    time.sleep(0.15)
    pub.publish(_lt_rt_held())
    time.sleep(settle)


def ensure_walk_policy(
    joy_pub,
    target: str = WALK_POLICY_FOLLOW,
    *,
    max_steps: int = 24,
    force_standby_lb: bool = False,
    dry_run: bool = False,
) -> bool:
    """
    将运控切到 target walk 策略（默认 amp_right_hold）。

    要求：joy_pub 已创建；机器人站稳、处于 STANDBY；不要在行走中途调用。
    返回是否确认切到 target（以 policy name 为准）。
    """
    if dry_run:
        print(f"\033[93m[POLICY]\033[0m DRY-RUN：跳过切换，目标仍为 {target}")
        return True
    if joy_pub is None or SimJoy is None:
        print("\033[91m[POLICY]\033[0m 无 /joy_msg，无法切策略")
        return False

    tracker = PolicyTracker()
    fsm = {"v": None}

    def _on_fsm(msg: Int32) -> None:
        fsm["v"] = int(msg.data)

    fsm_sub = rospy.Subscriber(FSM_TOPIC, Int32, _on_fsm, queue_size=5)
    time.sleep(0.4)

    for _ in range(5):
        joy_pub.publish(_lt_rt_held())
        time.sleep(0.05)

    # 仅在明确要求时发 LB（已在 RUNNING 时退回 STANDBY）。
    # 若已在 STANDBY，误发 LB 会进 RUNNING，导致无法切策略。
    if force_standby_lb:
        print("\033[93m[POLICY]\033[0m pulse LB → 尝试退回 STANDBY")
        _pulse(pub=joy_pub, lb=1.0, settle=1.8)

    if tracker.policy == target:
        print(f"\033[92m[POLICY]\033[0m 已是 {target}（无需切换）")
        _cleanup(fsm_sub, tracker)
        return True

    print(
        f"\033[93m[POLICY]\033[0m 当前={tracker.policy!r} → {target} (fsm={fsm['v']})"
    )

    for i in range(max_steps):
        if rospy.is_shutdown():
            break
        if tracker.policy == target:
            break
        before = time.time()
        # dpad_horizontal: -1 = Next（default_controller.cpp）
        print(f"\033[93m[POLICY]\033[0m Next ({i + 1}/{max_steps})")
        _pulse(joy_pub, dpad_horizontal=-1.0, settle=1.7)
        tracker.wait_update(since=before, timeout=2.5)
        print(
            f"\033[90m[POLICY]\033[0m 现在 policy={tracker.policy!r} "
            f"ctrl={tracker.ctrl_group!r}"
        )
        if tracker.policy == target:
            break

    ok = tracker.policy == target
    if ok:
        print(f"\033[92m[POLICY]\033[0m 已切换到 {target}")
    else:
        print(
            f"\033[91m[POLICY]\033[0m 未能确认 {target}，最后 "
            f"policy={tracker.policy!r}。请确认已 STANDBY 且 launch 已注册该策略。"
        )

    _cleanup(fsm_sub, tracker)
    return ok


def _cleanup(fsm_sub, tracker: PolicyTracker) -> None:
    try:
        fsm_sub.unregister()
    except Exception:
        pass
    try:
        tracker._sub.unregister()
    except Exception:
        pass
