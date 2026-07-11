#!/usr/bin/env python3
"""
Enter RUNNING via synthetic /joy_msg, then immediately walk forward briefly
before RUNNING can auto-revert to STANDBY (observed to happen ~7s after
entering RUNNING if idle).

Combines unlock_running.py's RUNNING-entry sequence with an immediate
/cmd_vel publish the instant "sim2real switch to running state" is seen
on /rosout — no gap between confirming RUNNING and moving, since a
separate confirm-then-move script would likely miss the window.

Also sets linear.z = 1.0 on every /cmd_vel message (not just 0.0 default),
matching what a teammate's mode_arbiter.py (on a different robot unit)
hardcodes on every leg command — this matches our own earlier finding
that the real joystick's idle trigger axis always publishes linear.z=1.0,
never 0, and the low-level controller may depend on that.

Usage:
  python3 unlock_and_walk.py --walk-sec 1.0 --linear-x 0.08
"""

from __future__ import annotations

import argparse
import re
import time
from pathlib import Path
from typing import Optional

import rospy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Log
from std_msgs.msg import Int32

try:
    from sim2real_msg.msg import Joy as SimJoy
except ImportError as exc:
    print("需要 sim2real_msg (source ~/sim2real/install/setup.bash)")
    raise SystemExit(1) from exc

FSM_TOPIC = "/fsm_state"
JOY_TOPIC = "/joy_msg"
ROSOUT_TOPIC = "/rosout"
CMD_VEL_TOPIC = "/cmd_vel"

FSM_PROTECTION_SHUTDOWN = 8
_RUNNING_RE = "sim2real switch to running state"
_POLICY_RE = re.compile(r"policy name:\s*\[([^\]]+)\]")


def read_last_policy_from_log() -> Optional[str]:
    for path in (
        Path.home() / ".ros" / "log" / "latest" / "rosout.log",
        Path("/home/nvidia/.ros/log/latest/rosout.log"),
    ):
        try:
            if not path.is_file():
                continue
            text = path.read_bytes()[-400_000:].decode("utf-8", errors="ignore")
            matches = _POLICY_RE.findall(text)
            if matches:
                return matches[-1].strip()
        except OSError:
            continue
    return None


class State:
    def __init__(self) -> None:
        self.fsm: Optional[int] = None
        self.running_hint = False
        self.running_hint_at = 0.0
        self.policy: Optional[str] = read_last_policy_from_log()
        self.policy_updated_at = 0.0
        rospy.Subscriber(FSM_TOPIC, Int32, self._on_fsm, queue_size=5)
        rospy.Subscriber(ROSOUT_TOPIC, Log, self._on_log, queue_size=50)

    def _on_fsm(self, msg: Int32) -> None:
        self.fsm = int(msg.data)

    def _on_log(self, msg: Log) -> None:
        text = msg.msg or ""
        if _RUNNING_RE in text:
            self.running_hint = True
            self.running_hint_at = time.time()
        m = _POLICY_RE.search(text)
        if m:
            self.policy = m.group(1).strip()
            self.policy_updated_at = time.time()


def make_joy(**fields) -> "SimJoy":
    msg = SimJoy()
    for k, v in fields.items():
        setattr(msg, k, float(v))
    return msg


def publish_cmd(cmd_pub, joy_pub, linear_x: float, angular_z: float = 0.0) -> None:
    # Matches mode_arbiter.py's _publish_legs(): publish BOTH /joy_msg
    # (l_vertical/r_horizontal, mimicking the physical left stick) AND
    # /cmd_vel simultaneously on every cycle, not just /cmd_vel alone.
    joy_pub.publish(make_joy(l_vertical=linear_x, r_horizontal=angular_z))
    twist = Twist()
    twist.linear.x = float(linear_x)
    twist.linear.z = 1.0  # match teammate's mode_arbiter.py / real joystick idle baseline
    twist.angular.z = float(angular_z)
    cmd_pub.publish(twist)


def pulse(joy_pub, settle: float, **fields) -> None:
    joy_pub.publish(make_joy(lt=-1.0, rt=-1.0, **fields))
    time.sleep(0.15)
    joy_pub.publish(make_joy(lt=-1.0, rt=-1.0))
    time.sleep(settle)


def ensure_walk_policy(joy_pub, state: State, target: str, max_steps: int) -> bool:
    if state.policy == target:
        print(f"[POLICY] already {target}")
        return True
    print(f"[POLICY] current={state.policy!r} -> target={target!r} fsm={state.fsm}")
    for _ in range(5):
        joy_pub.publish(make_joy(lt=-1.0, rt=-1.0))
        time.sleep(0.05)
    for i in range(max_steps):
        if state.policy == target:
            break
        before = time.time()
        print(f"[POLICY] Next ({i + 1}/{max_steps})")
        pulse(joy_pub, settle=1.7, dpad_horizontal=-1.0)
        deadline = time.time() + 2.5
        while time.time() < deadline:
            if state.policy_updated_at >= before and state.policy:
                break
            time.sleep(0.05)
        print(f"[POLICY] now policy={state.policy!r}")
    ok = state.policy == target
    print(f"[POLICY] {'OK' if ok else 'FAILED'}: policy={state.policy!r}")
    return ok


def enter_running_and_walk(
    joy_pub, cmd_pub, state: State, walk_sec: float, linear_x: float, wait_timeout: float
) -> bool:
    since = time.time()
    state.running_hint = False
    print("[RUNNING] hold LT+RT, pulse LB ...")
    for _ in range(10):
        joy_pub.publish(make_joy(lt=-1.0, rt=-1.0, lb=0.0))
        time.sleep(0.02)
    joy_pub.publish(make_joy(lt=-1.0, rt=-1.0, lb=1.0))
    time.sleep(0.05)
    for _ in range(5):
        joy_pub.publish(make_joy(lt=-1.0, rt=-1.0, lb=0.0))
        time.sleep(0.02)
    for _ in range(5):
        joy_pub.publish(make_joy(lt=0.0, rt=0.0, lb=0.0))
        time.sleep(0.02)

    print(f"[RUNNING] waiting up to {wait_timeout:.1f}s for confirmation ...")
    t0 = time.time()
    while time.time() - t0 < wait_timeout:
        if state.running_hint and state.running_hint_at >= since:
            break
        time.sleep(0.02)

    if not (state.running_hint and state.running_hint_at >= since):
        print("[RUNNING] NOT CONFIRMED — not attempting to walk")
        return False

    print("[RUNNING] CONFIRMED — walking forward now")
    rate_dt = 1.0 / 20.0
    end = time.time() + walk_sec
    while time.time() < end:
        publish_cmd(cmd_pub, joy_pub, linear_x)
        time.sleep(rate_dt)
    publish_cmd(cmd_pub, joy_pub, 0.0)
    print("[WALK] done, zeroed velocity")
    return True


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Enter RUNNING then immediately walk forward briefly")
    p.add_argument("--walk-sec", type=float, default=1.0)
    p.add_argument("--linear-x", type=float, default=0.08)
    p.add_argument("--wait-timeout", type=float, default=3.0, help="max wait for RUNNING confirmation")
    p.add_argument("--skip-policy-switch", action="store_true")
    p.add_argument("--target-policy", default="amp_right_hold")
    p.add_argument("--policy-max-steps", type=int, default=24)
    return p


def main() -> None:
    args = build_parser().parse_args()
    rospy.init_node("unlock_and_walk", anonymous=True)
    joy_pub = rospy.Publisher(JOY_TOPIC, SimJoy, queue_size=10)
    cmd_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
    state = State()
    time.sleep(0.5)

    print(f"[STATE] fsm={state.fsm} policy={state.policy!r}")
    if state.fsm == FSM_PROTECTION_SHUTDOWN:
        print("[ABORT] fsm=8 PROTECTION_SHUTDOWN")
        raise SystemExit(1)

    if not args.skip_policy_switch:
        if not ensure_walk_policy(joy_pub, state, args.target_policy, args.policy_max_steps):
            print("[ABORT] policy switch failed")
            raise SystemExit(1)

    ok = enter_running_and_walk(
        joy_pub, cmd_pub, state, args.walk_sec, args.linear_x, args.wait_timeout
    )
    if not ok:
        publish_cmd(cmd_pub, joy_pub, 0.0)
        raise SystemExit(1)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
