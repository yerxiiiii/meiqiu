#!/usr/bin/env python3
"""
Switch to a target walk policy, then enter FSM RUNNING sub-state,
via synthetic /joy_msg (sim2real_msg/Joy) — no physical controller needed.

Background: /fsm_state stays at 5 (ExecDefault) for BOTH the STANDBY and
RUNNING sub-states — the numeric topic alone can never tell them apart.
The only way to confirm RUNNING is watching /rosout for the exact line
"sim2real switch to running state". Likewise, confirming a walk-policy
switch means watching /rosout for "policy name: [xxx]".

This mirrors the mechanism found in a teammate's mode_arbiter.py /
policy_switch.py on a different robot unit (212) — reimplemented here
standalone for chris (222), not copied verbatim, since paths/topics/
default policy differ. Two-step sequence:

  1. ensure_walk_policy(): hold LT+RT, pulse D-pad-horizontal ("Next" in
     the FSM's policy menu) repeatedly until /rosout confirms the target
     policy name, or --max-steps is exhausted.
  2. enter_running(): hold LT+RT, pulse LB, release — confirm via
     /rosout's "switch to running state" line.

Usage:
  python3 unlock_running.py --list-only          # just watch/report current policy+fsm, no input sent
  python3 unlock_running.py --target-policy amp_right_hold
  python3 unlock_running.py --skip-policy-switch  # only try entering RUNNING, skip policy cycling
"""

from __future__ import annotations

import argparse
import re
import time
from pathlib import Path
from typing import Optional

import rospy
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

FSM_EXEC_DEFAULT = 5
FSM_PROTECTION_SHUTDOWN = 8

_POLICY_RE = re.compile(r"policy name:\s*\[([^\]]+)\]")
_RUNNING_RE = "sim2real switch to running state"


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
        self.policy: Optional[str] = read_last_policy_from_log()
        self.running_hint = False
        self.running_hint_at = 0.0
        self.policy_updated_at = 0.0
        rospy.Subscriber(FSM_TOPIC, Int32, self._on_fsm, queue_size=5)
        rospy.Subscriber(ROSOUT_TOPIC, Log, self._on_log, queue_size=50)

    def _on_fsm(self, msg: Int32) -> None:
        self.fsm = int(msg.data)

    def _on_log(self, msg: Log) -> None:
        text = msg.msg or ""
        m = _POLICY_RE.search(text)
        if m:
            self.policy = m.group(1).strip()
            self.policy_updated_at = time.time()
        if _RUNNING_RE in text:
            self.running_hint = True
            self.running_hint_at = time.time()


def make_joy(**fields) -> "SimJoy":
    msg = SimJoy()
    for k, v in fields.items():
        setattr(msg, k, float(v))
    return msg


def pulse(pub, settle: float, **fields) -> None:
    pub.publish(make_joy(lt=-1.0, rt=-1.0, **fields))
    time.sleep(0.15)
    pub.publish(make_joy(lt=-1.0, rt=-1.0))
    time.sleep(settle)


def ensure_walk_policy(pub, state: State, target: str, max_steps: int) -> bool:
    if state.policy == target:
        print(f"[POLICY] already {target}")
        return True
    print(f"[POLICY] current={state.policy!r} -> target={target!r} fsm={state.fsm}")
    for _ in range(5):
        pub.publish(make_joy(lt=-1.0, rt=-1.0))
        time.sleep(0.05)
    for i in range(max_steps):
        if state.policy == target:
            break
        before = time.time()
        print(f"[POLICY] Next ({i + 1}/{max_steps})")
        pulse(pub, settle=1.7, dpad_horizontal=-1.0)
        deadline = time.time() + 2.5
        while time.time() < deadline:
            if state.policy_updated_at >= before and state.policy:
                break
            time.sleep(0.05)
        print(f"[POLICY] now policy={state.policy!r}")
    ok = state.policy == target
    print(f"[POLICY] {'OK' if ok else 'FAILED'}: policy={state.policy!r}")
    return ok


def enter_running(pub, state: State, settle: float) -> bool:
    since = time.time()
    state.running_hint = False
    print("[RUNNING] hold LT+RT, pulse LB ...")
    for _ in range(10):
        pub.publish(make_joy(lt=-1.0, rt=-1.0, lb=0.0))
        time.sleep(0.02)
    pub.publish(make_joy(lt=-1.0, rt=-1.0, lb=1.0))
    time.sleep(0.05)
    for _ in range(5):
        pub.publish(make_joy(lt=-1.0, rt=-1.0, lb=0.0))
        time.sleep(0.02)

    t0 = time.time()
    while time.time() - t0 < settle:
        if state.running_hint and state.running_hint_at >= since:
            break
        time.sleep(0.05)

    for _ in range(5):
        pub.publish(make_joy(lt=0.0, rt=0.0, lb=0.0))
        time.sleep(0.02)

    ok = state.running_hint and state.running_hint_at >= since
    print(f"[RUNNING] {'CONFIRMED (rosout: switch to running state)' if ok else 'NOT CONFIRMED'}")
    return ok


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Switch policy + enter RUNNING via synthetic /joy_msg")
    p.add_argument("--target-policy", default="amp_right_hold")
    p.add_argument("--max-steps", type=int, default=24)
    p.add_argument("--settle", type=float, default=1.6)
    p.add_argument("--skip-policy-switch", action="store_true")
    p.add_argument("--list-only", action="store_true", help="just report current fsm/policy, send nothing")
    return p


def main() -> None:
    args = build_parser().parse_args()
    rospy.init_node("unlock_running", anonymous=True)
    pub = rospy.Publisher(JOY_TOPIC, SimJoy, queue_size=10)
    state = State()
    time.sleep(0.5)

    print(f"[STATE] fsm={state.fsm} policy={state.policy!r}")
    if args.list_only:
        return

    if state.fsm == FSM_PROTECTION_SHUTDOWN:
        print("[ABORT] fsm=8 PROTECTION_SHUTDOWN — clear that first, this won't help")
        raise SystemExit(1)

    if not args.skip_policy_switch:
        if not ensure_walk_policy(pub, state, args.target_policy, args.max_steps):
            print("[ABORT] policy switch failed, not attempting RUNNING")
            raise SystemExit(1)

    enter_running(pub, state, args.settle)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
