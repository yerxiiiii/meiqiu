# -*- coding: utf-8 -*-
"""跟随/运动控制源互斥：避免多个节点双写 /cmd_vel、/joy_msg。"""

from __future__ import annotations

import os
import subprocess
from typing import List, Optional, Tuple


def _pgrep(pattern: str) -> List[int]:
    try:
        out = subprocess.check_output(
            ["pgrep", "-f", pattern], text=True, stderr=subprocess.DEVNULL
        )
        return [int(x) for x in out.split() if x.strip().isdigit()]
    except (subprocess.CalledProcessError, FileNotFoundError, ValueError):
        return []


def _systemctl_active(unit: str) -> bool:
    try:
        out = subprocess.check_output(
            ["systemctl", "is-active", unit],
            text=True,
            stderr=subprocess.DEVNULL,
        )
        return out.strip() == "active"
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False


def check_follow_conflicts(self_pid: Optional[int] = None) -> Tuple[bool, List[str]]:
    """
    检测是否有其它跟随/运动控制源在跑。
    返回 (ok, warnings)。ok=False 表示强烈建议先停再继续。
    """
    me = self_pid if self_pid is not None else os.getpid()
    warns: List[str] = []

    # systemd units that publish motion
    for unit, tip in (
        (
            "uwb-follow.service",
            "uwb-follow.service 仍为 active → 会与 arbiter 双写 /cmd_vel。"
            "请执行: sudo systemctl stop uwb-follow.service"
            " && sudo systemctl disable uwb-follow.service",
        ),
    ):
        if _systemctl_active(unit):
            warns.append(tip)

    # process patterns that publish /cmd_vel or /joy_msg
    checks = (
        ("uwb_follow.py", "uwb_follow.py（独立跟随）"),
        ("moon/brain/mode_arbiter.py", "mode_arbiter.py"),
        ("person_follow_ros1.py", "person_follow_ros1.py（视觉跟随）"),
        ("guide_demo_node.py", "guide_demo_node.py（固定路线带路）"),
        ("keyboard_teleop.py", "keyboard_teleop.py"),
        ("unlock_and_walk.py", "unlock_and_walk.py"),
    )
    for pat, label in checks:
        pids = [p for p in _pgrep(pat) if p != me]
        if not pids:
            continue
        if "mode_arbiter" in pat:
            warns.append(f"已有其它 mode_arbiter 在跑 pid={pids}，请勿多开。")
        else:
            warns.append(
                f"检测到 {label} 进程 pid={pids}，请先停掉再启 arbiter，避免抢 /cmd_vel。"
            )

    return (len(warns) == 0), warns


def print_conflict_report(ok: bool, warns: List[str], *, fatal: bool = False) -> None:
    if ok:
        print("\033[92m[MUTEX]\033[0m 未发现跟随双写冲突")
        return
    for w in warns:
        print(f"\033[91m[MUTEX]\033[0m {w}")
    if fatal:
        print("\033[91m[MUTEX]\033[0m 已中止启动（可用 --ignore-mutex 强制）")
    else:
        print("\033[93m[MUTEX]\033[0m 继续运行有风险，建议先处理上述冲突")
