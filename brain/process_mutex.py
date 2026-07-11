# -*- coding: utf-8 -*-
"""跟随控制源互斥：避免 mode_arbiter 与 uwb_follow 双写 /cmd_vel。"""

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
    检测是否有其它跟随控制源在跑。
    返回 (ok, warnings)。ok=False 表示强烈建议先停再继续。
    """
    me = self_pid if self_pid is not None else os.getpid()
    warns: List[str] = []

    if _systemctl_active("uwb-follow.service"):
        warns.append(
            "uwb-follow.service 仍为 active → 会与 arbiter 双写 /cmd_vel。"
            "请执行: sudo systemctl stop uwb-follow.service"
        )

    for pat, label in (
        ("uwb_follow.py", "uwb_follow.py"),
        ("moon/brain/mode_arbiter.py", "mode_arbiter.py"),
    ):
        pids = [p for p in _pgrep(pat) if p != me]
        # 自己就是 arbiter 时，忽略自身匹配
        if "mode_arbiter" in pat and not pids:
            continue
        if "uwb_follow" in pat and pids:
            warns.append(
                f"检测到 {label} 进程 pid={pids}，请先停掉再启 arbiter，避免抢控制。"
            )
        if "mode_arbiter" in pat and pids:
            warns.append(
                f"已有其它 mode_arbiter 在跑 pid={pids}，请勿多开。"
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
