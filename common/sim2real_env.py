# -*- coding: utf-8 -*-
"""Resolve sim2real workspace path portably across machines.

Priority:
  1. $SIM2REAL_WS
  2. ~/sim2real
  3. ~/sim2real_master-feature-master_and_slave  (legacy long name)
  4. /home/nvidia/sim2real
  5. /home/nvidia/sim2real_master-feature-master_and_slave

Prefer creating a symlink:  ln -s <your_ws> ~/sim2real
or export SIM2REAL_WS=/path/to/ws
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import List


def _candidates() -> List[Path]:
    home = Path.home()
    out: List[Path] = []
    env = os.environ.get("SIM2REAL_WS", "").strip()
    if env:
        out.append(Path(env).expanduser())
    out.extend(
        [
            home / "sim2real",
            home / "sim2real_master-feature-master_and_slave",
            Path("/home/nvidia/sim2real"),
            Path("/home/nvidia/sim2real_master-feature-master_and_slave"),
        ]
    )
    # dedupe while preserving order
    seen = set()
    uniq: List[Path] = []
    for p in out:
        key = str(p.resolve()) if p.exists() else str(p)
        if key in seen:
            continue
        seen.add(key)
        uniq.append(p)
    return uniq


def resolve_sim2real_ws() -> Path:
    """Return first workspace that has install/ or devel/ setup.bash."""
    for ws in _candidates():
        if (ws / "install" / "setup.bash").is_file() or (
            ws / "devel" / "setup.bash"
        ).is_file():
            return ws
    # Fallback: preferred short name (caller may still fail clearly)
    return Path.home() / "sim2real"


def resolve_setup_bash() -> Path:
    """Path to install/setup.bash (or devel if install missing)."""
    ws = resolve_sim2real_ws()
    install = ws / "install" / "setup.bash"
    if install.is_file():
        return install
    devel = ws / "devel" / "setup.bash"
    if devel.is_file():
        return devel
    return install


def source_setup_cmd() -> str:
    """Shell fragment: source <setup.bash>"""
    return f"source {resolve_setup_bash()}"


def joy_teleop_restore_cmd() -> str:
    """Shell command used after killing joy_teleop."""
    return (
        f"{source_setup_cmd()} && "
        "roslaunch sim2real_master joy_teleop.launch use_filter:=true &"
    )


def moon_root() -> Path:
    """moon repo root (parent of common/)."""
    return Path(__file__).resolve().parent.parent
