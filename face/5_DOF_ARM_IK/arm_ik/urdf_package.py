#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位 PiPlusPro URDF 包内的主 .urdf 文件。"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path
from typing import List, Optional, Union

DEFAULT_PACKAGE_NAME = "PiPlusPro_S_12L10A2G2H1W_ZedMini"
DEFAULT_URDF_NAMES = (
    "PiPlusPro_S_12L10A2G2H1W_ZedMini_260322.urdf",
    "PiPlusPro_S_12L10A2G2H1W_ZedMini.urdf",
    "robot.urdf",
)


def _expand_xacro(xacro_path: Path, out_path: Path) -> bool:
    if not shutil.which("xacro"):
        return False
    try:
        proc = subprocess.run(
            ["xacro", str(xacro_path)],
            check=True,
            capture_output=True,
            text=True,
        )
        out_path.write_text(proc.stdout, encoding="utf-8")
        return True
    except (subprocess.CalledProcessError, OSError):
        return False


def find_urdf_in_package(package_dir: Path) -> Path:
    """
    在 URDF 包目录中查找主 urdf：
      1) urdf/*.urdf（非 .xacro）
      2) 包根目录 *.urdf
      3) 若有 .xacro 且系统有 xacro，生成 urdf/_generated.urdf
    """
    package_dir = package_dir.resolve()
    if not package_dir.is_dir():
        raise FileNotFoundError(f"URDF 包目录不存在: {package_dir}")

    search_dirs: List[Path] = []
    urdf_sub = package_dir / "urdf"
    if urdf_sub.is_dir():
        search_dirs.append(urdf_sub)
    search_dirs.append(package_dir)

    for folder in search_dirs:
        for name in DEFAULT_URDF_NAMES:
            p = folder / name
            if p.is_file():
                return p

    candidates: List[Path] = []
    for folder in search_dirs:
        candidates.extend(sorted(folder.glob("*.urdf")))
    if len(candidates) == 1:
        return candidates[0]
    if len(candidates) > 1:
        for c in candidates:
            if "ZedMini" in c.name or "PiPlusPro" in c.name:
                return c
        raise FileNotFoundError(
            f"目录 {package_dir} 中存在多个 .urdf，请指定文件: "
            + ", ".join(x.name for x in candidates),
        )

    xacro_candidates: List[Path] = []
    for folder in search_dirs:
        xacro_candidates.extend(sorted(folder.glob("*.xacro")))
    if xacro_candidates:
        xacro_main = xacro_candidates[0]
        for x in xacro_candidates:
            if x.stem == package_dir.name or "robot" in x.name.lower():
                xacro_main = x
                break
        gen = urdf_sub / "_generated_from_xacro.urdf" if urdf_sub.is_dir() else package_dir / "_generated_from_xacro.urdf"
        if _expand_xacro(xacro_main, gen):
            return gen
        raise FileNotFoundError(
            f"仅找到 xacro ({xacro_main.name})，请安装 ros-xacro 并执行: "
            f"xacro {xacro_main} > urdf/robot.urdf",
        )

    raise FileNotFoundError(
        f"在 {package_dir} 下未找到 .urdf；请确认已拷贝完整包（含 urdf/ 子目录）",
    )


def resolve_urdf_path(path: Union[str, Path]) -> Path:
    """路径可为 .urdf 文件或包目录。"""
    p = Path(path).resolve()
    if p.is_file() and p.suffix.lower() == ".urdf":
        return p
    if p.is_dir():
        return find_urdf_in_package(p)
    if p.is_file() and p.suffix.lower() == ".xacro":
        out = p.parent / "_generated_from_xacro.urdf"
        if _expand_xacro(p, out):
            return out
    raise FileNotFoundError(f"无效的 URDF 路径: {path}")


def default_urdf_file(project_root: Optional[Path] = None) -> Path:
    root = project_root or Path(__file__).resolve().parents[1]
    return root / "urdf" / DEFAULT_URDF_NAMES[0]


def default_package_dir(project_root: Optional[Path] = None) -> Path:
    return default_urdf_file(project_root).parent
