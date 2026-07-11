#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""解析 URDF 并打印右臂 5 关节链、零位 FK。"""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from arm_ik.right_arm_ik import RightArmIKSolver, load_standing_home_q  # noqa: E402
from arm_ik.urdf_package import default_urdf_file, resolve_urdf_path  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="检查右臂 URDF 运动学链")
    parser.add_argument(
        "urdf",
        nargs="?",
        default=str(default_urdf_file(ROOT)),
        help="URDF 文件路径",
    )
    parser.add_argument(
        "--config",
        default=str(ROOT / "config" / "right_arm.yaml"),
        help="关节名配置",
    )
    args = parser.parse_args()

    urdf_file = resolve_urdf_path(args.urdf)
    print(f"[inspect] URDF: {urdf_file}")

    solver = RightArmIKSolver.from_urdf(urdf_file, config_path=args.config)
    print("base_link:", solver.chain.base_link)
    print("ee_link:", solver.chain.ee_link)
    print("joints:", solver.joint_names)
    lo, hi = solver.chain.limits()
    for i, name in enumerate(solver.joint_names):
        j = solver.chain.joints[i]
        print(
            f"  [{i}] {name}: parent={j.parent} child={j.child} "
            f"axis={j.axis} origin_xyz={j.origin_xyz} "
            f"limits=[{lo[i]:.3f}, {hi[i]:.3f}]",
        )
    q0 = (lo + hi) * 0.5
    t = solver.fk(q0)
    print("FK @ q_mid xyz:", t[:3, 3])
    import numpy as np

    qz = np.zeros(5)
    print("FK @ q=0 伸直 xyz (torso):", solver.fk(qz)[:3, 3])
    try:
        qs = load_standing_home_q(args.config)
        print(
            "FK @ standing_home_q 站立 xyz (torso):",
            solver.fk(qs)[:3, 3],
            "q=", [round(v, 3) for v in qs],
        )
    except KeyError as e:
        print("standing_home_q:", e)
    if solver.torso_mount is not None:
        t_bt = solver.torso_mount.compute(0.0)
        print("T_base_torso @ waist=0, xyz:", t_bt[:3, 3])


if __name__ == "__main__":
    main()
