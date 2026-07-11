#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘控制右臂末端位姿 → IK → 关节角；可选下发实机。

平移（config teleop 实机标定方向，base_link）:
  W/S 前/后    A/D 左/右    Q/E 上/下
  方向键: ↑↓ 前/后  ←→ 左/右  PgUp/PgDn 上/下

旋转 (RPY 增量, 绕 base_link 固定轴):
  7/8  Roll±(X)  4/5  Pitch±(Y)  1/2  Yaw±(Z)
  I/K  Roll±     J/L  Pitch±     U/O  Yaw±

夹爪（实机 r_claw_joint）:
  F     开合切换

其它:
  空格  一键回到站立默认位（standing_home_q，立即下发）
  H     帮助   ESC  退出

IK 原点: 站立默认关节角；解算尽力贴近目标，不丢弃近似解。
"""

from __future__ import annotations

import argparse
import math
import select
import sys
import time
import termios
import tty
from pathlib import Path
from typing import Optional

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from arm_ik import (  # noqa: E402
    IKResult,
    IKTaskMode,
    RightArmIKSolver,
    load_standing_home_q,
    load_teleop_axes,
)
from arm_ik.robot_bridge import RightArmRobotBridge, load_robot_config  # noqa: E402
from arm_ik.transforms import rpy_matrix  # noqa: E402

FRAME_NAME = "base_link"


def matrix_to_rpy_deg(r: np.ndarray) -> tuple[float, float, float]:
    sy = math.sqrt(float(r[0, 0] ** 2 + r[1, 0] ** 2))
    if sy > 1e-6:
        roll = math.atan2(float(r[2, 1]), float(r[2, 2]))
        pitch = math.atan2(float(-r[2, 0]), sy)
        yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
    else:
        roll = math.atan2(float(-r[1, 2]), float(r[1, 1]))
        pitch = math.atan2(float(-r[2, 0]), sy)
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def apply_delta_pose(
    t: np.ndarray,
    dpos: np.ndarray,
    drot_rpy: np.ndarray,
) -> np.ndarray:
    out = t.copy()
    out[:3, 3] += dpos
    r_delta = rpy_matrix(drot_rpy[0], drot_rpy[1], drot_rpy[2])
    out[:3, :3] = r_delta @ out[:3, :3]
    return out


def fk_in_base(solver: RightArmIKSolver, q: np.ndarray, q_waist: float) -> np.ndarray:
    t_torso = solver.fk(q)
    if solver.torso_mount is None:
        return t_torso
    return solver.torso_mount.compute(q_waist) @ t_torso


def ik_from_base_target(
    solver: RightArmIKSolver,
    target_base: np.ndarray,
    q_seed: np.ndarray,
    q_waist: float,
    mode: IKTaskMode,
):
    if solver.torso_mount is None:
        return solver.ik(target_base, q_seed=q_seed, mode=mode)
    target_torso = solver.torso_mount.target_in_torso_frame(target_base, q_waist)
    return solver.ik(target_torso, q_seed=q_seed, mode=mode)


class KeyReader:
    def __init__(self):
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)

    def __enter__(self):
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, *args):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def read_key(self) -> str:
        if not select.select([sys.stdin], [], [], 0.05)[0]:
            return ""
        ch = sys.stdin.read(1)
        if ch != "\x1b":
            return ch
        if not select.select([sys.stdin], [], [], 0.02)[0]:
            return ch
        seq = sys.stdin.read(2)
        return ch + seq


ARROW_MAP = {
    "[A": "left",
    "[B": "down",
    "[C": "right",
    "[D": "up",
    "[5": "pgup",
    "[6": "pgdown",
}

HELP = __doc__


def run_teleop(
    solver: RightArmIKSolver,
    pos_step: float,
    rot_step_deg: float,
    ik_mode: IKTaskMode,
    q_waist: float = 0.0,
    robot: Optional[RightArmRobotBridge] = None,
    ik_origin_q: Optional[np.ndarray] = None,
    teleop_axes: Optional[dict[str, np.ndarray]] = None,
) -> None:
    if ik_origin_q is None:
        raise ValueError("缺少 standing_home_q（IK 站立原点）")
    ik_origin_q = np.asarray(ik_origin_q, dtype=float).copy()
    axes = teleop_axes or load_teleop_axes(
        ROOT / "config" / "right_arm.yaml",
    )
    q = ik_origin_q.copy()

    target_base = fk_in_base(solver, q, q_waist)
    target_cmd = target_base.copy()
    rot_step = math.radians(rot_step_deg)
    last_key_t = 0.0
    key_debounce_s = 0.05
    last_print_t = 0.0
    status_print_interval = 0.2

    if solver.torso_mount is not None:
        _t_tb = np.linalg.inv(solver.torso_mount.compute(q_waist))

        def target_torso_fn(target_base: np.ndarray) -> np.ndarray:
            return _t_tb @ target_base
    else:

        def target_torso_fn(target_base: np.ndarray) -> np.ndarray:
            return target_base

    def reset_to_standing_origin() -> None:
        nonlocal q, target_base, target_cmd
        q = ik_origin_q.copy()
        target_base = fk_in_base(solver, q, q_waist)
        target_cmd = target_base.copy()

    def push_robot(smooth: bool = True) -> None:
        if robot is not None:
            robot.set_arm_goal(q, smooth=smooth)

    def solve_ik_fast(q_seed: np.ndarray) -> IKResult:
        tgt = target_torso_fn(target_cmd)
        return solver.ik(tgt, q_seed=q_seed, mode=ik_mode)

    def do_ik() -> None:
        nonlocal q, last_print_t
        res = solve_ik_fast(q)
        if (
            res.position_error_m > 0.025
            and ik_mode != IKTaskMode.POSITION
        ):
            res_pos = solver.ik(
                target_torso_fn(target_cmd),
                q_seed=res.q,
                mode=IKTaskMode.POSITION,
            )
            if res_pos.position_error_m < res.position_error_m:
                res = res_pos
        q = res.q.copy()
        push_robot(smooth=True)
        now = time.time()
        if now - last_print_t < status_print_interval:
            return
        last_print_t = now
        rpy_cmd = matrix_to_rpy_deg(target_cmd[:3, :3])
        claw_s = ""
        if robot is not None:
            claw_s = f" claw={'开' if robot.claw_is_open else '合'}"
        tag = "OK" if res.message == "converged" else "~"
        print(
            f"\rIK {tag}  "
            f"cmd=[{target_cmd[0,3]:+.3f},{target_cmd[1,3]:+.3f},{target_cmd[2,3]:+.3f}] "
            f"err={res.position_error_m*1000:.1f}mm "
            f"it={res.iterations}{claw_s}   ",
            end="",
            flush=True,
        )

    def go_home_stand() -> None:
        nonlocal q
        reset_to_standing_origin()
        if robot is not None:
            robot.set_arm_goal(q, smooth=False)
        print(
            "\n→ 已回站立默认位 "
            f"q=[{', '.join(f'{v:+.2f}' for v in q)}]",
            end="",
            flush=True,
        )

    def move_axis(direction: str, sign: float = 1.0) -> None:
        vec = axes[direction] * (sign * pos_step)
        move(dpos=vec)

    def move(dpos=None, drot=None) -> None:
        nonlocal target_cmd
        dpos = np.zeros(3) if dpos is None else np.asarray(dpos, float)
        drot = np.zeros(3) if drot is None else np.asarray(drot, float)
        target_cmd = apply_delta_pose(target_cmd, dpos, drot)
        do_ik()

    def accept_key() -> bool:
        nonlocal last_key_t
        now = time.time()
        if now - last_key_t < key_debounce_s:
            return False
        last_key_t = now
        return True

    print(HELP)
    if solver.torso_mount is None:
        print("警告: URDF 无腰-躯干链，base 与 torso 视为同一系")
    if robot is None:
        mode_note = "仅 IK（加 --robot 或去掉 --sim-only 才发实机）"
    elif getattr(robot, "_dry_run", False):
        mode_note = "联调 --dry-run（不发布）"
    else:
        backend = getattr(robot, "effective_backend", "?")
        mode_note = f"实机 → {backend}"
    print(f"{FRAME_NAME} 末端键盘控制已启动 — {mode_note}")
    ax_s = " ".join(
        f"{k}=[{v[0]:+.0f},{v[1]:+.0f},{v[2]:+.0f}]"
        for k, v in axes.items()
    )
    print(f"平移标定(base_link): {ax_s}")
    origin_xyz = target_cmd[:3, 3]
    if robot is not None:
        q_robot = robot.get_arm_q()
        robot.hold_current_pose()
        delta = float(np.linalg.norm(q_robot - ik_origin_q))
        if delta > 0.15:
            print(
                f"提示: 实机关节与 standing_home_q 差 {delta:.2f} rad，"
                "请确认已 Start 站立或更新 config",
            )
    print(
        f"\nIK 原点=站立默认 q=[{', '.join(f'{v:+.2f}' for v in ik_origin_q)}] "
        f"base xyz=[{origin_xyz[0]:+.3f},{origin_xyz[1]:+.3f},{origin_xyz[2]:+.3f}] "
        f"(非 q=0 伸直姿；启动不下发，空格站立复原)",
        flush=True,
    )

    with KeyReader() as keys:
        while True:
            k = keys.read_key()
            if not k:
                continue
            if k in ("\x03", "\x1b"):
                if k == "\x1b":
                    print("\n退出")
                break
            if k in ("h", "H", "?"):
                print(HELP)
                continue
            if k == " ":
                go_home_stand()
                continue
            if k in ("f", "F"):
                if robot is None:
                    print("\n夹爪需实机模式（默认 run 脚本已 --robot）", end="", flush=True)
                    continue
                opened = robot.toggle_claw()
                print(f"\n夹爪 → {'张开' if opened else '闭合'}", end="", flush=True)
                continue

            if not accept_key():
                continue

            arrow = ARROW_MAP.get(k[1:] if k.startswith("\x1b") else "", "")
            if arrow == "up":
                move_axis("forward")
            elif arrow == "down":
                move_axis("back")
            elif arrow == "left":
                move_axis("left")
            elif arrow == "right":
                move_axis("right")
            elif arrow == "pgup":
                move_axis("up")
            elif arrow == "pgdown":
                move_axis("down")
            elif k in ("w", "W"):
                move_axis("forward")
            elif k in ("s", "S"):
                move_axis("back")
            elif k in ("a", "A"):
                move_axis("left")
            elif k in ("d", "D"):
                move_axis("right")
            elif k in ("q", "Q"):
                move_axis("up")
            elif k in ("e", "E"):
                move_axis("down")
            elif k in ("7", "i", "I"):
                move(drot=[rot_step, 0, 0])
            elif k in ("8", "k", "K"):
                move(drot=[-rot_step, 0, 0])
            elif k in ("4", "j", "J"):
                move(drot=[0, rot_step, 0])
            elif k in ("5", "l", "L"):
                move(drot=[0, -rot_step, 0])
            elif k in ("1", "u", "U"):
                move(drot=[0, 0, rot_step])
            elif k in ("2", "o", "O"):
                move(drot=[0, 0, -rot_step])

    if robot is not None:
        robot.stop()


def main():
    parser = argparse.ArgumentParser(description="键盘末端 IK（可选实机）")
    parser.add_argument(
        "--config",
        default=str(ROOT / "config" / "right_arm.yaml"),
    )
    parser.add_argument("--pos-step", type=float, default=0.01, help="平移步长 m")
    parser.add_argument(
        "--rot-step-deg", type=float, default=5.0, help="旋转步长 deg",
    )
    parser.add_argument(
        "--q-waist",
        type=float,
        default=0.0,
        help="当前 waist_yaw_joint 角 (rad)，用于 base↔torso 变换",
    )
    parser.add_argument(
        "--ik-mode",
        choices=("tool_z", "full", "position"),
        default="tool_z",
        help="IK: full=位置+姿态, tool_z=位置+末端Z, position=仅位置",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--robot",
        action="store_true",
        help="向实机发布（run_keyboard_demo.sh 默认已开启）",
    )
    mode.add_argument(
        "--sim-only",
        action="store_true",
        help="仅 IK，不连接 ROS / 不发实机",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="连接 ROS 但不发布 JointState",
    )
    parser.add_argument(
        "--no-fsm",
        action="store_true",
        help="不检查 FSM=EXEC_DEFAULT(5)",
    )
    parser.add_argument(
        "--develop",
        action="store_true",
        help="等待 FSM=EXEC_DEVELOP(16) 再连 lowlevel（完整控臂，推荐）",
    )
    args = parser.parse_args()

    mode_map = {
        "tool_z": IKTaskMode.POSITION_TOOL_Z,
        "full": IKTaskMode.POSITION_ORIENTATION,
        "position": IKTaskMode.POSITION,
    }
    solver = RightArmIKSolver.from_config(args.config)
    ik_origin_q = load_standing_home_q(args.config)
    teleop_axes = load_teleop_axes(args.config)

    robot_bridge: Optional[RightArmRobotBridge] = None
    use_robot = args.robot or not args.sim_only
    if use_robot:
        import rospy

        rospy.init_node("arm_ik_keyboard_teleop", anonymous=True)
        rb_cfg = load_robot_config(args.config)
        robot_bridge = RightArmRobotBridge(
            solver.joint_names,
            rb_cfg,
            dry_run=args.dry_run,
            check_fsm=not args.no_fsm,
            prefer_develop=args.develop,
        )
        ready = robot_bridge.wait_for_ready()
        if not args.dry_run:
            be = robot_bridge.effective_backend
            if ready:
                if be == "lowlevel":
                    print("实机就绪: lowlevel 双臂；按 F 切换夹爪")
                else:
                    print(
                        "实机就绪: 仅右臂 → /pi_plus_absolute；"
                        "请先手柄 Start 进入站立 (fsm_state=5)",
                    )
            else:
                print(
                    "请先 Start 站立: rostopic echo /fsm_state 应为 5",
                )
        else:
            print("dry-run: 已连 ROS，不发布关节指令")

    try:
        run_teleop(
            solver,
            pos_step=args.pos_step,
            rot_step_deg=args.rot_step_deg,
            ik_mode=mode_map[args.ik_mode],
            q_waist=args.q_waist,
            robot=robot_bridge,
            ik_origin_q=ik_origin_q,
            teleop_axes=teleop_axes,
        )
    finally:
        if robot_bridge is not None:
            robot_bridge.stop()


if __name__ == "__main__":
    main()
