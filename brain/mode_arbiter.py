#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
中央决策节点：语音切模式，唯一下发头/腿指令。

  /moon/voice_cmd  → 模式切换
  /moon/face       → FACE_LOOK 时控头
  UWB 串口         → UWB_FOLLOW 时算意图
  /moon/obstacle   → 门控（默认关；联调加 --enable-obstacle-gate）

运行：
  source .../install/setup.bash
  sudo systemctl stop uwb-follow.service   # 避免双写
  python3 /home/nvidia/moon/brain/mode_arbiter.py --dry-run
  python3 /home/nvidia/moon/brain/mode_arbiter.py
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from typing import Optional

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String

_BRAIN = os.path.dirname(os.path.abspath(__file__))
_MOON = os.path.dirname(_BRAIN)
_VISION = os.path.join(_MOON, "vision")
for p in (_MOON, _BRAIN, _VISION):
    if p not in sys.path:
        sys.path.insert(0, p)

from modes import (  # noqa: E402
    FACE_LAYOUT_LEN,
    TOPIC_CAMERA_OWNER,
    TOPIC_FACE,
    TOPIC_MODE,
    TOPIC_OBSTACLE,
    TOPIC_VOICE_CMD,
    VOICE_CMD_PREPARE_FOLLOW,
    VOICE_TO_MODE,
    Mode,
)
from camera_owner import CameraOwner  # noqa: E402
from joy_monitor import JoyMonitor  # noqa: E402
from neck_control import (  # noqa: E402
    ABSOLUTE_TOPIC,
    HEAD_PITCH_JOINT,
    HEAD_YAW_JOINT,
    NeckServo,
)
from uwb_intent import (  # noqa: E402
    CMD_VEL_X_SCALE,
    CMD_VEL_YAW_SCALE,
    UWB_TIMEOUT,
    FollowController,
    UWBParser,
    apply_soft_scales,
    find_uwb_port,
    try_open_serial,
)
from obstacle_state import ObstacleState  # noqa: E402
from safety_gate import SIDESTEP_BLEND, apply_safety_gate  # noqa: E402
from person_mask import estimate_person_center_m  # noqa: E402
from policy_switch import (  # noqa: E402
    WALK_POLICY_DEFAULT,
    WALK_POLICY_FOLLOW,
    ensure_walk_policy,
)
from fsm_guard import FsmGuard  # noqa: E402
from process_mutex import check_follow_conflicts, print_conflict_report  # noqa: E402

try:
    from sim2real_msg.msg import Joy as SimJoy
except ImportError:
    SimJoy = None

CONTROL_HZ = 50.0
# 关闭上层 soft/hold：交给运控 amp_right_hold 的 cmd_vel_filter_scale（与遥控同路）
SOFT_START_SEC = 0.0
POST_RUNNING_HOLD_SEC = 0.0
OBSTACLE_TIMEOUT = 0.8
# 与 uwb_follow 对齐：先测跟随默认关门控；联调用 --enable-obstacle-gate
ENABLE_OBSTACLE_GATE = False
OBSTACLE_REQUIRED = False
USE_OBSTACLE_SIDESTEP = True
AUTO_ENTER_RUNNING = True
FOLLOW_WALK_POLICY = WALK_POLICY_FOLLOW
ENTER_RUNNING_SETTLE_SEC = 1.6
POLICY_SETTLE_SEC = 0.8
# 误拨到 standby 后等站稳再二次 LB
STANDBY_RECOVER_SEC = 2.0


class ModeArbiter:
    def __init__(self, dry_run: bool = False, manage_camera: bool = True):
        self.dry_run = dry_run
        self.mode = Mode.IDLE
        self.neck = NeckServo()
        self.follow = FollowController()
        self.joy_mon = JoyMonitor()
        self.cam = CameraOwner(enabled=manage_camera and not dry_run)
        self.fsm = FsmGuard()

        self._face = {"dx": 0.0, "dy": 0.0, "has": False, "valid": False, "t": 0.0}
        self._obs = ObstacleState(valid=False)
        self._obs_t = 0.0

        self._ser = None
        self._parser: Optional[UWBParser] = None
        self._uwb_data = None
        self._uwb_t = 0.0
        self._teleop_killed = False
        self._running_sent = False
        self._motion_armed = False
        self._policy_ready = False
        self._policy_retry_at = 0.0
        self._policy_lock = threading.Lock()
        self._policy_seq = 0
        self._follow_policy_prepared = False
        self._soft_t0: Optional[float] = None
        self._hold_until: Optional[float] = None  # 进 RUNNING 后零速截止时刻
        self._last_loop = time.time()
        self._print_i = 0

        self.mode_pub = rospy.Publisher(TOPIC_MODE, String, queue_size=1, latch=True)
        self.cam_pub = rospy.Publisher(
            TOPIC_CAMERA_OWNER, String, queue_size=1, latch=True
        )
        self.neck_pub = rospy.Publisher(ABSOLUTE_TOPIC, JointState, queue_size=10)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.joy_pub = None
        if SimJoy is not None:
            self.joy_pub = rospy.Publisher("/joy_msg", SimJoy, queue_size=10)

        rospy.Subscriber(TOPIC_VOICE_CMD, String, self._on_voice, queue_size=5)
        rospy.Subscriber(TOPIC_FACE, Float32MultiArray, self._on_face, queue_size=1)
        rospy.Subscriber(
            TOPIC_OBSTACLE, Float32MultiArray, self._on_obstacle, queue_size=1
        )

        self._publish_mode()
        self.cam.apply_mode(self.mode)
        self._publish_cam_owner()

    def _publish_mode(self) -> None:
        self.mode_pub.publish(String(data=self.mode.value))

    def _publish_cam_owner(self) -> None:
        owner = self.cam.owner or "none"
        self.cam_pub.publish(String(data=owner))

    def _bump_policy_seq(self) -> int:
        self._policy_seq += 1
        return self._policy_seq

    def _on_voice(self, msg: String) -> None:
        key = (msg.data or "").strip().lower()
        if key == VOICE_CMD_PREPARE_FOLLOW:
            print(
                "\033[90m[VOICE]\033[0m prepare_follow 已弃用（开麦不再预切策略）"
            )
            return
        new_mode = VOICE_TO_MODE.get(key)
        if new_mode is None:
            print(f"\033[93m[VOICE]\033[0m 未知命令: {msg.data!r}")
            return
        if new_mode == self.mode:
            print(f"\033[90m[VOICE]\033[0m 已在模式 {self.mode.value}")
            return
        print(
            f"\033[92m[VOICE]\033[0m {msg.data} → {self.mode.value} → {new_mode.value}"
        )
        self._leave_mode(self.mode)
        self.mode = new_mode
        self._enter_mode(new_mode)
        self._publish_mode()

    def _switch_walk_policy(self, target: str, *, label: str = "") -> bool:
        if self.joy_pub is None:
            return False
        tag = f" ({label})" if label else ""
        print(f"\033[93m[POLICY]\033[0m 请求切换 → {target}{tag}", flush=True)
        with self._policy_lock:
            return ensure_walk_policy(
                self.joy_pub,
                target,
                dry_run=self.dry_run,
            )

    def _restore_default_policy(self, *, label: str = "idle") -> bool:
        """同步恢复 amp（关麦/停跟随时必须完成，不能异步被 joy_teleop 打断）。"""
        if self.dry_run or self.joy_pub is None:
            return True
        self._bump_policy_seq()
        if not self._teleop_killed:
            self._kill_teleop()
        ok = self._switch_walk_policy(WALK_POLICY_DEFAULT, label=label)
        self._follow_policy_prepared = False
        return ok

    def _restore_default_policy_async(self) -> None:
        if self.dry_run or self.joy_pub is None:
            return
        seq = self._bump_policy_seq()

        def _run() -> None:
            if seq != self._policy_seq:
                print(
                    "\033[90m[POLICY]\033[0m 跳过后台恢复 amp（已有新的策略操作）",
                    flush=True,
                )
                return
            if not self._teleop_killed:
                self._kill_teleop()
            self._switch_walk_policy(WALK_POLICY_DEFAULT, label="stop/idle async")
            self._follow_policy_prepared = False

        threading.Thread(target=_run, daemon=True).start()

    def _leave_mode(self, mode: Mode) -> None:
        if mode == Mode.UWB_FOLLOW:
            self._publish_legs(0.0, 0.0)
            self._close_uwb()
            self._running_sent = False
            self._motion_armed = False
            had_follow_policy = self._policy_ready
            self._policy_ready = False
            self._soft_t0 = None
            self._hold_until = None
            self._follow_policy_prepared = False
            if had_follow_policy and not self.dry_run:
                ok = self._restore_default_policy(label="leave UWB_FOLLOW")
                if not ok:
                    print(
                        "\033[91m[POLICY]\033[0m 同步恢复 amp 失败，后台重试",
                        flush=True,
                    )
                    self._restore_default_policy_async()
            if self._teleop_killed and not self.dry_run:
                self._restore_teleop()
        if mode == Mode.FACE_LOOK:
            self.neck.reset_home()
            self._publish_neck(0.0, 0.0)

    def _enter_mode(self, mode: Mode) -> None:
        self.cam.apply_mode(mode)
        self._publish_cam_owner()
        if mode == Mode.UWB_FOLLOW:
            self._bump_policy_seq()
            # 先停 joy_teleop，避免它 50Hz 覆盖 arbiter 的 dpad 脉冲导致策略切不动
            if not self.dry_run:
                self._kill_teleop()
            ok = self._switch_walk_policy(
                FOLLOW_WALK_POLICY, label="enter UWB_FOLLOW"
            )
            self._policy_ready = ok
            self._follow_policy_prepared = ok
            self._policy_retry_at = time.time()
            if ok and not self.dry_run:
                time.sleep(POLICY_SETTLE_SEC)
            if not ok and not self.dry_run:
                print(
                    f"\033[91m[POLICY]\033[0m 未切到 {FOLLOW_WALK_POLICY}，"
                    "拒绝自动进 RUNNING（请手柄确认 STANDBY 后重试口令）"
                )
        if mode == Mode.IDLE:
            self._publish_legs(0.0, 0.0)
            self._publish_neck(0.0, 0.0)

    def _abort_follow(self, reason: str) -> None:
        print(f"\033[91m[SAFE]\033[0m 退出跟随: {reason}")
        self._publish_legs(0.0, 0.0)
        self._leave_mode(Mode.UWB_FOLLOW)
        self.mode = Mode.IDLE
        self._publish_mode()
        self.cam.apply_mode(Mode.IDLE)
        self._publish_cam_owner()

    def _on_face(self, msg: Float32MultiArray) -> None:
        d = msg.data
        if len(d) < FACE_LAYOUT_LEN:
            return
        self._face = {
            "dx": float(d[0]),
            "dy": float(d[1]),
            "has": bool(d[2] >= 0.5),
            "valid": bool(d[3] >= 0.5),
            "t": time.time(),
        }

    def _on_obstacle(self, msg: Float32MultiArray) -> None:
        self._obs = ObstacleState.from_list(msg.data, stamp=time.time())
        self._obs_t = time.time()

    def _kill_teleop(self) -> None:
        import subprocess

        subprocess.run(["rosnode", "kill", "/joy_teleop"], capture_output=True)
        time.sleep(0.5)
        self._teleop_killed = True
        print("\033[93m[CTRL]\033[0m 已停 joy_teleop")

    def _restore_teleop(self) -> None:
        import subprocess

        try:
            from common.sim2real_env import joy_teleop_restore_cmd
            setup = joy_teleop_restore_cmd()
        except Exception:
            setup = (
                "source /home/nvidia/sim2real/install/setup.bash && "
                "roslaunch sim2real_master joy_teleop.launch use_filter:=true &"
            )
        subprocess.Popen(
            ["bash", "-c", setup],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self._teleop_killed = False
        print("\033[93m[CTRL]\033[0m 正在恢复 joy_teleop")

    def _close_uwb(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self._parser = None
        self._uwb_data = None

    def _ensure_uwb(self) -> bool:
        if self._parser is not None:
            return True
        port = find_uwb_port()
        if port is None:
            return False
        ser = try_open_serial(port)
        if ser is None:
            return False
        self._ser = ser
        self._parser = UWBParser(ser)
        print(f"\033[92m[UWB]\033[0m 已连接 {port}")
        return True

    def _publish_neck(self, yaw: float, pitch: float) -> None:
        if self.dry_run:
            return
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [HEAD_YAW_JOINT, HEAD_PITCH_JOINT]
        msg.position = [float(yaw), float(pitch)]
        self.neck_pub.publish(msg)

    def _publish_legs(
        self, fwd: float, rot: float, lb: float = 0.0, lt: float = 0.0, rt: float = 0.0
    ) -> None:
        if self.dry_run:
            return
        if self.joy_pub is not None:
            joy = SimJoy()
            joy.lt = float(lt)
            joy.rt = float(rt)
            joy.l_vertical = float(fwd)
            joy.r_horizontal = float(rot)
            joy.lb = float(lb)
            self.joy_pub.publish(joy)
        twist = Twist()
        twist.linear.x = float(fwd) * CMD_VEL_X_SCALE
        twist.linear.z = 1.0
        twist.angular.z = float(rot) * CMD_VEL_YAW_SCALE
        self.cmd_pub.publish(twist)

    def _lb_edge_pulse(self) -> None:
        """LT+RT 按住后点一次 LB（运控 STANDBY↔RUNNING 切换边沿）。"""
        for _ in range(10):
            self._publish_legs(0.0, 0.0, lb=0.0, lt=-1.0, rt=-1.0)
            time.sleep(0.02)
        self._publish_legs(0.0, 0.0, lb=1.0, lt=-1.0, rt=-1.0)
        time.sleep(0.08)
        for _ in range(5):
            self._publish_legs(0.0, 0.0, lb=0.0, lt=-1.0, rt=-1.0)
            time.sleep(0.02)
        for _ in range(5):
            self._publish_legs(0.0, 0.0, lb=0.0, lt=0.0, rt=0.0)
            time.sleep(0.02)

    def _arm_after_running(self) -> None:
        """确认 RUNNING 后：先原地零速 hold，再开始 soft 爬升。"""
        self._motion_armed = True
        now = time.time()
        self._hold_until = now + POST_RUNNING_HOLD_SEC
        self._soft_t0 = self._hold_until  # soft 从 hold 结束后才计
        print(
            f"\033[93m[RUNNING]\033[0m 原地踏步 {POST_RUNNING_HOLD_SEC:.1f}s "
            f"后再软启速度（{SOFT_START_SEC:.0f}s）"
        )

    def _enter_running_pulse(self) -> bool:
        """
        确保运控在 RUNNING 再发跟随速度。

        LT+RT+LB 是切换而非“只进 RUNNING”：已在 RUNNING 时再点会掉回 STANDBY。
        必须以 rosout `switch to running` 为准，禁止 fsm=5 兜底武装。
        """
        if self.dry_run:
            self._arm_after_running()
            return True
        if self.joy_pub is None:
            return False

        fsm, _, _ = self.fsm.snapshot()
        if fsm is not None and not self.fsm.allows_motion():
            print(f"\033[91m[RUNNING]\033[0m fsm={fsm} 非可行走态，拒绝进 RUNNING")
            return False

        # arbiter 重启后 hint 可能丢：先从 rosout 日志恢复
        if not self.fsm.is_running():
            self.fsm.recover_running_from_log()

        # 已在 RUNNING：勿再点 LB
        if self.fsm.is_running():
            print("\033[92m[RUNNING]\033[0m 已在 RUNNING，跳过 LB")
            self._arm_after_running()
            return True

        def _try_once(tag: str) -> bool:
            since = time.time()
            print(f"\033[93m[RUNNING]\033[0m {tag}：按住 LT+RT 并点 LB…")
            self._lb_edge_pulse()
            ok = self.fsm.wait_running_hint(
                since=since, timeout=ENTER_RUNNING_SETTLE_SEC
            )
            if ok:
                print("\033[92m[RUNNING]\033[0m 已确认 rosout: switch to running")
                return True
            if self.fsm.saw_standby_since(since):
                print(
                    "\033[93m[RUNNING]\033[0m 本次 LB 落到 standby"
                    "（多半原先已在 RUNNING，误拨出）—"
                    f"等 {STANDBY_RECOVER_SEC:.1f}s 站稳再试"
                )
                time.sleep(STANDBY_RECOVER_SEC)
            return False

        if _try_once("第1次"):
            self._arm_after_running()
            return True

        # 再试一次（边沿被吞，或刚误拨到 standby 已站稳）
        if self.fsm.is_running() or _try_once("第2次"):
            self._arm_after_running()
            return True

        print(
            "\033[91m[RUNNING]\033[0m 未确认 switch to running，"
            "拒绝发跟随速度（请先 STANDBY 后重试口令）"
        )
        self._motion_armed = False
        return False

    def tick(self) -> None:
        now = time.time()
        dt = max(1e-3, min(0.2, now - self._last_loop))
        self._last_loop = now

        # 任意模式：保护关机立即停腿
        if self.fsm.is_fatal() and self.mode == Mode.UWB_FOLLOW:
            fsm, _, _ = self.fsm.snapshot()
            self._abort_follow(f"fsm={fsm} PROTECTION/ERROR")
            return

        if self.joy_mon.blocks():
            if self.mode == Mode.UWB_FOLLOW:
                self._publish_legs(0.0, 0.0)
                # 开麦 R 会触发 joy 活动；未开始发跟随速度前不拦截 RUNNING/UWB 流程
                if self._motion_armed:
                    self._print_i += 1
                    if self._print_i % 50 == 0:
                        print("\033[93m[JOY]\033[0m 手柄占用，暂停跟随下发")
                    return
            elif self.mode != Mode.IDLE:
                return

        if self.mode == Mode.IDLE:
            return

        if self.mode == Mode.FACE_LOOK:
            face_age = now - self._face["t"]
            valid = self._face["valid"] and face_age < 0.5
            has = valid and self._face["has"]
            yaw, pitch = self.neck.update_from_face(
                self._face["dx"], self._face["dy"], has, now, dt
            )
            self._publish_neck(yaw, pitch)
            self._print_i += 1
            if self._print_i % 25 == 0:
                tag = "DRY" if self.dry_run else "FACE"
                print(
                    f"\033[92m[{tag}]\033[0m "
                    f"dx={self._face['dx']:+.2f} dy={self._face['dy']:+.2f} "
                    f"has={has} → yaw={math.degrees(yaw):+.1f}° "
                    f"pitch={math.degrees(pitch):+.1f}°"
                )
            return

        if self.mode == Mode.UWB_FOLLOW:
            if not self._policy_ready and not self.dry_run:
                if now - self._policy_retry_at >= 8.0:
                    self._policy_retry_at = now
                    print(
                        f"\033[93m[POLICY]\033[0m 重试切换 {FOLLOW_WALK_POLICY} ...",
                        flush=True,
                    )
                    if not self._teleop_killed:
                        self._kill_teleop()
                    ok = self._switch_walk_policy(
                        FOLLOW_WALK_POLICY, label="retry"
                    )
                    if ok:
                        self._policy_ready = True
                        time.sleep(POLICY_SETTLE_SEC)
                if not self._policy_ready:
                    self._print_i += 1
                    if self._print_i % 50 == 0:
                        print(
                            f"\033[91m[POLICY]\033[0m 等待 {FOLLOW_WALK_POLICY}，"
                            "不下发跟随速度"
                        )
                    self._publish_legs(0.0, 0.0)
                    return

            if not self._ensure_uwb():
                self._print_i += 1
                if self._print_i % 50 == 0:
                    print("\033[90m[UWB]\033[0m 等待串口...")
                return
            try:
                data = self._parser.read_latest()
            except (OSError, Exception):
                print("\033[91m[UWB]\033[0m 串口断开")
                self._close_uwb()
                self._publish_legs(0.0, 0.0)
                return
            if data:
                self._uwb_data = data
                self._uwb_t = now
            if self._uwb_data is None or (now - self._uwb_t) > UWB_TIMEOUT:
                # 串口已开但无 ###1.9：与手柄不同，此时没有“前进意图”，只能零速
                self._publish_legs(0.0, 0.0)
                self._print_i += 1
                if self._print_i % 50 == 0:
                    print(
                        "\033[90m[UWB]\033[0m 无有效帧（暂停）—"
                        "检查标签供电/距离，需 ###1.9 才进入 FOLLOW"
                    )
                return

            if not self.fsm.allows_motion() and not self.dry_run:
                self._publish_legs(0.0, 0.0)
                self._print_i += 1
                if self._print_i % 25 == 0:
                    fsm, _, _ = self.fsm.snapshot()
                    print(f"\033[91m[SAFE]\033[0m fsm={fsm} 禁止发速")
                return

            fwd, rot, state, filt_ang = self.follow.calculate(self._uwb_data)
            gate = "OFF"
            if ENABLE_OBSTACLE_GATE:
                age = now - self._obs_t
                stale = self._obs_t <= 0 or age > OBSTACLE_TIMEOUT
                person_m = estimate_person_center_m(
                    uwb_distance_cm=self._uwb_data.distance
                )
                fwd, rot, gate = apply_safety_gate(
                    fwd,
                    rot,
                    self._obs,
                    use_rotate_bias=USE_OBSTACLE_SIDESTEP,
                    stale=stale,
                    required=OBSTACLE_REQUIRED,
                    sidestep_blend=SIDESTEP_BLEND,
                    person_center_m=person_m,
                )

            if AUTO_ENTER_RUNNING and not self._running_sent:
                self._running_sent = True
                if not self._enter_running_pulse():
                    self._abort_follow("未能进入 RUNNING")
                    return

            if not self._motion_armed and not self.dry_run:
                self._publish_legs(0.0, 0.0)
                return

            # 进 RUNNING 后先原地零速，再 soft 爬升
            if self._hold_until is not None and now < self._hold_until:
                self._publish_legs(0.0, 0.0, lt=1.0, rt=1.0)
                self._print_i += 1
                if self._print_i % 25 == 0:
                    left = self._hold_until - now
                    print(
                        f"\033[93m[HOLD]\033[0m 原地踏步中 "
                        f"剩余 {left:.1f}s dist={self._uwb_data.distance:.0f}cm"
                    )
                return

            soft = 1.0
            if self._soft_t0 is not None and SOFT_START_SEC > 0:
                soft = min(
                    1.0, max(0.0, (time.time() - self._soft_t0) / SOFT_START_SEC)
                )
            fwd, rot = apply_soft_scales(fwd, rot, soft)

            # 与 keyboard_teleop 一致：跟随时保持 lt/rt=1 心跳 + cmd_vel
            # state=FOLLOW/BACK 才有前进意图；KEEPING/ALIGNED 为零速保持
            self._publish_legs(fwd, rot, lt=1.0, rt=1.0)
            self._print_i += 1
            if self._print_i % 10 == 0:
                tag = "DRY" if self.dry_run else "FOLLOW"
                fsm, run_h, _ = self.fsm.snapshot()
                if not run_h and not self.dry_run:
                    print(
                        "\033[91m[FOLLOW]\033[0m 有意图但非 RUNNING，"
                        "运控会忽略速度 — 检查进态"
                    )
                print(
                    f"\033[92m[{tag}]\033[0m dist={self._uwb_data.distance:.1f}cm "
                    f"ang={filt_ang:.1f} {state} fwd={fwd:.2f} rot={rot:.2f} "
                    f"gate={gate} soft={soft:.2f} fsm={fsm} run={int(run_h)}"
                )

    def shutdown(self) -> None:
        self._leave_mode(self.mode)
        self.mode = Mode.IDLE
        self._publish_mode()
        self.cam.shutdown()
        self.fsm.unregister()
        if self._teleop_killed and not self.dry_run:
            self._restore_teleop()


def main():
    ap = argparse.ArgumentParser(description="Moon 中央决策 / 语音切模式")
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="不发 /cmd_vel、/joy_msg、/pi_plus_absolute，不启相机子进程",
    )
    ap.add_argument(
        "--no-camera-manage",
        action="store_true",
        help="不由 arbiter 启停 ZED 节点（手动开 face_obs / zed_obstacle）",
    )
    ap.add_argument(
        "--no-obstacle-gate",
        action="store_true",
        help="忽略障碍门控（默认已关，保留兼容）",
    )
    ap.add_argument(
        "--enable-obstacle-gate",
        action="store_true",
        help="开启 /moon/obstacle 门控（避障联调）",
    )
    ap.add_argument(
        "--ignore-mutex",
        action="store_true",
        help="忽略 uwb_follow / 双 arbiter 冲突检测",
    )
    args = ap.parse_args()

    global ENABLE_OBSTACLE_GATE
    if args.enable_obstacle_gate:
        ENABLE_OBSTACLE_GATE = True
    if args.no_obstacle_gate:
        ENABLE_OBSTACLE_GATE = False

    if not args.dry_run and not args.ignore_mutex:
        ok, warns = check_follow_conflicts()
        print_conflict_report(ok, warns, fatal=False)
        # 有 uwb_follow 冲突时仍允许启动但强提示；双 arbiter 则退出
        for w in warns:
            if "其它 mode_arbiter" in w:
                print_conflict_report(ok, warns, fatal=True)
                sys.exit(1)

    rospy.init_node("moon_mode_arbiter", anonymous=False)
    arb = ModeArbiter(
        dry_run=args.dry_run,
        manage_camera=not args.no_camera_manage,
    )

    print()
    print("╔══════════════════════════════════════════════════╗")
    print("║     Moon 中央决策 (voice → mode → actuate)       ║")
    print("║  小派看我→FACE_LOOK  小派我们走→UWB_FOLLOW       ║")
    print("║  小派停止→IDLE                                   ║")
    print(f"║  跟随运控策略: {FOLLOW_WALK_POLICY:<28s} ║")
    print(f"║  障碍门控: {'ON ' if ENABLE_OBSTACLE_GATE else 'OFF':<33s} ║")
    print("╚══════════════════════════════════════════════════╝")
    print()
    if args.dry_run:
        print("\033[93m[NOTE]\033[0m DRY-RUN：只打印，不控头/腿，不启相机")
    print(f"\033[93m[NOTE]\033[0m 订 {TOPIC_VOICE_CMD}，发 {TOPIC_MODE}")
    print(
        f"\033[93m[NOTE]\033[0m UWB_FOLLOW: {FOLLOW_WALK_POLICY} → RUNNING → soft cmd_vel"
    )
    print("\033[93m[NOTE]\033[0m fsm=8(PROTECTION) 立即零速退出跟随")
    print("\033[93m[NOTE]\033[0m 模拟口令: python3 /home/nvidia/moon/voice/voice_sim.py")
    print()

    rate = rospy.Rate(CONTROL_HZ)
    try:
        while not rospy.is_shutdown():
            arb.tick()
            rate.sleep()
    finally:
        arb.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\narbiter 退出")
    except rospy.ROSInterruptException:
        pass
