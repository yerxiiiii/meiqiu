#!/usr/bin/env python3
"""
Mini Pi Plus - 固定场地语音带路 demo

推荐顺序（真机）:
  1. sim2real 已自动运行，机器人已站立 (fsm=5)
  2. 启动本节点（先不抢 /cmd_vel）
  3. ARMED 后，接管 /cmd_vel 前会自动通过 /joy_msg 发 LT+RT+LB
     序列尝试进入 RUNNING 子状态（fsm=5 同时覆盖 STANDBY/RUNNING，
     /fsm_state 数值本身无法区分，需要监听 /rosout 的
     "sim2real switch to running state" 确认）。此前误以为只要
     fsm=5 就能走，实际上还卡在 STANDBY 子状态，故补上这一步。
     不再依赖人工按手柄。
  4. RUNNING 确认后立即执行路线（避免空闲 ~7s 后自动退回 STANDBY）
  5. 语音动作结束 / 停下 / Ctrl+C -> 零速，并立刻恢复 /joy_teleop（不长期占遥控）

用法:
  python3 guide_demo_node.py --dry-run --fast-dry-run --text "小派带我去炳胜餐厅"
  python3 guide_demo_node.py --text "小派带我去炳胜餐厅"
  python3 guide_demo_node.py --smoke-move 1.5
"""

from __future__ import annotations

import argparse
import os
import re
import signal
import subprocess
import sys
import threading
import time
from enum import Enum
from typing import Any, Dict, List, Optional, Set

import yaml

try:
    import rospy
    from geometry_msgs.msg import Twist
    from rosgraph_msgs.msg import Log
    from std_msgs.msg import Int32, String
except ImportError as exc:  # pragma: no cover
    print("需要 ROS1 rospy: source /opt/ros/noetic/setup.bash", file=sys.stderr)
    raise SystemExit(1) from exc

try:
    from sim2real_msg.msg import Joy as SimJoy
except ImportError:
    SimJoy = None  # type: ignore

_RUNNING_LOG_HINT = "sim2real switch to running state"
WAKE_WINDOW_SEC = 5.0  # 唤醒词与后续指令允许分成两句话的最大间隔（秒）


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_DEST = os.path.join(ROOT, "config", "destinations.yaml")
DEFAULT_SAFETY = os.path.join(ROOT, "config", "safety.yaml")


class GuideState(str, Enum):
    WAIT_ARM = "WAIT_ARM"  # 等手柄解锁 / fsm 可走
    ARMED = "ARMED"  # 已可走，可接受带路命令
    LEAD_TO_DEST = "LEAD_TO_DEST"
    PAUSED = "PAUSED"
    ARRIVED = "ARRIVED"
    STOPPED = "STOPPED"
    IDLE = "IDLE"


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def match_destination(text: str, destinations: Dict[str, Any]) -> Optional[str]:
    text = text.strip()
    for dest_id, cfg in destinations.items():
        for kw in cfg.get("keywords", []) + [cfg.get("name", "")]:
            if kw and kw in text:
                return dest_id
    return None


def _levenshtein(a: str, b: str) -> int:
    if a == b:
        return 0
    if len(a) < len(b):
        a, b = b, a
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i] + [0] * len(b)
        for j, cb in enumerate(b, 1):
            cur[j] = min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb))
        prev = cur
    return prev[-1]


def _fuzzy_contains(text: str, word: str, max_dist: int = 1) -> bool:
    """ASR 对生僻短词（如"小派"）容易听成音近但常见的词（如"老派"）。
    在允许 +-1 字长、编辑距离 <= max_dist 的窗口里模糊查找 word 是否出现在 text 里，
    弥补精确子串匹配漏掉这类近音误听的问题。"""
    if word in text:
        return True
    n = len(word)
    for m in (n - 1, n, n + 1):
        if m < 1 or m > len(text):
            continue
        for i in range(len(text) - m + 1):
            if _levenshtein(text[i : i + m], word) <= max_dist:
                return True
    return False


def _any_fuzzy(text: str, words: List[str], max_dist: int = 1) -> bool:
    return any(_fuzzy_contains(text, w, max_dist) for w in words)


def _is_short_command(text: str, words: List[str]) -> bool:
    """整句几乎就是关键词本身（KWS 单条命中），允许不带唤醒词。"""
    t = (text or "").strip()
    if not t or not words:
        return False
    if any(t == w for w in words):
        return True
    return _any_fuzzy(t, words) and len(t) <= max(len(w) for w in words) + 1


def parse_voice_command(text: str, dest_cfg: Dict[str, Any]) -> Optional[str]:
    text = (text or "").strip()
    if not text:
        return None
    keywords = dest_cfg.get("keywords", {})
    stop_words = keywords.get("stop", ["停下", "停止", "停车"])
    wake_words = keywords.get("wake", ["小派"])
    walk_test_words = keywords.get("walk_test", ["出发"])
    uwb_follow_words = keywords.get("uwb_follow", ["跟我走", "跟着我"])
    go_to_prefix = keywords.get("go_to_prefix", ["带我去", "去", "带路"])
    guide_short = keywords.get(
        "guide_short", ["带路", "带我去炳胜", "带我去炳胜餐厅"]
    )
    for w in stop_words:
        if w in text:
            return "stop"
    has_wake = _any_fuzzy(text, wake_words)
    has_walk = _any_fuzzy(text, walk_test_words)
    has_uwb = _any_fuzzy(text, uwb_follow_words)
    # 先判跟随再判出发：walk_test 含「走起」，模糊匹配会误伤「跟我走」
    # KWS 常拆句：开麦后单独「跟我走」「出发」「带路」也接受
    if has_uwb and (has_wake or _is_short_command(text, uwb_follow_words)):
        return "uwb_follow"
    if has_walk and (has_wake or _is_short_command(text, walk_test_words)):
        return "walk_test"
    dest_id = match_destination(text, dest_cfg.get("destinations", {}))
    has_go_prefix = any(p in text for p in go_to_prefix)
    # 「小派带我去炳胜餐厅 / 带路」→ go_to:<dest> → destinations.yaml 预制轨迹
    if dest_id and (
        has_wake
        or has_go_prefix
        or _is_short_command(text, guide_short)
    ):
        return f"go_to:{dest_id}"
    return None


def list_cmd_vel_publishers() -> List[str]:
    try:
        out = subprocess.check_output(
            ["rostopic", "info", "/cmd_vel"],
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=3,
        )
    except (subprocess.SubprocessError, FileNotFoundError):
        return []
    pubs: List[str] = []
    in_pub = False
    for line in out.splitlines():
        if line.startswith("Publishers:"):
            in_pub = True
            continue
        if in_pub:
            if line.startswith("Subscribers:"):
                break
            line = line.strip()
            if line.startswith("*"):
                pubs.append(line[1:].strip().split()[0])
    return pubs


def kill_node(name: str) -> bool:
    try:
        subprocess.check_call(
            ["rosnode", "kill", name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )
        return True
    except (subprocess.SubprocessError, FileNotFoundError):
        return False


def restart_joy_teleop() -> None:
    try:
        subprocess.Popen(
            ["rosrun", "joy_teleop", "joy_teleop.py", "__name:=joy_teleop"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        pass


def is_trigger_pressed(v: float) -> bool:
    """Xbox 类手柄：扳机松开约 +1，按下约 -1。"""
    return v < -0.5


def is_button_pressed(v: float) -> bool:
    return v > 0.5


class GuideDemoNode:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.dest_cfg = load_yaml(args.dest_config)
        self.safety = load_yaml(args.safety_config)
        self.dry_run = bool(args.dry_run)
        self.state = GuideState.WAIT_ARM
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._armed = threading.Event()
        self._obstacle = "clear"
        self._route_thread: Optional[threading.Thread] = None
        self._took_over_joy = False
        self._fsm: Optional[int] = None
        self._pending_cmd: Optional[str] = None
        self._unlock_combo_seen = False
        self._running_hint = False
        self._running_hint_at = 0.0
        self._entered_running = False
        self._last_wake_at = 0.0

        self.cmd_topic = self.safety.get("cmd_vel_topic", "/cmd_vel")
        self.max_lin = float(self.safety.get("max_linear_x", 0.12))
        self.max_ang = float(self.safety.get("max_angular_z", 0.25))
        self.hz = float(self.safety.get("publish_hz", 20))
        self.segment_hold = float(self.safety.get("segment_stop_hold", 0.15))
        defaults = self.dest_cfg.get("defaults", {})
        self.default_lin = float(defaults.get("linear_x", 0.08))
        self.default_ang = float(defaults.get("angular_z", 0.20))
        wt = self.dest_cfg.get("walk_test_params", {})
        self.walk_test_reps = int(wt.get("reps", 3))
        self.walk_test_seconds = float(wt.get("seconds", 1.0))
        self.walk_test_linear_x = float(wt.get("linear_x", self.default_lin))
        self.walk_test_pause = float(wt.get("pause", 1.5))

        # 可走 fsm：现场默认常见为 5(ExecDefault)。可用 --ready-fsm 覆盖。
        ready = self.safety.get("ready_fsm_states", [5, 6])
        if args.ready_fsm:
            ready = [int(x) for x in args.ready_fsm.split(",")]
        self.ready_fsm: Set[int] = set(int(x) for x in ready)

        if self.dry_run:
            print("=" * 60, flush=True)
            print("DRY-RUN：只打印，不发 /cmd_vel", flush=True)
            print("=" * 60, flush=True)
            self.pub = None
            self.state_pub = None
            self.brain_voice_pub = None
            self._armed.set()
            self.state = GuideState.ARMED
        else:
            rospy.init_node("guide_demo_node", anonymous=False)
            self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
            self.state_pub = rospy.Publisher(
                self.safety.get("state_topic", "/guide/state"), String, queue_size=1
            )
            # UWB 跟随走中央决策：命中后发 /moon/voice_cmd = uwb_follow
            self.brain_voice_pub = rospy.Publisher(
                self.safety.get("brain_voice_topic", "/moon/voice_cmd"),
                String,
                queue_size=1,
            )
            rospy.Subscriber(
                self.safety.get("voice_command_topic", "/guide/voice_command"),
                String,
                self._on_voice_cmd,
                queue_size=1,
            )
            rospy.Subscriber(
                self.safety.get("obstacle_topic", "/guide/obstacle_state"),
                String,
                self._on_obstacle,
                queue_size=1,
            )
            rospy.Subscriber("/fsm_state", Int32, self._on_fsm, queue_size=1)
            self.joy_pub = None
            if SimJoy is not None:
                rospy.Subscriber("/joy_msg", SimJoy, self._on_joy_msg, queue_size=1)
                self.joy_pub = rospy.Publisher("/joy_msg", SimJoy, queue_size=10)
            rospy.Subscriber("/rosout", Log, self._on_rosout, queue_size=50)
            rospy.on_shutdown(self._on_shutdown)
            self._print_boot_banner()

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _print_boot_banner(self) -> None:
        pubs = list_cmd_vel_publishers()
        self.log("启动顺序（全自动，无需手柄）:")
        self.log(f"  1) 等待 /fsm_state in {sorted(self.ready_fsm)} 后 ARMED")
        self.log("  2) ARMED 后接管 /cmd_vel，自动发 LT+RT+LB 进入 RUNNING")
        self.log("  3) RUNNING 确认后立即执行带路")
        self.log(f"当前 cmd_vel publishers={pubs}")
        if self.args.skip_arm_check:
            self.log("已 --skip-arm-check，直接 ARMED（仅调试用）")
            self._set_armed("skip_arm_check")

    def log(self, msg: str) -> None:
        prefix = "[DRY-RUN] " if self.dry_run else f"[{self.state.value}] "
        print(f"{prefix}{msg}", flush=True)

    def set_state(self, state: GuideState) -> None:
        self.state = state
        self.log(f"state -> {state.value}")
        if self.state_pub is not None:
            self.state_pub.publish(String(data=state.value))

    def _on_fsm(self, msg: Int32) -> None:
        self._fsm = int(msg.data)
        if self._armed.is_set():
            return
        if self._fsm in self.ready_fsm:
            self._set_armed(f"fsm_state={self._fsm}")

    def _on_joy_msg(self, msg) -> None:
        # 仅作提示：检测到解锁组合键
        if is_trigger_pressed(msg.lt) and is_trigger_pressed(msg.rt) and is_button_pressed(msg.lb):
            if not self._unlock_combo_seen:
                self._unlock_combo_seen = True
                self.log("检测到手柄解锁组合 LT+RT+LB（仍以 /fsm_state 进入 ARMED 为准）")

    def _on_rosout(self, msg) -> None:
        text = msg.msg or ""
        if _RUNNING_LOG_HINT in text:
            self._running_hint = True
            self._running_hint_at = time.time()

    def _set_armed(self, reason: str) -> None:
        if self._armed.is_set():
            return
        self._armed.set()
        self.set_state(GuideState.ARMED)
        self.log(f"ARMED: {reason}")
        self.log("现在可以接受带路命令；即将在运动前接管 /cmd_vel")

    def wait_until_armed(self, timeout: Optional[float] = None) -> bool:
        if self.dry_run or self.args.skip_arm_check:
            self._set_armed("dry-run/skip")
            return True
        self.set_state(GuideState.WAIT_ARM)
        self.log(
            f"等待可走信号: 目标 fsm={sorted(self.ready_fsm)} "
            f"(当前 fsm={self._fsm})；ARMED 后自动进 RUNNING，无需手柄"
        )
        ok = self._armed.wait(timeout=timeout)
        if not ok:
            self.log("等待 ARMED 超时，放弃运动")
        return ok

    def takeover_cmd_vel(self) -> bool:
        """仅在 ARMED 且即将运动时调用：暂停手柄对 /cmd_vel 的占用，
        并尝试进入 RUNNING 子状态（走路真正需要的状态，fsm=5 不够）。
        若检测到 UWB/arbiter/视觉跟随等其它运动源，拒绝接管并返回 False。"""
        if self.dry_run or not self.args.takeover or self._took_over_joy:
            return True
        pubs = list_cmd_vel_publishers()
        # 禁止与 UWB / 中央决策抢控制：这些节点不是手柄，不能随便 kill
        blocked = [
            p
            for p in pubs
            if any(
                k in p
                for k in (
                    "uwb_follow",
                    "moon_mode_arbiter",
                    "mode_arbiter",
                    "person_follower",
                    "person_follow",
                )
            )
        ]
        if blocked:
            self.log(
                f"ABORT takeover: 检测到其它运动源仍在发 /cmd_vel: {blocked}；"
                "请先 stop uwb-follow / mode_arbiter / person_follow 再带路"
            )
            return False
        foreign = [p for p in pubs if "guide_demo" not in p]
        for name in foreign:
            self.log(f"takeover: 暂停 {name}（避免零速覆盖）")
            if kill_node(name) and "joy_teleop" in name:
                self._took_over_joy = True
        time.sleep(0.3)
        self.log(f"takeover 后 publishers={list_cmd_vel_publishers()}")
        self._enter_running()
        return True

    def _joy_pulse(self, **fields) -> None:
        if self.joy_pub is None:
            return
        msg = SimJoy()
        for k, v in fields.items():
            setattr(msg, k, float(v))
        self.joy_pub.publish(msg)

    def _enter_running(self, attempts: int = 3, wait_timeout: float = 6.0) -> bool:
        """通过 /joy_msg 模拟 LT+RT+LB，尝试把 fsm=5 的 STANDBY 子状态切到
        RUNNING（真正能响应 /cmd_vel 走路的状态）。用 /rosout 里的
        "sim2real switch to running state" 确认，而不是只看 /fsm_state
        （STANDBY/RUNNING 数值相同，光看 fsm 分不出来）。
        实测该组合键触发不完全可靠，常需要 2 次左右，故重试几次。"""
        if self.dry_run or self.joy_pub is None or self._entered_running:
            return True
        for i in range(attempts):
            since = time.time()
            self._running_hint = False
            self.log(f"尝试进入 RUNNING ({i + 1}/{attempts})：LT+RT+LB ...")
            for _ in range(10):
                self._joy_pulse(lt=-1.0, rt=-1.0, lb=0.0)
                time.sleep(0.02)
            self._joy_pulse(lt=-1.0, rt=-1.0, lb=1.0)
            time.sleep(0.05)
            for _ in range(5):
                self._joy_pulse(lt=-1.0, rt=-1.0, lb=0.0)
                time.sleep(0.02)
            for _ in range(5):
                self._joy_pulse(lt=0.0, rt=0.0, lb=0.0)
                time.sleep(0.02)

            t0 = time.time()
            while time.time() - t0 < wait_timeout:
                if self._running_hint and self._running_hint_at >= since:
                    self.log("RUNNING 已确认 (rosout: switch to running state)")
                    self._entered_running = True
                    return True
                time.sleep(0.05)
            self.log("本次未确认 RUNNING，重试")
        self.log("多次尝试仍未确认 RUNNING；仍会继续发 /cmd_vel，但可能走不动")
        return False

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        linear_x = clamp(linear_x, -self.max_lin, self.max_lin)
        angular_z = clamp(angular_z, -self.max_ang, self.max_ang)
        if self.dry_run:
            return
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.z = 1.0  # 与真实手柄扳机空闲基线一致（sim2real 可能据此判定"有效遥操作"）
        msg.angular.z = angular_z
        self.pub.publish(msg)

    def zero_velocity(self, hold: float = 0.0) -> None:
        self.publish_cmd(0.0, 0.0)
        if hold > 0:
            time.sleep(hold)

    def emergency_stop(self, *_args) -> None:
        self._stop_event.set()
        self.set_state(GuideState.STOPPED)
        for _ in range(5):
            self.publish_cmd(0.0, 0.0)
            time.sleep(0.02)
        self.log("emergency stop: zero /cmd_vel")
        # 停下后立刻把手柄还回去，避免语音急停后遥控一直失灵
        self._restore_joy()
        if self._armed.is_set():
            self.set_state(GuideState.ARMED)
        self.log("急停后已归还遥控，语音待机")

    def _restore_joy(self) -> None:
        if self._took_over_joy:
            self.log("恢复 /joy_teleop ...")
            restart_joy_teleop()
            self._took_over_joy = False

    def _release_to_standby(self, reason: str = "") -> None:
        """语音动作结束后：零速 + 立刻归还遥控，回到可继续听语音的待机。"""
        self.zero_velocity()
        self._restore_joy()
        if self._armed.is_set():
            self.set_state(GuideState.ARMED)
        else:
            self.set_state(GuideState.IDLE)
        msg = "已归还遥控，语音待机"
        if reason:
            msg = f"{reason}；{msg}"
        self.log(msg)

    def _on_shutdown(self) -> None:
        self.emergency_stop()
        self._restore_joy()

    def _signal_handler(self, signum, _frame) -> None:
        self.log(f"caught signal {signum}")
        self.emergency_stop()
        self._restore_joy()
        if not self.dry_run and rospy.core.is_initialized():
            rospy.signal_shutdown("signal")
        raise SystemExit(0)

    def _on_voice_cmd(self, msg: String) -> None:
        self.handle_command(msg.data)

    def _on_obstacle(self, msg: String) -> None:
        level = (msg.data or "clear").strip().lower()
        if level not in ("clear", "slow", "stop"):
            return
        with self._lock:
            prev = self._obstacle
            self._obstacle = level
        if level == "stop" and prev != "stop":
            self.log("obstacle stop")
            self.zero_velocity()
            if self.state == GuideState.LEAD_TO_DEST:
                self.set_state(GuideState.PAUSED)
        elif level == "clear" and self.state == GuideState.PAUSED:
            self.set_state(GuideState.LEAD_TO_DEST)

    def _has_wake_word(self, text: str) -> bool:
        wake_words = self.dest_cfg.get("keywords", {}).get("wake", ["小派"])
        return _any_fuzzy(text, wake_words)

    def _parse_with_wake_memory(self, text: str) -> str:
        """先按原样解析；若失败且本句没有唤醒词，但最近 WAKE_WINDOW_SEC 秒内
        听到过唤醒词（ASR 常把"小派，出发"这类带停顿的话拆成两句），
        就假装唤醒词也在本句里再解析一次。"""
        parsed = parse_voice_command(text, self.dest_cfg) or ""
        has_wake = self._has_wake_word(text)
        if has_wake:
            self._last_wake_at = time.time()
        if not parsed and not has_wake:
            if time.time() - self._last_wake_at < WAKE_WINDOW_SEC:
                wake_words = self.dest_cfg.get("keywords", {}).get("wake", ["小派"])
                retry = parse_voice_command(f"{wake_words[0]}{text}", self.dest_cfg)
                if retry:
                    self.log(f"唤醒词记忆命中（{WAKE_WINDOW_SEC:.0f}s 内）：补上后重新解析")
                    parsed = retry
        return parsed

    def handle_command(self, raw: str) -> None:
        cmd = raw.strip()
        if cmd.startswith("go_to:") or cmd in ("stop", "walk_test", "uwb_follow"):
            parsed = cmd
        else:
            parsed = self._parse_with_wake_memory(cmd)

        self.log(f"command: {raw!r} -> {parsed!r}")
        if parsed == "stop":
            self.emergency_stop()
            # 同步通知 brain 停跟随
            self._publish_brain_voice("stop")
            return
        if not (
            parsed.startswith("go_to:")
            or parsed in ("walk_test", "uwb_follow")
        ):
            return

        if not self._armed.is_set() and not self.dry_run and not self.args.skip_arm_check:
            self._pending_cmd = parsed
            self.log("尚未 ARMED：命令已缓存")
            return

        if parsed == "uwb_follow":
            self.start_uwb_follow()
            return

        if parsed == "walk_test":
            self.start_walk_test()
            return

        dest_id = parsed.split(":", 1)[1]
        self.start_route(dest_id)

    def _publish_brain_voice(self, cmd: str) -> None:
        if self.dry_run:
            self.log(f"[DRY-RUN] would publish /moon/voice_cmd: {cmd}")
            return
        if self.brain_voice_pub is None:
            self.log(f"brain_voice_pub 未就绪，跳过 /moon/voice_cmd={cmd}")
            return
        self.brain_voice_pub.publish(String(data=cmd))
        self.log(f"已发 /moon/voice_cmd: {cmd}")

    def start_uwb_follow(self) -> None:
        """语音「小派，跟我走」→ 交给 mode_arbiter 的 UWB_FOLLOW 模式。
        本节点不抢 /cmd_vel，避免与跟随冲突。"""
        self.log("UWB 跟随请求：转发 brain，guide 不占遥控")
        # 若之前带路占过 joy，先还回去
        self._restore_joy()
        self._publish_brain_voice("uwb_follow")
        if self._armed.is_set():
            self.set_state(GuideState.ARMED)
        self.log("已请求 uwb_follow（需 mode_arbiter 在跑才会真正跟随）")

    def start_route(self, dest_id: str) -> None:
        destinations = self.dest_cfg.get("destinations", {})
        if dest_id not in destinations:
            self.log(f"unknown destination: {dest_id}")
            return
        if self._route_thread and self._route_thread.is_alive():
            self.log("route already running")
            return
        if not self.wait_until_armed(timeout=self.args.arm_timeout):
            return
        if not self.takeover_cmd_vel():
            return

        self._stop_event.clear()
        self.set_state(GuideState.LEAD_TO_DEST)
        self._route_thread = threading.Thread(
            target=self._run_route, args=(dest_id,), daemon=True
        )
        self._route_thread.start()

    def start_walk_test(self) -> None:
        """语音 "小派，出发" 触发：短距离直线走 N 次，验证解锁/走路可重复。"""
        if self._route_thread and self._route_thread.is_alive():
            self.log("route/walk-test already running")
            return
        if not self.wait_until_armed(timeout=self.args.arm_timeout):
            return
        if not self.takeover_cmd_vel():
            return

        self._stop_event.clear()
        self.set_state(GuideState.LEAD_TO_DEST)
        self._route_thread = threading.Thread(target=self._run_walk_test, daemon=True)
        self._route_thread.start()

    def _run_walk_test(self) -> None:
        reps = self.walk_test_reps
        self.log(
            f"walk-test: {reps}x forward {self.walk_test_seconds}s "
            f"@ {self.walk_test_linear_x}"
        )
        try:
            for i in range(reps):
                if self._stop_event.is_set():
                    break
                self.log(f"walk-test rep {i + 1}/{reps}")
                if not self._run_timed_motion(
                    self.walk_test_seconds, self.walk_test_linear_x, 0.0
                ):
                    break
                if i < reps - 1:
                    self.zero_velocity(self.walk_test_pause)
        finally:
            # emergency_stop 已归还遥控时不再重复；正常结束则立刻归还
            if self._took_over_joy:
                reason = (
                    "walk-test done"
                    if not self._stop_event.is_set()
                    else "walk-test 中止"
                )
                self._release_to_standby(reason)
            else:
                self.zero_velocity()

    def _wait_if_paused(self) -> bool:
        while not self._stop_event.is_set():
            with self._lock:
                obs = self._obstacle
            if obs != "stop" and self.state != GuideState.PAUSED:
                return True
            if obs == "stop":
                self.zero_velocity()
                time.sleep(0.05)
                continue
            return True
        return False

    def _run_timed_motion(self, duration: float, lin: float, ang: float) -> bool:
        rate_dt = 1.0 / max(self.hz, 1.0)
        end = time.time() + duration
        last_log = 0.0
        while time.time() < end:
            if self._stop_event.is_set():
                self.zero_velocity()
                return False
            if not self._wait_if_paused():
                self.zero_velocity()
                return False
            with self._lock:
                obs = self._obstacle
            scale = 0.5 if obs == "slow" else 1.0
            lx, az = lin * scale, ang * scale
            self.publish_cmd(lx, az)
            now = time.time()
            if not self.dry_run and now - last_log > 0.5:
                self.log(f"cmd_vel x={lx:.3f} yaw={az:.3f}")
                last_log = now
            if self.dry_run and self.args.fast_dry_run:
                break
            time.sleep(rate_dt)
        self.zero_velocity(self.segment_hold)
        return True

    def _speak(self, text: str) -> None:
        self.log(f"SPEAK: {text}")

    def _run_route(self, dest_id: str) -> None:
        cfg = self.dest_cfg["destinations"][dest_id]
        route: List[Dict[str, Any]] = cfg.get("route", [])
        self.log(f"LEAD_TO_DEST -> {cfg.get('name', dest_id)} ({len(route)} segments)")

        try:
            for i, seg in enumerate(route):
                if self._stop_event.is_set():
                    break
                action = seg.get("action", "")
                self.log(f"segment[{i}] action={action} params={seg}")
                if action == "speak":
                    self._speak(seg.get("text", ""))
                elif action == "wait":
                    duration = float(seg.get("duration", 1.0))
                    if self.args.fast_dry_run and self.dry_run:
                        time.sleep(0.05)
                    else:
                        end = time.time() + duration
                        while time.time() < end and not self._stop_event.is_set():
                            if not self._wait_if_paused():
                                break
                            time.sleep(0.05)
                    self.zero_velocity(self.segment_hold)
                elif action == "forward":
                    if not self._run_timed_motion(
                        float(seg.get("duration", 1.0)),
                        float(seg.get("linear_x", self.default_lin)),
                        0.0,
                    ):
                        break
                elif action == "turn":
                    if not self._run_timed_motion(
                        float(seg.get("duration", 1.0)),
                        0.0,
                        float(seg.get("angular_z", self.default_ang)),
                    ):
                        break
                else:
                    self.log(f"unknown action: {action}")
                self.zero_velocity(self.segment_hold)

            if not self._stop_event.is_set():
                self.set_state(GuideState.ARRIVED)
                arrive = cfg.get("arrive_speak")
                if arrive:
                    self._speak(arrive)
        finally:
            if self._took_over_joy:
                reason = (
                    "route arrived"
                    if not self._stop_event.is_set()
                    else "route 中止"
                )
                self._release_to_standby(reason)
            else:
                self.zero_velocity()

    def run_smoke_move(self, seconds: float) -> None:
        if not self.wait_until_armed(timeout=self.args.arm_timeout):
            return
        if not self.takeover_cmd_vel():
            return
        self.log(f"smoke-move forward {seconds}s @ {self.default_lin}")
        self.set_state(GuideState.LEAD_TO_DEST)
        try:
            self._run_timed_motion(seconds, self.default_lin, 0.0)
        finally:
            if self._took_over_joy:
                self._release_to_standby("smoke-move done")
            else:
                self.zero_velocity()

    def spin(self) -> None:
        try:
            if self.args.smoke_move is not None:
                if self.dry_run:
                    self.log("smoke-move 需要真机，去掉 --dry-run")
                    return
                self.run_smoke_move(float(self.args.smoke_move))
                return

            if self.args.text:
                # 真机：先等 ARMED，再执行；命令可提前注入
                if not self.dry_run and not self.args.skip_arm_check:
                    self.handle_command(self.args.text)  # 可能先缓存
                    if not self.wait_until_armed(timeout=self.args.arm_timeout):
                        return
                    if self._pending_cmd:
                        cmd = self._pending_cmd
                        self._pending_cmd = None
                        self.handle_command(cmd)
                    elif self.state == GuideState.ARMED:
                        self.handle_command(self.args.text)
                else:
                    self.handle_command(self.args.text)
                if self._route_thread:
                    self._route_thread.join()
                return

            self.log("waiting for /guide/voice_command ...")
            if self.dry_run:
                return
            # 后台等 ARMED；命令到达时再执行
            rate = rospy.Rate(2)
            while not rospy.is_shutdown():
                if self._pending_cmd and self._armed.is_set():
                    cmd = self._pending_cmd
                    self._pending_cmd = None
                    self.handle_command(cmd)
                rate.sleep()
        finally:
            if not self.dry_run:
                self.zero_velocity()
                self._restore_joy()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Fixed-route voice guide demo")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--fast-dry-run", action="store_true")
    p.add_argument("--text", type=str, default="")
    p.add_argument("--smoke-move", type=float, default=None, metavar="SEC")
    p.add_argument(
        "--takeover",
        dest="takeover",
        action="store_true",
        default=True,
        help="ARMED 后运动前暂停 joy_teleop（默认开）",
    )
    p.add_argument("--no-takeover", dest="takeover", action="store_false")
    p.add_argument(
        "--skip-arm-check",
        action="store_true",
        help="跳过可走检测（危险，仅调试）",
    )
    p.add_argument(
        "--ready-fsm",
        type=str,
        default="",
        help="可走 fsm_state 列表，逗号分隔，默认 5,6",
    )
    p.add_argument(
        "--arm-timeout",
        type=float,
        default=120.0,
        help="等待 ARMED 超时秒数",
    )
    p.add_argument("--dest-config", default=DEFAULT_DEST)
    p.add_argument("--safety-config", default=DEFAULT_SAFETY)
    return p


def main() -> None:
    args = build_parser().parse_args()
    if args.fast_dry_run:
        args.dry_run = True
    GuideDemoNode(args).spin()


if __name__ == "__main__":
    main()
