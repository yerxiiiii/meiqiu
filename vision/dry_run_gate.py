#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
避障干跑：只订 /moon/obstacle，模拟「想前进」，打印门控结果。
不发 /joy_msg、不发 /cmd_vel → 完全不控腿。

用法：
  # 终端1：视觉
  python3 /home/nvidia/moon/vision/zed_obstacle_node.py

  # 终端2：干跑
  source .../install/setup.bash
  python3 /home/nvidia/moon/vision/dry_run_gate.py

手挡镜头 / 走开，看 gate 是否 STOP / SLOW / CLEAR。
浏览器 FPV: http://localhost:8080/（需 SSH -L 8080）
"""

from __future__ import annotations

import math
import os
import sys
import time

import rospy
from std_msgs.msg import Float32MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from obstacle_state import ObstacleState
from safety_gate import apply_safety_gate

# 模拟 UWB「想往前走」的摇杆（不真发）
FAKE_FWD = 0.50
FAKE_ROT = 0.0
OBSTACLE_TIMEOUT = 0.8
PRINT_HZ = 5.0


def main():
    rospy.init_node("dry_run_gate", anonymous=True)
    state = {"obs": ObstacleState(valid=False), "t": 0.0}

    def cb(msg: Float32MultiArray):
        state["obs"] = ObstacleState.from_list(msg.data, stamp=time.time())
        state["t"] = time.time()

    rospy.Subscriber("/moon/obstacle", Float32MultiArray, cb, queue_size=1)

    print()
    print("╔══════════════════════════════════════════════════╗")
    print("║     避障干跑 dry_run_gate（不控腿）              ║")
    print("║  假想 fwd=%.2f，只打印门控后的 fwd/rot           ║" % FAKE_FWD)
    print("╚══════════════════════════════════════════════════╝")
    print()
    print("请先启动 zed_obstacle_node；用手挡/离开镜头验证 STOP/CLEAR")
    print()

    period = 1.0 / PRINT_HZ
    while not rospy.is_shutdown():
        obs = state["obs"]
        age = time.time() - state["t"] if state["t"] > 0 else 999.0
        stale = state["t"] <= 0.0 or age > OBSTACLE_TIMEOUT
        fwd, rot, reason = apply_safety_gate(
            FAKE_FWD, FAKE_ROT, obs,
            use_rotate_bias=True,
            stale=stale,
            required=False,
        )

        def fmt(d):
            return f"{d:4.2f}" if math.isfinite(d) else " nan"

        if stale:
            print(f"\033[90m[DRY]\033[0m 等待 /moon/obstacle ... age={age:.1f}s")
        else:
            print(
                f"\033[92m[DRY]\033[0m "
                f"L:{fmt(obs.left_m)} C:{fmt(obs.center_m)} R:{fmt(obs.right_m)} "
                f"| cap:{obs.forward_cap:4.2f} bias:{obs.rotate_bias:+5.2f} "
                f"| fake_fwd:{FAKE_FWD:.2f} → out_fwd:{fwd:5.2f} out_rot:{rot:+5.2f} "
                f"| {reason}"
            )
        time.sleep(period)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n干跑结束")
    except rospy.ROSInterruptException:
        pass
