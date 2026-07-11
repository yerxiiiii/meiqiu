#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在 STANDBY 下通过 /joy_msg 十字键切换走路策略到 lr（lowerBody）。

用法（机器人站稳、不要在走的时候切）:
  1. 若正在走: 按手柄 LB 进 STANDBY，或本脚本加 --standby
  2. python3 moon/switch_to_lr.py
  3. 看日志出现: algorithm: [lr]  ctrl group:(lowerBody)
  4. 再 LB 回 RUNNING，然后跑 upper_body_teleop.py
"""

import argparse
import time

import rospy
from sim2real_msg.msg import Joy
from std_msgs.msg import Int32

JOY_TOPIC = "/joy_msg"
FSM_TOPIC = "/fsm_state"
FSM_EXEC_DEFAULT = 5


def _idle_joy() -> Joy:
    msg = Joy()
    msg.lt = 1.0
    msg.rt = 1.0
    return msg


def _pulse(pub, **fields) -> None:
    """发一帧带边沿的 joy，再回空闲。"""
    on = _idle_joy()
    for k, v in fields.items():
        setattr(on, k, v)
    pub.publish(on)
    time.sleep(0.15)
    pub.publish(_idle_joy())
    time.sleep(0.4)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--standby",
        action="store_true",
        help="先发一次 LB，尝试从 RUNNING 切到 STANDBY",
    )
    ap.add_argument(
        "--steps",
        type=int,
        default=1,
        help="十字键右按几次（amp→lr 通常 1 次；若当前是 footstep 可能要 2）",
    )
    ap.add_argument(
        "--left",
        action="store_true",
        help="用十字键左（Prev）而不是右（Next）",
    )
    args = ap.parse_args()

    rospy.init_node("switch_to_lr", anonymous=True)
    pub = rospy.Publisher(JOY_TOPIC, Joy, queue_size=5)
    fsm = {"v": None}

    def _on_fsm(msg: Int32) -> None:
        fsm["v"] = int(msg.data)

    rospy.Subscriber(FSM_TOPIC, Int32, _on_fsm, queue_size=5)
    time.sleep(0.5)

    rospy.loginfo("fsm_state=%s (5=EXEC_DEFAULT)", fsm["v"])
    if fsm["v"] is not None and fsm["v"] != FSM_EXEC_DEFAULT:
        rospy.logwarn("FSM 不是 EXEC_DEFAULT，切换可能无效")

    # 心跳，避免 joy 超时
    for _ in range(5):
        pub.publish(_idle_joy())
        time.sleep(0.05)

    if args.standby:
        rospy.loginfo("pulse LB → STANDBY/STANDING")
        _pulse(pub, lb=1.0)
        time.sleep(1.6)

    # dpad_horizontal: 1=左(Prev), -1=右(Next)  （见 default_controller.cpp）
    horiz = 1.0 if args.left else -1.0
    direction = "left/Prev" if args.left else "right/Next"
    for i in range(max(1, args.steps)):
        rospy.loginfo("pulse dpad %s (%d/%d) → changePolicy", direction, i + 1, args.steps)
        _pulse(pub, dpad_horizontal=horiz)
        time.sleep(1.6)

    rospy.loginfo(
        "done. 确认日志: 'algorithm: [lr]' / 'ctrl group:(lowerBody)'。"
        "然后 LB 进 RUNNING，再跑 upper_body_teleop.py"
    )


if __name__ == "__main__":
    main()
