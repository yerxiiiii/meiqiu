# -*- coding: utf-8 -*-
"""模式与语音命令契约。"""

from __future__ import annotations

from enum import Enum


class Mode(str, Enum):
    IDLE = "IDLE"
    FACE_LOOK = "FACE_LOOK"
    UWB_FOLLOW = "UWB_FOLLOW"


# /moon/voice_cmd 载荷（std_msgs/String.data）
VOICE_CMD_FACE_LOOK = "face_look"
VOICE_CMD_UWB_FOLLOW = "uwb_follow"
VOICE_CMD_STOP = "stop"

VOICE_TO_MODE = {
    VOICE_CMD_FACE_LOOK: Mode.FACE_LOOK,
    VOICE_CMD_UWB_FOLLOW: Mode.UWB_FOLLOW,
    VOICE_CMD_STOP: Mode.IDLE,
    # 别名
    "idle": Mode.IDLE,
    "look": Mode.FACE_LOOK,
    "follow": Mode.UWB_FOLLOW,
}

# 话题
TOPIC_VOICE_CMD = "/moon/voice_cmd"
TOPIC_MODE = "/moon/mode"
TOPIC_FACE = "/moon/face"
TOPIC_OBSTACLE = "/moon/obstacle"
TOPIC_CAMERA_OWNER = "/moon/camera_owner"

# 运控 walk 策略名（与 sim2real rl_config / AMP_RIGHT_HOLD.md 一致）
WALK_POLICY_FOLLOW = "amp_right_hold"

# /moon/face Float32MultiArray: [dx_n, dy_n, has_face, valid]
FACE_LAYOUT_LEN = 4
