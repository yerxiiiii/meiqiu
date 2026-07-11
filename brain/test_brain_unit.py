#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""brain 契约单测（无 ROS / 不控腿）。"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from modes import VOICE_TO_MODE, Mode
from neck_control import NeckServo
from uwb_intent import FollowController, UWBData
from camera_owner import CameraOwner


def main():
    assert VOICE_TO_MODE["face_look"] == Mode.FACE_LOOK
    assert VOICE_TO_MODE["uwb_follow"] == Mode.UWB_FOLLOW
    assert VOICE_TO_MODE["stop"] == Mode.IDLE
    print("  OK modes")

    n = NeckServo()
    y, p = n.update_from_face(0.5, 0.0, True, 1.0, 0.05)
    assert abs(y) > 0
    print("  OK neck_control")

    c = FollowController()
    fwd, rot, st, ang = c.calculate(UWBData(100, 0, 100, 0, 0, True))
    assert st in ("DIST_ONLY", "FOLLOW", "KEEPING", "BACK")
    print("  OK uwb_intent")

    cam = CameraOwner(enabled=False)
    cam.apply_mode(Mode.FACE_LOOK)
    assert cam.owner == "face"
    cam.apply_mode(Mode.IDLE)
    assert cam.owner is None
    print("  OK camera_owner")
    print("brain unit checks passed")


if __name__ == "__main__":
    main()
