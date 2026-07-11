#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
脸观测节点：ZED RGB + MediaPipe → /moon/face
只感知，不发脖子/腿指令。

  /moon/face Float32MultiArray: [dx_n, dy_n, has_face, valid]

由 mode_arbiter 在 FACE_LOOK 时拉起；勿与 zed_obstacle_node 同时开。
"""

from __future__ import annotations

import time

import cv2
import mediapipe as mp
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

TOPIC = "/moon/face"
PUBLISH_HZ = 15.0
PROC_MAX_W = 400
DETECT_CONFIDENCE = 0.4
TRACK_CONFIDENCE = 0.5
MESH_INTERVAL = 2


def open_zed():
    import pyzed.sl as sl

    zed = sl.Camera()
    init = sl.InitParameters()
    init.depth_mode = sl.DEPTH_MODE.NONE  # 只要 RGB
    init.camera_resolution = sl.RESOLUTION.VGA
    init.camera_fps = 30
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        # 部分 SDK 无 NONE，回退 PERFORMANCE
        init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED open failed: {status}")
    return sl, zed, sl.RuntimeParameters()


def make_msg(dx: float, dy: float, has_face: bool, valid: bool) -> Float32MultiArray:
    msg = Float32MultiArray()
    msg.layout = MultiArrayLayout(
        dim=[MultiArrayDimension(label="face", size=4, stride=4)],
        data_offset=0,
    )
    msg.data = [
        float(dx),
        float(dy),
        1.0 if has_face else 0.0,
        1.0 if valid else 0.0,
    ]
    return msg


def main():
    rospy.init_node("moon_face_obs", anonymous=False)
    pub = rospy.Publisher(TOPIC, Float32MultiArray, queue_size=2)

    print()
    print("╔══════════════════════════════════════════════════╗")
    print("║     Face Obs → /moon/face（不控头/腿）           ║")
    print("╚══════════════════════════════════════════════════╝")
    print()

    try:
        sl, zed, runtime = open_zed()
    except Exception as e:
        rospy.logerr("无法打开 ZED: %s", e)
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            pub.publish(make_msg(0, 0, False, False))
            rate.sleep()
        return

    image_mat = sl.Mat()
    face_mesh = mp.solutions.face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=False,
        min_detection_confidence=DETECT_CONFIDENCE,
        min_tracking_confidence=TRACK_CONFIDENCE,
    )
    rate = rospy.Rate(PUBLISH_HZ)
    frame_i = 0
    cached = (0.0, 0.0, False)
    print(f"\033[92m[FACE]\033[0m 已打开，发布 {TOPIC} @ {PUBLISH_HZ} Hz")

    try:
        while not rospy.is_shutdown():
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                pub.publish(make_msg(0, 0, False, False))
                rate.sleep()
                continue

            zed.retrieve_image(image_mat, sl.VIEW.LEFT)
            bgra = image_mat.get_data()
            bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
            h, w = bgr.shape[:2]
            scale = PROC_MAX_W / max(w, 1)
            if scale < 1.0:
                small = cv2.resize(
                    bgr,
                    (PROC_MAX_W, max(1, int(h * scale))),
                    interpolation=cv2.INTER_AREA,
                )
            else:
                small = bgr
            rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
            if not rgb.flags["C_CONTIGUOUS"]:
                rgb = np.ascontiguousarray(rgb)

            frame_i += 1
            run = frame_i % MESH_INTERVAL == 0 or not cached[2]
            if run:
                res = face_mesh.process(rgb)
                if res.multi_face_landmarks:
                    lm = res.multi_face_landmarks[0]
                    xs = [p.x for p in lm.landmark]
                    ys = [p.y for p in lm.landmark]
                    cx = sum(xs) / len(xs)
                    cy = sum(ys) / len(ys)
                    dx = (cx - 0.5) * 2.0
                    dy = (cy - 0.5) * 2.0
                    cached = (dx, dy, True)
                else:
                    cached = (0.0, 0.0, False)

            dx, dy, has = cached
            pub.publish(make_msg(dx, dy, has, True))

            if frame_i % 15 == 0:
                print(
                    f"\033[92m[FACE]\033[0m has={has} dx={dx:+.2f} dy={dy:+.2f}"
                )

            rate.sleep()
    finally:
        face_mesh.close()
        try:
            zed.close()
        except Exception:
            pass
        print("\033[93m[FACE]\033[0m 已关闭")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nface_obs 退出")
    except rospy.ROSInterruptException:
        pass
