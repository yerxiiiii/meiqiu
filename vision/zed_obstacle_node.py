#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZED 深度障碍节点
================
只做感知：读 ZED 深度 → 左/中/右 ROI → 发布 /moon/obstacle
不发布 /joy_msg、/cmd_vel。

运行：
  source .../install/setup.bash
  python3 /home/nvidia/moon/vision/zed_obstacle_node.py
"""

from __future__ import annotations

import math
import os
import sys
import time

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

# 同目录导入
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from obstacle_state import ObstacleState
from safety_gate import compute_caps_from_zones
from fpv_mjpeg import MjpegStream

# ------------------------------------------------------------
# 参数
# ------------------------------------------------------------
PUBLISH_HZ = 15.0
TOPIC = "/moon/obstacle"

# 深度模式：NEURAL 更准但更重；PERFORMANCE 更轻
DEPTH_MODE_NAME = "PERFORMANCE"  # PERFORMANCE | NEURAL | ULTRA
DEPTH_MAX_M = 3.0
DEPTH_MIN_M = 0.3

# ROI：相对图像宽高的比例（中下前方，避开天空）
# 三区水平划分，垂直取图像中下部
ROI_Y0, ROI_Y1 = 0.35, 0.85
ROI_LEFT = (0.05, 0.32)
ROI_CENTER = (0.35, 0.65)
ROI_RIGHT = (0.68, 0.95)

# 百分位：用较近的深度代表障碍（抗噪）
DEPTH_PERCENTILE = 15.0

# SSH 远程第一视角：本机开 MJPEG，笔记本端口转发后用浏览器看
ENABLE_FPV_STREAM = True
FPV_PORT = 8080
FPV_DRAW_ROI = True  # 画面上画出左/中/右检测框


def _open_zed():
    import pyzed.sl as sl

    zed = sl.Camera()
    init = sl.InitParameters()
    mode = getattr(sl.DEPTH_MODE, DEPTH_MODE_NAME, sl.DEPTH_MODE.PERFORMANCE)
    init.depth_mode = mode
    init.coordinate_units = sl.UNIT.METER
    init.depth_maximum_distance = DEPTH_MAX_M
    init.depth_minimum_distance = DEPTH_MIN_M
    init.camera_resolution = sl.RESOLUTION.VGA  # 避障够用，降负载
    init.camera_fps = 30

    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED open failed: {status}")
    runtime = sl.RuntimeParameters()
    return sl, zed, runtime


def _roi_percentile(depth: np.ndarray, x0: float, x1: float, y0: float, y1: float) -> float:
    h, w = depth.shape[:2]
    xa, xb = int(w * x0), int(w * x1)
    ya, yb = int(h * y0), int(h * y1)
    patch = depth[ya:yb, xa:xb].astype(np.float32).ravel()
    valid = patch[np.isfinite(patch) & (patch > DEPTH_MIN_M) & (patch < DEPTH_MAX_M)]
    if valid.size < 30:
        return float("nan")
    return float(np.percentile(valid, DEPTH_PERCENTILE))


def _make_msg(state: ObstacleState) -> Float32MultiArray:
    msg = Float32MultiArray()
    msg.layout = MultiArrayLayout(
        dim=[MultiArrayDimension(label="obstacle", size=6, stride=6)],
        data_offset=0,
    )
    msg.data = state.to_list()
    return msg


def _draw_fpv(frame_bgra, left: float, center: float, right: float, cap: float):
    """ZED LEFT 图是 BGRA；画 ROI 与距离文字后返回 BGR。"""
    import cv2

    bgr = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)
    h, w = bgr.shape[:2]
    y0, y1 = int(h * ROI_Y0), int(h * ROI_Y1)

    def box(x0r, x1r, color, label, dist):
        x0, x1 = int(w * x0r), int(w * x1r)
        cv2.rectangle(bgr, (x0, y0), (x1, y1), color, 2)
        d = f"{dist:.2f}m" if math.isfinite(dist) else "n/a"
        cv2.putText(
            bgr, f"{label} {d}", (x0 + 4, max(20, y0 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA,
        )

    if FPV_DRAW_ROI:
        box(*ROI_LEFT, (80, 180, 255), "L", left)
        box(*ROI_CENTER, (80, 255, 120), "C", center)
        box(*ROI_RIGHT, (80, 180, 255), "R", right)
        cv2.putText(
            bgr, f"cap={cap:.2f}", (10, h - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2, cv2.LINE_AA,
        )
    return bgr


def main():
    rospy.init_node("zed_obstacle", anonymous=False)
    pub = rospy.Publisher(TOPIC, Float32MultiArray, queue_size=2)

    print()
    print("╔══════════════════════════════════════════════════╗")
    print("║         ZED 障碍感知 (/moon/obstacle)            ║")
    print("║  只发布深度门控，不控制电机                       ║")
    print("╚══════════════════════════════════════════════════╝")
    print()

    fpv = None
    if ENABLE_FPV_STREAM:
        fpv = MjpegStream(port=FPV_PORT)
        fpv.start()
        print(f"\033[92m[FPV]\033[0m http://<机器人IP>:{FPV_PORT}/  或 SSH 转发后开本机浏览器")
        print(f"\033[93m[FPV]\033[0m 笔记本执行: ssh -L {FPV_PORT}:localhost:{FPV_PORT} nvidia@<机器人IP>")
        print(f"\033[93m[FPV]\033[0m 然后浏览器打开: http://localhost:{FPV_PORT}/")

    try:
        sl, zed, runtime = _open_zed()
    except Exception as e:
        rospy.logerr("无法打开 ZED: %s", e)
        # 持续发 invalid，让跟随侧可知视觉挂了
        rate = rospy.Rate(2)
        bad = ObstacleState(valid=False, forward_cap=0.0)
        while not rospy.is_shutdown():
            pub.publish(_make_msg(bad))
            rate.sleep()
        return

    depth_mat = sl.Mat()
    image_mat = sl.Mat()
    rate = rospy.Rate(PUBLISH_HZ)
    tick = 0
    print(f"\033[92m[ZED]\033[0m 已打开，发布 {TOPIC} @ {PUBLISH_HZ} Hz")

    try:
        while not rospy.is_shutdown():
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                state = ObstacleState(valid=False, forward_cap=0.0)
                pub.publish(_make_msg(state))
                rate.sleep()
                continue

            zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
            depth = depth_mat.get_data()
            # ZED Mat 可能是 HxWxC；取第一通道
            if depth.ndim == 3:
                depth = depth[:, :, 0]

            left = _roi_percentile(depth, *ROI_LEFT, ROI_Y0, ROI_Y1)
            center = _roi_percentile(depth, *ROI_CENTER, ROI_Y0, ROI_Y1)
            right = _roi_percentile(depth, *ROI_RIGHT, ROI_Y0, ROI_Y1)
            cap, bias = compute_caps_from_zones(left, center, right)

            state = ObstacleState(
                left_m=left,
                center_m=center,
                right_m=right,
                forward_cap=cap,
                rotate_bias=bias,
                valid=True,
                stamp=time.time(),
            )
            pub.publish(_make_msg(state))

            if fpv is not None:
                zed.retrieve_image(image_mat, sl.VIEW.LEFT)
                frame = image_mat.get_data()
                fpv.update_bgr(_draw_fpv(frame, left, center, right, cap))

            tick += 1
            if tick % 15 == 0:
                def fmt(d):
                    return f"{d:4.2f}" if math.isfinite(d) else " nan"
                print(
                    f"\033[92m[OBS]\033[0m L:{fmt(left)} C:{fmt(center)} R:{fmt(right)} "
                    f"| cap:{cap:4.2f} bias:{bias:+5.2f}"
                )

            rate.sleep()
    finally:
        if fpv is not None:
            fpv.stop()
        try:
            zed.close()
        except Exception:
            pass
        print("\033[93m[ZED]\033[0m 已关闭")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n视觉节点退出")
    except rospy.ROSInterruptException:
        pass
