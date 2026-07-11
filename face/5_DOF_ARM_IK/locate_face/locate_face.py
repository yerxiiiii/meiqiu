#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================
# locate_face.py  ——  人脸定位 + 脖子视觉跟随 (face-centering)
#
# 思路:
#   1. ZED Mini 左眼采集图像 (沿用 gaze_robot.py 的相机/显示框架)
#   2. MediaPipe Face Mesh / Face Detection 找到当前人脸框中心
#   3. 计算脸中心相对画面中心的归一化像素偏差 (dx_n, dy_n) ∈ [-1, 1]
#   4. 比例增量控制:
#         delta_yaw   = -K_YAW  * dx_n     (脸偏右 -> 头向右转)
#         delta_pitch = +K_PITCH * dy_n    (脸偏下 -> 头向下俯, +pitch=低头)
#      经死区/速率限制/软限位后更新 yaw_target / pitch_target
#   5. 独立的控制线程以 PUBLISH_RATE_HZ 持续把最新 target 发到
#      sim2real /pi_plus_absolute  (FSM 必须先进入 EXEC_DEFAULT(5))
#
# 安全:
#   - 启动时等待 /fsm_state == EXEC_DEFAULT(5) 才开始下发命令
#   - 没检测到人脸时 target 保持不变(不会乱扫)
#   - 软限位: yaw ±YAW_LIMIT_DEG, pitch ∈ [PITCH_UP_DEG, PITCH_DOWN_DEG]
#   - Ctrl+C / ESC / 窗口关闭时自动回中
#
# 运行:
#     python locate_face.py
# ==============================================================

import os
import subprocess
import threading
import time
import math
import argparse

# ===== SSH 无 DISPLAY 兜底: 必须在 import cv2 之前设 =====
if not os.environ.get("DISPLAY"):
    os.environ["DISPLAY"] = ":0"
if not os.environ.get("XAUTHORITY"):
    _xauth = os.path.expanduser("~/.Xauthority")
    if os.path.exists(_xauth):
        os.environ["XAUTHORITY"] = _xauth

import cv2
import numpy as np
import mediapipe as mp

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

# =====================================================
#                    可调参数
# =====================================================

# ----- ROS / sim2real 接口 -----
ABSOLUTE_TOPIC = "/pi_plus_absolute"
HEAD_YAW_JOINT = "head_yaw_joint"      # 左右(+ 向左,- 向右)
HEAD_PITCH_JOINT = "head_pitch_joint"  # 上下(- 抬头,+ 低头)

# ----- 相机配置 (沿用 gaze_robot.py) -----
# ZED Mini 双目可选分辨率: 4416x1242@15/3840x1080@30/2560x720@60/1344x376@100
WIDTH, HEIGHT = 2560, 720       # 这里用 720p,响应更快;高清可改回 4416x1242
TARGET_FPS = 30
CAM_ID = 0
USE_MJPG = False                # ZED Mini 不支持 MJPG
ZED_STEREO = True               # 双目拼接,只取左半边
PROC_MAX_W = 960                # 喂给 MediaPipe 的最大宽度(降采样保证流畅)

# ----- 显示 -----
FULLSCREEN = True
WINDOW_NAME = "Locate Face (Orin Nano)"

# ----- 人脸检测器 -----
DETECT_CONFIDENCE = 0.4         # Face Detection 阈值
TRACK_CONFIDENCE = 0.5          # Face Mesh 跟踪阈值
ROI_PAD_RATIO = 0.30            # 跟丢后用 detection 找 ROI 的扩边比例

# ----- 视觉伺服控制律 -----
# 死区: 脸中心偏差 |dx_n| < DEAD_BAND 视为已对中,不更新 target
# 单位是"画面半宽"的归一化值,0.05 ≈ 画面 5% 宽度的中心容差
DEAD_BAND_X = 0.04
DEAD_BAND_Y = 0.05

# 比例增益(度/单位归一化偏差):
# 直观: dx_n=1(脸完全在画面右边缘)时,头一帧最多多转 K_YAW 度.
#   ZED Mini 单眼水平 FOV ≈ 87°,半视场 ≈ 43°,所以理论上"完美对齐"应取 43.
#   实际取小一点更稳,避免过冲;视觉伺服会持续闭环修正.
K_YAW_DEG = 20.0
K_PITCH_DEG = 15.0

# 每帧目标角度变化的最大幅度(度),防止突然甩头
MAX_STEP_YAW_DEG = 6.0
MAX_STEP_PITCH_DEG = 5.0

# 目标位置低通滤波系数 ∈ (0, 1] (越小越平滑,响应也越慢)
# 经过死区 + 速率限制后再做一次 EMA,抹掉视觉抖动残留
TARGET_EMA_ALPHA = 0.6

# ----- 软限位(单位:度) -----
# 来自 pd.yaml 软件限位 ±1.58 rad ≈ ±90.5°,这里取保守值
YAW_LIMIT_DEG = 80.0
PITCH_UP_DEG = -40.0     # 抬头极限
PITCH_DOWN_DEG = 60.0    # 低头极限

# ----- 控制线程发布频率 -----
PUBLISH_RATE_HZ = 50

# ----- 是否发送速度前馈 -----
# 视觉伺服下,target 来自视觉,velocity 容易噪;先关掉,只发位置
ENABLE_VEL_FEEDFORWARD = False

# ----- FSM 守门 (参考 neck_move_demo.py) -----
FSM_STATE_TOPIC = "/fsm_state"
FSM_EXEC_DEFAULT = 5
FSM_GATE_ENABLED = True
FSM_WAIT_TIMEOUT = 30.0

# ----- 找不到人脸超时(秒): 超过则平滑回中 -----
# >0 启用; 0 表示"保持最后位置不动"
NO_FACE_RETURN_HOME_SEC = 1.0
# 回中速率(度/秒): 越大回得越快;过大会瞬间甩头
RETURN_HOME_RATE_DEG_PER_SEC = 45.0

# =====================================================
#                    工具函数 / 类
# =====================================================


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def open_camera() -> cv2.VideoCapture:
    cap = cv2.VideoCapture(CAM_ID, cv2.CAP_V4L2)
    if USE_MJPG:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def detect_screen_size(default=(1920, 1080)):
    """通过 xrandr 读主屏分辨率,失败则用默认值。"""
    try:
        env = os.environ.copy()
        env.setdefault("DISPLAY", ":0")
        out = subprocess.check_output(
            ["xrandr"], env=env, stderr=subprocess.DEVNULL, timeout=2
        ).decode()
        for line in out.splitlines():
            if "*" in line:
                token = line.strip().split()[0]
                w, h = token.split("x")
                return int(w), int(h)
    except Exception:
        pass
    return default


def fit_letterbox(img, target_w, target_h):
    """等比缩放到 target,留黑边,保持长宽比。"""
    src_h, src_w = img.shape[:2]
    scale = min(target_w / src_w, target_h / src_h)
    new_w, new_h = int(src_w * scale), int(src_h * scale)
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    x = (target_w - new_w) // 2
    y = (target_h - new_h) // 2
    canvas[y:y + new_h, x:x + new_w] = resized
    return canvas


def compute_proc_size(src_w, src_h, max_w):
    if src_w <= max_w:
        return src_w, src_h
    scale = max_w / src_w
    return int(src_w * scale), int(src_h * scale)


# ----- FSM 状态监听 -----
class FsmStateMonitor:
    """订阅 /fsm_state 维护最新 FSM 状态。"""

    _NAME_MAP = {
        0: "INIT", 1: "ERROR",
        2: "CANDIDATE_DEFAULT", 3: "CANDIDATE_CUSTOM",
        4: "CANDIDATE_REMOTE",
        5: "EXEC_DEFAULT", 6: "EXEC_CUSTOM", 7: "EXEC_REMOTE",
        8: "PROTECTION_SHUTDOWN",
        9: "CANDIDATE_CALIBRATION", 10: "EXEC_CALIBRATING",
        11: "EXEC_CALIB_OK", 12: "EXEC_CALIB_FAILED",
        13: "CANDIDATE_TEACHING", 14: "EXEC_TEACHING",
        15: "CANDIDATE_DEVELOP", 16: "EXEC_DEVELOP",
    }

    def __init__(self, topic: str = FSM_STATE_TOPIC):
        self._lock = threading.Lock()
        self._state = None
        self._sub = rospy.Subscriber(topic, Int32, self._cb, queue_size=10)

    def _cb(self, msg):
        with self._lock:
            self._state = int(msg.data)

    @property
    def state(self):
        with self._lock:
            return self._state

    @classmethod
    def state_name(cls, v) -> str:
        return cls._NAME_MAP.get(v, f"UNKNOWN({v})")

    def is_exec_default(self) -> bool:
        return self.state == FSM_EXEC_DEFAULT

    def wait_for_exec_default(self, timeout: float = FSM_WAIT_TIMEOUT) -> bool:
        deadline = time.time() + timeout
        last_log_t = 0.0
        warned_timeout = False
        while not rospy.is_shutdown():
            s = self.state
            if s == FSM_EXEC_DEFAULT:
                return True
            now = time.time()
            if now - last_log_t >= 1.0:
                if s is None:
                    rospy.logwarn(
                        "[FSM] 还没收到 %s, 请确认 sim2real_master 已启动",
                        FSM_STATE_TOPIC,
                    )
                else:
                    rospy.logwarn(
                        "[FSM] 当前状态 %s(%d) != EXEC_DEFAULT(5)",
                        self.state_name(s), s,
                    )
                last_log_t = now
            if not warned_timeout and now > deadline:
                rospy.logerr(
                    "[FSM] 等待 %.0fs 仍未进入 EXEC_DEFAULT,继续等...",
                    timeout,
                )
                warned_timeout = True
            time.sleep(0.1)
        return False


# ----- 共享的脖子目标角度(线程安全) -----
class NeckTarget:
    """yaw / pitch 的目标值(弧度),供控制线程读取,视觉线程更新。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._yaw = 0.0
        self._pitch = 0.0
        self._updated_t = time.time()

    def set(self, yaw_rad: float, pitch_rad: float):
        with self._lock:
            self._yaw = yaw_rad
            self._pitch = pitch_rad
            self._updated_t = time.time()

    def get(self):
        with self._lock:
            return self._yaw, self._pitch

    @property
    def last_update_age(self):
        with self._lock:
            return time.time() - self._updated_t


# ----- 控制线程: 固定频率发布最新 target 到 master -----
class NeckController(threading.Thread):
    """独立线程: 以 PUBLISH_RATE_HZ 把 NeckTarget 持续发到 ABSOLUTE_TOPIC。

    保证视觉处理慢一拍时,master 端依旧能稳定收到命令(避免被认为信号丢失)。
    """

    def __init__(self, target: NeckTarget, fsm: 'FsmStateMonitor | None'):
        super().__init__(daemon=True)
        self._target = target
        self._fsm = fsm
        self._stop_evt = threading.Event()
        self._pub = rospy.Publisher(ABSOLUTE_TOPIC, JointState, queue_size=10)
        self._rate = rospy.Rate(PUBLISH_RATE_HZ)

    @property
    def num_subscribers(self) -> int:
        return self._pub.get_num_connections()

    def stop(self):
        self._stop_evt.set()

    def publish_center_blocking(self, duration: float = 0.5):
        """阻塞地连发回中指令(用于退出时)。"""
        msg = JointState()
        msg.name = [HEAD_YAW_JOINT, HEAD_PITCH_JOINT]
        msg.position = [0.0, 0.0]
        msg.velocity = []
        msg.effort = []
        end_t = time.time() + duration
        while time.time() < end_t:
            msg.header.stamp = rospy.Time.now()
            try:
                self._pub.publish(msg)
            except Exception:
                break
            time.sleep(1.0 / max(PUBLISH_RATE_HZ, 1))

    def run(self):
        msg = JointState()
        msg.name = [HEAD_YAW_JOINT, HEAD_PITCH_JOINT]
        msg.velocity = []
        msg.effort = []
        warned_no_fsm = False
        while not self._stop_evt.is_set() and not rospy.is_shutdown():
            # FSM 守门: 不在 EXEC_DEFAULT 时不下发命令(避免在 INIT/PROTECTION 下乱抖)
            if self._fsm is not None and not self._fsm.is_exec_default():
                if not warned_no_fsm:
                    rospy.logwarn_throttle(
                        2.0,
                        "[ctrl] FSM 当前 %s,暂停下发(等待 EXEC_DEFAULT)",
                        FsmStateMonitor.state_name(self._fsm.state),
                    )
                    warned_no_fsm = True
                self._rate.sleep()
                continue
            warned_no_fsm = False
            yaw, pitch = self._target.get()
            msg.position = [yaw, pitch]
            if ENABLE_VEL_FEEDFORWARD:
                msg.velocity = [0.0, 0.0]
            msg.header.stamp = rospy.Time.now()
            try:
                self._pub.publish(msg)
            except Exception as e:
                rospy.logerr_throttle(2.0, "[ctrl] publish 异常: %s", e)
            self._rate.sleep()


# =====================================================
#                    视觉与控制律
# =====================================================


def face_bbox_from_landmarks(landmarks, w, h, pad=10):
    """从 mediapipe Face Mesh 整脸 landmarks 取像素边界框。"""
    xs = [p.x for p in landmarks.landmark]
    ys = [p.y for p in landmarks.landmark]
    x1 = max(0, int(min(xs) * w) - pad)
    y1 = max(0, int(min(ys) * h) - pad)
    x2 = min(w - 1, int(max(xs) * w) + pad)
    y2 = min(h - 1, int(max(ys) * h) + pad)
    return x1, y1, x2, y2


def detect_face_roi_bbox(face_detector, rgb, w, h, pad_ratio=ROI_PAD_RATIO):
    """用 Face Detection 在全图上找最大人脸,返回扩边后的像素 ROI 框。"""
    det = face_detector.process(rgb)
    if not det.detections:
        return None
    best = max(det.detections, key=lambda d: d.score[0])
    rel = best.location_data.relative_bounding_box
    bx = rel.xmin * w
    by = rel.ymin * h
    bw = rel.width * w
    bh = rel.height * h
    pad_x = bw * pad_ratio
    pad_y = bh * pad_ratio
    x1 = max(0, int(bx - pad_x))
    y1 = max(0, int(by - pad_y))
    x2 = min(w, int(bx + bw + pad_x))
    y2 = min(h, int(by + bh + pad_y))
    if x2 - x1 < 20 or y2 - y1 < 20:
        return None
    return (x1, y1, x2, y2)


def update_target_from_error(
    yaw_cur_rad: float, pitch_cur_rad: float,
    dx_n: float, dy_n: float,
    state: dict,
):
    """根据归一化像素偏差 (dx_n, dy_n) 计算新的 yaw/pitch 目标(弧度)。

    控制律(每帧):
      1) 死区: |dx_n| < DEAD_BAND_X 时 dx_n 视为 0 (y 同理)
      2) 比例增量: delta = -K * dx_n (yaw)  /  +K * dy_n (pitch)  单位度
      3) 速率限制: |delta| 不超过 MAX_STEP_DEG
      4) EMA 滤波: 用 TARGET_EMA_ALPHA 与上一次平滑后的 target 融合
      5) 软限位: clamp 到 YAW_LIMIT_DEG / [PITCH_UP_DEG, PITCH_DOWN_DEG]

    state: 跨帧记忆字典(包含上一拍 ema 后的 yaw/pitch_rad)。
    """
    # 1) 死区
    if abs(dx_n) < DEAD_BAND_X:
        dx_n = 0.0
    if abs(dy_n) < DEAD_BAND_Y:
        dy_n = 0.0

    # 2) 比例(度)
    delta_yaw_deg = -K_YAW_DEG * dx_n
    delta_pitch_deg = +K_PITCH_DEG * dy_n

    # 3) 速率限制
    delta_yaw_deg = clamp(delta_yaw_deg, -MAX_STEP_YAW_DEG, MAX_STEP_YAW_DEG)
    delta_pitch_deg = clamp(
        delta_pitch_deg, -MAX_STEP_PITCH_DEG, MAX_STEP_PITCH_DEG,
    )

    # 用 "当前正在被发布的" target 作为起点(即 state 里保存的 ema 后值)
    base_yaw_rad = state.get("yaw_rad", yaw_cur_rad)
    base_pitch_rad = state.get("pitch_rad", pitch_cur_rad)
    raw_yaw_rad = base_yaw_rad + math.radians(delta_yaw_deg)
    raw_pitch_rad = base_pitch_rad + math.radians(delta_pitch_deg)

    # 4) EMA 与基准 (即 base_*) 融合, alpha 越小越粘滞
    a = TARGET_EMA_ALPHA
    yaw_new = base_yaw_rad * (1 - a) + raw_yaw_rad * a
    pitch_new = base_pitch_rad * (1 - a) + raw_pitch_rad * a

    # 5) 软限位
    yaw_new = clamp(
        yaw_new,
        -math.radians(YAW_LIMIT_DEG),
        +math.radians(YAW_LIMIT_DEG),
    )
    pitch_new = clamp(
        pitch_new,
        math.radians(PITCH_UP_DEG),
        math.radians(PITCH_DOWN_DEG),
    )

    state["yaw_rad"] = yaw_new
    state["pitch_rad"] = pitch_new
    return yaw_new, pitch_new


# =====================================================
#                    主流程
# =====================================================


def main():
    parser = argparse.ArgumentParser(description="locate_face: 视觉伺服脖子对中")
    parser.add_argument(
        "--no-gui", action="store_true",
        help="不开图形窗口(纯后台,仅日志)",
    )
    parser.add_argument(
        "--no-fsm", action="store_true",
        help="跳过 FSM 守门(直接下发命令,谨慎使用)",
    )
    args = parser.parse_args()

    rospy.init_node("locate_face", anonymous=False, disable_signals=False)

    # ----- 相机 -----
    cap = open_camera()
    if not cap.isOpened():
        rospy.logerr("[cam] 无法打开相机 /dev/video%d", CAM_ID)
        return
    raw_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    raw_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    real_fps = cap.get(cv2.CAP_PROP_FPS)
    real_w = raw_w // 2 if ZED_STEREO else raw_w
    real_h = raw_h
    proc_w, proc_h = compute_proc_size(real_w, real_h, PROC_MAX_W)
    rospy.loginfo(
        "[cam] 原始 %dx%d@%.1ffps  实际左眼 %dx%d  MediaPipe %dx%d",
        raw_w, raw_h, real_fps, real_w, real_h, proc_w, proc_h,
    )

    # ----- MediaPipe -----
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=False,   # 这里不需要瞳孔,关掉省算力
        min_detection_confidence=DETECT_CONFIDENCE,
        min_tracking_confidence=TRACK_CONFIDENCE,
    )
    face_mesh_roi = mp_face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=False,
        min_detection_confidence=DETECT_CONFIDENCE,
        min_tracking_confidence=TRACK_CONFIDENCE,
    )
    mp_face_detection = mp.solutions.face_detection
    face_detector = mp_face_detection.FaceDetection(
        model_selection=1, min_detection_confidence=DETECT_CONFIDENCE,
    )

    # ----- 显示 -----
    screen_w, screen_h = (0, 0)
    if not args.no_gui:
        screen_w, screen_h = detect_screen_size()
        rospy.loginfo(
            "[gui] 屏幕 %dx%d 全屏=%s  (ESC/q 退出, f 切全屏)",
            screen_w, screen_h, FULLSCREEN,
        )
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        if FULLSCREEN:
            cv2.setWindowProperty(
                WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN,
            )
    is_fullscreen = FULLSCREEN

    # ----- FSM / 控制线程 -----
    fsm = None if args.no_fsm else FsmStateMonitor()
    target = NeckTarget()  # 默认 0,0 (回中位)
    controller = NeckController(target, fsm)

    def on_shutdown():
        rospy.logwarn("[shutdown] 退出 -> 回中")
        controller.stop()
        try:
            controller.publish_center_blocking(0.5)
        except Exception:
            pass

    rospy.on_shutdown(on_shutdown)
    controller.start()

    # 等订阅者(master)上线
    t0 = time.time()
    while controller.num_subscribers == 0 and time.time() - t0 < 3.0 \
            and not rospy.is_shutdown():
        time.sleep(0.1)
    if controller.num_subscribers == 0:
        rospy.logwarn(
            "[ctrl] %s 上还没有订阅者(master 可能未启动),继续运行",
            ABSOLUTE_TOPIC,
        )
    else:
        rospy.loginfo(
            "[ctrl] %d 订阅者已连接到 %s",
            controller.num_subscribers, ABSOLUTE_TOPIC,
        )

    # 等 FSM 进入 EXEC_DEFAULT
    if fsm is not None:
        rospy.loginfo("[FSM] 等待 EXEC_DEFAULT(5)...")
        fsm.wait_for_exec_default(FSM_WAIT_TIMEOUT)
        rospy.loginfo("[FSM] OK,进入视觉伺服循环")

    # ----- 视觉伺服主循环 -----
    ctrl_state = {"yaw_rad": 0.0, "pitch_rad": 0.0}  # 已发出的最新 target
    last_face_t = time.time()
    last_loop_t = time.time()
    homing_logged = False  # 防止"开始回中"日志刷屏
    fps_t0 = time.time()
    fps_frames = 0
    fps_show = 0.0
    last_log_t = 0.0

    while not rospy.is_shutdown() and cap.isOpened():
        loop_now = time.time()
        dt_frame = max(1e-3, min(0.2, loop_now - last_loop_t))  # 防极端值
        last_loop_t = loop_now
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn_throttle(2.0, "[cam] 抓帧失败")
            continue
        if ZED_STEREO:
            frame = frame[:, : frame.shape[1] // 2]

        # FPS
        fps_frames += 1
        if fps_frames >= 10:
            now = time.time()
            fps_show = fps_frames / (now - fps_t0)
            fps_t0 = now
            fps_frames = 0

        # 降采样喂 MediaPipe
        if (proc_w, proc_h) != (real_w, real_h):
            proc_bgr = cv2.resize(frame, (proc_w, proc_h))
        else:
            proc_bgr = frame
        rgb = cv2.cvtColor(proc_bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        res = face_mesh.process(rgb)

        # ROI 兜底
        used_roi = False
        if not res.multi_face_landmarks:
            bbox = detect_face_roi_bbox(face_detector, rgb, proc_w, proc_h)
            if bbox is not None:
                x1, y1, x2, y2 = bbox
                roi = rgb[y1:y2, x1:x2].copy()
                roi.flags.writeable = False
                res2 = face_mesh_roi.process(roi)
                if res2.multi_face_landmarks:
                    # 把 ROI 内归一化坐标映射回 proc 全图
                    rw_roi = x2 - x1
                    rh_roi = y2 - y1
                    for lms in res2.multi_face_landmarks:
                        for lm in lms.landmark:
                            lm.x = (lm.x * rw_roi + x1) / proc_w
                            lm.y = (lm.y * rh_roi + y1) / proc_h
                    res = res2
                    used_roi = True

        # ----- 控制律 -----
        h, w = frame.shape[:2]
        cx_img, cy_img = w / 2.0, h / 2.0
        face_cx = face_cy = None
        face_bbox_disp = None
        if res.multi_face_landmarks:
            lm = res.multi_face_landmarks[0]
            # 归一化坐标 -> 单眼显示图像素坐标
            fx1, fy1, fx2, fy2 = face_bbox_from_landmarks(lm, w, h, pad=12)
            face_bbox_disp = (fx1, fy1, fx2, fy2)
            face_cx = (fx1 + fx2) / 2.0
            face_cy = (fy1 + fy2) / 2.0

            # 归一化偏差: 以画面半宽/半高为 1
            dx_n = (face_cx - cx_img) / (w / 2.0)
            dy_n = (face_cy - cy_img) / (h / 2.0)

            cur_yaw, cur_pitch = target.get()
            new_yaw, new_pitch = update_target_from_error(
                cur_yaw, cur_pitch, dx_n, dy_n, ctrl_state,
            )
            target.set(new_yaw, new_pitch)
            last_face_t = loop_now
            homing_logged = False
        else:
            # 找不到脸: 默认目标保持不动; 丢失超 NO_FACE_RETURN_HOME_SEC 秒后
            # 以 RETURN_HOME_RATE_DEG_PER_SEC 的速率平滑朝 (0, 0) 回中
            lost_dur = loop_now - last_face_t
            if (NO_FACE_RETURN_HOME_SEC > 0
                    and lost_dur > NO_FACE_RETURN_HOME_SEC):
                cur_yaw, cur_pitch = target.get()
                step_rad = math.radians(
                    RETURN_HOME_RATE_DEG_PER_SEC * dt_frame
                )
                # 朝 0 移动,步长不超过当前剩余距离
                new_yaw = (cur_yaw - math.copysign(
                    min(step_rad, abs(cur_yaw)), cur_yaw
                )) if abs(cur_yaw) > 1e-4 else 0.0
                new_pitch = (cur_pitch - math.copysign(
                    min(step_rad, abs(cur_pitch)), cur_pitch
                )) if abs(cur_pitch) > 1e-4 else 0.0
                target.set(new_yaw, new_pitch)
                ctrl_state["yaw_rad"] = new_yaw
                ctrl_state["pitch_rad"] = new_pitch
                if not homing_logged:
                    rospy.loginfo(
                        "[homing] 丢失 %.1fs > %.1fs,开始平滑回中",
                        lost_dur, NO_FACE_RETURN_HOME_SEC,
                    )
                    homing_logged = True

        # ----- 可视化 -----
        if not args.no_gui:
            draw_scale = max(1.0, h / 720.0)
            thick1 = max(1, int(2 * draw_scale))
            thick2 = max(2, int(3 * draw_scale))

            # 画面中心十字
            cx_i, cy_i = int(cx_img), int(cy_img)
            cv2.drawMarker(
                frame, (cx_i, cy_i), (255, 255, 255),
                cv2.MARKER_CROSS, max(20, int(30 * draw_scale)),
                thickness=thick1,
            )

            # 死区方框(画面正中央)
            dx_pix = int(DEAD_BAND_X * w / 2.0)
            dy_pix = int(DEAD_BAND_Y * h / 2.0)
            cv2.rectangle(
                frame,
                (cx_i - dx_pix, cy_i - dy_pix),
                (cx_i + dx_pix, cy_i + dy_pix),
                (90, 90, 90), thick1,
            )

            if face_bbox_disp is not None:
                fx1, fy1, fx2, fy2 = face_bbox_disp
                col = (0, 255, 0) if not used_roi else (0, 255, 255)
                cv2.rectangle(frame, (fx1, fy1), (fx2, fy2), col, thick1)
                cv2.circle(
                    frame, (int(face_cx), int(face_cy)),
                    max(4, int(6 * draw_scale)), col, -1,
                )
                # 中心 -> 脸中心 的偏差线
                cv2.line(
                    frame, (cx_i, cy_i),
                    (int(face_cx), int(face_cy)),
                    col, thick1,
                )
                tag = "FACE" + (" [ROI]" if used_roi else "")
                cv2.putText(
                    frame, tag, (fx1, max(0, fy1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7 * draw_scale, col, thick1,
                )

            tgt_yaw, tgt_pitch = target.get()
            fsm_text = "off"
            if fsm is not None:
                s = fsm.state
                fsm_text = (
                    f"{FsmStateMonitor.state_name(s)}({s})"
                    if s is not None else "wait"
                )
            status_lines = [
                f"FPS {fps_show:5.1f}",
                f"target yaw  = {math.degrees(tgt_yaw):+6.2f} deg",
                f"target pitch= {math.degrees(tgt_pitch):+6.2f} deg",
                f"FSM {fsm_text}",
            ]
            if face_bbox_disp is not None:
                dx_n = (face_cx - cx_img) / (w / 2.0)
                dy_n = (face_cy - cy_img) / (h / 2.0)
                status_lines.insert(
                    1, f"dx={dx_n:+.2f}  dy={dy_n:+.2f}",
                )
            else:
                status_lines.insert(1, "no face")

            y_text = int(40 * draw_scale)
            for line in status_lines:
                cv2.putText(
                    frame, line, (20, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7 * draw_scale,
                    (0, 255, 0), thick1,
                )
                y_text += int(34 * draw_scale)

            lost_dur = loop_now - last_face_t
            if face_bbox_disp is not None:
                head_color = (0, 200, 255)
                head_text = "TRACKING"
            elif (NO_FACE_RETURN_HOME_SEC > 0
                    and lost_dur > NO_FACE_RETURN_HOME_SEC):
                head_color = (255, 200, 0)
                head_text = f"HOMING ({lost_dur:.1f}s)"
            else:
                head_color = (0, 0, 255)
                head_text = (
                    f"NO FACE ({lost_dur:.1f}s)"
                    if NO_FACE_RETURN_HOME_SEC > 0 else "NO FACE"
                )
            cv2.putText(
                frame, head_text, (20, h - int(20 * draw_scale)),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0 * draw_scale,
                head_color, thick2,
            )

            show = fit_letterbox(frame, screen_w, screen_h)
            cv2.imshow(WINDOW_NAME, show)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):
                break
            if key == ord("f"):
                is_fullscreen = not is_fullscreen
                cv2.setWindowProperty(
                    WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN if is_fullscreen
                    else cv2.WINDOW_NORMAL,
                )

        # 定期终端日志
        now = time.time()
        if now - last_log_t > 1.0:
            tgt_yaw, tgt_pitch = target.get()
            rospy.loginfo(
                "[track] face=%s  FPS=%.1f  yaw=%+6.2f° pitch=%+6.2f°",
                "Y" if res.multi_face_landmarks else "N",
                fps_show, math.degrees(tgt_yaw), math.degrees(tgt_pitch),
            )
            last_log_t = now

    # 退出清理
    rospy.loginfo("[exit] 主循环结束,清理资源")
    cap.release()
    if not args.no_gui:
        cv2.destroyAllWindows()
    controller.stop()
    try:
        controller.publish_center_blocking(0.5)
    except Exception:
        pass


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
