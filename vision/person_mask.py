# -*- coding: utf-8 -*-
"""
目标人深度估计接口（骨架）。

真机联调避障时：用检测/分割把「跟随目标」从障碍里剔掉。
本文件先提供契约与无模型占位，不依赖 GPU 权重下载。

并行工程可替换 estimate_person_center_m()：
  - YOLO-person / NanoDet / 自研 RKNN
  - 输入：ZED 左目 + 深度；输出：目标人中心深度(m)
"""

from __future__ import annotations

from typing import Optional


def estimate_person_center_m(
    *,
    uwb_distance_cm: Optional[float] = None,
    detector_center_m: Optional[float] = None,
) -> Optional[float]:
    """
    返回目标人深度(m)，供 safety_gate person_center_m 使用。

    优先级：
      1) 视觉检测器给出的 detector_center_m（并行工程接入）
      2) UWB 距离换算（粗近似，仅作「别把人当墙」的弱提示）
    """
    if detector_center_m is not None and detector_center_m > 0.1:
        return float(detector_center_m)
    if uwb_distance_cm is not None and uwb_distance_cm > 20.0:
        # UWB 是斜距/标签距，粗略当水平深度用；宁可偏松也不要误刹
        return float(uwb_distance_cm) * 0.01
    return None
