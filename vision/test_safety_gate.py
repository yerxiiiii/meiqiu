#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""safety_gate 单测：无相机、无 ROS、不控腿。"""

from __future__ import annotations

import math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from obstacle_state import ObstacleState
from safety_gate import apply_safety_gate, compute_caps_from_zones, STOP_DIST


def _approx(a, b, eps=1e-3):
    return abs(a - b) <= eps


def test_stop_when_near():
    cap, bias = compute_caps_from_zones(2.0, STOP_DIST - 0.05, 2.0)
    assert cap == 0.0, cap
    fwd, rot, reason = apply_safety_gate(
        0.5, 0.0,
        ObstacleState(2.0, STOP_DIST - 0.05, 2.0, cap, bias, True),
    )
    assert fwd == 0.0 and reason.startswith("STOP"), (fwd, reason)


def test_clear_far():
    cap, bias = compute_caps_from_zones(2.5, 2.5, 2.5)
    assert _approx(cap, 1.0), cap
    fwd, rot, reason = apply_safety_gate(
        0.5, 0.1,
        ObstacleState(2.5, 2.5, 2.5, cap, bias, True),
    )
    assert _approx(fwd, 0.5) and reason == "CLEAR", (fwd, reason)


def test_sidestep_prefers_open_side():
    # 中间近、右边更通 → bias > 0（建议右转）；默认 blend<1 仍应同号
    cap, bias = compute_caps_from_zones(0.5, 0.55, 2.0)
    assert bias > 0, bias
    fwd, rot, reason = apply_safety_gate(
        0.3, 0.0,
        ObstacleState(0.5, 0.55, 2.0, cap, bias, True),
        use_rotate_bias=True,
        sidestep_blend=1.0,
    )
    assert rot > 0 and "SIDE" in reason, (rot, reason)


def test_sidestep_soft_blend():
    cap, bias = compute_caps_from_zones(0.5, 0.55, 2.0)
    _, rot_full, _ = apply_safety_gate(
        0.3, 0.0,
        ObstacleState(0.5, 0.55, 2.0, cap, bias, True),
        sidestep_blend=1.0,
    )
    _, rot_soft, _ = apply_safety_gate(
        0.3, 0.0,
        ObstacleState(0.5, 0.55, 2.0, cap, bias, True),
        sidestep_blend=0.35,
    )
    assert abs(rot_soft) < abs(rot_full) + 1e-6, (rot_soft, rot_full)


def test_person_not_hard_stop():
    # 中区很近本应 STOP，但 person_center 匹配时抬高 cap
    near = STOP_DIST - 0.05
    cap, bias = compute_caps_from_zones(2.0, near, 2.0)
    assert cap == 0.0
    fwd, rot, reason = apply_safety_gate(
        0.5, 0.0,
        ObstacleState(2.0, near, 2.0, cap, bias, True),
        person_center_m=near,
    )
    assert fwd > 0.0 and "PERSON" in reason, (fwd, reason)


def test_no_vision_optional():
    fwd, rot, reason = apply_safety_gate(
        0.5, 0.0, ObstacleState(valid=False), stale=True, required=False,
    )
    assert _approx(fwd, 0.5) and reason == "VISION_OPTIONAL"


def test_no_vision_required():
    fwd, rot, reason = apply_safety_gate(
        0.5, 0.2, ObstacleState(valid=False), stale=True, required=True,
    )
    assert fwd == 0.0 and rot == 0.0 and reason == "NO_VISION"


def main():
    tests = [
        test_stop_when_near,
        test_clear_far,
        test_sidestep_prefers_open_side,
        test_sidestep_soft_blend,
        test_person_not_hard_stop,
        test_no_vision_optional,
        test_no_vision_required,
    ]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"  OK  {t.__name__}")
        except Exception as e:
            failed += 1
            print(f"FAIL  {t.__name__}: {e}")
    print()
    if failed:
        print(f"❌ {failed}/{len(tests)} failed")
        sys.exit(1)
    print(f"✅ {len(tests)}/{len(tests)} passed（纯逻辑，未碰腿）")


if __name__ == "__main__":
    main()
