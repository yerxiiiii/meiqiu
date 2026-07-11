#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
5-DOF 数值逆解（阻尼最小二乘）。

5 自由度无法同时精确满足 6 维位姿，支持：
  - position: 仅位置
  - position_tool_z: 位置 + 末端 z 轴（5 约束，推荐）
  - position_orientation: 位置 + 完整姿态（软约束）
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np

from .forward_kinematics import ForwardKinematics
from .transforms import (
    clamp_vector,
    rotation_error,
    split_pose,
    tool_z_error,
)
from .urdf_parser import ArmChain


class IKTaskMode(str, Enum):
    POSITION = "position"
    POSITION_TOOL_Z = "position_tool_z"
    POSITION_ORIENTATION = "position_orientation"


@dataclass
class IKConfig:
    max_iterations: int = 120
    position_tolerance_m: float = 2e-3
    orientation_tolerance_rad: float = 0.05
    damping: float = 0.08
    step_scale: float = 0.85
    position_weight: float = 1.0
    orientation_weight: float = 0.35
    jacobian_delta: float = 1e-4
    use_geometric_jacobian: bool = True
    stall_iterations: int = 5
    # True：未严格收敛也返回当前最优 q，便于 5-DOF 尽力贴近目标
    accept_best_effort: bool = True


@dataclass
class IKResult:
    success: bool
    q: np.ndarray
    iterations: int
    position_error_m: float
    orientation_error_rad: float
    message: str


class NumericalIK:
    def __init__(
        self,
        fk: ForwardKinematics,
        chain: ArmChain,
        config: Optional[IKConfig] = None,
    ):
        self.fk = fk
        self.chain = chain
        self.cfg = config or IKConfig()
        self.q_lo, self.q_hi = chain.limits()

    def solve(
        self,
        target_T: np.ndarray,
        q_seed: Optional[np.ndarray] = None,
        mode: IKTaskMode = IKTaskMode.POSITION_TOOL_Z,
    ) -> IKResult:
        n = self.chain.dof
        q = (
            np.zeros(n, dtype=float)
            if q_seed is None
            else np.asarray(q_seed, dtype=float).copy()
        )
        q = clamp_vector(q, self.q_lo, self.q_hi)
        p_tgt, r_tgt = split_pose(target_T)

        best_q = q.copy()
        best_pos = float("inf")
        best_ori = float("inf")
        stall = 0
        tol = self.cfg.position_tolerance_m
        last_it = 0

        use_geom = self.cfg.use_geometric_jacobian
        for it in range(1, self.cfg.max_iterations + 1):
            last_it = it
            if use_geom:
                j6, t_cur = self.fk.geometric_jacobian(q)
            else:
                t_cur = self.fk.compute(q)
                j6 = None
            p_cur, r_cur = split_pose(t_cur)
            e_task, pos_err, ori_err = self._task_error(
                p_cur, r_cur, p_tgt, r_tgt, mode,
            )

            pos_ok = pos_err < tol
            ori_ok = (
                mode == IKTaskMode.POSITION
                or ori_err < self.cfg.orientation_tolerance_rad
            )
            if pos_ok and ori_ok:
                return IKResult(
                    success=True,
                    q=q,
                    iterations=it,
                    position_error_m=pos_err,
                    orientation_error_rad=ori_err,
                    message="converged",
                )

            if pos_err < best_pos - 1e-6:
                stall = 0
                best_pos = pos_err
                best_ori = ori_err
                best_q = q.copy()
            else:
                stall += 1
                if stall >= self.cfg.stall_iterations:
                    break

            j = (
                self._jacobian_from_geom(j6, mode)
                if j6 is not None
                else self._task_jacobian(q, r_cur, mode)
            )
            jj_t = j @ j.T
            lam2 = self.cfg.damping ** 2
            try:
                dq = j.T @ np.linalg.solve(
                    jj_t + lam2 * np.eye(jj_t.shape[0]), e_task,
                )
            except np.linalg.LinAlgError:
                dq = j.T @ np.linalg.lstsq(
                    jj_t + lam2 * np.eye(jj_t.shape[0]),
                    e_task,
                    rcond=None,
                )[0]
                if not np.all(np.isfinite(dq)):
                    continue
            scales = (1.0, 0.5) if pos_err > tol * 2 else (1.0,)
            best_step_q = q
            best_step_pos = pos_err
            for scale in scales:
                q_try = clamp_vector(
                    q + self.cfg.step_scale * scale * dq,
                    self.q_lo,
                    self.q_hi,
                )
                t_try = self.fk.compute(q_try)
                p_try, r_try = split_pose(t_try)
                _, pe_try, oe_try = self._task_error(
                    p_try, r_try, p_tgt, r_tgt, mode,
                )
                if pe_try < best_step_pos:
                    best_step_pos = pe_try
                    best_step_q = q_try
                    if pe_try < best_pos - 1e-6:
                        best_pos = pe_try
                        best_ori = oe_try
                        best_q = q_try.copy()
            q = best_step_q

        msg = "best_effort"
        ok = self.cfg.accept_best_effort
        if (
            best_pos < self.cfg.position_tolerance_m
            and (
                mode == IKTaskMode.POSITION
                or best_ori < self.cfg.orientation_tolerance_rad
            )
        ):
            msg = "converged"
            ok = True
        return IKResult(
            success=ok,
            q=best_q,
            iterations=last_it if last_it > 0 else self.cfg.max_iterations,
            position_error_m=best_pos,
            orientation_error_rad=best_ori,
            message=msg,
        )

    def _task_error(
        self,
        p_cur: np.ndarray,
        r_cur: np.ndarray,
        p_tgt: np.ndarray,
        r_tgt: np.ndarray,
        mode: IKTaskMode,
    ) -> tuple[np.ndarray, float, float]:
        e_pos = (p_tgt - p_cur) * self.cfg.position_weight
        pos_err = float(np.linalg.norm(e_pos))
        if mode == IKTaskMode.POSITION:
            return e_pos, pos_err, 0.0
        if mode == IKTaskMode.POSITION_TOOL_Z:
            e_ori = tool_z_error(r_cur, r_tgt) * self.cfg.orientation_weight
            ori_err = float(np.linalg.norm(e_ori))
            return np.concatenate([e_pos, e_ori]), pos_err, ori_err
        e_ori = rotation_error(r_cur, r_tgt) * self.cfg.orientation_weight
        ori_err = float(np.linalg.norm(e_ori))
        return np.concatenate([e_pos, e_ori]), pos_err, ori_err

    def _jacobian_from_geom(
        self, j6: np.ndarray, mode: IKTaskMode,
    ) -> np.ndarray:
        if mode == IKTaskMode.POSITION:
            return j6[0:3, :] * self.cfg.position_weight
        return np.vstack(
            [
                j6[0:3, :] * self.cfg.position_weight,
                j6[3:6, :] * self.cfg.orientation_weight,
            ],
        )

    def _task_jacobian(
        self,
        q: np.ndarray,
        r_cur: np.ndarray,
        mode: IKTaskMode,
    ) -> np.ndarray:
        return self._numeric_jacobian(q, r_cur, mode)

    def _numeric_jacobian(
        self,
        q: np.ndarray,
        r_cur: np.ndarray,
        mode: IKTaskMode,
    ) -> np.ndarray:
        """数值雅可比（回退）。"""
        n = self.chain.dof
        t0 = self.fk.compute(q)
        p0, r0 = split_pose(t0)
        r_tgt = r_cur
        h = self.cfg.jacobian_delta
        if mode == IKTaskMode.POSITION:
            m = 3
        else:
            m = 6
        j = np.zeros((m, n), dtype=float)
        for i in range(n):
            dq = np.zeros(n, dtype=float)
            dq[i] = h
            t1 = self.fk.compute(q + dq)
            p1, r1 = split_pose(t1)
            j[0:3, i] = (p1 - p0) / h
            if mode == IKTaskMode.POSITION_TOOL_Z:
                j[3:6, i] = (tool_z_error(r1, r0)) / h
            elif mode == IKTaskMode.POSITION_ORIENTATION:
                j[3:6, i] = (rotation_error(r1, r0)) / h
        if mode == IKTaskMode.POSITION:
            j *= self.cfg.position_weight
        elif mode == IKTaskMode.POSITION_TOOL_Z:
            j[0:3] *= self.cfg.position_weight
            j[3:6] *= self.cfg.orientation_weight
        else:
            j[0:3] *= self.cfg.position_weight
            j[3:6] *= self.cfg.orientation_weight
        return j
