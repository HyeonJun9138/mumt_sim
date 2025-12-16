"""
Quick 3D waypoint-following demo using the existing UAV dynamics.

- Uses sim.core.uav.UAV for kinematics.
- Simple PID-ish yaw/altitude/throttle loop to track a list of waypoints.
- Matplotlib 3D plot for trajectory and waypoints.

추가 기능
- --tune 실행 시 자동 튜닝 수행
- 튜닝 도중 더 좋은 best 값이 나오면 즉시 --save 파일로 중간 저장(체크포인트)
- 실행 시 --load를 주지 않아도 --save 파일이 있으면 자동으로 로드해서 그 값으로 시뮬 실행
- --report로 튜닝 로그(best score, 단계 등) 저장
"""

from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider
import sys

# Add project root so `sim` package resolves when running directly.
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.config import clamp, wrap_deg
from sim.core.uav import UAV, UAVParams


# -----------------------------
# Gains
# -----------------------------
@dataclass
class PIDGains:
    yaw: float = 0.9
    yaw_i: float = 0.0
    pitch_rate: float = 1.4
    pitch_damp: float = 1.0
    alt_pitch: float = 0.03  # deg command per meter of altitude error
    alt_i: float = 0.0       # integral term for altitude error -> pitch bias
    throttle: float = 0.1
    throttle_alt: float = 0.0008  # throttle bias per meter altitude error
    throttle_i: float = 0.0       # integral term for speed error -> throttle bias
    lookahead_m: float = 120.0
    freeze_yaw_dist: float = 50.0
    freeze_alt_ratio: float = 0.5


# -----------------------------
# Save / Load helpers (atomic)
# -----------------------------
def _save_json_atomic(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)


def save_gains(path: str | Path, gains: PIDGains) -> None:
    p = Path(path)
    _save_json_atomic(p, gains.__dict__)


def save_report(path: str | Path, report: dict) -> None:
    p = Path(path)
    _save_json_atomic(p, report)


def load_gains(path: str | Path) -> PIDGains | None:
    p = Path(path)
    if not p.exists():
        return None
    try:
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)

        # 호환성: 혹시 {"gains": {...}} 형태로 저장된 경우도 처리
        if isinstance(data, dict) and "gains" in data and isinstance(data["gains"], dict):
            data = data["gains"]

        if not isinstance(data, dict):
            return None

        return PIDGains(**data)
    except Exception as e:
        print(f"[load] failed to load {p}: {e}")
        return None


# -----------------------------
# Simulation
# -----------------------------
def simulate_waypoints(
    waypoints: Iterable[Tuple[float, float, float]],
    dt: float = 0.01,
    total_time: float = 120.0,
    speed_target: float = 80.0,
    pos_tol: float = 30.0,
    gains: PIDGains = PIDGains(),
    return_errors: bool = False,
    # 튜닝용 시작조건
    start_offset_xy: Tuple[float, float] = (-300.0, -300.0),
    start_yaw_deg: float | None = None,
    start_speed: float | None = None,
    # 튜닝용 메타 반환
    return_info: bool = False,
):
    uav = UAV(UAVParams())
    wp_list: List[Tuple[float, float, float]] = list(waypoints)
    if not wp_list:
        raise ValueError("waypoints list is empty")

    uav.s.x = wp_list[0][0] + float(start_offset_xy[0])
    uav.s.y = wp_list[0][1] + float(start_offset_xy[1])
    uav.s.z = wp_list[0][2]
    if start_yaw_deg is not None:
        uav.s.yaw = wrap_deg(float(start_yaw_deg))
    if start_speed is not None:
        uav.s.u = float(start_speed)

    traj: list[tuple[float, float, float]] = []
    curr_idx = 0

    def _heading_to_target(dx: float, dy: float) -> float:
        # yaw definition matches sim (positive yaw is clockwise, -y forward)
        return wrap_deg(math.degrees(math.atan2(-dy, dx)))

    t = 0.0
    errs_xy: list[float] = []
    errs_alt: list[float] = []

    yaw_int = 0.0
    alt_int = 0.0
    speed_int = 0.0

    # Anti-windup clamps
    yaw_int_max = 100.0
    alt_int_max = 10.0
    speed_int_max = 10.0

    # 튜닝 점수용 메트릭들
    sat_yaw = 0
    sat_pitch = 0
    sat_roll = 0
    sat_thr = 0
    effort = 0.0
    min_u = float("inf")
    aborted = False

    eps = 1e-6

    max_roll_rate = getattr(uav.p, "max_roll_rate_dps", None)
    if max_roll_rate is None:
        max_roll_rate = 1e9  # 파라미터가 없으면 사실상 포화 없음

    while t < total_time and curr_idx < len(wp_list):
        tx, ty, tz = wp_list[curr_idx]
        dx = tx - uav.s.x
        dy = ty - uav.s.y
        dz = tz - uav.s.z
        dist_xy = math.hypot(dx, dy)

        if dist_xy < pos_tol and abs(dz) < pos_tol * 0.6:
            curr_idx += 1
            yaw_int = 0.0
            alt_int = 0.0
            speed_int = 0.0
            continue

        # Lookahead
        if dist_xy > 1e-3 and gains.lookahead_m > 0.0:
            lx = uav.s.x + dx / dist_xy * gains.lookahead_m
            ly = uav.s.y + dy / dist_xy * gains.lookahead_m
            desired_yaw = _heading_to_target(lx - uav.s.x, ly - uav.s.y)
        else:
            desired_yaw = _heading_to_target(dx, dy)

        freeze_yaw = dist_xy < gains.freeze_yaw_dist and abs(dz) > pos_tol * gains.freeze_alt_ratio
        if freeze_yaw:
            uav.cmd_yaw_rate = clamp(-uav.s.r * 0.5, -uav.p.max_yaw_rate_dps, uav.p.max_yaw_rate_dps)
        else:
            yaw_err = ((desired_yaw - uav.s.yaw + 540.0) % 360.0) - 180.0
            yaw_err = clamp(yaw_err, -60.0, 60.0)
            yaw_int = clamp(yaw_int + yaw_err * dt, -yaw_int_max, yaw_int_max)
            uav.cmd_yaw_rate = clamp(
                yaw_err * gains.yaw + yaw_int * gains.yaw_i,
                -uav.p.max_yaw_rate_dps,
                uav.p.max_yaw_rate_dps,
            )

        # Altitude -> pitch target -> pitch-rate loop
        desired_pitch = clamp(
            dz * gains.alt_pitch,
            -uav.p.pitch_limit_deg * 0.7,
            uav.p.pitch_limit_deg * 0.7,
        )
        alt_int = clamp(alt_int + dz * dt, -alt_int_max, alt_int_max)
        pitch_err = desired_pitch + alt_int * gains.alt_i - uav.s.pitch
        pitch_cmd = pitch_err * gains.pitch_rate - uav.s.q * gains.pitch_damp
        uav.cmd_pitch_rate = clamp(pitch_cmd, -uav.p.max_pitch_rate_dps, uav.p.max_pitch_rate_dps)

        # Roll damping
        uav.cmd_roll_rate = -uav.s.roll * 1.5

        # Throttle speed hold + altitude bias
        local_speed_target = speed_target * clamp(dist_xy / 300.0, 0.5, 1.0)
        speed_err = local_speed_target - uav.s.u
        speed_int = clamp(speed_int + speed_err * dt, -speed_int_max, speed_int_max)
        alt_bias = dz * gains.throttle_alt
        uav.cmd_throttle = clamp(speed_err * gains.throttle + speed_int * gains.throttle_i + alt_bias, -1.0, 1.0)

        # 포화 카운트 + effort 누적
        if abs(uav.cmd_yaw_rate) >= uav.p.max_yaw_rate_dps - eps:
            sat_yaw += 1
        if abs(uav.cmd_pitch_rate) >= uav.p.max_pitch_rate_dps - eps:
            sat_pitch += 1
        if abs(uav.cmd_roll_rate) >= float(max_roll_rate) - eps:
            sat_roll += 1
        if abs(uav.cmd_throttle) >= 1.0 - eps:
            sat_thr += 1

        effort += (
            abs(uav.cmd_yaw_rate) / (uav.p.max_yaw_rate_dps + eps)
            + abs(uav.cmd_pitch_rate) / (uav.p.max_pitch_rate_dps + eps)
            + abs(uav.cmd_roll_rate) / (float(max_roll_rate) + eps)
            + abs(uav.cmd_throttle)
        ) * dt

        uav.step(dt)

        # 안정성 체크
        if not (np.isfinite(uav.s.x) and np.isfinite(uav.s.y) and np.isfinite(uav.s.z) and np.isfinite(uav.s.u)):
            aborted = True
            break
        if abs(uav.s.z) > 20000.0:
            aborted = True
            break

        traj.append((uav.s.x, uav.s.y, uav.s.z))
        errs_xy.append(dist_xy)
        errs_alt.append(abs(dz))
        min_u = min(min_u, float(uav.s.u))
        t += dt

    traj_arr = np.array(traj) if traj else np.zeros((0, 3), dtype=float)

    info = {
        "finished": curr_idx >= len(wp_list),
        "curr_idx": curr_idx,
        "t": float(t),
        "steps": int(len(traj)),
        "effort": float(effort),
        "sat_yaw": int(sat_yaw),
        "sat_pitch": int(sat_pitch),
        "sat_roll": int(sat_roll),
        "sat_throttle": int(sat_thr),
        "sat_total": int(sat_yaw + sat_pitch + sat_roll + sat_thr),
        "aborted": bool(aborted),
        "min_u": float(min_u) if min_u != float("inf") else float("nan"),
    }

    if return_errors and return_info:
        return traj_arr, wp_list, np.array(errs_xy), np.array(errs_alt), info
    if return_errors:
        return traj_arr, wp_list, np.array(errs_xy), np.array(errs_alt)
    if return_info:
        return traj_arr, wp_list, info
    return traj_arr, wp_list


# -----------------------------
# Tuning utilities
# -----------------------------
@dataclass(frozen=True)
class ParamSpec:
    name: str
    low: float
    high: float
    scale: str = "linear"  # "linear" | "log"


def _rotate_waypoints_xy(wps: List[Tuple[float, float, float]], yaw_deg: float) -> List[Tuple[float, float, float]]:
    ang = math.radians(yaw_deg)
    c, s = math.cos(ang), math.sin(ang)
    out = []
    for x, y, z in wps:
        xr = c * x - s * y
        yr = s * x + c * y
        out.append((xr, yr, z))
    return out


def _scale_waypoints_xy(wps: List[Tuple[float, float, float]], scale: float) -> List[Tuple[float, float, float]]:
    return [(x * scale, y * scale, z) for (x, y, z) in wps]


def make_tuning_suite(base_wps: List[Tuple[float, float, float]]) -> List[List[Tuple[float, float, float]]]:
    suite: list[list[tuple[float, float, float]]] = []
    suite.append(list(base_wps))
    suite.append(_rotate_waypoints_xy(base_wps, 90.0))
    suite.append(_rotate_waypoints_xy(base_wps, 180.0))
    suite.append(_scale_waypoints_xy(base_wps, 1.25))
    suite.append([(x, y, z + 80.0) for (x, y, z) in base_wps])
    return suite


def _default_param_specs() -> List[ParamSpec]:
    return [
        ParamSpec("yaw", 0.2, 2.5, "linear"),
        ParamSpec("yaw_i", 0.0, 0.2, "linear"),
        ParamSpec("pitch_rate", 0.4, 3.2, "linear"),
        ParamSpec("pitch_damp", 0.2, 2.5, "linear"),
        ParamSpec("alt_pitch", 0.005, 0.09, "linear"),
        ParamSpec("alt_i", 0.0, 0.2, "linear"),
        ParamSpec("throttle", 0.02, 0.35, "linear"),
        ParamSpec("throttle_alt", 1e-4, 2.5e-3, "log"),
        ParamSpec("throttle_i", 0.0, 0.2, "linear"),
        ParamSpec("lookahead_m", 10.0, 220.0, "linear"),
        ParamSpec("freeze_yaw_dist", 0.0, 160.0, "linear"),
    ]


def _gains_to_params(g: PIDGains, specs: List[ParamSpec]) -> dict[str, float]:
    d = g.__dict__.copy()
    return {s.name: float(d[s.name]) for s in specs}


def _params_to_gains(params: dict[str, float]) -> PIDGains:
    return PIDGains(**params)


def _spec_bounds_x(specs: List[ParamSpec]) -> tuple[np.ndarray, np.ndarray]:
    lb = []
    ub = []
    for s in specs:
        if s.scale == "log":
            lb.append(math.log(s.low))
            ub.append(math.log(s.high))
        else:
            lb.append(s.low)
            ub.append(s.high)
    return np.array(lb, dtype=float), np.array(ub, dtype=float)


def _params_to_x(params: dict[str, float], specs: List[ParamSpec]) -> np.ndarray:
    x = np.zeros(len(specs), dtype=float)
    for i, s in enumerate(specs):
        v = float(params[s.name])
        x[i] = math.log(v) if s.scale == "log" else v
    return x


def _x_to_params(x: np.ndarray, specs: List[ParamSpec]) -> dict[str, float]:
    out: dict[str, float] = {}
    for i, s in enumerate(specs):
        out[s.name] = float(math.exp(x[i])) if s.scale == "log" else float(x[i])
    return out


def _lhs_candidates(specs: List[ParamSpec], n: int, rng: np.random.Generator) -> List[dict[str, float]]:
    d = len(specs)
    u = np.empty((n, d), dtype=float)
    for j in range(d):
        perm = rng.permutation(n)
        u[:, j] = (perm + rng.uniform(size=n)) / n

    cands: list[dict[str, float]] = []
    for i in range(n):
        p: dict[str, float] = {}
        for j, s in enumerate(specs):
            uj = float(u[i, j])
            if s.scale == "log":
                lo, hi = math.log(s.low), math.log(s.high)
                p[s.name] = math.exp(lo + uj * (hi - lo))
            else:
                p[s.name] = s.low + uj * (s.high - s.low)
        cands.append(p)
    return cands


def _score_run(errs_xy: np.ndarray, errs_alt: np.ndarray, info: dict, dt: float, speed_target: float) -> float:
    if info.get("aborted", False) or errs_xy.size == 0:
        return 3e6

    finished = bool(info.get("finished", False))
    steps = max(1, int(info.get("steps", 1)))

    iae_xy = float(np.sum(errs_xy) * dt)
    iae_alt = float(np.sum(errs_alt) * dt)

    p95_xy = float(np.percentile(errs_xy, 95))
    p95_alt = float(np.percentile(errs_alt, 95))
    max_xy = float(np.max(errs_xy))
    max_alt = float(np.max(errs_alt))

    sat_ratio = float(info.get("sat_total", 0)) / float(steps)

    t = max(1e-3, float(info.get("t", steps * dt)))
    eff = float(info.get("effort", 0.0)) / t

    min_u = float(info.get("min_u", float("nan")))
    speed_pen = 0.0
    if np.isfinite(min_u):
        speed_pen = max(0.0, (speed_target * 0.35 - min_u)) * 40.0

    base = (
        1.0 * iae_xy
        + 0.45 * p95_xy
        + 0.15 * max_xy
        + 1.25 * iae_alt
        + 0.55 * p95_alt
        + 0.15 * max_alt
        + 220.0 * sat_ratio
        + 6.0 * eff
        + speed_pen
    )

    if not finished:
        base += 1e6 + 5.0 * max_xy + 5.0 * max_alt

    return float(base)


def _evaluate_params(
    params: dict[str, float],
    waypoint_sets: List[List[Tuple[float, float, float]]],
    *,
    dt: float,
    total_time: float,
    speed_target: float,
    pos_tol: float,
    start_cases: List[tuple[Tuple[float, float], float, float]],
    cache: dict[tuple, float] | None = None,
) -> float:
    key = None
    if cache is not None:
        key = tuple((k, round(float(v), 6)) for k, v in sorted(params.items()))
        key = key + (("dt", round(dt, 4)), ("T", round(total_time, 2)), ("V", round(speed_target, 2)))
        if key in cache:
            return cache[key]

    gains = _params_to_gains(params)

    total = 0.0
    n = 0
    for wps in waypoint_sets:
        for (offset_xy, yaw0, u0) in start_cases:
            _, _, exy, ealt, info = simulate_waypoints(
                wps,
                dt=dt,
                total_time=total_time,
                speed_target=speed_target,
                pos_tol=pos_tol,
                gains=gains,
                return_errors=True,
                start_offset_xy=offset_xy,
                start_yaw_deg=yaw0,
                start_speed=u0,
                return_info=True,
            )
            total += _score_run(exy, ealt, info, dt=dt, speed_target=speed_target)
            n += 1

    score = total / max(1, n)
    if cache is not None and key is not None:
        cache[key] = score
    return float(score)


def _local_refine(
    start_params: dict[str, float],
    start_score: float,
    specs: List[ParamSpec],
    eval_fn,
    *,
    max_iters: int = 70,
    step_frac: float = 0.18,
    min_step_frac: float = 0.004,
    on_try=None,  # on_try(params, score) -> None
) -> tuple[dict[str, float], float]:
    lb, ub = _spec_bounds_x(specs)
    best_x = _params_to_x(start_params, specs)
    best_score = float(start_score)

    step = (ub - lb) * float(step_frac)
    min_step = (ub - lb) * float(min_step_frac)

    for _ in range(max_iters):
        improved = False
        for i in range(len(specs)):
            for sgn in (-1.0, 1.0):
                xt = best_x.copy()
                xt[i] = float(np.clip(xt[i] + sgn * step[i], lb[i], ub[i]))
                pt = _x_to_params(xt, specs)
                sc = float(eval_fn(pt))
                if on_try is not None:
                    on_try(pt, sc)
                if sc < best_score:
                    best_score = sc
                    best_x = xt
                    improved = True

        if not improved:
            step *= 0.5
            if float(np.max(step)) < float(np.max(min_step)):
                break

    return _x_to_params(best_x, specs), float(best_score)


def tune_pid_gains(
    base_waypoints: List[Tuple[float, float, float]],
    *,
    speed_target: float = 90.0,
    pos_tol: float = 30.0,
    n_global: int = 160,
    keep_top: int = 10,
    n_restarts: int = 6,
    local_iters: int = 70,
    seed: int = 0,
    dt_coarse: float = 0.02,
    T_coarse: float = 70.0,
    dt_fine: float = 0.01,
    T_fine: float = 95.0,
    # 체크포인트 저장
    save_path: str | None = None,
    report_path: str | None = None,
) -> tuple[PIDGains, dict]:
    rng = np.random.default_rng(seed)
    specs = _default_param_specs()
    waypoint_sets = make_tuning_suite(base_waypoints)

    start_cases = [
        ((-300.0, -300.0), 0.0, speed_target * 0.9),
        ((-520.0, 180.0), 90.0, speed_target * 0.8),
        ((220.0, -520.0), 180.0, speed_target * 1.0),
    ]

    cache_coarse: dict[tuple, float] = {}
    cache_fine: dict[tuple, float] = {}

    def eval_coarse(p: dict[str, float]) -> float:
        return _evaluate_params(
            p, waypoint_sets,
            dt=dt_coarse, total_time=T_coarse,
            speed_target=speed_target, pos_tol=pos_tol,
            start_cases=start_cases,
            cache=cache_coarse,
        )

    def eval_fine(p: dict[str, float]) -> float:
        return _evaluate_params(
            p, waypoint_sets,
            dt=dt_fine, total_time=T_fine,
            speed_target=speed_target, pos_tol=pos_tol,
            start_cases=start_cases,
            cache=cache_fine,
        )

    # 후보 생성: 기본 + (있다면) 이전 저장값 + LHS
    candidates: list[dict[str, float]] = []
    candidates.append(_gains_to_params(PIDGains(), specs))

    if save_path is not None:
        prev = load_gains(save_path)
        if prev is not None:
            candidates.append(_gains_to_params(prev, specs))
            print(f"[tune] resume candidate loaded from {save_path}: {prev}")

    candidates.extend(_lhs_candidates(specs, n_global, rng))

    best_fine_score = float("inf")
    best_params: dict[str, float] | None = None
    best_stage = "init"

    def checkpoint(params: dict[str, float], fine_score: float, stage: str) -> None:
        nonlocal best_fine_score, best_params, best_stage
        if fine_score + 1e-9 < best_fine_score:
            best_fine_score = float(fine_score)
            best_params = dict(params)
            best_stage = stage

            best_gains = _params_to_gains(best_params)
            print(f"[tune][best] score={best_fine_score:.2f} stage={best_stage} gains={best_gains}")

            if save_path is not None:
                save_gains(save_path, best_gains)
                print(f"[tune][save] updated {save_path}")

            if report_path is not None:
                rep = {
                    "timestamp": datetime.now().isoformat(timespec="seconds"),
                    "best_score": best_fine_score,
                    "best_stage": best_stage,
                    "best_params": {k: float(v) for k, v in best_params.items()},
                    "speed_target": float(speed_target),
                    "pos_tol": float(pos_tol),
                    "n_global": int(n_global),
                    "keep_top": int(keep_top),
                    "n_restarts": int(n_restarts),
                    "local_iters": int(local_iters),
                    "dt_coarse": float(dt_coarse),
                    "T_coarse": float(T_coarse),
                    "dt_fine": float(dt_fine),
                    "T_fine": float(T_fine),
                    "suite_size": int(len(waypoint_sets)),
                    "start_cases": start_cases,
                    "seed": int(seed),
                }
                save_report(report_path, rep)

    # 1) 전역 탐색(빠른 평가)
    scored_coarse: list[tuple[float, dict[str, float]]] = []
    best_coarse = float("inf")
    best_coarse_params: dict[str, float] | None = None

    for i, p in enumerate(candidates):
        sc = float(eval_coarse(p))
        scored_coarse.append((sc, p))

        if sc < best_coarse:
            best_coarse = sc
            best_coarse_params = p

            # 새 coarse best를 발견했을 때만 fine 평가해서 체크포인트 갱신 시도
            sc_f = float(eval_fine(p))
            checkpoint(p, sc_f, stage="coarse-best")

        if (i + 1) % 25 == 0 or (i + 1) == len(candidates):
            print(f"[tune:coarse] {i+1}/{len(candidates)} coarse_best={best_coarse:.2f}")

    scored_coarse.sort(key=lambda x: x[0])
    top = [p for (_, p) in scored_coarse[:max(keep_top, n_restarts)]]

    # 2) 상위 후보 정밀 평가
    refined_seed: list[tuple[float, dict[str, float]]] = []
    for i, p in enumerate(top):
        sc_f = float(eval_fine(p))
        refined_seed.append((sc_f, p))
        checkpoint(p, sc_f, stage="fine-seed")
    refined_seed.sort(key=lambda x: x[0])
    print(f"[tune:fine] seed best={refined_seed[0][0]:.2f}")

    # 3) 로컬 미세조정(중간 개선도 체크포인트)
    overall_best_score = float("inf")
    overall_best_params: dict[str, float] | None = None

    for r in range(min(n_restarts, len(refined_seed))):
        s0, p0 = refined_seed[r]

        def on_try(pt, sc):
            checkpoint(pt, float(sc), stage=f"local-r{r+1}")

        p_ref, s_ref = _local_refine(
            p0, s0, specs, eval_fine,
            max_iters=local_iters,
            on_try=on_try,
        )
        print(f"[tune:local] restart {r+1}/{min(n_restarts, len(refined_seed))} score={s_ref:.2f}")
        checkpoint(p_ref, float(s_ref), stage=f"local-final-r{r+1}")

        if s_ref < overall_best_score:
            overall_best_score = float(s_ref)
            overall_best_params = dict(p_ref)

    if overall_best_params is None:
        # 혹시라도 로컬에 못 들어간 경우
        overall_best_params = refined_seed[0][1]
        overall_best_score = refined_seed[0][0]

    best_gains = _params_to_gains(overall_best_params)

    # 최종 저장(한 번 더 확정)
    if save_path is not None:
        save_gains(save_path, best_gains)
        print(f"[tune][save] final {save_path}")
    report = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "best_score": float(overall_best_score),
        "best_stage": "final",
        "best_params": {k: float(v) for k, v in overall_best_params.items()},
        "speed_target": float(speed_target),
        "pos_tol": float(pos_tol),
        "n_global": int(n_global),
        "keep_top": int(keep_top),
        "n_restarts": int(n_restarts),
        "local_iters": int(local_iters),
        "dt_coarse": float(dt_coarse),
        "T_coarse": float(T_coarse),
        "dt_fine": float(dt_fine),
        "T_fine": float(T_fine),
        "suite_size": int(len(waypoint_sets)),
        "start_cases": start_cases,
        "seed": int(seed),
    }
    if report_path is not None:
        save_report(report_path, report)

    return best_gains, report


def sweep_time_scales(
    base_waypoints: List[Tuple[float, float, float]],
    time_scales: List[float],
    *,
    speed_target: float = 90.0,
    pos_tol: float = 30.0,
    n_global: int = 160,
    keep_top: int = 10,
    n_restarts: int = 6,
    local_iters: int = 70,
    seed: int = 0,
    dt_coarse: float = 0.02,
    T_coarse: float = 70.0,
    dt_fine: float = 0.01,
    T_fine: float = 95.0,
    save_dir: str | Path = "test/uav_pid_db",
    db_name: str = "uav_pid_db.json",
    aggregate_path: str | Path | None = None,
):
    """
    Run tuning for multiple time_scale values (dt scaled accordingly) and save a DB JSON with per-scale gains.
    Also writes an aggregated DB if aggregate_path is provided.
    """
    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    db_path = save_dir / db_name

    prefix = Path(db_name).stem.replace("_db", "")
    records = []
    for ts in time_scales:
        ts = float(ts)
        rep_path = save_dir / f"{prefix}_scale_{ts:.2f}.report.json"
        gains_path = save_dir / f"{prefix}_scale_{ts:.2f}.json"
        print(f"[sweep] time_scale={ts:.2f} dt_coarse={dt_coarse*ts:.4f} dt_fine={dt_fine*ts:.4f}")
        gains_ts, rep_ts = tune_pid_gains(
            base_waypoints,
            speed_target=speed_target,
            pos_tol=pos_tol,
            n_global=n_global,
            keep_top=keep_top,
            n_restarts=n_restarts,
            local_iters=local_iters,
            seed=seed,
            dt_coarse=dt_coarse * ts,
            T_coarse=T_coarse,
            dt_fine=dt_fine * ts,
            T_fine=T_fine,
            save_path=gains_path,
            report_path=rep_path,
        )
        records.append(
            {
                "time_scale": ts,
                "dt_coarse": dt_coarse * ts,
                "dt_fine": dt_fine * ts,
                "best_score": float(rep_ts["best_score"]),
                "best_stage": rep_ts.get("best_stage"),
                "gains": gains_ts.__dict__,
                "report_path": str(rep_path),
                "gains_path": str(gains_path),
            }
        )
    _save_json_atomic(db_path, {"records": records})
    if aggregate_path is not None:
        agg_path = Path(aggregate_path)
        agg_path.parent.mkdir(parents=True, exist_ok=True)
        _save_json_atomic(agg_path, {"records": records})
        print(f"[sweep] saved aggregate DB to {agg_path}")
    print(f"[sweep] saved DB with {len(records)} entries to {db_path}")
    return db_path, records


# -----------------------------
# Plot / GUI
# -----------------------------
def plot_trajectory(traj: np.ndarray, waypoints: List[Tuple[float, float, float]]):
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], label="trajectory", color="tab:blue")
    wps = np.array(waypoints)
    ax.scatter(wps[:, 0], wps[:, 1], wps[:, 2], color="red", marker="o", s=50, label="waypoints")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()
    ax.set_title("UAV PID waypoint tracking demo")
    plt.tight_layout()
    plt.show()


def interactive_gui(
    waypoints: List[Tuple[float, float, float]],
    gains: PIDGains,
    dt: float = 0.01,
    total_time: float = 160.0,
    speed_target: float = 90.0,
    pos_tol: float = 30.0,
    save_path: str | None = None,  # 현재 슬라이더 값을 저장하고 싶으면 전달
):
    traj, wps = simulate_waypoints(waypoints, dt=dt, total_time=total_time, speed_target=speed_target, gains=gains)

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(221, projection="3d")
    ax.set_title("Trajectory (3D)")
    line_traj, = ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], label="trajectory", color="tab:blue")
    wps_arr = np.array(wps)
    ax.scatter(wps_arr[:, 0], wps_arr[:, 1], wps_arr[:, 2], color="red", marker="o", s=50, label="waypoints")
    ax.legend()

    ax_err = fig.add_subplot(222)
    _, _, errs_xy, errs_alt = simulate_waypoints(
        waypoints, dt=dt, total_time=total_time, speed_target=speed_target, gains=gains, return_errors=True
    )
    err_line_xy, = ax_err.plot(errs_xy, label="XY error (m)", color="tab:blue")
    err_line_alt, = ax_err.plot(errs_alt, label="Alt error (m)", color="tab:orange")
    ax_err.set_title("Errors over time")
    ax_err.set_xlabel("Step")
    ax_err.set_ylabel("Error (m)")
    ax_err.legend()

    slider_params = [
        ("yaw", gains.yaw, 0.2, 2.0),
        ("yaw_i", gains.yaw_i, 0.0, 0.2),
        ("pitch_rate", gains.pitch_rate, 0.5, 3.0),
        ("pitch_damp", gains.pitch_damp, 0.2, 2.5),
        ("alt_pitch", gains.alt_pitch, 0.0, 0.08),
        ("alt_i", gains.alt_i, 0.0, 0.2),
        ("throttle", gains.throttle, 0.05, 0.3),
        ("throttle_alt", gains.throttle_alt, 0.0001, 0.002),
        ("throttle_i", gains.throttle_i, 0.0, 0.2),
        ("lookahead_m", gains.lookahead_m, 0.0, 200.0),
        ("freeze_yaw_dist", gains.freeze_yaw_dist, 0.0, 120.0),
        ("speed_target", speed_target, 40.0, 140.0),
    ]
    slider_objs = {}

    left_col_x = 0.1
    right_col_x = 0.55
    height = 0.03
    spacing = 0.005
    split = (len(slider_params) + 1) // 2
    for idx, (name, val, vmin, vmax) in enumerate(slider_params):
        col = 0 if idx < split else 1
        row = idx if col == 0 else idx - split
        axpos = [left_col_x if col == 0 else right_col_x, 0.05 + row * (height + spacing), 0.32, height]
        sa = fig.add_axes(axpos)
        slider = Slider(
            ax=sa, label=name,
            valmin=vmin, valmax=vmax,
            valinit=val,
            valstep=(vmax - vmin) / 200.0
        )
        slider_objs[name] = slider

    ax_button_apply = fig.add_axes([0.82, 0.9, 0.12, 0.05])
    button_apply = Button(ax_button_apply, "Apply", color="lightgray", hovercolor="0.9")

    ax_button_save = fig.add_axes([0.82, 0.84, 0.12, 0.05])
    button_save = Button(ax_button_save, "Save", color="lightgray", hovercolor="0.9")

    status_text = fig.text(0.55, 0.86, "", fontsize=9)

    def _read_gains_from_sliders() -> tuple[PIDGains, float]:
        new_gains = PIDGains(
            yaw=float(slider_objs["yaw"].val),
            yaw_i=float(slider_objs["yaw_i"].val),
            pitch_rate=float(slider_objs["pitch_rate"].val),
            pitch_damp=float(slider_objs["pitch_damp"].val),
            alt_pitch=float(slider_objs["alt_pitch"].val),
            alt_i=float(slider_objs["alt_i"].val),
            throttle=float(slider_objs["throttle"].val),
            throttle_alt=float(slider_objs["throttle_alt"].val),
            throttle_i=float(slider_objs["throttle_i"].val),
            lookahead_m=float(slider_objs["lookahead_m"].val),
            freeze_yaw_dist=float(slider_objs["freeze_yaw_dist"].val),
            freeze_alt_ratio=gains.freeze_alt_ratio,  # GUI에서 조정 안 하는 값 유지
        )
        new_speed = float(slider_objs["speed_target"].val)
        return new_gains, new_speed

    def apply(event=None):
        new_gains, new_speed = _read_gains_from_sliders()

        t_traj, _, e_xy, e_alt = simulate_waypoints(
            waypoints,
            dt=dt,
            total_time=total_time,
            speed_target=new_speed,
            gains=new_gains,
            return_errors=True,
        )
        line_traj.set_data(t_traj[:, 0], t_traj[:, 1])
        line_traj.set_3d_properties(t_traj[:, 2])

        err_line_xy.set_ydata(e_xy)
        err_line_xy.set_xdata(np.arange(len(e_xy)))
        err_line_alt.set_ydata(e_alt)
        err_line_alt.set_xdata(np.arange(len(e_alt)))
        ax_err.relim()
        ax_err.autoscale_view()

        status_text.set_text(
            f"mean XY {np.mean(e_xy):.1f} m, max XY {np.max(e_xy):.1f} m | "
            f"mean alt {np.mean(e_alt):.1f} m, max alt {np.max(e_alt):.1f} m"
        )

        if t_traj.shape[0] > 0:
            ax.set_xlim(np.min(t_traj[:, 0]), np.max(t_traj[:, 0]))
            ax.set_ylim(np.min(t_traj[:, 1]), np.max(t_traj[:, 1]))
            ax.set_zlim(np.min(t_traj[:, 2]), np.max(t_traj[:, 2]))

        fig.canvas.draw_idle()

    def save_current(event=None):
        if save_path is None:
            status_text.set_text("save_path가 없어서 저장 불가(--save를 넘겨줘)")
            fig.canvas.draw_idle()
            return
        new_gains, _ = _read_gains_from_sliders()
        save_gains(save_path, new_gains)
        status_text.set_text(f"saved gains to {save_path}")
        fig.canvas.draw_idle()

    button_apply.on_clicked(apply)
    button_save.on_clicked(save_current)

    apply()
    plt.show()


# -----------------------------
# Main
# -----------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PID waypoint tracker demo with auto-tuning and checkpoint saving.")
    parser.add_argument("--tune", action="store_true", help="run auto-tuning and update best gains while tuning")
    parser.add_argument("--save", type=str, default="test/uav_pid_db/uav_pid_tuned_gains.json", help="path to save tuned gains")
    parser.add_argument("--report", type=str, help="path to save tuning report json (default: <save>.report.json)")
    parser.add_argument("--load", type=str, help="load gains from json (overrides auto-load from --save if given)")
    parser.add_argument("--no-gui", action="store_true", help="do not open GUI (useful if only tuning)")
    parser.add_argument(
        "--sweep-time-scales",
        type=str,
        help="comma-separated time_scale values to tune per-scale gains (dt scaled) and save DB; disables GUI",
    )
    parser.add_argument(
        "--sweep-dir",
        type=str,
        default="test/uav_pid_db",
        help="directory to save per-scale gains/reports (default: test/uav_pid_db)",
    )
    parser.add_argument(
        "--aggregate",
        type=str,
        default="sim/runtime/controllers/uav_pid_db.json",
        help="path to save aggregated DB json for simulator (default: sim/runtime/controllers/uav_pid_db.json)",
    )
    args = parser.parse_args()

    wp_demo = [
        (0.0, 0.0, 300.0),
        (800.0, 0.0, 300.0),
        (800.0, 800.0, 350.0),
        (0.0, 800.0, 350.0),
        (-400.0, 400.0, 400.0),
    ]

    save_path = args.save
    report_path = args.report
    save_path = str(Path(save_path))
    report_path = str(Path(report_path)) if report_path else None
    if report_path is None:
        sp = Path(save_path)
        if sp.suffix:
            report_path = str(sp.with_suffix(".report.json"))
        else:
            report_path = str(sp.with_name(sp.name + ".report.json"))

    gains = PIDGains()

    # 1) 명시적 --load가 있으면 그걸 우선
    if args.load:
        g = load_gains(args.load)
        if g is not None:
            gains = g
            print(f"[load] gains loaded from {args.load}: {gains}")

    # 2) --load가 없으면 --save 파일이 있으면 자동 로드
    else:
        g = load_gains(save_path)
        if g is not None:
            gains = g
            print(f"[auto-load] gains loaded from {save_path}: {gains}")

    # 2.5) time_scale sweep DB 생성 모드
    if args.sweep_time_scales:
        scales = [float(s.strip()) for s in args.sweep_time_scales.split(",") if s.strip()]
        if not scales:
            print("[sweep] no valid time_scale values provided")
        else:
            sweep_time_scales(
                wp_demo,
                scales,
                speed_target=90.0,
                pos_tol=30.0,
                n_global=180,
                keep_top=12,
                n_restarts=7,
                local_iters=80,
                seed=0,
                dt_coarse=0.02,
                T_coarse=70.0,
                dt_fine=0.01,
                T_fine=95.0,
                save_dir=Path(args.sweep_dir),
                db_name="uav_pid_db.json",
                aggregate_path=args.aggregate,
            )
        exit(0)

    # 3) 튜닝 수행 (중간 best 값은 바로바로 --save에 저장됨)
    if args.tune:
        tuned, rep = tune_pid_gains(
            wp_demo,
            speed_target=90.0,
            pos_tol=30.0,
            n_global=180,
            keep_top=12,
            n_restarts=7,
            local_iters=80,
            seed=0,
            dt_coarse=0.02,
            T_coarse=70.0,
            dt_fine=0.01,
            T_fine=95.0,
            save_path=save_path,
            report_path=report_path,
        )
        gains = tuned
        print(f"[tune] final best gains: {gains} (score {rep['best_score']:.2f})")
        print(f"[tune] gains saved to {save_path}")
        print(f"[tune] report saved to {report_path}")

    if not args.no_gui:
        interactive_gui(wp_demo, gains, dt=0.01, total_time=160.0, speed_target=90.0, save_path=save_path)
