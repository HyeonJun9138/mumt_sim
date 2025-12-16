from __future__ import annotations

import json
import math
import re
from typing import Any, TYPE_CHECKING

import numpy as np

from sim.runtime.controllers import DEFAULT_TUNED_GAINS, PIDGains, WaypointPIDController, WaypointTarget, load_pid_gains
from sim.runtime.operation_mode import OperationContext, OperationMode, build_operation_mode

if TYPE_CHECKING:
    from sim.runtime.app import SimulationApp


def load_gain_db(app: "SimulationApp", path) -> list[dict] | None:
    key = str(path.resolve())
    if key in app.pid_db_cache:
        return app.pid_db_cache[key]
    # 1) Primary: aggregated DB file
    if path.exists():
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            records = data.get("records") if isinstance(data, dict) else None
            if isinstance(records, list):
                app.pid_db_cache[key] = records
                return records
        except Exception as e:
            print(f"[pid-db] failed to load {path}: {e}")
    # 2) Fallback: scan scale-specific files (e.g., pid_tuned_scale_*.json)
    records: list[dict] = []
    prefix = path.stem
    if prefix.endswith("_db"):
        prefix = prefix[: -len("_db")]
    pattern = f"{prefix}_scale_*.json"
    for f in sorted(path.parent.glob(pattern)):
        m = re.search(r"_scale_([0-9.]+)", f.stem)
        if not m:
            continue
        try:
            ts = float(m.group(1))
        except Exception:
            continue
        g = load_pid_gains(f)
        if g is None:
            continue
        records.append(
            {
                "time_scale": ts,
                "gains": g.__dict__,
                "gains_path": str(f),
            }
        )
    if records:
        app.pid_db_cache[key] = records
        print(f"[pid-db] built ad-hoc records from pattern {pattern} ({len(records)} entries)")
        return records
    return None


def pick_gains_for_scale(
    app: "SimulationApp", db_path, fallback_path, time_scale: float, craft_label: str | None = None
) -> PIDGains:
    records = load_gain_db(app, db_path)
    label = craft_label or db_path.stem
    chosen_ts: float | None = None
    chosen_path: str | None = None
    if records:
        best_rec = None
        best_diff = float("inf")
        for r in records:
            ts = r.get("time_scale")
            g = r.get("gains")
            if ts is None or not isinstance(g, dict):
                continue
            diff = abs(float(ts) - float(time_scale))
            if diff < best_diff:
                best_diff = diff
                best_rec = r
        if best_rec:
            gains_dict = best_rec.get("gains")
            try:
                gains_obj = PIDGains(**gains_dict)
                chosen_ts = float(best_rec.get("time_scale")) if best_rec.get("time_scale") is not None else None
                chosen_path = best_rec.get("gains_path") or str(db_path)
                choice = (chosen_ts, chosen_path, round(time_scale, 2))
                prev = app.pid_choice_log.get(label)
                if prev != choice:
                    ts_disp = chosen_ts if chosen_ts is not None else time_scale
                    print(f"[pid-db] {label}: time_scale={time_scale:.2f} -> record ts={ts_disp:.2f} from {chosen_path}")
                    app.pid_choice_log[label] = choice
                return gains_obj
            except Exception as e:
                print(f"[pid-db] failed to build gains from record: {e}")
    # fallback: flat gains file
    g = load_pid_gains(fallback_path)
    if g is not None:
        source = str(fallback_path)
        choice = (None, source, round(time_scale, 2))
        prev = app.pid_choice_log.get(label)
        if prev != choice:
            print(f"[pid-db] {label}: time_scale={time_scale:.2f} -> fallback gains {source}")
            app.pid_choice_log[label] = choice
        return g
    source = "DEFAULT_TUNED_GAINS"
    choice = (None, source, round(time_scale, 2))
    prev = app.pid_choice_log.get(label)
    if prev != choice:
        print(f"[pid-db] {label}: time_scale={time_scale:.2f} -> using default tuned gains")
        app.pid_choice_log[label] = choice
    return DEFAULT_TUNED_GAINS


def get_operation_mode_handler(app: "SimulationApp", mode_id: int | None) -> OperationMode | None:
    if mode_id is None:
        return None
    if mode_id in app.operation_mode_handlers:
        return app.operation_mode_handlers[mode_id]
    try:
        handler = build_operation_mode(mode_id)
    except Exception as exc:
        print(f"[sim-log] unsupported operation mode {mode_id}: {exc}")
        return None
    app.operation_mode_handlers[mode_id] = handler
    return handler


def update_filming_target(app: "SimulationApp", idx: int, tgt: WaypointTarget | None, dt: float) -> None:
    """운용 모드 핸들러에 위임해 촬영 타깃 갱신."""
    uav = app.state.uavs[idx]
    filming_prop = tgt.filming if tgt else None
    current_wp_id = tgt.wp_id if tgt else None

    if filming_prop is None:
        app.uav_line_search_state[idx] = None
        app.uav_filming_props[idx] = None
        app.uav_filming_target[idx] = app._default_downward_target(uav)
        return

    handler = get_operation_mode_handler(app, filming_prop.get("operationMode"))
    if handler is None or app.dem is None:
        app.uav_line_search_state[idx] = None
        app.uav_filming_target[idx] = app._default_downward_target(uav)
        return

    ctx = OperationContext(dem=app.dem, default_target_fn=app._default_downward_target)
    prev_state = app.uav_line_search_state[idx] if idx < len(app.uav_line_search_state) else None

    try:
        result = handler.apply(
            uav=uav,
            filming_prop=filming_prop,
            ctx=ctx,
            dt=float(dt or 0.0),
            current_wp_id=current_wp_id,
            prev_state=prev_state,
        )
    except Exception as exc:
        print(f"[sim-log] operation mode {filming_prop.get('operationMode')} failed: {exc}")
        app.uav_line_search_state[idx] = None
        app.uav_filming_target[idx] = app._default_downward_target(uav)
        return

    app.uav_filming_target[idx] = result.target or app._default_downward_target(uav)
    app.uav_line_search_state[idx] = result.state
    if result.reset_debug or result.debug is not None:
        app.uav_line_search_debug[idx] = result.debug


def init_uav_autopilots(app: "SimulationApp") -> None:
    """비행 경로 기반 PID 오토파일럿 초기화."""
    if not app.state:
        return

    app.uav_autopilots = [None for _ in app.state.uavs]
    app.uav_filming_props = [None for _ in app.state.uavs]
    app.uav_current_wp_ids = [None for _ in app.state.uavs]
    app.uav_line_search_state = [None for _ in app.state.uavs]
    app.uav_filming_target = [None for _ in app.state.uavs]
    app.uav_line_search_debug = [None for _ in app.state.uavs]
    if not (app.enable_uav_autopilot and app.enable_flight_paths):
        return
    if not getattr(app.state, "flight_paths", None):
        return

    time_scale = app.state.time_scale["value"] if app.state else 1.0
    for idx, plist in enumerate(app.state.flight_paths):
        if not plist:
            continue
        targets: list[WaypointTarget] = []
        for wp in plist:
            pos = None
            speed = None
            filming = None
            wp_id = None
            hover_time = None
            loiter = None
            hover_prop = None
            if isinstance(wp, dict):
                pos = wp.get("pos")
                speed = wp.get("speed")
                filming = wp.get("filming")
                wp_id = wp.get("wp_id")
                hover_time = wp.get("hover_time")
                loiter = wp.get("loiter")
                hover_prop = wp.get("hover_prop") or wp.get("hovering")
                if hover_time is None and isinstance(hover_prop, dict):
                    hover_time = hover_prop.get("time")
            elif isinstance(wp, (list, tuple)) and len(wp) == 3:
                pos = wp
            if pos is None:
                continue
            try:
                px, py, pz = pos
            except Exception:
                continue
            try:
                speed_val = float(speed) if speed is not None else None
            except Exception:
                speed_val = None
            targets.append(
                WaypointTarget(
                    pos=(float(px), float(py), float(pz)),
                    speed=speed_val,
                    filming=filming,
                    wp_id=int(wp_id) if wp_id is not None else None,
                    hover_time=float(hover_time) if hover_time is not None else None,
                    loiter=loiter,
                )
            )

        if targets:
            is_uav = app.state.uav_types[idx] == "UAV"
            gains_path = app.pid_db_path_uav if is_uav else app.pid_db_path_lah
            fallback_path = app.pid_gains_path if is_uav else app.pid_gains_path_lah
            label = ("UAV" if is_uav else "LAH") + f"{idx + 1}"
            gains = pick_gains_for_scale(app, gains_path, fallback_path, time_scale, craft_label=label)
            app.uav_autopilots[idx] = WaypointPIDController(
                app.state.uavs[idx],
                targets,
                gains=gains,
                speed_target=90.0 if is_uav else 60.0,
                pos_tol=30.0,
                name=("UAV" if is_uav else "LAH") + f"{idx + 1}",
                allow_hover=not is_uav,
            )
            app.uav_filming_props[idx] = targets[0].filming if targets[0].filming else None
            app.uav_current_wp_ids[idx] = int(targets[0].wp_id) if targets[0].wp_id is not None else None
            print(f"[pid-autopilot] armed for {('UAV' if is_uav else 'LAH')}{idx + 1} with {len(targets)} waypoints.")


def update_autopilots(app: "SimulationApp", dt_sim: float, wall_dt: float) -> None:
    """
    고정 스텝으로 PID 오토파일럿을 advance (time_scale이 커져도 안정적).
    """
    if not app.state or not app.uav_autopilots or not app.enable_uav_autopilot:
        return
    time_scale = app.state.time_scale["value"] if app.state else 1.0
    ctrl_step = float(app.sim_step)
    for idx, ap in enumerate(app.uav_autopilots):
        if ap is None or app.state.crashed[idx]:
            continue
        is_uav = app.state.uav_types[idx] == "UAV"
        gains_path = app.pid_db_path_uav if is_uav else app.pid_db_path_lah
        fallback_path = app.pid_gains_path if is_uav else app.pid_gains_path_lah
        label = ("UAV" if is_uav else "LAH") + f"{idx + 1}"
        ap.gains = pick_gains_for_scale(app, gains_path, fallback_path, time_scale, craft_label=label)
        pending = app.state.pending_states[idx]
        if pending:
            # Use buffered state samples (arrive at sim_step spacing) to keep control in sync.
            wall_slice = wall_dt / len(pending)
            for state_tuple in pending:
                (
                    app.state.uavs[idx].s.x,
                    app.state.uavs[idx].s.y,
                    app.state.uavs[idx].s.z,
                    app.state.uavs[idx].s.roll,
                    app.state.uavs[idx].s.pitch,
                    app.state.uavs[idx].s.yaw,
                    app.state.uavs[idx].s.u,
                    app.state.uavs[idx].s.p,
                    app.state.uavs[idx].s.q,
                    app.state.uavs[idx].s.r,
                ) = state_tuple
                ap.update(ctrl_step, dem=app.dem, wall_dt=wall_slice)
                update_filming_target(app, idx, ap.current_target(), ctrl_step)
        else:
            # No buffered states (e.g., worker lag) -> single control update for the elapsed sim time.
            step = float(dt_sim) if dt_sim and dt_sim > 0 else ctrl_step
            ap.update(step, dem=app.dem, wall_dt=wall_dt)
            update_filming_target(app, idx, ap.current_target(), step)

        # Track current filming property for this UAV (current target of autopilot).
        tgt = ap.current_target()
        app.uav_filming_props[idx] = tgt.filming if tgt else None
        app.uav_current_wp_ids[idx] = int(tgt.wp_id) if (tgt and tgt.wp_id is not None) else None
        if tgt and tgt.filming:
            fov = tgt.filming.get("fieldOfView")
            if isinstance(fov, (int, float)):
                app.state.fov_diag = float(fov)
