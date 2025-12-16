from __future__ import annotations

import math
import random
import time
from typing import TYPE_CHECKING

import numpy as np
import pygame

from sim.agent_status import LAH_FUEL_BURN_LPS, MAX_FUEL_L
from sim.config import FPS, WORLD_HALF, clamp
from sim.entities.entities import Missile, MovingTarget
from sim.entities.threat import AirDefenseThreat
from sim.runtime.constants import INITIAL_OFFSETS, TARGET_ROAM_RADIUS_M, TARGET_SPEED_RANGE, THREAT_RADAR, THREAT_WEAPON
from sim.world.dem import check_los, ray_intersect_dem

if TYPE_CHECKING:
    from sim.runtime.app import SimulationApp

MAX_TARGET_RANGE_M = 5000.0


def tick_time(app: "SimulationApp"):
    ms = app.clock.tick(FPS)
    raw_dt = max(0.0001, min(0.05, ms / 1000.0))
    app.dt_smoothed = 0.85 * app.dt_smoothed + 0.15 * raw_dt
    dt = app.dt_smoothed
    with app.state.time_lock:
        time_scale_val = app.state.time_scale["value"]
    dt_sim = dt * time_scale_val
    # Rotor angle uses fixed step scaled for stable visuals (not tied to frame jitter).
    dt_sim_step = app.sim_step * time_scale_val
    # Spin rotor visuals; bumped multiplier for a faster-looking main rotor.
    app.rotor_angle = (app.rotor_angle + 1440.0 * dt_sim_step) % 360.0
    if app.debug_frames < 3:
        print(f"[sim-log] frame start dt={dt:.4f} dt_sim={dt_sim:.4f} fuel_lah1={app.state.fuel_levels[0]:.1f}")
    return dt, dt_sim, dt_sim_step, time_scale_val


def consume_fuel(app: "SimulationApp", dt_sim):
    for i, _ in enumerate(app.state.uav_types):
        if not app.state.crashed[i]:
            app.state.fuel_levels[i] = max(0.0, app.state.fuel_levels[i] - LAH_FUEL_BURN_LPS * dt_sim)


def sample_cpu(app: "SimulationApp"):
    if app.psutil is None:
        return
    now = time.time()
    if now - app.cpu_last_sample >= 0.5:
        app.cpu_percent = app.psutil.cpu_percent(percpu=True)
        app.cpu_last_sample = now


def reset_all_craft(app: "SimulationApp", ground_clearance_m: float | None = None):
    for i, u in enumerate(app.state.uavs):
        with app.state.uav_locks[i]:
            u.reset()
            sp = app.state.initial_spawn_points[i] if i < len(app.state.initial_spawn_points) else None
            if sp is not None:
                u.s.x, u.s.y = sp[0], sp[1]
                spawn_z = sp[2] if len(sp) >= 3 else u.s.z
            else:
                u.s.x += INITIAL_OFFSETS[i][0]
                u.s.y += INITIAL_OFFSETS[i][1]
                spawn_z = u.s.z
            if ground_clearance_m is not None and app.dem is not None:
                ground_z = app.dem.get_height(u.s.x, u.s.y)
                u.s.z = ground_z + ground_clearance_m
            else:
                u.s.z = spawn_z
            app.state.crashed[i] = False
            app.state.crippled[i] = False
            app.state.trails[i].clear()
            app.state.fuel_levels[i] = MAX_FUEL_L
            reset_msg = {
                "type": "reset",
                "state": app._uav_state_tuple(u),
                "time_scale": app.state.time_scale["value"],
            }
            if app.state.proc_cmd_queues:
                app.state.proc_cmd_queues[i].put(reset_msg)


def spawn_target_near_camera(app: "SimulationApp"):
    cx, cy, cz = app.cam.target
    ox = random.uniform(-150, 150)
    oy = random.uniform(-150, 150)
    tx = clamp(cx + ox, -WORLD_HALF, WORLD_HALF)
    ty = clamp(cy + oy, -WORLD_HALF, WORLD_HALF)
    app.state.targets.append(
        MovingTarget(
            x=tx,
            y=ty,
            vmin=TARGET_SPEED_RANGE[0],
            vmax=TARGET_SPEED_RANGE[1],
            roam_center=(tx, ty),
            roam_radius=TARGET_ROAM_RADIUS_M,
            threat=AirDefenseThreat(radar=THREAT_RADAR, weapon=THREAT_WEAPON),
        )
    )


def fire_missile(app: "SimulationApp"):
    active_idx = app.state.active_idx
    if app.state.uav_types[active_idx] != "LAH":
        print("[fire] Missiles available only on LAH (slots 1-3)")
        return
    if not app.state.targets:
        return
    uav = app.state.uavs[active_idx]
    with app.state.uav_locks[active_idx]:
        nearest = min(
            app.state.targets,
            key=lambda T: (T.x - uav.s.x) ** 2 + (T.y - uav.s.y) ** 2 + (T.z - uav.s.z) ** 2,
        )
        yaw_rad = math.radians(uav.s.yaw)
        launch_x = uav.s.x + math.cos(yaw_rad) * 8.0
        launch_y = uav.s.y + math.sin(yaw_rad) * -8.0
        launch_z = uav.s.z
    app.state.missiles.append(Missile(launch_x, launch_y, launch_z, nearest))


def capture_snapshots_and_trails(app: "SimulationApp"):
    snapshots = []
    for i, u in enumerate(app.state.uavs):
        # Incorporate any buffered states to avoid skipping points at high time-scale.
        pending = app.state.pending_states[i]
        if pending:
            trail = app.state.trails[i]
            for state_tuple in pending:
                sx, sy, sz, sroll, spitch, syaw, su, sp, sq, sr = state_tuple
                if trail:
                    prev = trail[-1]
                    if math.dist((sx, sy, sz), prev) >= 5.0:
                        trail.append((sx, sy, sz))
                else:
                    trail.append((sx, sy, sz))
            app.state.pending_states[i] = []
        with app.state.uav_locks[i]:
            s = u.s
            pos = (s.x, s.y, s.z, s.roll, s.pitch, s.yaw, u.s.u)
        snapshots.append(pos)
        trail = app.state.trails[i]
        if trail:
            prev = trail[-1]
            seg = math.dist(pos[:3], prev)
            if seg >= 5.0:
                trail.append(pos[:3])
        else:
            trail.append(pos[:3])
        total = 0.0
        cutoff_idx = 0
        for j in range(len(trail) - 1, 0, -1):
            total += math.dist(trail[j], trail[j - 1])
            if total > 1000.0:
                cutoff_idx = j - 1
                break
        if cutoff_idx > 0:
            app.state.trails[i] = trail[cutoff_idx:]
        elif total <= 0.0 and len(trail) > 2:
            app.state.trails[i] = trail[-2:]
    active_snap = snapshots[app.state.active_idx]
    app.cam.target[0] = active_snap[0]
    app.cam.target[1] = active_snap[1]
    app.cam.target[2] = active_snap[2]
    if app.debug_frames < 1:
        print("[sim-log] after snapshots/trails")
    return snapshots, active_snap


def nearest_target(app: "SimulationApp", active_snap):
    if not app.state.targets:
        return None
    candidates = []
    for t in app.state.targets:
        d2 = (t.x - active_snap[0]) ** 2 + (t.y - active_snap[1]) ** 2 + (t.z - active_snap[2]) ** 2
        if d2 <= MAX_TARGET_RANGE_M**2:
            candidates.append((d2, t))
    if not candidates:
        return None
    return min(candidates, key=lambda p: p[0])[1]


def default_downward_target(app: "SimulationApp", uav):
    # look straight down relative to aircraft (pitch -90)
    dir_vec = np.array([0.0, 0.0, -1.0], dtype=float)
    origin = np.array([uav.s.x, uav.s.y, uav.s.z], dtype=float)
    hit = ray_intersect_dem(origin, dir_vec, app.dem)
    if hit is not None:
        return tuple(hit.tolist())
    return tuple((origin + dir_vec * 1000.0).tolist())


def update_targets_and_missiles(app: "SimulationApp", dt_sim):
    if app.state.targets_move:
        for t in app.state.targets:
            t.step(dt_sim, dem=app.dem)
    for m in list(app.state.missiles):
        m.step(dt_sim)
        if (not m.active) and (m.exploded and m.explode_time > 1.2 and len(m.sparks) == 0):
            app.state.missiles.remove(m)
    if app.debug_frames < 1:
        print("[sim-log] after target/missile step")


def evaluate_threats(app: "SimulationApp", dt_sim):
    detection_lines = []
    if app.state.threat_kill_disabled:
        # Clear any lingering crippled flags while kills are disabled.
        for i in range(len(app.state.crippled)):
            app.state.crippled[i] = False
    for idx, u in enumerate(app.state.uavs):
        with app.state.uav_locks[idx]:
            upos = np.array([u.s.x, u.s.y, u.s.z], dtype=float)
        for t in app.state.targets:
            # Skip far targets to avoid heavy LOS/DEM work.
            if np.linalg.norm(np.array([t.x - upos[0], t.y - upos[1], t.z - upos[2]])) > MAX_TARGET_RANGE_M:
                continue
            if not getattr(t, "threat", None):
                continue
            # If threat lethality is disabled, skip kill logic entirely.
            if app.state.threat_kill_disabled or getattr(getattr(t.threat, "weapon", None), "omega", 1.0) == 0.0:
                # Still track detection for HUD lines but never apply damage.
                los = check_los(upos, np.array([t.x, t.y, t.z]), app.dem)
                t.threat.detection_prob(float(np.linalg.norm(np.array([t.x, t.y, t.z]) - upos)), los, dt_sim)
                detection_lines.append(
                    {
                        "tpos": (t.x, t.y, t.z),
                        "upos": tuple(upos),
                        "los": los,
                        "detected": t.threat.state.detected,
                    }
                )
                continue
            range_m = float(np.linalg.norm(np.array([t.x, t.y, t.z]) - upos))
            if range_m < 1e-3:
                continue
            los = check_los(upos, np.array([t.x, t.y, t.z]), app.dem)
            t.threat.detection_prob(range_m, los, dt_sim)
            pk = t.threat.kill_prob(range_m, dt_sim)
            detection_lines.append(
                {
                    "tpos": (t.x, t.y, t.z),
                    "upos": tuple(upos),
                    "los": los,
                    "detected": t.threat.state.detected,
                }
            )
            if pk > 0.0 and random.random() < pk:
                with app.state.uav_locks[idx]:
                    app.state.crippled[idx] = True
                    u.cmd_throttle = -1.0
                    u.cmd_pitch_rate = -u.p.max_pitch_rate_dps * 0.8
                    spin_sign = 1.0 if random.random() < 0.5 else -1.0
                    u.cmd_roll_rate = u.p.max_roll_rate_dps * 0.6 * spin_sign
                    u.cmd_yaw_rate = u.p.max_yaw_rate_dps * 0.6 * spin_sign
    if app.debug_frames < 1:
        print("[sim-log] after threat eval")
    return detection_lines


def remove_destroyed_targets(app: "SimulationApp"):
    app.state.targets = [t for t in app.state.targets if getattr(t, "alive", True)]
