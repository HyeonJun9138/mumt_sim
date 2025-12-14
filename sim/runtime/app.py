import json
import math
import os
import random
import threading
import time
from pathlib import Path

import numpy as np
import pygame
from pygame.locals import DOUBLEBUF, OPENGL
from OpenGL.GL import (
    glBegin,
    glBlendFunc,
    glClear,
    glClearColor,
    glColor3f,
    glColor4f,
    glDisable,
    glEnable,
    glEnd,
    glFogf,
    glFogfv,
    glFogi,
    glHint,
    glLineWidth,
    glLineStipple,
    glLoadIdentity,
    glMatrixMode,
    GL_MODELVIEW,
    GL_PROJECTION,
    glVertex3f,
    glViewport,
    GL_BLEND,
    GL_COLOR_BUFFER_BIT,
    GL_DEPTH_BUFFER_BIT,
    GL_DEPTH_TEST,
    GL_EXP2,
    GL_FOG,
    GL_FOG_COLOR,
    GL_FOG_DENSITY,
    GL_FOG_HINT,
    GL_FOG_MODE,
    GL_LINES,
    GL_LINE_STRIP,
    GL_LINE_STIPPLE,
    GL_NICEST,
    GL_POINTS,
    GL_SRC_ALPHA,
    GL_ONE_MINUS_SRC_ALPHA,
    GLfloat,
    glGetString,
    GL_VENDOR,
    GL_RENDERER,
    GL_VERSION,
    GL_SHADING_LANGUAGE_VERSION,
    glGetError,
    glGetDoublev,
    glGetIntegerv,
    GL_MODELVIEW_MATRIX,
    GL_PROJECTION_MATRIX,
    GL_VIEWPORT,
    glPointSize,
)
from OpenGL.GLU import gluPerspective, gluProject

from sim.agent_status import LAH_FUEL_BURN_LPS, MAX_FUEL_L, build_agent_status_snapshot
from sim.config import (
    CLEAR_COLOR,
    DEM_CACHE_MOVE_THRESHOLD_M,
    DEM_FILE,
    DEM_PREFETCH_RADIUS_M,
    FPS,
    FOG_COLOR,
    FOG_DENSITY,
    REPO_ROOT,
    RENDER_RADIUS_M,
    WIN_H,
    WIN_W,
    WORLD_HALF,
    clamp,
)
from sim.core.camera import OrbitCamera
from sim.entities.entities import Missile, MovingTarget
from sim.entities.threat import AirDefenseThreat
from sim.render.draw import (
    draw_axes,
    draw_camera_footprint,
    draw_dem,
    draw_grid,
    draw_hud,
    draw_labels,
    draw_lah,
    draw_uav,
)
from sim.world.dem import DEM, check_los
from .autopilot import build_missions, update_autopilot
from .constants import INITIAL_OFFSETS, TARGET_ROAM_RADIUS_M, TARGET_SPEED_RANGE, THREAT_RADAR, THREAT_WEAPON
from .state import SimulationState, build_initial_state
from .workers import uav_worker


def _log(msg: str):
    print(f"[sim-log] {msg}", flush=True)


class SimulationApp:
    def __init__(self):
        self.cam = OrbitCamera()
        self.clock: pygame.time.Clock | None = None
        self.dem: DEM | None = None
        self.state: SimulationState | None = None
        self.orbit = False
        self.pan = False
        self.last = (0, 0)
        self.debug_frames = 0
        self.dt_smoothed = 1 / 60
        self.running = True
        self.rotor_angle = 0.0
        self.log_thread: threading.Thread | None = None
        self.log_stop_event: threading.Event | None = None
        self.log_file_path: Path | None = None
        self.mission_name: str = ""

    def _mission_for_idx(self, idx: int):
        if not self.state or idx >= len(self.state.missions):
            return None
        return self.state.missions[idx]

    def _autopilot_active_for(self, idx: int) -> bool:
        if not self.state:
            return False
        mission = self._mission_for_idx(idx)
        return bool(self.state.autopilot_enabled and mission and mission.active and not mission.completed)

    def setup(self):
        self.dem = DEM(str(DEM_FILE))
        self._log_dem_info()
        self._init_display()
        self.state = build_initial_state()
        self._start_workers()
        self._start_logger()
        _log("entering main loop")

    def run(self):
        self.setup()
        try:
            while self.running:
                dt, dt_sim, time_scale_val = self._tick_time()
                self._consume_fuel(dt_sim)
                keys = pygame.key.get_pressed()
                self._apply_continuous_input(keys, dt)
                self._process_events()
                self._update_autopilot(dt_sim)
                snapshots, active_snap = self._capture_snapshots_and_trails()
                nearest = self._nearest_target(active_snap)
                self._update_targets_and_missiles(dt_sim)
                detection_lines = self._evaluate_threats(dt_sim)
                self._remove_destroyed_targets()
                self.state.latest_agent_status_0401 = build_agent_status_snapshot(
                    self.state.uav_types,
                    snapshots,
                    self.state.fuel_levels,
                    self.state.crashed,
                    self.state.crippled,
                    self.dem,
                    self.state.targets,
                    self.state.fov_diag,
                )
                uav_lon, uav_lat = self.dem.env_to_lonlat(active_snap[0], active_snap[1])
                self._render_frame(
                    snapshots, active_snap, nearest, detection_lines, dt_sim, time_scale_val, uav_lon, uav_lat
                )
        finally:
            self._shutdown()

    def _log_dem_info(self):
        assert self.dem is not None
        print(f"[DEM] file: {DEM_FILE}")
        print(
            f"[DEM] shape: {self.dem.elevation.shape} lon/lat bounds: ({self.dem.xmin:.5f},{self.dem.ymin:.5f})-({self.dem.xmax:.5f},{self.dem.ymax:.5f})"
        )
        print(f"[DEM] elevation min/max: {self.dem.min_elev:.1f}/{self.dem.max_elev:.1f}")

    def _init_display(self):
        pygame.init()
        caption = os.getenv("SIM_INSTANCE_NAME", "UAV 6-DoF 3D + DEM + Missile (modular)")
        pygame.display.set_caption(caption)
        print(f"[sim] window caption: {caption}")
        pygame.display.set_mode((WIN_W, WIN_H), DOUBLEBUF | OPENGL)
        self.clock = pygame.time.Clock()

        print("[GL] vendor:", glGetString(GL_VENDOR))
        print("[GL] renderer:", glGetString(GL_RENDERER))
        print("[GL] version:", glGetString(GL_VERSION))
        print("[GL] GLSL:", glGetString(GL_SHADING_LANGUAGE_VERSION))

        glViewport(0, 0, WIN_W, WIN_H)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glClearColor(*CLEAR_COLOR)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60.0, WIN_W / WIN_H, 0.1, 30000.0)

        glEnable(GL_FOG)
        fog_color = (GLfloat * 4)(*FOG_COLOR)
        glFogfv(GL_FOG_COLOR, fog_color)
        glFogi(GL_FOG_MODE, GL_EXP2)
        glFogf(GL_FOG_DENSITY, FOG_DENSITY)
        glHint(GL_FOG_HINT, GL_NICEST)

    def _start_workers(self):
        assert self.dem is not None
        assert self.state is not None
        self.state.workers = [
            threading.Thread(
                target=uav_worker,
                args=(
                    i,
                    u,
                    self.dem,
                    self.state.run_event,
                    self.state.crashed,
                    self.state.crippled,
                    self.state.uav_locks[i],
                    self.state.time_scale,
                    self.state.time_lock,
                ),
                daemon=True,
            )
            for i, u in enumerate(self.state.uavs)
        ]
        for t in self.state.workers:
            t.start()

    def _start_logger(self):
        log_dir = REPO_ROOT / "log"
        log_dir.mkdir(parents=True, exist_ok=True)
        existing = sorted(log_dir.glob("*_0401.njson"))
        mission_idx = len(existing) + 1
        self.mission_name = f"임시{mission_idx}"
        self.log_file_path = log_dir / f"{self.mission_name}_0401.njson"
        self.log_stop_event = threading.Event()

        def _loop():
            while not self.log_stop_event.is_set():
                if self.state and self.state.latest_agent_status_0401:
                    try:
                        with self.log_file_path.open("a", encoding="utf-8") as f:
                            f.write(json.dumps(self.state.latest_agent_status_0401, ensure_ascii=False) + "\n")
                    except Exception as e:
                        print(f"[log] write failed: {e}")
                time.sleep(0.2)  # 5 Hz

        self.log_thread = threading.Thread(target=_loop, daemon=True)
        self.log_thread.start()

    def _tick_time(self):
        ms = self.clock.tick(FPS)
        raw_dt = max(0.0001, min(0.05, ms / 1000.0))
        self.dt_smoothed = 0.85 * self.dt_smoothed + 0.15 * raw_dt
        dt = self.dt_smoothed
        with self.state.time_lock:
            time_scale_val = self.state.time_scale["value"]
        dt_sim = dt * time_scale_val
        self.rotor_angle = (self.rotor_angle + 720.0 * dt_sim) % 360.0
        if self.debug_frames < 3:
            _log(f"frame start dt={dt:.4f} dt_sim={dt_sim:.4f} fuel_lah1={self.state.fuel_levels[0]:.1f}")
        return dt, dt_sim, time_scale_val

    def _consume_fuel(self, dt_sim):
        for i, _ in enumerate(self.state.uav_types):
            if not self.state.crashed[i]:
                self.state.fuel_levels[i] = max(0.0, self.state.fuel_levels[i] - LAH_FUEL_BURN_LPS * dt_sim)

    def _apply_continuous_input(self, keys, dt):
        active_idx = self.state.active_idx
        uav = self.state.uavs[active_idx]
        is_lah = self.state.uav_types[active_idx] == "LAH"
        autop_active = self._autopilot_active_for(active_idx)
        if not autop_active:
            with self.state.uav_locks[active_idx]:
                if not self.state.crippled[active_idx]:
                    if keys[pygame.K_w]:
                        uav.cmd_throttle = 1.0
                    elif keys[pygame.K_s]:
                        uav.cmd_throttle = -1.0
                    else:
                        uav.cmd_throttle = 0.0

                    if keys[pygame.K_SPACE]:
                        uav.cmd_hover() if is_lah else uav.cmd_straight()
                    else:
                        if keys[pygame.K_LEFT]:
                            uav.cmd_left()
                        elif keys[pygame.K_RIGHT]:
                            uav.cmd_right()
                        else:
                            uav.cmd_yaw_rate = 0.0
                            uav.cmd_roll_rate = -uav.s.roll * 2.0
                        if keys[pygame.K_UP]:
                            uav.cmd_climb()
                        elif keys[pygame.K_DOWN]:
                            uav.cmd_descend()
                        else:
                            uav.neutralize_pitch()
        if keys[pygame.K_e]:
            self.state.fov_diag += 15.0 * dt
        if keys[pygame.K_d]:
            self.state.fov_diag -= 15.0 * dt
        self.state.fov_diag = clamp(self.state.fov_diag, 1.2, 31.2)
        if self.debug_frames < 1:
            _log("after input/keys")

    def _process_events(self):
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                self.running = False
            elif e.type == pygame.KEYDOWN:
                self._handle_keydown(e.key)
            elif e.type == pygame.MOUSEBUTTONDOWN:
                self._handle_mouse_down(e)
            elif e.type == pygame.MOUSEBUTTONUP:
                self._handle_mouse_up(e)
            elif e.type == pygame.MOUSEMOTION:
                self._handle_mouse_motion(e)
        if self.debug_frames < 1:
            _log("after events loop")

    def _handle_keydown(self, key):
        if key in (pygame.K_ESCAPE, pygame.K_q):
            self.running = False
        elif key == pygame.K_r:
            self._reset_all_craft(stop_autopilot=True)
        elif key == pygame.K_f:
            self.state.fog_enabled = not self.state.fog_enabled
        elif key == pygame.K_n:
            self.state.targets_move = not self.state.targets_move
        elif key == pygame.K_p:
            self._start_autopilot_missions()
        elif key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            with self.state.time_lock:
                self.state.time_scale["value"] = clamp(self.state.time_scale["value"] + 1.0, 0.1, 100.0)
            print(f"[time] scale -> {self.state.time_scale['value']:.1f}x")
        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            with self.state.time_lock:
                self.state.time_scale["value"] = clamp(self.state.time_scale["value"] - 1.0, 0.1, 100.0)
            print(f"[time] scale -> {self.state.time_scale['value']:.1f}x")
        elif key == pygame.K_g:
            self._spawn_target_near_camera()
        elif key == pygame.K_m:
            self._fire_missile()
        elif key == pygame.K_1:
            self.state.active_idx = 0
        elif key == pygame.K_2 and len(self.state.uavs) > 1:
            self.state.active_idx = 1
        elif key == pygame.K_3 and len(self.state.uavs) > 2:
            self.state.active_idx = 2
        elif key == pygame.K_4 and len(self.state.uavs) > 3:
            self.state.active_idx = 3
        elif key == pygame.K_5 and len(self.state.uavs) > 4:
            self.state.active_idx = 4
        elif key == pygame.K_6 and len(self.state.uavs) > 5:
            self.state.active_idx = 5

    def _handle_mouse_down(self, event):
        if event.button == 3:
            self.orbit = True
            self.last = event.pos
        elif event.button == 2:
            self.pan = True
            self.last = event.pos
        elif event.button == 4:
            self.cam.distance = max(50.0, self.cam.distance * 0.9)
        elif event.button == 5:
            self.cam.distance = min(8000.0, self.cam.distance * 1.1)

    def _handle_mouse_up(self, event):
        if event.button == 3:
            self.orbit = False
        elif event.button == 2:
            self.pan = False

    def _handle_mouse_motion(self, event):
        if self.orbit:
            dx = event.pos[0] - self.last[0]
            dy = event.pos[1] - self.last[1]
            self.cam.yaw += dx * 0.3
            self.cam.pitch = clamp(self.cam.pitch - dy * 0.3, -89.0, 89.0)
            self.last = event.pos
        elif self.pan:
            dx = event.pos[0] - self.last[0]
            dy = event.pos[1] - self.last[1]
            pan_scale = self.cam.distance * 0.002
            self.cam.target[0] -= dx * pan_scale
            self.cam.target[1] += dy * pan_scale
            self.last = event.pos

    def _reset_all_craft(self, stop_autopilot: bool = False, ground_clearance_m: float | None = None):
        if stop_autopilot:
            self.state.autopilot_enabled = False
            self.state.missions = []
        for i, u in enumerate(self.state.uavs):
            with self.state.uav_locks[i]:
                u.reset()
                sp = self.state.initial_spawn_points[i] if i < len(self.state.initial_spawn_points) else None
                if sp is not None:
                    u.s.x, u.s.y = sp[0], sp[1]
                    spawn_z = sp[2] if len(sp) >= 3 else u.s.z
                else:
                    u.s.x += INITIAL_OFFSETS[i][0]
                    u.s.y += INITIAL_OFFSETS[i][1]
                    spawn_z = u.s.z
                if ground_clearance_m is not None and self.dem is not None:
                    ground_z = self.dem.get_height(u.s.x, u.s.y)
                    u.s.z = ground_z + ground_clearance_m
                else:
                    u.s.z = spawn_z
                self.state.crashed[i] = False
                self.state.crippled[i] = False
                self.state.trails[i].clear()
                self.state.fuel_levels[i] = MAX_FUEL_L

    def _start_autopilot_missions(self):
        if not self.dem:
            print("[auto] DEM not loaded; cannot start autopilot missions.")
            return
        self._reset_all_craft(stop_autopilot=False, ground_clearance_m=50.0)
        origins = [(u.s.x, u.s.y, u.s.z) for u in self.state.uavs]
        self.state.missions = build_missions(origins, self.dem)
        self.state.autopilot_enabled = True
        print(f"[auto] Autopilot started for {len(self.state.missions)} craft (5 waypoints each, +50m AGL)")

    def _spawn_target_near_camera(self):
        cx, cy, cz = self.cam.target
        ox = random.uniform(-150, 150)
        oy = random.uniform(-150, 150)
        tx = clamp(cx + ox, -WORLD_HALF, WORLD_HALF)
        ty = clamp(cy + oy, -WORLD_HALF, WORLD_HALF)
        self.state.targets.append(
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

    def _fire_missile(self):
        active_idx = self.state.active_idx
        if self.state.uav_types[active_idx] != "LAH":
            print("[fire] Missiles available only on LAH (slots 1-3)")
            return
        if not self.state.targets:
            return
        uav = self.state.uavs[active_idx]
        with self.state.uav_locks[active_idx]:
            nearest = min(
                self.state.targets,
                key=lambda T: (T.x - uav.s.x) ** 2 + (T.y - uav.s.y) ** 2 + (T.z - uav.s.z) ** 2,
            )
            yaw_rad = math.radians(uav.s.yaw)
            launch_x = uav.s.x + math.cos(yaw_rad) * 8.0
            launch_y = uav.s.y + math.sin(yaw_rad) * -8.0
            launch_z = uav.s.z
        self.state.missiles.append(Missile(launch_x, launch_y, launch_z, nearest))

    def _capture_snapshots_and_trails(self):
        snapshots = []
        for i, u in enumerate(self.state.uavs):
            with self.state.uav_locks[i]:
                s = u.s
                pos = (s.x, s.y, s.z, s.roll, s.pitch, s.yaw, u.s.u)
            snapshots.append(pos)
            trail = self.state.trails[i]
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
                self.state.trails[i] = trail[cutoff_idx:]
            elif total <= 0.0 and len(trail) > 2:
                self.state.trails[i] = trail[-2:]
        active_snap = snapshots[self.state.active_idx]
        self.cam.target[0] = active_snap[0]
        self.cam.target[1] = active_snap[1]
        self.cam.target[2] = active_snap[2]
        if self.debug_frames < 1:
            _log("after snapshots/trails")
        return snapshots, active_snap

    def _nearest_target(self, active_snap):
        if not self.state.targets:
            return None
        return min(
            self.state.targets,
            key=lambda T: (T.x - active_snap[0]) ** 2 + (T.y - active_snap[1]) ** 2 + (T.z - active_snap[2]) ** 2,
        )

    def _update_autopilot(self, dt_sim):
        if not self.state or not self.dem:
            return
        update_autopilot(self.state, self.dem, dt_sim)

    def _update_targets_and_missiles(self, dt_sim):
        if self.state.targets_move:
            for t in self.state.targets:
                t.step(dt_sim, dem=self.dem)
        for m in list(self.state.missiles):
            m.step(dt_sim)
            if (not m.active) and (m.exploded and m.explode_time > 1.2 and len(m.sparks) == 0):
                self.state.missiles.remove(m)
        if self.debug_frames < 1:
            _log("after target/missile step")

    def _evaluate_threats(self, dt_sim):
        detection_lines = []
        for idx, u in enumerate(self.state.uavs):
            with self.state.uav_locks[idx]:
                upos = np.array([u.s.x, u.s.y, u.s.z], dtype=float)
            for t in self.state.targets:
                if not getattr(t, "threat", None):
                    continue
                range_m = float(np.linalg.norm(np.array([t.x, t.y, t.z]) - upos))
                if range_m < 1e-3:
                    continue
                los = check_los(upos, np.array([t.x, t.y, t.z]), self.dem)
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
                    with self.state.uav_locks[idx]:
                        self.state.crippled[idx] = True
                        u.cmd_throttle = -1.0
                        u.cmd_pitch_rate = -u.p.max_pitch_rate_dps * 0.8
                        spin_sign = 1.0 if random.random() < 0.5 else -1.0
                        u.cmd_roll_rate = u.p.max_roll_rate_dps * 0.6 * spin_sign
                        u.cmd_yaw_rate = u.p.max_yaw_rate_dps * 0.6 * spin_sign
        if self.debug_frames < 1:
            _log("after threat eval")
        return detection_lines

    def _remove_destroyed_targets(self):
        self.state.targets = [t for t in self.state.targets if getattr(t, "alive", True)]

    def _render_frame(self, snapshots, active_snap, nearest, detection_lines, dt_sim, time_scale_val, uav_lon, uav_lat):
        is_lah = self.state.uav_types[self.state.active_idx] == "LAH"

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self.cam.apply()

        if self.state.fog_enabled:
            glEnable(GL_FOG)
        else:
            glDisable(GL_FOG)

        glDisable(GL_BLEND)
        draw_grid(size=int(RENDER_RADIUS_M), step=100)
        draw_axes(50)
        draw_dem(
            self.dem,
            self.cam,
            center_xy=(active_snap[0], active_snap[1]),
            radius_m=RENDER_RADIUS_M,
            prefetch_radius_m=DEM_PREFETCH_RADIUS_M,
            move_threshold=DEM_CACHE_MOVE_THRESHOLD_M,
            z_scale=1.0,
        )
        glEnable(GL_BLEND)

        for idx, trail in enumerate(self.state.trails):
            if len(trail) >= 2:
                glDisable(GL_LINE_STIPPLE)
                glLineWidth(2.0 if idx == self.state.active_idx else 1.5)
                if idx == self.state.active_idx:
                    glColor3f(0.2, 0.6, 1.0)
                else:
                    glColor3f(0.4, 0.5, 0.7)
                glBegin(GL_LINE_STRIP)
                for p in trail:
                    glVertex3f(*p)
                glEnd()

        for i, u in enumerate(self.state.uavs):
            with self.state.uav_locks[i]:
                if self.state.uav_types[i] == "LAH":
                    draw_lah(u, self.rotor_angle)
                else:
                    draw_uav(u)
        for t in self.state.targets:
            t.draw()
        for m in self.state.missiles:
            m.draw(self.dem)
        if detection_lines:
            glLineWidth(1.0)
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(1, 0x0F0F)
            for info in detection_lines:
                if info["los"]:
                    glColor4f(1.0, 0.25, 0.25, 0.5)
                else:
                    glColor4f(0.45, 0.45, 0.45, 0.5)
                glBegin(GL_LINES)
                glVertex3f(*info["tpos"])
                glVertex3f(*info["upos"])
                glEnd()
            glDisable(GL_LINE_STIPPLE)

        mission = self._mission_for_idx(self.state.active_idx)
        if mission and mission.waypoints and self.state.autopilot_enabled:
            glPointSize(7.0)
            glColor3f(0.95, 0.85, 0.2)
            glBegin(GL_POINTS)
            for wp in mission.waypoints:
                glVertex3f(wp.x, wp.y, wp.z)
            glEnd()
            glPointSize(1.0)

        model = glGetDoublev(GL_MODELVIEW_MATRIX)
        proj = glGetDoublev(GL_PROJECTION_MATRIX)
        viewport = glGetIntegerv(GL_VIEWPORT)
        label_entries = []
        for idx, snap in enumerate(snapshots):
            sx, sy, sz = gluProject(snap[0], snap[1], snap[2] + 10.0, model, proj, viewport)
            if 0.0 <= sz <= 1.0:
                color = (255, 80, 80) if idx == self.state.active_idx else (220, 220, 220)
                name = f"LAH{idx + 1}" if idx < 3 else f"UAV{idx - 2}"
                speed_text = f"{snap[6]:.0f}m/s"
                state_tag = "FALL" if self.state.crippled[idx] else ("CRASH" if self.state.crashed[idx] else "")
                mission = self._mission_for_idx(idx)
                wp_info = ""
                if mission and mission.waypoints:
                    wp_info = f" crt_wp:{mission.current_idx + 1}/{len(mission.waypoints)}"
                text = f"{name} {speed_text}{wp_info}" if not state_tag else f"{name} {speed_text} {state_tag}{wp_info}"
                label_entries.append((sx, sy + 8, text, color))
        for tidx, t in enumerate(self.state.targets):
            sx, sy, sz = gluProject(t.x, t.y, t.z + 10.0, model, proj, viewport)
            if 0.0 <= sz <= 1.0:
                label_entries.append((sx, sy + 8, f"target{tidx + 1}", (255, 200, 80)))
        if mission and mission.waypoints and self.state.autopilot_enabled:
            for wp in mission.waypoints:
                sx, sy, sz = gluProject(wp.x, wp.y, wp.z + 2.0, model, proj, viewport)
                if 0.0 <= sz <= 1.0:
                    label_entries.append((sx, sy + 6, wp.name, (240, 210, 90)))
        draw_labels(label_entries)

        footprint_area = None
        los_clear = False
        if nearest and (not is_lah):
            uav_pos = np.array([active_snap[0], active_snap[1], active_snap[2]], dtype=float)
            tgt_pos = np.array([nearest.x, nearest.y, nearest.z], dtype=float)
            los_clear = check_los(uav_pos, tgt_pos, self.dem)
            glLineWidth(2.0)
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(1, 0x3333)
            glColor3f(0.6, 0.6, 0.6)
            glBegin(GL_LINES)
            glVertex3f(*uav_pos)
            glVertex3f(*tgt_pos)
            glEnd()
            glDisable(GL_LINE_STIPPLE)
            with self.state.uav_locks[self.state.active_idx]:
                footprint_area = draw_camera_footprint(
                    self.state.uavs[self.state.active_idx],
                    (nearest.x, nearest.y, nearest.z),
                    fov_diag_deg=self.state.fov_diag,
                    dem=self.dem,
                    aspect_ratio=16 / 9,
                    z_scale=1.0,
                )

        lines = [
            f"Active: {'LAH' if is_lah else 'UAV'}{self.state.active_idx + 1}  pos (m): x={active_snap[0]:7.1f}  y={active_snap[1]:7.1f}  z={active_snap[2]:6.1f}",
            f"pos (lat/lon): lat={uav_lat:9.5f}  lon={uav_lon:10.5f}",
            f"spd (m/s): {active_snap[6]:5.1f}   yaw={active_snap[5]:6.1f}deg  pitch={active_snap[4]:5.1f}deg  roll={active_snap[3]:5.1f}deg",
            f"FOV (diag): {self.state.fov_diag:4.1f}deg   LOS: {1 if los_clear else 0}   dt={dt_sim*1000:.1f}ms  time x{time_scale_val:.1f}",
            f"Targets: {len(self.state.targets)}  Missiles: {len(self.state.missiles)}",
            "1-6: switch craft (1-3 LAH, 4-6 UAV) | M: Fire (LAH only) | N: Toggle targets | G: Spawn target | P: Autopilot WPs | Wheel: Zoom | RMB drag: Orbit | MMB drag: Pan",
        ]
        if mission and mission.waypoints:
            if self._autopilot_active_for(self.state.active_idx):
                status = "ON"
            elif mission.completed:
                status = "DONE"
            elif self.state.autopilot_enabled:
                status = "HOLD"
            else:
                status = "OFF"
            lines.append(f"Autopilot: {status} wp {mission.current_idx + 1}/{len(mission.waypoints)} (P to regenerate)")
        if footprint_area is not None:
            lines.append(f"Footprint area: {footprint_area:8.1f} m^2")
        if any(self.state.crippled) and not any(self.state.crashed):
            crippled_ids = [str(i + 1) for i, c in enumerate(self.state.crippled) if c]
            lines.append(f"Status: DAMAGED UAVs: {', '.join(crippled_ids)} (falling)")
        if any(self.state.crashed):
            crashed_ids = [str(i + 1) for i, c in enumerate(self.state.crashed) if c]
            lines.append(f"Status: CRASHED craft: {', '.join(crashed_ids)} (press R to reset)")
        draw_hud(lines)

        if self.debug_frames < 3:
            print(
                f"[frame] cam_target=({self.cam.target[0]:.1f},{self.cam.target[1]:.1f},{self.cam.target[2]:.1f}) "
                f"cam_dist={self.cam.distance:.1f} uav_pos=({active_snap[0]:.1f},{active_snap[1]:.1f},{active_snap[2]:.1f}) "
                f"time_scale={time_scale_val:.1f} fog={self.state.fog_enabled}"
            )
            err = glGetError()
            if err:
                print(f"[GL ERROR] frame: {err}")
            self.debug_frames += 1

        pygame.display.flip()
        if self.debug_frames < 2:
            _log("after display flip")

    def _shutdown(self):
        if self.state:
            self.state.run_event.clear()
            for t in self.state.workers:
                t.join(timeout=0.2)
        if self.log_stop_event:
            self.log_stop_event.set()
        if self.log_thread:
            self.log_thread.join(timeout=0.5)
        pygame.quit()
