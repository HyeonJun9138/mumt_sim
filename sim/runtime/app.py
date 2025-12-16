import json
import re
import math
import os
import queue
import random
import threading
import time
import multiprocessing
from pathlib import Path

import numpy as np
import pygame
from pygame.locals import DOUBLEBUF, OPENGL
try:
    import psutil
except ImportError:
    psutil = None
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
    glOrtho,
    glPushMatrix,
    glPopMatrix,
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
    GL_QUADS,
)
from OpenGL.GLU import gluPerspective, gluProject

from sim.agent_status import LAH_FUEL_BURN_LPS, MAX_FUEL_L, build_agent_status_snapshot
from sim.config import (
    CLEAR_COLOR,
    DEM_FILE,
    DEM_CACHE_MOVE_THRESHOLD_M,
    DEM_PREFETCH_RADIUS_M,
    FPS,
    FOG_COLOR,
    FOG_DENSITY,
    MAP_DIR,
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
from sim.runtime import scenario_loader
from sim.runtime.controllers import DEFAULT_TUNED_GAINS, PIDGains, WaypointPIDController, WaypointTarget, load_pid_gains
from sim.render.draw import (
    draw_axes,
    draw_camera_footprint,
    draw_dem,
    draw_grid,
    draw_hud,
    draw_labels,
    draw_lah,
    draw_polyline,
    draw_uav,
)
from sim.world.dem import DEM, check_los, ray_intersect_dem
from .constants import INITIAL_OFFSETS, TARGET_ROAM_RADIUS_M, TARGET_SPEED_RANGE, THREAT_RADAR, THREAT_WEAPON
from .state import SimulationState, build_initial_state
from .workers import run_uav_process

# Targets farther than this (from active craft) are ignored for threat/LOS to avoid heavy DEM work.
# Allow LOS/threat checks for targets farther from the active aircraft; mission
# targets can spawn >10 km away from takeoff points.
MAX_TARGET_RANGE_M = 15000.0


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
        log_0401_env = os.getenv("SIM_LOG_0401_DIR")
        self.log_dir_0401: Path = Path(log_0401_env) if log_0401_env else REPO_ROOT / "log"
        log_0402_env = os.getenv("SIM_LOG_0402_DIR")
        self.log_dir_0402: Path = Path(log_0402_env) if log_0402_env else self.log_dir_0401
        self.log0402_file_path: Path | None = None
        self.logged_target_ids_0402: set[int] = set()
        self.mission_name: str = ""
        self.cpu_count = os.cpu_count() or 0
        self.cpu_percent = None
        self.cpu_last_sample = 0.0
        self.sim_step = 0.01  # fixed simulation step (seconds)
        self.mission_reference_path: Path | None = None
        self.flight_path_root: Path = REPO_ROOT / "log" / "EP-000001" / "database_EP_000001" / "FlightPath"
        self.enable_flight_paths: bool = False
        self.enable_uav_autopilot: bool = True
        self.pid_gains_path: Path = REPO_ROOT / "test" / "uav_pid_db" / "uav_pid_tuned_gains.json"
        self.pid_gains_path_lah: Path = REPO_ROOT / "test" / "lah_pid_db" / "lah_pid_tuned_gains.json"
        self.pid_db_path_uav: Path = REPO_ROOT / "sim" / "runtime" / "controllers" / "uav_pid_db.json"
        self.pid_db_path_lah: Path = REPO_ROOT / "sim" / "runtime" / "controllers" / "lah_pid_db.json"
        self.pid_db_cache: dict[str, list[dict]] = {}
        self.uav_autopilots: list[WaypointPIDController | None] = []
        self.uav_filming_props: list[dict | None] = []
        self.uav_current_wp_ids: list[int | None] = []
        self.uav_line_search_state: list[dict | None] = []
        self.uav_filming_target: list[tuple[float, float, float] | None] = []
        self.targets_loaded_from_file: bool = False
        self.mission_root: Path = REPO_ROOT / "log" / "EP-000001" / "database_EP_000001"
        self.selected_dem_file: Path | None = None

    def _apply_env_mission_root(self):
        """Allow SIM_DB_ROOT to override mission reference / flight path roots."""
        env_root = os.getenv("SIM_DB_ROOT")
        if not env_root:
            return
        root = Path(env_root)
        if not root.exists():
            print(f"[env] SIM_DB_ROOT does not exist: {root}")
            return
        self.mission_root = root
        if self.mission_reference_path is None:
            ref_dir = root / "MissionReferenceInfo"
            ref_files = sorted(ref_dir.glob("*.json"))
            if ref_files:
                self.mission_reference_path = ref_files[0]
        self.flight_path_root = root / "FlightPath"
        self.enable_flight_paths = True

    def _guess_anchor_lonlat(self) -> tuple[float, float] | None:
        """Try to pick a representative lon/lat from mission reference for DEM selection."""
        if not self.mission_reference_path or not self.mission_reference_path.exists():
            return None
        try:
            data = json.loads(self.mission_reference_path.read_text(encoding="utf-8"))
        except Exception:
            return None
        sources = data.get("takeOverInfoList") or data.get("rtbCoordinateList") or []
        for item in sources:
            coord = item.get("coordinate") if isinstance(item, dict) else None
            if not coord:
                coord = item if isinstance(item, dict) else None
            if not coord:
                continue
            try:
                lon = float(coord.get("longitude"))
                lat = float(coord.get("latitude"))
                return (lon, lat)
            except Exception:
                continue
        return None


    def _find_dem_tile_for_lonlat(self, lon: float, lat: float) -> Path | None:
        """Find a DEM tile in MAP_DIR whose filename suggests it covers the lon/lat."""
        pattern = re.compile(r"([ns])(\d+)_([ew])(\d+)", re.IGNORECASE)
        for tif in sorted(MAP_DIR.glob("*.tif")):
            m = pattern.search(tif.name)
            if not m:
                continue
            ns, lat_str, ew, lon_str = m.groups()
            try:
                lat_base = float(lat_str)
                lon_base = float(lon_str)
                if ns.lower() == "s":
                    lat_base = -lat_base
                if ew.lower() == "w":
                    lon_base = -lon_base
            except Exception:
                continue
            # Simple check: does the integer degree match?
            if int(math.floor(lat)) == int(lat_base) and int(math.floor(lon)) == int(lon_base):
                return tif
        return None

    def _load_targets_from_targetinfo(self):
        """Replace default targets with those from TargetInfo under the current mission root."""
        if not self.dem or not self.state:
            return
        targets = scenario_loader.load_targets_from_db(self.mission_root, self.dem)
        if not targets:
            return
        self.state.targets = targets
        self.targets_loaded_from_file = True
        print(f"[scenario] loaded {len(targets)} targets from TargetInfo -> {self.mission_root}")

    def _uav_state_tuple(self, uav):
        return (
            uav.s.x,
            uav.s.y,
            uav.s.z,
            uav.s.roll,
            uav.s.pitch,
            uav.s.yaw,
            uav.s.u,
            uav.s.p,
            uav.s.q,
            uav.s.r,
        )

    def setup(self):
        self.state = build_initial_state()
        self._apply_env_mission_root()
        dem_path = DEM_FILE
        anchor = self._guess_anchor_lonlat()
        if anchor:
            lon, lat = anchor
            cand = self._find_dem_tile_for_lonlat(lon, lat)
            if cand:
                dem_path = cand
        self.selected_dem_file = dem_path
        # Load DEM tiles; seed with DEFAULT_DEM_NAME so the initial active tile matches mission area.
        self.dem = DEM(str(dem_path))
        self._log_dem_info()
        self._load_targets_from_targetinfo()
        # Debug: print initial spawn positions and target positions
        if self.state and self.state.initial_spawn_points:
            lines = []
            for idx, sp in enumerate(self.state.initial_spawn_points):
                lines.append(f"{idx+1}:{sp[0]:.1f},{sp[1]:.1f},{sp[2]:.1f}")
            print("[spawn] initial spawn points:", " | ".join(lines))
        if self.state and self.state.targets:
            lines = []
            for t in self.state.targets:
                lines.append(f"id={getattr(t,'id','?')} xy=({t.x:.1f},{t.y:.1f}) z={t.z:.1f}")
            print("[targets] current targets:", " | ".join(lines))
        self._apply_mission_reference_spawns()
        if self.targets_loaded_from_file:
            pass
        elif self.mission_reference_path:
            self._relocate_targets_near_spawn()
        else:
            self._ensure_spawn_above_dem()
        if self.enable_flight_paths:
            self._load_flight_paths()
        self._init_uav_autopilots()
        self._init_display()
        self._start_workers()
        self._start_logger()
        _log("entering main loop")

    def run(self):
        self.setup()
        try:
            while self.running:
                dt, dt_sim, dt_sim_step, time_scale_val = self._tick_time()
                self._sample_cpu()
                self._poll_worker_states()
                self._consume_fuel(dt_sim)
                keys = pygame.key.get_pressed()
                self._apply_continuous_input(keys, dt)
                self._process_events()
                snapshots, active_snap = self._capture_snapshots_and_trails()
                nearest = self._nearest_target(active_snap)
                self._update_targets_and_missiles(dt_sim)
                detection_lines = self._evaluate_threats(dt_sim)
                # Run autopilots over the full sim dt using fixed-size control steps to stay stable at high time_scale.
                self._update_autopilots(dt_sim)
                self._send_controls_to_workers(time_scale_val)
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
                self._log_target_detections(snapshots)
                uav_lon, uav_lat = self.dem.env_to_lonlat(active_snap[0], active_snap[1])
                self._render_frame(
                    snapshots, active_snap, nearest, detection_lines, dt_sim_step, time_scale_val, uav_lon, uav_lat
                )
        finally:
            self._shutdown()

    def _log_dem_info(self):
        assert self.dem is not None
        print(f"[DEM] tiles dir: {MAP_DIR}")
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
        self.state.workers = []
        self.state.proc_cmd_queues = []
        self.state.proc_state_queues = []
        self.state.proc_stop_events = []

        for i, u in enumerate(self.state.uavs):
            cmd_q = multiprocessing.Queue()
            state_q = multiprocessing.Queue()
            stop_evt = multiprocessing.Event()
            p = multiprocessing.Process(
                target=run_uav_process,
                args=(
                    i,
                    self.state.uav_types[i],
                    u.p,
                    self._uav_state_tuple(u),
                    str(MAP_DIR),
                    cmd_q,
                    state_q,
                    stop_evt,
                ),
                daemon=True,
            )
            p.start()
            self.state.workers.append(p)
            self.state.proc_cmd_queues.append(cmd_q)
            self.state.proc_state_queues.append(state_q)
            self.state.proc_stop_events.append(stop_evt)

        # Kick off with initial control + time_scale so workers start with the same settings.
        self._send_controls_to_workers(self.state.time_scale["value"])

    def _start_logger(self):
        log_dir = self.log_dir_0401
        log_dir.mkdir(parents=True, exist_ok=True)
        name_from_env = os.getenv("SIM_MISSION_NAME")
        if name_from_env:
            self.mission_name = name_from_env
        else:
            existing = sorted(log_dir.glob("*_0401.njson"))
            mission_idx = len(existing) + 1
            self.mission_name = f"임시{mission_idx}"
        self.log_file_path = log_dir / f"{self.mission_name}_0401.njson"
        self.log_dir_0402.mkdir(parents=True, exist_ok=True)
        self.log0402_file_path = self.log_dir_0402 / f"{self.mission_name}_0402.njson"
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

    def _camera_forward_vector(self, idx: int, uav_pos: np.ndarray) -> np.ndarray | None:
        filming_target = None
        if idx < len(self.uav_filming_target):
            filming_target = self.uav_filming_target[idx]
        if filming_target is None and self.state:
            filming_prop = self.uav_filming_props[idx] if idx < len(self.uav_filming_props) else None
            try:
                filming_target = self._compute_filming_target(idx, filming_prop)
            except Exception:
                filming_target = None
        if filming_target is None:
            return np.array([0.0, 0.0, -1.0], dtype=float)
        forward = np.array(filming_target, dtype=float) - uav_pos
        norm = float(np.linalg.norm(forward))
        if norm < 1e-6:
            return np.array([0.0, 0.0, -1.0], dtype=float)
        return forward / norm

    def _target_seen_by_any_uav(self, target, target_id: int, snapshots, timestamp: int | None) -> dict | None:
        if not (self.state and self.dem):
            return None
        tgt_vec = np.array([target.x, target.y, target.z], dtype=float)
        max_angle = float(self.state.fov_diag) * 0.5
        for idx, snap in enumerate(snapshots):
            if idx >= len(self.state.uavs):
                break
            if self.state.crashed[idx] or self.state.crippled[idx]:
                continue
            uav_pos = np.array(snap[:3], dtype=float)
            forward = self._camera_forward_vector(idx, uav_pos)
            if forward is None:
                continue
            to_tgt = tgt_vec - uav_pos
            norm_tgt = float(np.linalg.norm(to_tgt))
            if norm_tgt < 1e-6:
                continue
            cosang = float(np.dot(forward, to_tgt) / (np.linalg.norm(forward) * norm_tgt))
            cosang = max(-1.0, min(1.0, cosang))
            ang = math.degrees(math.acos(cosang))
            if ang > max_angle:
                continue
            if not check_los(uav_pos, tgt_vec, self.dem):
                continue
            lon, lat = self.dem.env_to_lonlat(target.x, target.y)
            coord = {"latitude": lat, "longitude": lon, "altitude": float(target.z)}
            fov_val = float(self.state.fov_diag)
            return {
                "timestamp": timestamp,
                "roiInfo": {"aircraftID": idx + 1, "coordinate": coord, "fov": fov_val},
                "targetList": [
                    {
                        "targetID": target_id,
                        "targetType": 1,
                        "coordinate": coord,
                        "watcher": {"aircraftID": idx + 1},
                        "targetInFrame": 1,
                        "isDestroyed": 0,
                        "threat": 100.0,
                    }
                ],
            }
        return None

    def _log_target_detections(self, snapshots):
        if not (self.state and self.state.targets and self.log0402_file_path and self.dem):
            return
        if not self.state.latest_agent_status_0401:
            return
        timestamp = self.state.latest_agent_status_0401.get("timestamp")
        if timestamp is None:
            return
        for tidx, tgt in enumerate(self.state.targets):
            target_id = getattr(tgt, "id", None) or tidx + 1
            if target_id in self.logged_target_ids_0402:
                continue
            record = self._target_seen_by_any_uav(tgt, target_id, snapshots, timestamp)
            if record is None:
                continue
            try:
                with self.log0402_file_path.open("a", encoding="utf-8") as f:
                    f.write(json.dumps(record, ensure_ascii=False) + "\n")
                self.logged_target_ids_0402.add(target_id)
            except Exception as e:
                print(f"[log0402] write failed: {e}")

    def _poll_worker_states(self):
        if not self.state or not self.state.proc_state_queues:
            return
        for idx, q in enumerate(self.state.proc_state_queues):
            try:
                while True:
                    msg = q.get_nowait()
                    state_tuple = msg.get("state")
                    if state_tuple:
                        with self.state.uav_locks[idx]:
                            (
                                self.state.uavs[idx].s.x,
                                self.state.uavs[idx].s.y,
                                self.state.uavs[idx].s.z,
                                self.state.uavs[idx].s.roll,
                                self.state.uavs[idx].s.pitch,
                                self.state.uavs[idx].s.yaw,
                                self.state.uavs[idx].s.u,
                                self.state.uavs[idx].s.p,
                                self.state.uavs[idx].s.q,
                                self.state.uavs[idx].s.r,
                            ) = state_tuple
                            self.state.pending_states[idx].append(state_tuple)
                    if "crippled" in msg:
                        self.state.crippled[idx] = msg["crippled"]
                    if "crashed" in msg:
                        self.state.crashed[idx] = msg["crashed"]
            except queue.Empty:
                continue

    def _send_controls_to_workers(self, time_scale_val: float):
        if not self.state or not self.state.proc_cmd_queues:
            return
        for i, u in enumerate(self.state.uavs):
            msg = {
                "type": "control",
                "cmds": {
                    "yaw_rate": u.cmd_yaw_rate,
                    "pitch_rate": u.cmd_pitch_rate,
                    "roll_rate": u.cmd_roll_rate,
                    "throttle": u.cmd_throttle,
                },
                "crippled": self.state.crippled[i],
                "crashed": self.state.crashed[i],
                "time_scale": time_scale_val,
            }
            self.state.proc_cmd_queues[i].put(msg)

    def _tick_time(self):
        ms = self.clock.tick(FPS)
        raw_dt = max(0.0001, min(0.05, ms / 1000.0))
        self.dt_smoothed = 0.85 * self.dt_smoothed + 0.15 * raw_dt
        dt = self.dt_smoothed
        with self.state.time_lock:
            time_scale_val = self.state.time_scale["value"]
        dt_sim = dt * time_scale_val
        # Rotor angle uses fixed step scaled for stable visuals (not tied to frame jitter).
        dt_sim_step = self.sim_step * time_scale_val
        # Spin rotor visuals; bumped multiplier for a faster-looking main rotor.
        self.rotor_angle = (self.rotor_angle + 1440.0 * dt_sim_step) % 360.0
        if self.debug_frames < 3:
            _log(f"frame start dt={dt:.4f} dt_sim={dt_sim:.4f} fuel_lah1={self.state.fuel_levels[0]:.1f}")
        return dt, dt_sim, dt_sim_step, time_scale_val

    def _consume_fuel(self, dt_sim):
        for i, _ in enumerate(self.state.uav_types):
            if not self.state.crashed[i]:
                self.state.fuel_levels[i] = max(0.0, self.state.fuel_levels[i] - LAH_FUEL_BURN_LPS * dt_sim)

    def _apply_continuous_input(self, keys, dt):
        active_idx = self.state.active_idx
        uav = self.state.uavs[active_idx]
        is_lah = self.state.uav_types[active_idx] == "LAH"
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
            self._reset_all_craft()
        elif key == pygame.K_f:
            self.state.fog_enabled = not self.state.fog_enabled
        elif key == pygame.K_n:
            self.state.targets_move = not self.state.targets_move
        elif key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            with self.state.time_lock:
                self.state.time_scale["value"] = clamp(self.state.time_scale["value"] + 1.0, 0.1, 100.0)
            ts = self.state.time_scale["value"]
            print(f"[time] scale -> {ts:.1f}x (dt~{self.dt_smoothed:.4f}s, dt_sim~{self.dt_smoothed * ts:.4f}s)")
        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            with self.state.time_lock:
                self.state.time_scale["value"] = clamp(self.state.time_scale["value"] - 1.0, 0.1, 100.0)
            ts = self.state.time_scale["value"]
            print(f"[time] scale -> {ts:.1f}x (dt~{self.dt_smoothed:.4f}s, dt_sim~{self.dt_smoothed * ts:.4f}s)")
        elif key == pygame.K_g:
            self._spawn_target_near_camera()
        elif key == pygame.K_m:
            self._fire_missile()
        elif key == pygame.K_o:
            self.state.threat_kill_disabled = True
            print("[threat] attacks disabled (kill switch 'O').")
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

    def _reset_all_craft(self, ground_clearance_m: float | None = None):
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
                reset_msg = {
                    "type": "reset",
                    "state": self._uav_state_tuple(u),
                    "time_scale": self.state.time_scale["value"],
                }
                if self.state.proc_cmd_queues:
                    self.state.proc_cmd_queues[i].put(reset_msg)

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
            # Incorporate any buffered states to avoid skipping points at high time-scale.
            pending = self.state.pending_states[i]
            if pending:
                trail = self.state.trails[i]
                for state_tuple in pending:
                    sx, sy, sz, sroll, spitch, syaw, su, sp, sq, sr = state_tuple
                    if trail:
                        prev = trail[-1]
                        if math.dist((sx, sy, sz), prev) >= 5.0:
                            trail.append((sx, sy, sz))
                    else:
                        trail.append((sx, sy, sz))
                self.state.pending_states[i] = []
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
        candidates = []
        for t in self.state.targets:
            d2 = (t.x - active_snap[0]) ** 2 + (t.y - active_snap[1]) ** 2 + (t.z - active_snap[2]) ** 2
            if d2 <= MAX_TARGET_RANGE_M**2:
                candidates.append((d2, t))
        if not candidates:
            return None
        return min(candidates, key=lambda p: p[0])[1]

    def _compute_filming_target(self, idx: int, filming_prop: dict | None):
        """Legacy single-shot compute (fallback only)."""
        return self._default_downward_target(self.state.uavs[idx])

    def _default_downward_target(self, uav):
        # look straight down relative to aircraft (pitch -90)
        dir_vec = np.array([0.0, 0.0, -1.0], dtype=float)
        origin = np.array([uav.s.x, uav.s.y, uav.s.z], dtype=float)
        hit = ray_intersect_dem(origin, dir_vec, self.dem)
        if hit is not None:
            return tuple(hit.tolist())
        return tuple((origin + dir_vec * 1000.0).tolist())

    def _load_gain_db(self, path: Path) -> list[dict] | None:
        key = str(path.resolve())
        if key in self.pid_db_cache:
            return self.pid_db_cache[key]
        # 1) Primary: aggregated DB file
        if path.exists():
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                records = data.get("records") if isinstance(data, dict) else None
                if isinstance(records, list):
                    self.pid_db_cache[key] = records
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
            self.pid_db_cache[key] = records
            print(f"[pid-db] built ad-hoc records from pattern {pattern} ({len(records)} entries)")
            return records
        return None

    def _pick_gains_for_scale(self, db_path: Path, fallback_path: Path, time_scale: float) -> PIDGains:
        records = self._load_gain_db(db_path)
        if records:
            best = None
            best_diff = float("inf")
            for r in records:
                ts = r.get("time_scale")
                g = r.get("gains")
                if ts is None or not isinstance(g, dict):
                    continue
                diff = abs(float(ts) - float(time_scale))
                if diff < best_diff:
                    best_diff = diff
                    best = g
            if best:
                try:
                    return PIDGains(**best)
                except Exception as e:
                    print(f"[pid-db] failed to build gains from record: {e}")
        # fallback: flat gains file
        g = load_pid_gains(fallback_path)
        return g if g is not None else DEFAULT_TUNED_GAINS

    def _convert_line_search_points(self, line_search: dict) -> list[tuple[float, float, float]]:
        coords = line_search.get("coordinateList") or []
        pts: list[tuple[float, float, float]] = []
        for c in coords:
            lon = c.get("longitude")
            lat = c.get("latitude")
            alt = c.get("altitude", 0.0)
            if lon is None or lat is None:
                continue
            try:
                x, y = self.dem.lonlat_to_env(lon, lat)
                z = self.dem.get_height(x, y) if alt == 0 else alt
                pts.append((float(x), float(y), float(z)))
            except Exception:
                continue
        return pts

    def _build_line_search_segments(self, pts: list[tuple[float, float, float]]) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
        """Build line segments from point list in pairs (p0,p1), ignoring lone tail."""
        segs = []
        for i in range(0, len(pts) - 1, 2):
            p0 = pts[i]
            p1 = pts[i + 1]
            segs.append((p0, p1))
        return segs

    def _update_filming_target(self, idx: int, tgt: WaypointTarget | None, dt: float):
        """Update filming target over time (supports mode 1/2/4)."""
        uav = self.state.uavs[idx]
        filming_prop = tgt.filming if tgt else None
        current_wp_id = tgt.wp_id if tgt else None

        if filming_prop is None:
            self.uav_line_search_state[idx] = None
            self.uav_filming_props[idx] = None
            self.uav_filming_target[idx] = self._default_downward_target(uav)
            return

        mode = filming_prop.get("operationMode")
        if mode == 4:
            aircraft_fixed = filming_prop.get("aircraftFixed") or {}
            pitch_deg = float(aircraft_fixed.get("gimbalPitch", 0.0))
            yaw_offset_deg = float(aircraft_fixed.get("gimbalYaw", 0.0))
            yaw_deg = uav.s.yaw + yaw_offset_deg
            yaw_rad = math.radians(yaw_deg)
            pitch_rad = math.radians(pitch_deg)
            cp = math.cos(pitch_rad)
            dir_vec = np.array(
                [cp * math.cos(yaw_rad), -cp * math.sin(yaw_rad), math.sin(pitch_rad)],
                dtype=float,
            )
            origin = np.array([uav.s.x, uav.s.y, uav.s.z], dtype=float)
            hit = ray_intersect_dem(origin, dir_vec, self.dem)
            if hit is not None:
                self.uav_filming_target[idx] = tuple(hit.tolist())
            else:
                self.uav_filming_target[idx] = tuple((origin + dir_vec * 2000.0).tolist())
            self.uav_line_search_state[idx] = None
            return

        if mode == 1:
            coord_orient = filming_prop.get("coordinateOrientation") or {}
            coord = coord_orient.get("coordinate") or {}
            lon = coord.get("longitude")
            lat = coord.get("latitude")
            alt = coord.get("altitude", 0.0)
            if lon is None or lat is None:
                self.uav_filming_target[idx] = self._default_downward_target(uav)
            else:
                try:
                    x, y = self.dem.lonlat_to_env(lon, lat)
                    z = self.dem.get_height(x, y) if alt == 0 else alt
                    self.uav_filming_target[idx] = (x, y, z)
                except Exception:
                    self.uav_filming_target[idx] = self._default_downward_target(uav)
            self.uav_line_search_state[idx] = None
            return

        if mode == 2:
            state = self.uav_line_search_state[idx]
            line_search = filming_prop.get("lineSearch") or {}
            if (
                state is None
                or state.get("wp_id") != current_wp_id
                or state.get("filming_id") != id(filming_prop)
            ):
                pts = self._convert_line_search_points(line_search)
                segs = self._build_line_search_segments(pts)
                if not segs:
                    self.uav_filming_target[idx] = self._default_downward_target(uav)
                    self.uav_line_search_state[idx] = None
                    self.uav_line_search_debug[idx] = None
                    return
                state = {
                    "segments": segs,
                    "seg_idx": 0,
                    "seg_t": 0.0,
                    "speed": float(line_search.get("searchSpeed", 0.0)),
                    "wp_id": current_wp_id,
                    "filming_id": id(filming_prop),
                }
                self.uav_line_search_debug[idx] = pts

            segs = state["segments"]
            seg_idx = state.get("seg_idx", 0)
            seg_t = state.get("seg_t", 0.0)
            speed = max(0.0, float(state.get("speed", 0.0)))
            dist_left = speed * max(0.0, dt)

            while dist_left > 0.0 and seg_idx < len(segs):
                p0, p1 = segs[seg_idx]
                seg_len = math.dist(p0, p1)
                if seg_len < 1e-6:
                    seg_idx += 1
                    seg_t = 0.0
                    continue
                advance_t = dist_left / seg_len
                new_t = seg_t + advance_t
                if new_t >= 1.0:
                    dist_left = (new_t - 1.0) * seg_len
                    seg_idx += 1
                    seg_t = 0.0
                else:
                    seg_t = new_t
                    dist_left = 0.0

            if seg_idx >= len(segs):
                seg_idx = len(segs) - 1
                seg_t = 1.0

            p0, p1 = segs[seg_idx]
            target = (
                p0[0] + (p1[0] - p0[0]) * seg_t,
                p0[1] + (p1[1] - p0[1]) * seg_t,
                p0[2] + (p1[2] - p0[2]) * seg_t,
            )
            self.uav_filming_target[idx] = target
            state["seg_idx"] = seg_idx
            state["seg_t"] = seg_t
            self.uav_line_search_state[idx] = state
            return

        # Other modes or unknown: default down
        self.uav_line_search_state[idx] = None
        self.uav_filming_target[idx] = self._default_downward_target(uav)

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
        if self.state.threat_kill_disabled:
            # Clear any lingering crippled flags while kills are disabled.
            for i in range(len(self.state.crippled)):
                self.state.crippled[i] = False
        for idx, u in enumerate(self.state.uavs):
            with self.state.uav_locks[idx]:
                upos = np.array([u.s.x, u.s.y, u.s.z], dtype=float)
            for t in self.state.targets:
                # Skip far targets to avoid heavy LOS/DEM work.
                if np.linalg.norm(np.array([t.x - upos[0], t.y - upos[1], t.z - upos[2]])) > MAX_TARGET_RANGE_M:
                    continue
                if not getattr(t, "threat", None):
                    continue
                # If threat lethality is disabled, skip kill logic entirely.
                if self.state.threat_kill_disabled or getattr(getattr(t.threat, "weapon", None), "omega", 1.0) == 0.0:
                    # Still track detection for HUD lines but never apply damage.
                    los = check_los(upos, np.array([t.x, t.y, t.z]), self.dem)
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

        footprint_corners = None

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

        self._draw_flight_paths()

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
            # Overlay bright point for visibility
            glPointSize(8.0)
            glColor3f(1.0, 0.1, 0.1)
            glBegin(GL_POINTS)
            glVertex3f(t.x, t.y, t.z)
            glEnd()
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

        model = glGetDoublev(GL_MODELVIEW_MATRIX)
        proj = glGetDoublev(GL_PROJECTION_MATRIX)
        viewport = glGetIntegerv(GL_VIEWPORT)
        label_entries = []
        path_labels = []
        mode_labels = {0: "없음", 1: "좌표 지향", 2: "구간탐색", 3: "자동추적", 4: "기체고정", 5: "자동주사"}
        for idx, snap in enumerate(snapshots):
            sx, sy, sz = gluProject(snap[0], snap[1], snap[2] + 10.0, model, proj, viewport)
            if 0.0 <= sz <= 1.0:
                color = (255, 80, 80) if idx == self.state.active_idx else (220, 220, 220)
                name = f"LAH{idx + 1}" if idx < 3 else f"UAV{idx - 2}"
                speed_text = f"{snap[6]:.0f}m/s"
                state_tag = "FALL" if self.state.crippled[idx] else ("CRASH" if self.state.crashed[idx] else "")
                text = f"{name} {speed_text}" if not state_tag else f"{name} {speed_text} {state_tag}"
                label_entries.append((sx, sy + 8, text, color))
                wp_id = self.uav_current_wp_ids[idx] if idx < len(self.uav_current_wp_ids) else None
                filming = self.uav_filming_props[idx] if idx < len(self.uav_filming_props) else None
                mode = filming.get("operationMode") if isinstance(filming, dict) else None
                mode_text = mode_labels.get(mode, "없음") if mode is not None else "없음"
                sub_color = (255, 220, 180) if idx == self.state.active_idx else (180, 180, 180)
                label_entries.append((sx, sy - 12, f"WP {wp_id if wp_id is not None else '-'} | 촬영 모드: {mode_text}", sub_color))
        # Flight path point labels (visible subset only)
        for info in getattr(self, "_flight_path_labels", []):
            px, py, pz = gluProject(info["pos"][0], info["pos"][1], info["pos"][2] + 5.0, model, proj, viewport)
            if 0.0 <= pz <= 1.0:
                path_labels.append((px, py, info["text"], (255, 215, 0)))
        label_entries.extend(path_labels)
        for tidx, t in enumerate(self.state.targets):
            sx, sy, sz = gluProject(t.x, t.y, t.z + 10.0, model, proj, viewport)
            if 0.0 <= sz <= 1.0:
                label_entries.append((sx, sy + 8, f"target{tidx + 1}", (255, 200, 80)))
        draw_labels(label_entries)

        footprint_area = None
        los_clear = False
        if not is_lah:
            filming_prop = None
            if self.uav_filming_props and self.state.active_idx < len(self.uav_filming_props):
                filming_prop = self.uav_filming_props[self.state.active_idx]
            filming_target = None
            if self.uav_filming_target and self.state.active_idx < len(self.uav_filming_target):
                filming_target = self.uav_filming_target[self.state.active_idx]
            if filming_target is None:
                filming_target = self._compute_filming_target(self.state.active_idx, filming_prop)

            tgt_pos = None
            if filming_target is not None:
                tgt_pos = np.array(filming_target, dtype=float)
            elif nearest is not None:
                tgt_pos = np.array([nearest.x, nearest.y, nearest.z], dtype=float)

            # Draw line-search polyline debug (always, if exists).
            if (
                self.uav_line_search_debug
                and self.state.active_idx < len(self.uav_line_search_debug)
                and self.uav_line_search_debug[self.state.active_idx]
            ):
                draw_polyline(
                    self.uav_line_search_debug[self.state.active_idx],
                    color=(0.4, 1.0, 0.4),
                    width=2.0,
                    stipple=True,
                )

            if tgt_pos is not None:
                uav_pos = np.array([active_snap[0], active_snap[1], active_snap[2]], dtype=float)
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
                    footprint_area, footprint_corners = draw_camera_footprint(
                        self.state.uavs[self.state.active_idx],
                        tgt_pos,
                        fov_diag_deg=self.state.fov_diag,
                        dem=self.dem,
                        aspect_ratio=16 / 9,
                        z_scale=1.0,
                    )

        lines = [
            f"Active: {'LAH' if is_lah else 'UAV'}{self.state.active_idx + 1}  pos (m): x={active_snap[0]:7.1f}  y={active_snap[1]:7.1f}  z={active_snap[2]:6.1f}",
            f"pos (lat/lon): lat={uav_lat:9.5f}  lon={uav_lon:10.5f}",
            f"spd (m/s): {active_snap[6]:5.1f}   yaw={active_snap[5]:6.1f}deg  pitch={active_snap[4]:5.1f}deg  roll={active_snap[3]:5.1f}deg",
            f"FOV (diag): {self.state.fov_diag:4.1f}deg   LOS: {1 if los_clear else 0}   dt(step)={dt_sim*1000:.1f}ms  time x{time_scale_val:.1f}",
            f"Targets: {len(self.state.targets)}  Missiles: {len(self.state.missiles)}",
            "1-6: switch craft (1-3 LAH, 4-6 UAV) | M: Fire (LAH only) | N: Toggle targets | G: Spawn target | Wheel: Zoom | RMB drag: Orbit | MMB drag: Pan",
        ]
        if self.cpu_percent is not None:
            # Show up to first 8 cores to keep HUD compact.
            loads = " ".join(f"{p:3.0f}%" for p in self.cpu_percent[:8])
            lines.append(f"CPU cores: {self.cpu_count}  workers: {len(self.state.workers)}  load: {loads}")
        if footprint_area is not None:
            lines.append(f"Footprint area: {footprint_area:8.1f} m^2")
        if any(self.state.crippled) and not any(self.state.crashed):
            crippled_ids = [str(i + 1) for i, c in enumerate(self.state.crippled) if c]
            lines.append(f"Status: DAMAGED UAVs: {', '.join(crippled_ids)} (falling)")
        if any(self.state.crashed):
            crashed_ids = [str(i + 1) for i, c in enumerate(self.state.crashed) if c]
            lines.append(f"Status: CRASHED craft: {', '.join(crashed_ids)} (press R to reset)")
        draw_hud(lines)
        self._draw_minimap(snapshots, active_snap)

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
            for evt in getattr(self.state, "proc_stop_events", []):
                evt.set()
            for t in self.state.workers:
                t.join(timeout=1.0)
        if self.log_stop_event:
            self.log_stop_event.set()
        if self.log_thread:
            self.log_thread.join(timeout=0.5)
        pygame.quit()

    def _apply_mission_reference_spawns(self):
        """Use mission reference info to place aircraft at takeoff/handover points."""
        if not self.mission_reference_path:
            return
        if not self.mission_reference_path.exists():
            print(f"[mission-ref] file not found: {self.mission_reference_path}")
            return
        try:
            data = json.loads(self.mission_reference_path.read_text(encoding="utf-8"))
        except Exception as e:
            print(f"[mission-ref] failed to load {self.mission_reference_path}: {e}")
            return
        take_over_map = {item.get("aircraftID"): item.get("coordinate") for item in data.get("takeOverInfoList", [])}
        rtb_list = data.get("rtbCoordinateList", [])

        def _env_from_coord(coord):
            lon = coord.get("longitude")
            lat = coord.get("latitude")
            alt = coord.get("altitude", 0.0)
            x, y = self.dem.lonlat_to_env(lon, lat)
            # Ensure tile switches if needed (propagate if out of bounds)
            self.dem.ensure_tile_for_env(x, y)
            ground = self.dem.get_height(x, y)
            return x, y, alt, ground

        # First pass: compute env coords for each craft; find anchor from first non-LAH craft.
        env_coords = []
        anchor_spawn = None
        for idx, uav_type in enumerate(self.state.uav_types):
            aircraft_id = idx + 1
            coord = take_over_map.get(aircraft_id)
            if coord is None and rtb_list:
                coord = rtb_list[min(idx, len(rtb_list) - 1)]
            if coord is None:
                env_coords.append(self.state.initial_spawn_points[idx])
                continue
            x, y, alt, ground = _env_from_coord(coord)
            env_coords.append((x, y, alt, ground))
            if anchor_spawn is None and uav_type != "LAH":
                anchor_spawn = (x, y, ground)

        # Fallback anchor: use first craft (even LAH) if no non-LAH found.
        if anchor_spawn is None and env_coords:
            ex, ey, ealt, eg = env_coords[0]
            anchor_spawn = (ex, ey, eg)

        new_spawns = []
        for idx, uav in enumerate(self.state.uavs):
            uav_type = self.state.uav_types[idx]
            if idx >= len(env_coords):
                new_spawns.append(self.state.initial_spawn_points[idx])
                continue
            x, y, alt, ground = env_coords[idx]
            if uav_type == "LAH" and anchor_spawn is not None:
                ax, ay, ag = anchor_spawn
                x = ax + random.uniform(-200.0, 200.0)
                y = ay + random.uniform(-200.0, 200.0)
                ground = self.dem.get_height(x, y)
                z = ground + 100.0
            else:
                z = alt if alt > 0 else ground + 50.0
            uav.s.x, uav.s.y, uav.s.z = x, y, z
            new_spawns.append((x, y, z))

        # Update stored spawn points so reset() keeps them.
        self.state.initial_spawn_points = new_spawns

    def _relocate_targets_near_spawn(self):
        """Place targets near first spawn point to avoid distant DEM/LOS churn."""
        if not self.state.targets or not self.state.initial_spawn_points:
            return
        if self.targets_loaded_from_file:
            return
        cx, cy, cz = self.state.initial_spawn_points[0]
        for t in self.state.targets:
            ox = random.uniform(-200.0, 200.0)
            oy = random.uniform(-200.0, 200.0)
            tx = cx + ox
            ty = cy + oy
            tz = self.dem.get_height(tx, ty) + 50.0
            t.x = tx
            t.y = ty
            t.z = tz
            if hasattr(t, "roam_center"):
                t.roam_center = (tx, ty)

    def _draw_flight_paths(self):
        """Draw flight path waypoints/segments; highlight within render radius."""
        if not getattr(self.state, "flight_paths", None):
            return
        cam_x, cam_y = self.cam.target[0], self.cam.target[1]
        self._flight_path_labels = []
        glLineWidth(1.0)
        glEnable(GL_LINE_STIPPLE)
        glLineStipple(1, 0x0F0F)
        for idx, plist in enumerate(self.state.flight_paths):
            if not plist:
                continue
            # Only keep points within render radius to reduce clutter.
            visible = []
            for wp in plist:
                x, y, z = wp["pos"]
                if math.hypot(x - cam_x, y - cam_y) <= RENDER_RADIUS_M:
                    visible.append(wp)
            if len(visible) < 2:
                continue
            # Base grey line
            glColor3f(0.5, 0.5, 0.5)
            glBegin(GL_LINE_STRIP)
            for wp in visible:
                glVertex3f(*wp["pos"])
            glEnd()
            # Highlight close-in segments (half radius) in yellow
            glColor3f(1.0, 0.9, 0.2)
            glBegin(GL_LINE_STRIP)
            for wp in visible:
                if math.hypot(wp["pos"][0] - cam_x, wp["pos"][1] - cam_y) <= RENDER_RADIUS_M * 0.6:
                    glVertex3f(*wp["pos"])
            glEnd()
            # Points
            glPointSize(4.0)
            glBegin(GL_POINTS)
            glColor3f(1.0, 0.9, 0.2)
            for wp in visible:
                glVertex3f(*wp["pos"])
                self._flight_path_labels.append(
                    {
                        "pos": wp["pos"],
                        "text": f"{'LAH' if idx < 3 else 'UAV'}{idx+1} - WP{wp.get('wp_id')}",
                    }
                )
            glEnd()
        glDisable(GL_LINE_STIPPLE)

    def _ensure_spawn_above_dem(self):
        """When no mission reference is provided, place spawns above terrain."""
        if not self.state or not self.state.initial_spawn_points:
            return
        new_spawns = []
        for idx, uav in enumerate(self.state.uavs):
            x, y, z = self.state.initial_spawn_points[idx]
            ground = self.dem.get_height(x, y)
            # LAH higher buffer, UAV lower buffer
            if self.state.uav_types[idx] == "LAH":
                z = max(z, ground + 120.0)
            else:
                z = max(z, ground + 80.0)
            uav.s.x, uav.s.y, uav.s.z = x, y, z
            new_spawns.append((x, y, z))
        self.state.initial_spawn_points = new_spawns

    def _draw_minimap(self, snapshots, active_snap):
        """Draw a small top-down minimap; center/scale to include all craft/paths/targets."""
        points: list[tuple[float, float]] = []
        if snapshots:
            points.extend((snap[0], snap[1]) for snap in snapshots)
        if self.state.targets:
            points.extend((t.x, t.y) for t in self.state.targets)
        if getattr(self.state, "flight_paths", None):
            for plist in self.state.flight_paths:
                for wp in plist:
                    try:
                        x, y, _ = wp.get("pos") if isinstance(wp, dict) else wp
                    except Exception:
                        continue
                    points.append((x, y))

        if points:
            xs, ys = zip(*points)
            cx = 0.5 * (min(xs) + max(xs))
            cy = 0.5 * (min(ys) + max(ys))
            span = max(max(xs) - min(xs), max(ys) - min(ys))
            map_half = clamp(max(span * 0.65, 5000.0), 5000.0, 80000.0)
        else:
            cx, cy = active_snap[0], active_snap[1]
            map_half = 5000.0
        width = 220
        height = 220
        margin = 12
        # Save viewport/projection/modelview
        glPushMatrix()
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(-map_half, map_half, -map_half, map_half, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        glViewport(WIN_W - width - margin, WIN_H - height - margin, width, height)
        glDisable(GL_DEPTH_TEST)

        # Background
        glBegin(GL_QUADS)
        glColor4f(0.05, 0.05, 0.05, 0.8)
        glVertex3f(-map_half, -map_half, 0)
        glVertex3f(map_half, -map_half, 0)
        glVertex3f(map_half, map_half, 0)
        glVertex3f(-map_half, map_half, 0)
        glEnd()

        # Grid cross
        glColor3f(0.2, 0.2, 0.2)
        glLineWidth(1.0)
        glBegin(GL_LINES)
        glVertex3f(-map_half, 0, 0)
        glVertex3f(map_half, 0, 0)
        glVertex3f(0, -map_half, 0)
        glVertex3f(0, map_half, 0)
        glEnd()

        # Flight path points (mission WPs): show all, highlight active (yellow)
        if getattr(self.state, "flight_paths", None):
            active_idx = self.state.active_idx
            for idx, plist in enumerate(self.state.flight_paths):
                if not plist:
                    continue
                base_color = (1.0, 1.0, 0.2)
                if idx == active_idx:
                    base_color = (1.0, 1.0, 0.4)
                glLineWidth(1.0)
                glEnable(GL_LINE_STIPPLE)
                glLineStipple(1, 0x1111)
                glColor3f(*base_color)
                glBegin(GL_LINE_STRIP)
                for wp in plist:
                    x, y, z = wp["pos"]
                    dx, dy = x - cx, y - cy
                    if abs(dx) <= map_half and abs(dy) <= map_half:
                        glVertex3f(dx, dy, 0)
                glEnd()
                glDisable(GL_LINE_STIPPLE)
                glPointSize(5.0 if idx == active_idx else 4.0)
                glColor3f(*base_color)
                glBegin(GL_POINTS)
                for wp in plist:
                    x, y, z = wp["pos"]
                    dx, dy = x - cx, y - cy
                    if abs(dx) <= map_half and abs(dy) <= map_half:
                        glVertex3f(dx, dy, 0)
                glEnd()

        # Targets
        if self.state.targets:
            glPointSize(7.0)
            glBegin(GL_POINTS)
            for t_idx, t in enumerate(self.state.targets):
                dx, dy = t.x - cx, t.y - cy
                if abs(dx) <= map_half and abs(dy) <= map_half:
                    glColor3f(1.0, 0.0, 0.0)  # red for targets
                    glVertex3f(dx, dy, 0)
                    # small square marker for visibility
                    glColor3f(1.0, 0.5, 0.1)
                    s = 40.0
                    glVertex3f(dx + s, dy, 0)
                    glVertex3f(dx - s, dy, 0)
                    glVertex3f(dx, dy + s, 0)
                    glVertex3f(dx, dy - s, 0)
            glEnd()

        # Active UAV position (relative to minimap center)
        glPointSize(6.0)
        glColor3f(1.0, 1.0, 1.0)
        glBegin(GL_POINTS)
        glVertex3f(active_snap[0] - cx, active_snap[1] - cy, 0.0)
        glEnd()

        # All UAV positions
        if snapshots:
            glPointSize(6.0)
            glBegin(GL_POINTS)
            for idx, snap in enumerate(snapshots):
                dx, dy = snap[0] - cx, snap[1] - cy
                if abs(dx) > map_half or abs(dy) > map_half:
                    continue
                is_lah = idx < 3
                color = (0.1, 0.6, 1.0) if is_lah else (0.1, 1.0, 0.4)  # LAH blue, UAV green
                if idx == self.state.active_idx:
                    color = tuple(min(1.0, c + 0.3) for c in color)
                glColor3f(*color)
                glVertex3f(dx, dy, 0)
            glEnd()

        glEnable(GL_DEPTH_TEST)
        # Restore matrices/viewport
        glPopMatrix()  # modelview
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()
        glViewport(0, 0, WIN_W, WIN_H)

    def _load_flight_paths(self):
        """Load flight path waypoints from log EP folder and map to each aircraft."""
        if not self.flight_path_root.exists():
            print(f"[flight-path] directory not found: {self.flight_path_root}")
            return
        paths_per_aircraft: list[list[dict]] = [[] for _ in self.state.uavs]

        def craft_idx_from_path_id(pid: int) -> int | None:
            prefix = int(str(pid)[0])
            mapping = {1: 0, 2: 1, 3: 2, 4: 3, 5: 4, 6: 5}
            return mapping.get(prefix)

        counts = [0 for _ in self.state.uavs]
        for fp in sorted(self.flight_path_root.glob("*.json")):
            try:
                data = json.loads(fp.read_text(encoding="utf-8"))
            except Exception as e:
                print(f"[flight-path] failed to load {fp}: {e}")
                continue
            pid = data.get("pathID")
            if pid is None:
                continue
            idx = craft_idx_from_path_id(int(pid))
            if idx is None or idx >= len(paths_per_aircraft):
                continue
            # Allow both waypointList (UAV) and lahWaypointList (LAH)
            wps = data.get("waypointList", []) or data.get("lahWaypointList", [])
            for wp in wps:
                coord = wp.get("coordinate") or {}
                lon = coord.get("longitude")
                lat = coord.get("latitude")
                alt = coord.get("altitude", 0.0)
                speed = wp.get("speed")
                hover = (wp.get("hovering") or {}).get("time")
                if lon is None or lat is None:
                    continue
                try:
                    x, y = self.dem.lonlat_to_env(lon, lat)
                    z = self.dem.get_height(x, y) if alt == 0 else alt
                except Exception as e:
                    print(f"[flight-path] coord out of DEM bounds for {fp}: {e}")
                    continue
                paths_per_aircraft[idx].append(
                    {
                        "pos": (x, y, z),
                        "wp_id": wp.get("waypointID"),
                        "path_id": pid,
                        "speed": speed,
                        "filming": wp.get("filmingProperty"),
                        "hover_time": hover,
                    }
                )
                counts[idx] += 1
        self.state.flight_paths = paths_per_aircraft
        for idx, cnt in enumerate(counts):
            if cnt > 0:
                name = "LAH" if idx < 3 else "UAV"
                print(f"[flight-path] loaded {cnt} waypoints for {name}{idx+1}")

    def _init_uav_autopilots(self):
        """Build PID waypoint followers for UAVs if flight paths are available."""
        if not self.state:
            return

        self.uav_autopilots = [None for _ in self.state.uavs]
        self.uav_filming_props = [None for _ in self.state.uavs]
        self.uav_current_wp_ids = [None for _ in self.state.uavs]
        self.uav_line_search_state = [None for _ in self.state.uavs]
        self.uav_filming_target = [None for _ in self.state.uavs]
        self.uav_line_search_debug = [None for _ in self.state.uavs]
        if not (self.enable_uav_autopilot and self.enable_flight_paths):
            return
        if not getattr(self.state, "flight_paths", None):
            return

        time_scale = self.state.time_scale["value"] if self.state else 1.0
        for idx, plist in enumerate(self.state.flight_paths):
            if not plist:
                continue
            targets: list[WaypointTarget] = []
            for wp in plist:
                pos = None
                speed = None
                filming = None
                wp_id = None
                hover_time = None
                if isinstance(wp, dict):
                    pos = wp.get("pos")
                    speed = wp.get("speed")
                    filming = wp.get("filming")
                    wp_id = wp.get("wp_id")
                    hover_time = wp.get("hover_time")
                elif isinstance(wp, (list, tuple)) and len(wp) == 3:
                    pos = wp
                if pos is None:
                    continue
                try:
                    px, py, pz = pos
                except Exception:
                    continue
                targets.append(
                    WaypointTarget(
                        pos=(float(px), float(py), float(pz)),
                        speed=speed,
                        filming=filming,
                        wp_id=int(wp_id) if wp_id is not None else None,
                        hover_time=float(hover_time) if hover_time is not None else None,
                    )
                )

            if targets:
                is_uav = self.state.uav_types[idx] == "UAV"
                gains_path = self.pid_db_path_uav if is_uav else self.pid_db_path_lah
                fallback_path = self.pid_gains_path if is_uav else self.pid_gains_path_lah
                gains = self._pick_gains_for_scale(gains_path, fallback_path, time_scale)
                self.uav_autopilots[idx] = WaypointPIDController(
                    self.state.uavs[idx],
                    targets,
                    gains=gains,
                    speed_target=90.0 if is_uav else 60.0,
                    pos_tol=30.0,
                    name=("UAV" if is_uav else "LAH") + f"{idx + 1}",
                    allow_hover=not is_uav,
                )
                self.uav_filming_props[idx] = targets[0].filming if targets[0].filming else None
                self.uav_current_wp_ids[idx] = int(targets[0].wp_id) if targets[0].wp_id is not None else None
                print(f"[pid-autopilot] armed for {('UAV' if is_uav else 'LAH')}{idx + 1} with {len(targets)} waypoints.")

    def _update_autopilots(self, dt_sim: float):
        """
        Advance UAV PID autopilots over the current simulation dt by sub-stepping
        with a fixed control step (self.sim_step) so high time_scale does not
        explode the controller.
        """
        if not self.state or not self.uav_autopilots or not self.enable_uav_autopilot:
            return
        time_scale = self.state.time_scale["value"] if self.state else 1.0
        ctrl_step = float(self.sim_step)
        for idx, ap in enumerate(self.uav_autopilots):
            if ap is None or self.state.crashed[idx]:
                continue
            is_uav = self.state.uav_types[idx] == "UAV"
            gains_path = self.pid_db_path_uav if is_uav else self.pid_db_path_lah
            fallback_path = self.pid_gains_path if is_uav else self.pid_gains_path_lah
            ap.gains = self._pick_gains_for_scale(gains_path, fallback_path, time_scale)
            pending = self.state.pending_states[idx]
            if pending:
                # Use buffered state samples (arrive at sim_step spacing) to keep control in sync.
                for state_tuple in pending:
                    (
                        self.state.uavs[idx].s.x,
                        self.state.uavs[idx].s.y,
                        self.state.uavs[idx].s.z,
                        self.state.uavs[idx].s.roll,
                        self.state.uavs[idx].s.pitch,
                        self.state.uavs[idx].s.yaw,
                        self.state.uavs[idx].s.u,
                        self.state.uavs[idx].s.p,
                        self.state.uavs[idx].s.q,
                        self.state.uavs[idx].s.r,
                    ) = state_tuple
                    ap.update(ctrl_step, dem=self.dem)
                    self._update_filming_target(idx, ap.current_target(), ctrl_step)
            else:
                # Fallback: advance over the reported dt_sim in fixed steps.
                remaining = max(0.0, float(dt_sim))
                while remaining > 0.0:
                    step = ctrl_step if remaining >= ctrl_step else remaining
                    ap.update(step, dem=self.dem)
                    self._update_filming_target(idx, ap.current_target(), step)
                    remaining -= step

            # Track current filming property for this UAV (current target of autopilot).
            tgt = ap.current_target()
            self.uav_filming_props[idx] = tgt.filming if tgt else None
            self.uav_current_wp_ids[idx] = int(tgt.wp_id) if (tgt and tgt.wp_id is not None) else None
            if tgt and tgt.filming:
                fov = tgt.filming.get("fieldOfView")
                if isinstance(fov, (int, float)):
                    self.state.fov_diag = float(fov)

    def _sample_cpu(self):
        if psutil is None:
            return
        now = time.time()
        if now - self.cpu_last_sample >= 0.5:
            self.cpu_percent = psutil.cpu_percent(percpu=True)
            self.cpu_last_sample = now
