import json
import os
import queue
import threading
import time
import multiprocessing
from multiprocessing import shared_memory
from pathlib import Path

import numpy as np
import pygame
try:
    import psutil
except ImportError:
    psutil = None

from sim.agent_status import build_agent_status_snapshot
from sim.config import DEM_FILE, MAP_DIR, REPO_ROOT, WIN_H, WIN_W
from sim.core.camera import OrbitCamera
from sim.world.dem import DEM
from sim.runtime.controllers import WaypointPIDController, WaypointTarget
from .autopilot_loop import get_operation_mode_handler, init_uav_autopilots, update_autopilots
from .mission_setup import (
    apply_mission_reference_spawns,
    ensure_spawn_above_dem,
    load_flight_paths,
    log_dem_info,
    relocate_targets_near_spawn,
)
from .operation_mode import OperationContext, OperationMode
from .render_helpers import init_display, render_frame
from .runtime_utils import (
    capture_snapshots_and_trails,
    consume_fuel,
    default_downward_target,
    evaluate_threats,
    nearest_target,
    remove_destroyed_targets,
    reset_all_craft,
    sample_cpu,
    tick_time,
    update_targets_and_missiles,
)
from .input_handlers import apply_continuous_input, process_events
from .state import SimulationState, build_initial_state
from .workers import run_uav_process

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
        self.pid_choice_log: dict[str, tuple[float | None, str, float]] = {}
        self.operation_mode_handlers: dict[int, OperationMode] = {}
        self.uav_autopilots: list[WaypointPIDController | None] = []
        self.uav_filming_props: list[dict | None] = []
        self.uav_current_wp_ids: list[int | None] = []
        self.uav_line_search_state: list[dict | None] = []
        self.uav_filming_target: list[tuple[float, float, float] | None] = []
        self.uav_line_search_debug: list[list[tuple[float, float, float]] | None] = []
        self._flight_path_labels: list[dict] = []
        self.psutil = psutil
        self.dem_shared: dict | None = None
        self.dem_shm: shared_memory.SharedMemory | None = None

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
        # Load DEM tiles; seed with DEFAULT_DEM_NAME so the initial active tile matches mission area.
        self.dem = DEM(str(DEM_FILE))
        log_dem_info(self.dem)
        apply_mission_reference_spawns(self)
        if self.mission_reference_path:
            relocate_targets_near_spawn(self)
        else:
            ensure_spawn_above_dem(self)
        if self.enable_flight_paths:
            load_flight_paths(self)
        init_uav_autopilots(self)
        self.clock = init_display()
        self._start_workers()
        self._start_logger()
        _log("entering main loop")

    def run(self):
        self.setup()
        try:
            while self.running:
                dt, dt_sim, dt_sim_step, time_scale_val = tick_time(self)
                sample_cpu(self)
                self._poll_worker_states()
                consume_fuel(self, dt_sim)
                keys = pygame.key.get_pressed()
                apply_continuous_input(self, keys, dt)
                process_events(self)
                # Run autopilots before snapshots so we can use the buffered worker states directly.
                update_autopilots(self, dt_sim, dt)
                self._send_controls_to_workers(time_scale_val)
                snapshots, active_snap = capture_snapshots_and_trails(self)
                nearest = nearest_target(self, active_snap)
                update_targets_and_missiles(self, dt_sim)
                detection_lines = evaluate_threats(self, dt_sim)
                remove_destroyed_targets(self)
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
                render_frame(
                    self,
                    snapshots,
                    active_snap,
                    nearest,
                    detection_lines,
                    dt_sim_step,
                    time_scale_val,
                    uav_lon,
                    uav_lat,
                )
        finally:
            self._shutdown()

    def _start_workers(self):
        assert self.dem is not None
        assert self.state is not None
        self._prepare_shared_dem()
        self.state.workers = []
        self.state.proc_cmd_queues = []
        self.state.proc_state_queues = []
        self.state.proc_stop_events = []

        for i, u in enumerate(self.state.uavs):
            cmd_q = multiprocessing.Queue(maxsize=4)
            state_q = multiprocessing.Queue(maxsize=64)
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
                    self.dem_shared,
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
                            pending = self.state.pending_states[idx]
                            pending.append(state_tuple)
                            if len(pending) > 256:
                                pending.pop(0)
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
            try:
                self.state.proc_cmd_queues[i].put_nowait(msg)
            except queue.Full:
                try:
                    _ = self.state.proc_cmd_queues[i].get_nowait()
                    self.state.proc_cmd_queues[i].put_nowait(msg)
                except Exception:
                    pass


    def _compute_filming_target(self, idx: int, filming_prop: dict | None):
        """Legacy single-shot compute (fallback only)."""
        return default_downward_target(self, self.state.uavs[idx])

    def _default_downward_target(self, uav):
        return default_downward_target(self, uav)

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
        if self.dem_shm:
            try:
                self.dem_shm.close()
                self.dem_shm.unlink()
            except FileNotFoundError:
                pass
        pygame.quit()

    def _prepare_shared_dem(self):
        """Copy DEM elevation into shared memory for worker reuse."""
        if not self.dem or getattr(self.dem, "elevation", None) is None:
            self.dem_shared = None
            return
        try:
            elev = self.dem.elevation
            self.dem_shm = shared_memory.SharedMemory(create=True, size=elev.nbytes)
            shm_arr = np.ndarray(elev.shape, dtype=elev.dtype, buffer=self.dem_shm.buf)
            np.copyto(shm_arr, elev)
            transform = getattr(self.dem, "transform", None)
            transform_tuple = (
                float(transform.a),
                float(transform.b),
                float(transform.c),
                float(transform.d),
                float(transform.e),
                float(transform.f),
            ) if transform is not None else None
            self.dem_shared = {
                "name": self.dem_shm.name,
                "shape": elev.shape,
                "dtype": str(elev.dtype),
                "ref_lon": getattr(self.dem, "ref_lon", 0.0),
                "ref_lat": getattr(self.dem, "ref_lat", 0.0),
                "scale_x": getattr(self.dem, "scale_x", 1.0),
                "scale_y": getattr(self.dem, "scale_y", 1.0),
                "bounds": (
                    getattr(self.dem, "xmin", 0.0),
                    getattr(self.dem, "ymin", 0.0),
                    getattr(self.dem, "xmax", 0.0),
                    getattr(self.dem, "ymax", 0.0),
                ),
                "env_bounds": getattr(self.dem, "env_bounds", (0.0, 0.0, 0.0, 0.0)),
                "world_env_bounds": getattr(self.dem, "world_env_bounds", (0.0, 0.0, 0.0, 0.0)),
                "min_elev": getattr(self.dem, "min_elev", 0.0),
                "max_elev": getattr(self.dem, "max_elev", 0.0),
                "transform": transform_tuple,
            }
        except Exception as e:
            print(f"[DEM] failed to set up shared DEM: {e}")
            self.dem_shared = None
