from dataclasses import dataclass, field
import threading
from typing import List, Tuple

from sim.agent_status import MAX_FUEL_L
from sim.config import DEFAULT_FOV_DIAG
from sim.core.lah import LAH, LAHParams
from sim.core.uav import UAV, UAVParams
from sim.entities.entities import MovingTarget
from sim.entities.threat import AirDefenseThreat
from .constants import INITIAL_OFFSETS, TARGET_ROAM_RADIUS_M, TARGET_SPEED_RANGE, TARGET_SPAWNS, THREAT_RADAR, THREAT_WEAPON


@dataclass
class SimulationState:
    uavs: list
    uav_types: List[str]
    uav_locks: List[threading.Lock]
    targets: List[MovingTarget]
    missiles: list
    trails: List[list]
    crashed: List[bool]
    crippled: List[bool]
    fuel_levels: List[float]
    run_event: threading.Event
    time_scale: dict
    time_lock: threading.Lock
    workers: List[threading.Thread]
    active_idx: int = 0
    fov_diag: float = DEFAULT_FOV_DIAG
    fog_enabled: bool = True
    targets_move: bool = True
    latest_agent_status_0401: dict | None = None
    initial_spawn_points: List[Tuple[float, float, float]] = field(default_factory=list)
    missions: list = field(default_factory=list)
    autopilot_enabled: bool = False


def build_initial_state():
    params_uav = UAVParams()
    params_lah = LAHParams()
    uavs = [LAH(params_lah) for _ in range(3)] + [UAV(params_uav) for _ in range(3)]
    uav_types = ["LAH"] * 3 + ["UAV"] * 3
    uav_locks = [threading.Lock() for _ in uavs]

    for u, (ox, oy) in zip(uavs, INITIAL_OFFSETS):
        u.s.x += ox
        u.s.y += oy

    targets = [
        MovingTarget(
            x=tx,
            y=ty,
            vmin=TARGET_SPEED_RANGE[0],
            vmax=TARGET_SPEED_RANGE[1],
            roam_center=(tx, ty),
            roam_radius=TARGET_ROAM_RADIUS_M,
            threat=AirDefenseThreat(radar=THREAT_RADAR, weapon=THREAT_WEAPON),
        )
        for tx, ty in TARGET_SPAWNS
    ]
    if not targets:
        targets.append(
            MovingTarget(
                x=300.0,
                y=0.0,
                vmin=TARGET_SPEED_RANGE[0],
                vmax=TARGET_SPEED_RANGE[1],
                roam_center=(300.0, 0.0),
                roam_radius=TARGET_ROAM_RADIUS_M,
                threat=AirDefenseThreat(radar=THREAT_RADAR, weapon=THREAT_WEAPON),
            )
        )
    missiles: list = []
    trails = [[] for _ in uavs]
    crashed = [False for _ in uavs]
    crippled = [False for _ in uavs]
    fuel_levels = [MAX_FUEL_L for _ in uav_types]

    run_event = threading.Event()
    run_event.set()
    time_scale = {"value": 1.0}
    time_lock = threading.Lock()

    return SimulationState(
        uavs=uavs,
        uav_types=uav_types,
        uav_locks=uav_locks,
        targets=targets,
        missiles=missiles,
        trails=trails,
        crashed=crashed,
        crippled=crippled,
        fuel_levels=fuel_levels,
        run_event=run_event,
        time_scale=time_scale,
        time_lock=time_lock,
        workers=[],
        initial_spawn_points=[(u.s.x, u.s.y, u.s.z) for u in uavs],
        missions=[],
        autopilot_enabled=False,
    )
