import queue
import time
from multiprocessing import Event, Queue
from typing import Any, Dict, Tuple

from sim.config import clamp
from sim.core.lah import LAH, LAHParams
from sim.core.uav import UAV, UAVParams
from sim.world.dem import DEM


def _make_airframe(uav_type: str, params: Any):
    if uav_type == "LAH":
        if not isinstance(params, LAHParams):
            params = LAHParams()
        airframe = LAH(params)
    else:
        if not isinstance(params, UAVParams):
            params = UAVParams()
        airframe = UAV(params)
    return airframe


def _apply_state(airframe, state_tuple: Tuple[float, ...]):
    (
        airframe.s.x,
        airframe.s.y,
        airframe.s.z,
        airframe.s.roll,
        airframe.s.pitch,
        airframe.s.yaw,
        airframe.s.u,
        airframe.s.p,
        airframe.s.q,
        airframe.s.r,
    ) = state_tuple


def run_uav_process(
    idx: int,
    uav_type: str,
    params: Any,
    initial_state: Tuple[float, ...],
    dem_file: str,
    cmd_queue: Queue,
    state_queue: Queue,
    stop_event: Event,
    dem_shared: dict | None = None,
):
    """
    Multiprocessing worker: integrate a single airframe and stream back state tuples.
    Messages on cmd_queue:
      {"type": "control", "cmds": {...}, "crippled": bool, "crashed": bool, "time_scale": float}
      {"type": "reset", "state": tuple}
      {"type": "time_scale", "value": float}
    """
    dem = DEM(shared=dem_shared) if dem_shared else DEM(dem_file)
    airframe = _make_airframe(uav_type, params)
    _apply_state(airframe, initial_state)

    crippled = False
    crashed = False
    time_scale = 1.0
    last_send = time.perf_counter()
    prev = last_send
    sim_since_send = 0.0
    sim_step = 0.01  # fixed integration step (sim seconds)
    send_sim_interval = 0.05  # target spacing in simulation seconds (20 Hz)

    while not stop_event.is_set():
        now = time.perf_counter()
        wall_dt = clamp(now - prev, 0.001, 0.05)
        prev = now
        sim_budget = wall_dt * time_scale
        sim_since_send += sim_budget
        step_budget = sim_budget

        try:
            while True:
                msg: Dict[str, Any] = cmd_queue.get_nowait()
                mtype = msg.get("type")
                if mtype == "control":
                    cmds = msg.get("cmds", {})
                    airframe.cmd_yaw_rate = cmds.get("yaw_rate", airframe.cmd_yaw_rate)
                    airframe.cmd_pitch_rate = cmds.get("pitch_rate", airframe.cmd_pitch_rate)
                    airframe.cmd_roll_rate = cmds.get("roll_rate", airframe.cmd_roll_rate)
                    airframe.cmd_throttle = cmds.get("throttle", airframe.cmd_throttle)
                    crippled = bool(msg.get("crippled", crippled))
                    crashed = bool(msg.get("crashed", crashed))
                    time_scale = float(msg.get("time_scale", time_scale))
                elif mtype == "reset":
                    _apply_state(airframe, msg.get("state", initial_state))
                    airframe.cmd_yaw_rate = 0.0
                    airframe.cmd_pitch_rate = 0.0
                    airframe.cmd_roll_rate = 0.0
                    airframe.cmd_throttle = 0.0
                    crippled = False
                    crashed = False
                    time_scale = float(msg.get("time_scale", time_scale))
                elif mtype == "time_scale":
                    time_scale = float(msg.get("value", time_scale))
                else:
                    # Unknown message type; ignore.
                    pass
        except queue.Empty:
            pass

        if not crashed:
            # Integrate in fixed sim_step chunks to avoid variable dt jitter.
            while step_budget > 0.0:
                step_dt = sim_step if step_budget >= sim_step else step_budget
                airframe.step(step_dt)
                if crippled:
                    airframe.s.z = max(0.0, airframe.s.z - 30.0 * step_dt)
                    airframe.s.u = max(0.0, airframe.s.u * 0.95)
                ground_z = dem.get_height(airframe.s.x, airframe.s.y)
                if airframe.s.z <= ground_z + 0.5:
                    # Terrain collision: mark crashed and pin to ground.
                    airframe.s.z = ground_z
                    airframe.s.u = 0.0
                    crashed = True
                    crippled = False
                    # Clear commands to avoid further motion until reset.
                    airframe.cmd_yaw_rate = 0.0
                    airframe.cmd_pitch_rate = 0.0
                    airframe.cmd_roll_rate = 0.0
                    airframe.cmd_throttle = 0.0
                    sim_since_send = send_sim_interval  # force immediate report
                    break
                step_budget -= step_dt

        if sim_since_send >= send_sim_interval:
            msg = {
                "idx": idx,
                "state": (
                    airframe.s.x,
                    airframe.s.y,
                    airframe.s.z,
                    airframe.s.roll,
                    airframe.s.pitch,
                    airframe.s.yaw,
                    airframe.s.u,
                    airframe.s.p,
                    airframe.s.q,
                    airframe.s.r,
                ),
                "crippled": crippled,
                "crashed": crashed,
            }
            try:
                state_queue.put_nowait(msg)
            except queue.Full:
                # Drop oldest style: clear one slot then retry once.
                try:
                    state_queue.get_nowait()
                    state_queue.put_nowait(msg)
                except Exception:
                    pass
            last_send = now
            sim_since_send = 0.0

        time.sleep(0.002)
