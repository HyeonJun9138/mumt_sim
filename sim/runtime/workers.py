import time


def uav_worker(idx, uav, dem, run_event, crashed_flags, crippled_flags, lock, time_scale, time_lock):
    """Background integrator for each UAV."""
    prev = time.perf_counter()
    while run_event.is_set():
        now = time.perf_counter()
        dt = max(0.001, min(0.05, now - prev))
        prev = now
        with time_lock:
            ts = time_scale["value"]
        dt *= ts
        with lock:
            if not crashed_flags[idx]:
                uav.step(dt)
                if crippled_flags[idx]:
                    # Force rapid descent and spin regardless of commands
                    uav.s.z = max(0.0, uav.s.z - 30.0 * dt)
                    uav.s.u = max(0.0, uav.s.u * 0.95)
                # Safety: prevent terrain crashes during simulation; clamp above ground and keep flying.
                ground_z = dem.get_height(uav.s.x, uav.s.y)
                if uav.s.z <= ground_z + 5.0:
                    uav.s.z = ground_z + 80.0  # lift back above terrain
                    crashed_flags[idx] = False
                    crippled_flags[idx] = False
        time.sleep(0.002)
