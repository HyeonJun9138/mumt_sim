'''

 --- 조작 키 안내 ---
# W/S: 스로틀 업/다운
# 방향키 좌/우: 요(회전) + 롤 입력, 방향키 상/하: 피치 업/다운
# SPACE: 기체 자세 수평으로 복귀
# R: UAV 상태 리셋
# E/D: 카메라 대각 FOV 확대/축소
# M: 가장 가까운 타겟으로 미사일 발사
# N: 타겟 이동 on/off 토글
# G: 카메라 기준 주변에 타겟 하나 추가 생성
# 마우스 휠: 줌(카메라 거리 조정)
# 마우스 오른쪽 드래그: 궤도(orbit) 회전, 중간 버튼 드래그: 패닝
# ESC/Q: 종료

'''


import math
import random
import threading
import time
import numpy as np
import pygame
from pygame.locals import DOUBLEBUF, OPENGL
from OpenGL.GL import (
    glBegin,
    glBlendFunc,
    glClear,
    glClearColor,
    glColor3f,
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
    GL_LINE_LOOP,
    GL_LINES,
    GL_LINE_STRIP,
    GL_LINE_STIPPLE,
    GL_NICEST,
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
)
from OpenGL.GLU import gluPerspective, gluProject

from sim.core.camera import OrbitCamera
from sim.config import (
    CLEAR_COLOR,
    DEFAULT_FOV_DIAG,
    DEM_FILE,
    FPS,
    FOG_COLOR,
    FOG_DENSITY,
    WIN_W,
    WIN_H,
    WORLD_HALF,
    RENDER_RADIUS_M,
    DEM_PREFETCH_RADIUS_M,
    DEM_CACHE_MOVE_THRESHOLD_M,
    clamp,
)
from sim.world.dem import DEM, check_los
from sim.render.draw import draw_axes, draw_camera_footprint, draw_dem, draw_grid, draw_hud, draw_uav, draw_labels
from sim.entities.entities import MovingTarget, Missile
from sim.entities.threat import AirDefenseThreat, RadarParams, WeaponParams, WeaponType
from sim.core.uav import UAV, UAVParams

TARGET_SPAWNS = [
    # (x, y) in meters; edit this list to pre-place multiple targets
    (-200.0, -150.0),
    (250.0, 180.0),
]
TARGET_ROAM_RADIUS_M = 250.0  # each target roams within ~500m diameter around its spawn
TARGET_SPEED_RANGE = (2.0, 7.0)  # m/s
# Threat defaults (back to moderate)
THREAT_RADAR = RadarParams()  # P_fa=1e-6, n=1.5, t_ref=5s ...
THREAT_WEAPON = WeaponParams(
    weapon_type=WeaponType.MISSILE,
    a_range=3500.0,
    b_slope=2.0,
    omega=0.8,
    t_fire=3.0,
)
THREAT_DEFAULT = AirDefenseThreat(radar=THREAT_RADAR, weapon=THREAT_WEAPON)


def uav_worker(idx, uav, dem, run_event, crashed_flags, crippled_flags, lock, time_scale, time_lock):
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
                ground_z = dem.get_height(uav.s.x, uav.s.y)
                if uav.s.z <= ground_z + 0.5:
                    crashed_flags[idx] = True
                    crippled_flags[idx] = False
                    uav.s.z = ground_z
                    uav.s.u = 0.0
                    uav.cmd_throttle = 0.0
                    uav.cmd_pitch_rate = 0.0
                    uav.cmd_yaw_rate = 0.0
                    uav.cmd_roll_rate = 0.0
        time.sleep(0.002)


def main():
    dem = DEM(str(DEM_FILE))
    print(f"[DEM] file: {DEM_FILE}")
    print(
        f"[DEM] shape: {dem.elevation.shape} lon/lat bounds: ({dem.xmin:.5f},{dem.ymin:.5f})-({dem.xmax:.5f},{dem.ymax:.5f})"
    )
    print(f"[DEM] elevation min/max: {dem.min_elev:.1f}/{dem.max_elev:.1f}")

    pygame.init()
    pygame.display.set_caption("UAV 6-DoF 3D + DEM + Missile (modular)")
    pygame.display.set_mode((WIN_W, WIN_H), DOUBLEBUF | OPENGL)
    clock = pygame.time.Clock()

    # GL info
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

    params = UAVParams()
    uavs = [UAV(params) for _ in range(3)]
    uav_locks = [threading.Lock() for _ in uavs]
    # spread initial spawn slightly
    offsets = [(-30.0, -30.0), (0.0, 0.0), (30.0, 30.0)]
    for u, (ox, oy) in zip(uavs, offsets):
        u.s.x += ox
        u.s.y += oy
    active_idx = 0
    cam = OrbitCamera()

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
        targets = [
            MovingTarget(
                x=300.0,
                y=0.0,
                vmin=TARGET_SPEED_RANGE[0],
                vmax=TARGET_SPEED_RANGE[1],
                roam_center=(300.0, 0.0),
                roam_radius=TARGET_ROAM_RADIUS_M,
                threat=AirDefenseThreat(radar=THREAT_RADAR, weapon=THREAT_WEAPON),
            )
        ]
    targets_move = True
    missiles = []
    trails = [[] for _ in uavs]
    crashed = [False for _ in uavs]
    crippled = [False for _ in uavs]

    run_event = threading.Event()
    run_event.set()
    time_scale = {"value": 1.0}
    time_lock = threading.Lock()
    workers = [
        threading.Thread(
            target=uav_worker,
            args=(i, u, dem, run_event, crashed, crippled, uav_locks[i], time_scale, time_lock),
            daemon=True,
        )
        for i, u in enumerate(uavs)
    ]
    for t in workers:
        t.start()

    orbit = False
    pan = False
    last = (0, 0)
    fov_diag = DEFAULT_FOV_DIAG
    fog_enabled = True
    debug_frames = 0

    dt_smoothed = 1 / 60
    running = True
    while running:
        ms = clock.tick(FPS)
        raw_dt = max(0.0001, min(0.05, ms / 1000.0))
        dt_smoothed = 0.85 * dt_smoothed + 0.15 * raw_dt
        dt = dt_smoothed
        with time_lock:
            time_scale_val = time_scale["value"]
        dt_sim = dt * time_scale_val

        keys = pygame.key.get_pressed()
        uav = uavs[active_idx]
        with uav_locks[active_idx]:
            if not crippled[active_idx]:
                if keys[pygame.K_w]:
                    uav.cmd_throttle = 1.0
                elif keys[pygame.K_s]:
                    uav.cmd_throttle = -1.0
                else:
                    uav.cmd_throttle = 0.0

                if keys[pygame.K_SPACE]:
                    uav.cmd_straight()
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
            fov_diag += 15.0 * dt
        if keys[pygame.K_d]:
            fov_diag -= 15.0 * dt
        fov_diag = clamp(fov_diag, 1.2, 31.2)

        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif e.key == pygame.K_r:
                    for i, u in enumerate(uavs):
                        with uav_locks[i]:
                            u.reset()
                            u.s.x += offsets[i][0]
                            u.s.y += offsets[i][1]
                            crashed[i] = False
                            crippled[i] = False
                            trails[i].clear()
                elif e.key == pygame.K_f:
                    fog_enabled = not fog_enabled
                elif e.key == pygame.K_n:
                    targets_move = not targets_move
                elif e.key == pygame.K_p:
                    print("[auto] Autopilot not available (manual only)")
                elif e.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    with time_lock:
                        time_scale["value"] = clamp(time_scale["value"] + 1.0, 0.1, 100.0)
                    print(f"[time] scale -> {time_scale['value']:.1f}x")
                elif e.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    with time_lock:
                        time_scale["value"] = clamp(time_scale["value"] - 1.0, 0.1, 100.0)
                    print(f"[time] scale -> {time_scale['value']:.1f}x")
                elif e.key == pygame.K_g:
                    cx, cy, cz = cam.target
                    ox = random.uniform(-150, 150)
                    oy = random.uniform(-150, 150)
                    tx = clamp(cx + ox, -WORLD_HALF, WORLD_HALF)
                    ty = clamp(cy + oy, -WORLD_HALF, WORLD_HALF)
                    targets.append(
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
                elif e.key == pygame.K_m:
                    if targets:
                        with uav_locks[active_idx]:
                            nearest = min(
                                targets,
                                key=lambda T: (T.x - uav.s.x) ** 2 + (T.y - uav.s.y) ** 2 + (T.z - uav.s.z) ** 2,
                            )
                            yaw_rad = math.radians(uav.s.yaw)
                            launch_x = uav.s.x + math.cos(yaw_rad) * 8.0
                            launch_y = uav.s.y + math.sin(yaw_rad) * -8.0
                            launch_z = uav.s.z
                        missiles.append(Missile(launch_x, launch_y, launch_z, nearest))
                elif e.key == pygame.K_1:
                    active_idx = 0
                elif e.key == pygame.K_2 and len(uavs) > 1:
                    active_idx = 1
                elif e.key == pygame.K_3 and len(uavs) > 2:
                    active_idx = 2
            elif e.type == pygame.MOUSEBUTTONDOWN:
                if e.button == 3:
                    orbit = True
                    last = e.pos
                elif e.button == 2:
                    pan = True
                    last = e.pos
                elif e.button == 4:
                    cam.distance = max(50.0, cam.distance * 0.9)
                elif e.button == 5:
                    cam.distance = min(8000.0, cam.distance * 1.1)
            elif e.type == pygame.MOUSEBUTTONUP:
                if e.button == 3:
                    orbit = False
                elif e.button == 2:
                    pan = False
            elif e.type == pygame.MOUSEMOTION:
                if orbit:
                    dx = e.pos[0] - last[0]
                    dy = e.pos[1] - last[1]
                    cam.yaw += dx * 0.3
                    cam.pitch = clamp(cam.pitch - dy * 0.3, -89.0, 89.0)
                    last = e.pos
                elif pan:
                    dx = e.pos[0] - last[0]
                    dy = e.pos[1] - last[1]
                    pan_scale = cam.distance * 0.002
                    cam.target[0] -= dx * pan_scale
                    cam.target[1] += dy * pan_scale
                    last = e.pos

        snapshots = []
        for i, u in enumerate(uavs):
            with uav_locks[i]:
                s = u.s
                pos = (s.x, s.y, s.z, s.roll, s.pitch, s.yaw, u.s.u)
            snapshots.append(pos)
            # Track trail with spacing and 1 km length limit
            trail = trails[i]
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
                trails[i] = trail[cutoff_idx:]
            elif total <= 0.0 and len(trail) > 2:
                trails[i] = trail[-2:]
        active_snap = snapshots[active_idx]
        # Snap camera target to active UAV (no smoothing to reduce overhead)
        cam.target[0] = active_snap[0]
        cam.target[1] = active_snap[1]
        cam.target[2] = active_snap[2]

        if targets_move:
            for t in targets:
                t.step(dt_sim, dem=dem)
        for m in list(missiles):
            m.step(dt_sim)
            if (not m.active) and (m.exploded and m.explode_time > 1.2 and len(m.sparks) == 0):
                missiles.remove(m)
        detection_lines = []
        # Threat engagement: targets attack UAVs if detected
        for idx, u in enumerate(uavs):
            with uav_locks[idx]:
                upos = np.array([u.s.x, u.s.y, u.s.z], dtype=float)
            for t in targets:
                if not getattr(t, "threat", None):
                    continue
                range_m = float(np.linalg.norm(np.array([t.x, t.y, t.z]) - upos))
                if range_m < 1e-3:
                    continue
                los = check_los(upos, np.array([t.x, t.y, t.z]), dem)
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
                    with uav_locks[idx]:
                        crippled[idx] = True
                        # Force uncontrolled descent/spin
                        u.cmd_throttle = -1.0
                        u.cmd_pitch_rate = -u.p.max_pitch_rate_dps * 0.8
                        spin_sign = 1.0 if random.random() < 0.5 else -1.0
                        u.cmd_roll_rate = u.p.max_roll_rate_dps * 0.6 * spin_sign
                        u.cmd_yaw_rate = u.p.max_yaw_rate_dps * 0.6 * spin_sign
        # Remove destroyed targets
        targets = [t for t in targets if getattr(t, "alive", True)]

        nearest = (
            min(
                targets,
                key=lambda T: (T.x - active_snap[0]) ** 2 + (T.y - active_snap[1]) ** 2 + (T.z - active_snap[2]) ** 2,
            )
            if targets
            else None
        )

        uav_lon, uav_lat = dem.env_to_lonlat(active_snap[0], active_snap[1])

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        cam.apply()

        if fog_enabled:
            glEnable(GL_FOG)
        else:
            glDisable(GL_FOG)

        glDisable(GL_BLEND)
        draw_grid(size=int(RENDER_RADIUS_M), step=100)
        draw_axes(50)
        draw_dem(
            dem,
            cam,
            center_xy=(active_snap[0], active_snap[1]),
            radius_m=RENDER_RADIUS_M,
            prefetch_radius_m=DEM_PREFETCH_RADIUS_M,
            move_threshold=DEM_CACHE_MOVE_THRESHOLD_M,
            z_scale=1.0,
        )
        glEnable(GL_BLEND)

        for idx, trail in enumerate(trails):
            if len(trail) >= 2:
                glDisable(GL_LINE_STIPPLE)
                glLineWidth(2.0 if idx == active_idx else 1.5)
                glColor3f(0.2, 0.6, 1.0) if idx == active_idx else glColor3f(0.4, 0.5, 0.7)
                glBegin(GL_LINE_STRIP)
                for p in trail:
                    glVertex3f(*p)
                glEnd()

        for i, u in enumerate(uavs):
            with uav_locks[i]:
                draw_uav(u)
        for t in targets:
            t.draw()
        for m in missiles:
            m.draw(dem)
        # Threat LOS lines (targets -> UAVs)
        if detection_lines:
            glLineWidth(1.5)
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(1, 0x0F0F)
            for info in detection_lines:
                if info["los"]:
                    glColor3f(1.0, 0.2, 0.2) if info["detected"] else glColor3f(1.0, 0.2, 0.2)
                else:
                    glColor3f(0.45, 0.45, 0.45)
                glBegin(GL_LINES)
                glVertex3f(*info["tpos"])
                glVertex3f(*info["upos"])
                glEnd()
            glDisable(GL_LINE_STIPPLE)

        # 3D -> 2D labels
        model = glGetDoublev(GL_MODELVIEW_MATRIX)
        proj = glGetDoublev(GL_PROJECTION_MATRIX)
        viewport = glGetIntegerv(GL_VIEWPORT)
        label_entries = []
        for idx, snap in enumerate(snapshots):
            sx, sy, sz = gluProject(snap[0], snap[1], snap[2] + 10.0, model, proj, viewport)
            if 0.0 <= sz <= 1.0:
                color = (255, 80, 80) if idx == active_idx else (220, 220, 220)
                label_entries.append((sx, sy + 8, f"uav{idx + 1}", color))
        for tidx, t in enumerate(targets):
            sx, sy, sz = gluProject(t.x, t.y, t.z + 10.0, model, proj, viewport)
            if 0.0 <= sz <= 1.0:
                label_entries.append((sx, sy + 8, f"target{tidx + 1}", (255, 200, 80)))
        draw_labels(label_entries)

        if nearest:
            uav_pos = np.array([active_snap[0], active_snap[1], active_snap[2]], dtype=float)
            tgt_pos = np.array([nearest.x, nearest.y, nearest.z], dtype=float)
            los_clear = check_los(uav_pos, tgt_pos, dem)
            glLineWidth(2.0)
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(1, 0x3333)  # wide-gap dash
            glColor3f(0.6, 0.6, 0.6)
            glBegin(GL_LINES)
            glVertex3f(*uav_pos)
            glVertex3f(*tgt_pos)
            glEnd()
            glDisable(GL_LINE_STIPPLE)
            with uav_locks[active_idx]:
                footprint_area = draw_camera_footprint(
                    uav,
                    (nearest.x, nearest.y, nearest.z),
                    fov_diag_deg=fov_diag,
                    dem=dem,
                    aspect_ratio=16 / 9,
                    z_scale=1.0,
                )
        else:
            footprint_area = None
            los_clear = False

        lines = [
            f"Active: UAV{active_idx + 1}  pos (m): x={active_snap[0]:7.1f}  y={active_snap[1]:7.1f}  z={active_snap[2]:6.1f}",
            f"pos (lat/lon): lat={uav_lat:9.5f}  lon={uav_lon:10.5f}",
            f"spd (m/s): {active_snap[6]:5.1f}   yaw={active_snap[5]:6.1f}deg  pitch={active_snap[4]:5.1f}deg  roll={active_snap[3]:5.1f}deg",
            f"FOV (diag): {fov_diag:4.1f}deg   LOS: {1 if los_clear else 0}   dt={dt_sim*1000:.1f}ms  time x{time_scale_val:.1f}",
            f"Targets: {len(targets)}  Missiles: {len(missiles)}",
            "1/2/3: switch UAV | M: Fire | N: Toggle targets | G: Spawn target | Wheel: Zoom | RMB drag: Orbit | MMB drag: Pan",
        ]
        if footprint_area is not None:
            lines.append(f"Footprint area: {footprint_area:8.1f} m^2")
        if any(crippled) and not any(crashed):
            crippled_ids = [str(i + 1) for i, c in enumerate(crippled) if c]
            lines.append(f"Status: DAMAGED UAVs: {', '.join(crippled_ids)} (falling)")
        if any(crashed):
            crashed_ids = [str(i + 1) for i, c in enumerate(crashed) if c]
            lines.append(f"Status: CRASHED UAVs: {', '.join(crashed_ids)} (press R to reset)")
        draw_hud(lines)

        if debug_frames < 3:
            print(
                f"[frame] cam_target=({cam.target[0]:.1f},{cam.target[1]:.1f},{cam.target[2]:.1f}) "
                f"cam_dist={cam.distance:.1f} uav_pos=({active_snap[0]:.1f},{active_snap[1]:.1f},{active_snap[2]:.1f}) "
                f"time_scale={time_scale_val:.1f} fog={fog_enabled}"
            )
            err = glGetError()
            if err:
                print(f"[GL ERROR] frame: {err}")
            debug_frames += 1

        pygame.display.flip()

    run_event.clear()
    for t in workers:
        t.join(timeout=0.2)
    pygame.quit()


if __name__ == "__main__":
    main()
