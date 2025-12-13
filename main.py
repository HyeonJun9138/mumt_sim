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
)
from OpenGL.GLU import gluPerspective

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
from sim.render.draw import draw_axes, draw_camera_footprint, draw_dem, draw_grid, draw_hud, draw_uav
from sim.entities.entities import MovingTarget, Missile
from sim.core.uav import UAV, UAVParams


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
    uav = UAV(params)
    cam = OrbitCamera()

    targets = [MovingTarget(x=300.0, y=0.0)]
    targets_move = True
    missiles = []
    trail = []
    crashed = False

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

        keys = pygame.key.get_pressed()
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
                    uav.reset()
                    crashed = False
                    trail.clear()
                elif e.key == pygame.K_f:
                    fog_enabled = not fog_enabled
                elif e.key == pygame.K_n:
                    targets_move = not targets_move
                elif e.key == pygame.K_g:
                    cx, cy, cz = cam.target
                    ox = random.uniform(-150, 150)
                    oy = random.uniform(-150, 150)
                    targets.append(
                        MovingTarget(
                            x=clamp(cx + ox, -WORLD_HALF, WORLD_HALF),
                            y=clamp(cy + oy, -WORLD_HALF, WORLD_HALF),
                        )
                    )
                elif e.key == pygame.K_m:
                    if targets:
                        nearest = min(
                            targets,
                            key=lambda T: (T.x - uav.s.x) ** 2 + (T.y - uav.s.y) ** 2 + (T.z - uav.s.z) ** 2,
                        )
                        yaw_rad = math.radians(uav.s.yaw)
                        launch_x = uav.s.x + math.cos(yaw_rad) * 8.0
                        launch_y = uav.s.y + math.sin(yaw_rad) * -8.0
                        launch_z = uav.s.z
                        missiles.append(Missile(launch_x, launch_y, launch_z, nearest))
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

        lerp = 0.1
        cam.target[0] = (1 - lerp) * cam.target[0] + lerp * uav.s.x
        cam.target[1] = (1 - lerp) * cam.target[1] + lerp * uav.s.y
        cam.target[2] = (1 - lerp) * cam.target[2] + lerp * uav.s.z

        if not crashed:
            uav.step(dt)
            ground_z = dem.get_height(uav.s.x, uav.s.y)
            if uav.s.z <= ground_z + 0.5:
                crashed = True
                uav.s.z = ground_z
                uav.s.u = 0.0
                uav.cmd_throttle = 0.0
                uav.cmd_pitch_rate = 0.0
                uav.cmd_yaw_rate = 0.0
                uav.cmd_roll_rate = 0.0
        # Track trail with spacing and 1 km length limit
        pos = (uav.s.x, uav.s.y, uav.s.z)
        if trail:
            prev = trail[-1]
            seg = math.dist(pos, prev)
            if seg >= 5.0:
                trail.append(pos)
        else:
            trail.append(pos)
        # keep last 1 km of path
        total = 0.0
        cutoff_idx = 0
        for i in range(len(trail) - 1, 0, -1):
            total += math.dist(trail[i], trail[i - 1])
            if total > 1000.0:
                cutoff_idx = i - 1
                break
        if cutoff_idx > 0:
            trail = trail[cutoff_idx:]
        elif total <= 0.0 and len(trail) > 2:
            trail = trail[-2:]
        if targets_move:
            for t in targets:
                t.step(dt, dem=dem)
        for m in list(missiles):
            m.step(dt)
            if (not m.active) and (m.exploded and m.explode_time > 1.2 and len(m.sparks) == 0):
                missiles.remove(m)

        nearest = min(
            targets, key=lambda T: (T.x - uav.s.x) ** 2 + (T.y - uav.s.y) ** 2 + (T.z - uav.s.z) ** 2
        ) if targets else None

        uav_lon, uav_lat = dem.env_to_lonlat(uav.s.x, uav.s.y)

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
            center_xy=(uav.s.x, uav.s.y),
            radius_m=RENDER_RADIUS_M,
            prefetch_radius_m=DEM_PREFETCH_RADIUS_M,
            move_threshold=DEM_CACHE_MOVE_THRESHOLD_M,
            z_scale=1.0,
        )
        glEnable(GL_BLEND)

        # UAV trail (blue dashed line)
        if len(trail) >= 2:
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(1, 0x0F0F)
            glLineWidth(2.0)
            glColor3f(0.2, 0.6, 1.0)
            glBegin(GL_LINE_STRIP)
            for p in trail:
                glVertex3f(*p)
            glEnd()
            glDisable(GL_LINE_STIPPLE)

        draw_uav(uav)
        for t in targets:
            t.draw()
        for m in missiles:
            m.draw(dem)

        if nearest:
            uav_pos = np.array([uav.s.x, uav.s.y, uav.s.z], dtype=float)
            tgt_pos = np.array([nearest.x, nearest.y, nearest.z], dtype=float)
            los_clear = check_los(uav_pos, tgt_pos, dem)
            glLineWidth(2.0)
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(1, 0x0C0C)
            glColor3f(1, 1, 1) if los_clear else glColor3f(1, 0, 0)
            glBegin(GL_LINES)
            glVertex3f(*uav_pos)
            glVertex3f(*tgt_pos)
            glEnd()
            glDisable(GL_LINE_STIPPLE)
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
            f"pos (m): x={uav.s.x:7.1f}  y={uav.s.y:7.1f}  z={uav.s.z:6.1f}",
            f"pos (lat/lon): lat={uav_lat:9.5f}  lon={uav_lon:10.5f}",
            f"spd (m/s): {uav.s.u:5.1f}   yaw={uav.s.yaw:6.1f}deg  pitch={uav.s.pitch:5.1f}deg  roll={uav.s.roll:5.1f}deg",
            f"FOV (diag): {fov_diag:4.1f}deg   LOS: {1 if los_clear else 0}   dt={dt*1000:.1f}ms",
            f"Targets: {len(targets)}  Missiles: {len(missiles)}",
            "M: Fire | N: Toggle targets | G: Spawn target | Wheel: Zoom | RMB drag: Orbit | MMB drag: Pan",
        ]
        if footprint_area is not None:
            lines.append(f"Footprint area: {footprint_area:8.1f} m^2")
        if crashed:
            lines.append("Status: CRASHED (press R to reset)")
        draw_hud(lines)

        if debug_frames < 3:
            print(
                f"[frame] cam_target=({cam.target[0]:.1f},{cam.target[1]:.1f},{cam.target[2]:.1f}) "
                f"cam_dist={cam.distance:.1f} uav_pos=({uav.s.x:.1f},{uav.s.y:.1f},{uav.s.z:.1f}) fog={fog_enabled}"
            )
            err = glGetError()
            if err:
                print(f"[GL ERROR] frame: {err}")
            debug_frames += 1

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
