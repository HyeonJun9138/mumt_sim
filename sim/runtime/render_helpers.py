from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
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
    GL_NICEST,
    glLineWidth,
    glLineStipple,
    glLoadIdentity,
    glMatrixMode,
    glVertex3f,
    GL_SRC_ALPHA,
    GL_ONE_MINUS_SRC_ALPHA,
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
    GL_MODELVIEW,
    GL_PROJECTION,
    GL_QUADS,
    GLfloat,
    glGetDoublev,
    glGetIntegerv,
    GL_MODELVIEW_MATRIX,
    GL_PROJECTION_MATRIX,
    GL_VIEWPORT,
    glOrtho,
    glPointSize,
    GL_POINTS,
    glPopMatrix,
    glPushMatrix,
    glViewport,
    glGetString,
    GL_VENDOR,
    GL_RENDERER,
    GL_VERSION,
    GL_SHADING_LANGUAGE_VERSION,
    glGetError,
)
from OpenGL.GLU import gluPerspective, gluProject

from sim.config import (
    CLEAR_COLOR,
    DEM_CACHE_MOVE_THRESHOLD_M,
    DEM_PREFETCH_RADIUS_M,
    FOG_COLOR,
    FOG_DENSITY,
    RENDER_RADIUS_M,
    WIN_H,
    WIN_W,
)
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
from sim.world.dem import check_los

if TYPE_CHECKING:
    from sim.runtime.app import SimulationApp


def init_display():
    """OpenGL/pygame 초기화."""
    import os
    import pygame
    from pygame.locals import DOUBLEBUF, OPENGL

    pygame.init()
    caption = os.getenv("SIM_INSTANCE_NAME", "UAV 6-DoF 3D + DEM + Missile (modular)")
    pygame.display.set_caption(caption)
    print(f"[sim] window caption: {caption}")
    pygame.display.set_mode((WIN_W, WIN_H), DOUBLEBUF | OPENGL)

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
    return pygame.time.Clock()


def draw_flight_paths(app: "SimulationApp") -> None:
    """비행 경로 시각화."""
    if not getattr(app.state, "flight_paths", None):
        return
    cam_x, cam_y = app.cam.target[0], app.cam.target[1]
    app._flight_path_labels = []
    glLineWidth(1.0)
    glEnable(GL_LINE_STIPPLE)
    glLineStipple(1, 0x0F0F)
    for idx, plist in enumerate(app.state.flight_paths):
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
            app._flight_path_labels.append(
                {
                    "pos": wp["pos"],
                    "text": f"{'LAH' if idx < 3 else 'UAV'}{idx+1} - WP{wp.get('wp_id')}",
                }
            )
        glEnd()
    glDisable(GL_LINE_STIPPLE)


def draw_minimap(app: "SimulationApp", snapshots, active_snap) -> None:
    """우상단 미니맵."""
    map_half = 10000.0  # 20 km span
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

    ax, ay = active_snap[0], active_snap[1]
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

    # Flight path points (mission WPs): show all, highlight active
    if getattr(app.state, "flight_paths", None):
        active_idx = app.state.active_idx
        for idx, plist in enumerate(app.state.flight_paths):
            if not plist:
                continue
            base_color = (1.0, 0.9, 0.2) if idx == active_idx else (0.7, 0.7, 0.7)
            glLineWidth(1.0)
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(1, 0x1111)
            glColor3f(*base_color)
            glBegin(GL_LINE_STRIP)
            for wp in plist:
                x, y, z = wp["pos"]
                dx, dy = x - ax, y - ay
                if abs(dx) <= map_half and abs(dy) <= map_half:
                    glVertex3f(dx, dy, 0)
            glEnd()
            glDisable(GL_LINE_STIPPLE)
            glPointSize(4.0 if idx == active_idx else 3.0)
            glColor3f(*base_color)
            glBegin(GL_POINTS)
            for wp in plist:
                x, y, z = wp["pos"]
                dx, dy = x - ax, y - ay
                if abs(dx) <= map_half and abs(dy) <= map_half:
                    glVertex3f(dx, dy, 0)
            glEnd()

    # Active UAV position
    glPointSize(6.0)
    glColor3f(1.0, 0.2, 0.2)
    glBegin(GL_POINTS)
    glVertex3f(0.0, 0.0, 0.0)
    glEnd()

    glEnable(GL_DEPTH_TEST)
    # Restore matrices/viewport
    glPopMatrix()  # modelview
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
    glPopMatrix()
    glViewport(0, 0, WIN_W, WIN_H)


def render_frame(app: "SimulationApp", snapshots, active_snap, nearest, detection_lines, dt_sim, time_scale_val, uav_lon, uav_lat):
    """메인 프레임 렌더링."""
    import pygame

    is_lah = app.state.uav_types[app.state.active_idx] == "LAH"

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    app.cam.apply()

    footprint_corners = None

    if app.state.fog_enabled:
        glEnable(GL_FOG)
    else:
        glDisable(GL_FOG)

    glDisable(GL_BLEND)
    draw_grid(size=int(RENDER_RADIUS_M), step=100)
    draw_axes(50)
    draw_dem(
        app.dem,
        app.cam,
        center_xy=(active_snap[0], active_snap[1]),
        radius_m=RENDER_RADIUS_M,
        prefetch_radius_m=DEM_PREFETCH_RADIUS_M,
        move_threshold=DEM_CACHE_MOVE_THRESHOLD_M,
        z_scale=1.0,
    )
    glEnable(GL_BLEND)

    draw_flight_paths(app)

    for idx, trail in enumerate(app.state.trails):
        if len(trail) >= 2:
            glDisable(GL_LINE_STIPPLE)
            glLineWidth(2.0 if idx == app.state.active_idx else 1.5)
            if idx == app.state.active_idx:
                glColor3f(0.2, 0.6, 1.0)
            else:
                glColor3f(0.4, 0.5, 0.7)
            glBegin(GL_LINE_STRIP)
            for p in trail:
                glVertex3f(*p)
            glEnd()

    for i, u in enumerate(app.state.uavs):
        with app.state.uav_locks[i]:
            if app.state.uav_types[i] == "LAH":
                draw_lah(u, app.rotor_angle)
            else:
                draw_uav(u)
    for t in app.state.targets:
        t.draw()
    for m in app.state.missiles:
        m.draw(app.dem)
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
    mode_labels = {0: "없음", 1: "좌표 지정", 2: "구간탐색", 3: "자동추적", 4: "기체고정", 5: "자동주사"}
    for idx, snap in enumerate(snapshots):
        sx, sy, sz = gluProject(snap[0], snap[1], snap[2] + 10.0, model, proj, viewport)
        if 0.0 <= sz <= 1.0:
            color = (255, 80, 80) if idx == app.state.active_idx else (220, 220, 220)
            name = f"LAH{idx + 1}" if idx < 3 else f"UAV{idx - 2}"
            speed_text = f"{snap[6]:.0f}m/s"
            state_tag = "FALL" if app.state.crippled[idx] else ("CRASH" if app.state.crashed[idx] else "")
            text = f"{name} {speed_text}" if not state_tag else f"{name} {speed_text} {state_tag}"
            label_entries.append((sx, sy + 8, text, color))
            wp_id = app.uav_current_wp_ids[idx] if idx < len(app.uav_current_wp_ids) else None
            filming = app.uav_filming_props[idx] if idx < len(app.uav_filming_props) else None
            mode = filming.get("operationMode") if isinstance(filming, dict) else None
            mode_text = mode_labels.get(mode, "없음") if mode is not None else "없음"
            sub_color = (255, 220, 180) if idx == app.state.active_idx else (180, 180, 180)
            status_text = ""
            if idx < len(app.uav_autopilots):
                ap = app.uav_autopilots[idx]
                if ap is not None:
                    if getattr(ap, "is_loitering", False):
                        remaining = max(0.0, getattr(ap, "loiter_timer", 0.0))
                        status_text = f" | 상태: 선회중 ({remaining:.1f}s)"
                    elif getattr(ap, "is_hovering", False):
                        remaining = max(0.0, getattr(ap, "hover_timer", 0.0))
                        status_text = f" | 상태: 호버링 ({remaining:.1f}s)"
                    else:
                        status_text = " | 상태: 이동중"
            label_entries.append(
                (sx, sy - 12, f"WP {wp_id if wp_id is not None else '-'} | 촬영 모드: {mode_text}{status_text}", sub_color)
            )
    # Flight path point labels (visible subset only)
    for info in getattr(app, "_flight_path_labels", []):
        px, py, pz = gluProject(info["pos"][0], info["pos"][1], info["pos"][2] + 5.0, model, proj, viewport)
        if 0.0 <= pz <= 1.0:
            path_labels.append((px, py, info["text"], (255, 215, 0)))
    label_entries.extend(path_labels)
    for tidx, t in enumerate(app.state.targets):
        sx, sy, sz = gluProject(t.x, t.y, t.z + 10.0, model, proj, viewport)
        if 0.0 <= sz <= 1.0:
            label_entries.append((sx, sy + 8, f"target{tidx + 1}", (255, 200, 80)))
    draw_labels(label_entries)

    footprint_area = None
    los_clear = False
    if not is_lah:
        filming_prop = None
        if app.uav_filming_props and app.state.active_idx < len(app.uav_filming_props):
            filming_prop = app.uav_filming_props[app.state.active_idx]
        filming_target = None
        if app.uav_filming_target and app.state.active_idx < len(app.uav_filming_target):
            filming_target = app.uav_filming_target[app.state.active_idx]
        if filming_target is None:
            filming_target = app._compute_filming_target(app.state.active_idx, filming_prop)

        tgt_pos = None
        if filming_target is not None:
            tgt_pos = np.array(filming_target, dtype=float)
        elif nearest is not None:
            tgt_pos = np.array([nearest.x, nearest.y, nearest.z], dtype=float)

        # Draw line-search polyline debug (always, if exists).
        if (
            app.uav_line_search_debug
            and app.state.active_idx < len(app.uav_line_search_debug)
            and app.uav_line_search_debug[app.state.active_idx]
        ):
            draw_polyline(
                app.uav_line_search_debug[app.state.active_idx],
                color=(0.4, 1.0, 0.4),
                width=2.0,
                stipple=True,
            )

        if tgt_pos is not None:
            uav_pos = np.array([active_snap[0], active_snap[1], active_snap[2]], dtype=float)
            los_clear = check_los(uav_pos, tgt_pos, app.dem)
            glLineWidth(2.0)
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(1, 0x3333)
            glColor3f(0.6, 0.6, 0.6)
            glBegin(GL_LINES)
            glVertex3f(*uav_pos)
            glVertex3f(*tgt_pos)
            glEnd()
            glDisable(GL_LINE_STIPPLE)
            with app.state.uav_locks[app.state.active_idx]:
                footprint_area, footprint_corners = draw_camera_footprint(
                    app.state.uavs[app.state.active_idx],
                    tgt_pos,
                    fov_diag_deg=app.state.fov_diag,
                    dem=app.dem,
                    aspect_ratio=16 / 9,
                    z_scale=1.0,
                )

    lines = [
        f"Active: {'LAH' if is_lah else 'UAV'}{app.state.active_idx + 1}  pos (m): x={active_snap[0]:7.1f}  y={active_snap[1]:7.1f}  z={active_snap[2]:6.1f}",
        f"pos (lat/lon): lat={uav_lat:9.5f}  lon={uav_lon:10.5f}",
        f"spd (m/s): {active_snap[6]:5.1f}   yaw={active_snap[5]:6.1f}deg  pitch={active_snap[4]:5.1f}deg  roll={active_snap[3]:5.1f}deg",
        f"FOV (diag): {app.state.fov_diag:4.1f}deg   LOS: {1 if los_clear else 0}   dt(step)={dt_sim*1000:.1f}ms  time x{time_scale_val:.1f}",
        f"Targets: {len(app.state.targets)}  Missiles: {len(app.state.missiles)}",
        "1-6: switch craft (1-3 LAH, 4-6 UAV) | M: Fire (LAH only) | N: Toggle targets | G: Spawn target | Wheel: Zoom | RMB drag: Orbit | MMB drag: Pan",
    ]
    if app.cpu_percent is not None:
        # Show up to first 8 cores to keep HUD compact.
        loads = " ".join(f"{p:3.0f}%" for p in app.cpu_percent[:8])
        lines.append(f"CPU cores: {app.cpu_count}  workers: {len(app.state.workers)}  load: {loads}")
    if footprint_area is not None:
        lines.append(f"Footprint area: {footprint_area:8.1f} m^2")
    if any(app.state.crippled) and not any(app.state.crashed):
        crippled_ids = [str(i + 1) for i, c in enumerate(app.state.crippled) if c]
        lines.append(f"Status: DAMAGED UAVs: {', '.join(crippled_ids)} (falling)")
    if any(app.state.crashed):
        crashed_ids = [str(i + 1) for i, c in enumerate(app.state.crashed) if c]
        lines.append(f"Status: CRASHED craft: {', '.join(crashed_ids)} (press R to reset)")
    draw_hud(lines)
    draw_minimap(app, snapshots, active_snap)

    if app.debug_frames < 3:
        print(
            f"[frame] cam_target=({app.cam.target[0]:.1f},{app.cam.target[1]:.1f},{app.cam.target[2]:.1f}) "
            f"cam_dist={app.cam.distance:.1f} uav_pos=({active_snap[0]:.1f},{active_snap[1]:.1f},{active_snap[2]:.1f}) "
            f"time_scale={time_scale_val:.1f} fog={app.state.fog_enabled}"
        )
        err = glGetError()
        if err:
            print(f"[GL ERROR] frame: {err}")
        app.debug_frames += 1

    pygame.display.flip()
    if app.debug_frames < 2:
        print("[sim-log] after display flip")
