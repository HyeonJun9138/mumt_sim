import math
from typing import List, Optional

import numpy as np
import pygame
from OpenGL.GL import (
    glBegin,
    glBindTexture,
    glBlendFunc,
    glClearColor,
    glColor3f,
    glColor4f,
    glDisable,
    glEnable,
    glEnd,
    glLineWidth,
    glLineStipple,
    glLoadIdentity,
    glMatrixMode,
    glOrtho,
    glPixelStorei,
    glPointSize,
    glPopMatrix,
    glPushMatrix,
    glRotatef,
    glScalef,
    glTexCoord2f,
    glTexImage2D,
    glTexParameteri,
    glTexSubImage2D,
    glTranslatef,
    glVertex2f,
    glVertex3f,
    glGenTextures,
    glDeleteTextures,
    GL_BLEND,
    GL_DEPTH_TEST,
    GL_LINES,
    GL_LINE_LOOP,
    GL_LINE_STRIP,
    GL_LINE_STIPPLE,
    GL_MODELVIEW,
    GL_NEAREST,
    GL_ONE,
    GL_ONE_MINUS_SRC_ALPHA,
    GL_POINTS,
    GL_PROJECTION,
    GL_QUADS,
    GL_RGBA,
    GL_SRC_ALPHA,
    GL_TEXTURE_2D,
    GL_TEXTURE_MAG_FILTER,
    GL_TEXTURE_MIN_FILTER,
    GL_TRIANGLE_STRIP,
    GL_TRIANGLES,
    GL_UNPACK_ALIGNMENT,
    GL_UNSIGNED_BYTE,
    glGetError,
    glGetString,
    GL_VENDOR,
    GL_RENDERER,
    GL_VERSION,
    GL_SHADING_LANGUAGE_VERSION,
)
from OpenGL.GLU import gluPerspective

from sim.core.camera import OrbitCamera
from sim.config import WIN_W, WIN_H, clamp
from sim.world.dem import DEM, ray_intersect_dem
from sim.core.uav import UAV
from sim.core.lah import LAH

_hud_font = None
_hud_tex_id = None
_hud_size = (0, 0)
_dem_debug_logged = False
_gl_info_logged = False
_dem_step_smoothed = 2.0
_dem_cache = {"center": None, "radius": None, "row_min": 0, "row_max": 0, "col_min": 0, "col_max": 0}
_dem_clamp_logged = False
_dem_step_cap_logged = False


def draw_rejoin_points(points, color=(0.9, 0.2, 0.2)):
    """Draw small markers for rejoin helper points."""
    if not points:
        return
    glPointSize(8.0)
    glColor3f(*color)
    glBegin(GL_POINTS)
    for p in points:
        glVertex3f(p[0], p[1], p[2])
    glEnd()
    glPointSize(1.0)

def draw_axes(length=50.0, width=2.0):
    glLineWidth(width)
    glBegin(GL_LINES)
    glColor3f(1, 0, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(length, 0, 0)
    glColor3f(0, 1, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, length, 0)
    glColor3f(0, 0, 1)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, length)
    glEnd()


def draw_grid(size=2000, step=100):
    glColor3f(0.22, 0.24, 0.28)
    glLineWidth(1.0)
    glBegin(GL_LINES)
    for i in range(-size, size + 1, step):
        glVertex3f(i, -size, 0)
        glVertex3f(i, size, 0)
        glVertex3f(-size, i, 0)
        glVertex3f(size, i, 0)
    glEnd()


def draw_uav_mesh():
    glColor3f(0.9, 0.9, 0.95)
    glBegin(GL_TRIANGLES)
    glVertex3f(8.0, 0, 0)
    glVertex3f(-3.2, 1.2, 0.4)
    glVertex3f(-3.2, -1.2, 0.4)
    glVertex3f(8.0, 0, 0)
    glVertex3f(-3.2, -1.2, -0.4)
    glVertex3f(-3.2, 1.2, -0.4)
    glEnd()
    glBegin(GL_QUADS)
    glVertex3f(-1.0, -6.0, 0.0)
    glVertex3f(3.0, -6.0, 0.0)
    glVertex3f(3.0, 6.0, 0.0)
    glVertex3f(-1.0, 6.0, 0.0)
    glVertex3f(-5.0, -3.0, 0.0)
    glVertex3f(-4.0, -3.0, 0.0)
    glVertex3f(-4.0, 3.0, 0.0)
    glVertex3f(-5.0, 3.0, 0.0)
    glEnd()
    glBegin(GL_TRIANGLES)
    glVertex3f(-5.0, 0.0, 0.0)
    glVertex3f(-3.8, 0.0, 2.2)
    glVertex3f(-3.8, 0.0, 0.0)
    glEnd()


def draw_body_axes(length=12.0, width=2.5):
    glLineWidth(width)
    glBegin(GL_LINES)
    glColor3f(1, 0.2, 0.2)
    glVertex3f(0, 0, 0)
    glVertex3f(length, 0, 0)
    glColor3f(0.2, 1, 0.2)
    glVertex3f(0, 0, 0)
    glVertex3f(0, -length, 0)
    glColor3f(0.2, 0.5, 1)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, -length)
    glEnd()


def draw_uav(uav: UAV):
    s = uav.s
    glPushMatrix()
    glTranslatef(s.x, s.y, s.z)
    glRotatef(-s.yaw, 0, 0, 1)
    glRotatef(-s.pitch, 0, 1, 0)
    glRotatef(s.roll, 1, 0, 0)
    glScalef(2.0, 2.0, 2.0)
    draw_uav_mesh()
    draw_body_axes()
    glPopMatrix()


def draw_lah_mesh(main_rot_deg: float = 0.0, tail_rot_deg: float = 0.0):
    glColor3f(0.8, 0.85, 0.9)
    glBegin(GL_TRIANGLES)
    glVertex3f(7.0, 0, 0.0)
    glVertex3f(-4.0, 1.5, 0.6)
    glVertex3f(-4.0, -1.5, 0.6)
    glVertex3f(7.0, 0, 0.0)
    glVertex3f(-4.0, -1.5, -0.6)
    glVertex3f(-4.0, 1.5, -0.6)
    glEnd()
    glBegin(GL_QUADS)
    # tail boom
    glVertex3f(-4.0, -0.6, 0.0)
    glVertex3f(-10.0, -0.6, 0.0)
    glVertex3f(-10.0, 0.6, 0.0)
    glVertex3f(-4.0, 0.6, 0.0)
    # skids
    glVertex3f(-2.0, -2.8, -0.8)
    glVertex3f(4.0, -2.8, -0.8)
    glVertex3f(4.0, -2.3, -0.8)
    glVertex3f(-2.0, -2.3, -0.8)
    glVertex3f(-2.0, 2.8, -0.8)
    glVertex3f(4.0, 2.8, -0.8)
    glVertex3f(4.0, 2.3, -0.8)
    glVertex3f(-2.0, 2.3, -0.8)
    glEnd()
    # main rotor (cross)
    glColor3f(0.2, 0.2, 0.2)
    glLineWidth(2.0)
    glPushMatrix()
    glTranslatef(0, 0, 1.2)
    glRotatef(main_rot_deg, 0, 0, 1)
    glBegin(GL_LINES)
    glVertex3f(-6.0, 0, 0.0)
    glVertex3f(6.0, 0, 0.0)
    glVertex3f(0, -6.0, 0.0)
    glVertex3f(0, 6.0, 0.0)
    glEnd()
    glPopMatrix()
    # tail rotor
    glPushMatrix()
    glTranslatef(-10.0, 0, 0.0)
    glRotatef(tail_rot_deg, 1, 0, 0)
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0.8)
    glVertex3f(0, 0, -0.8)
    glEnd()
    glPopMatrix()


def draw_lah(lah: LAH, rotor_angle: float):
    s = lah.s
    glPushMatrix()
    glTranslatef(s.x, s.y, s.z)
    glRotatef(-s.yaw, 0, 0, 1)
    glRotatef(-s.pitch, 0, 1, 0)
    glRotatef(s.roll, 1, 0, 0)
    glScalef(2.0, 2.0, 2.0)
    draw_lah_mesh(main_rot_deg=rotor_angle, tail_rot_deg=rotor_angle * 4.0)
    draw_body_axes()
    glPopMatrix()


def draw_dem(
    dem: DEM,
    cam: OrbitCamera,
    center_xy=(0.0, 0.0),
    radius_m=2000.0,
    prefetch_radius_m=None,
    move_threshold=500.0,
    z_scale=1.0,
):
    dem.ensure_tile_for_env(center_xy[0], center_xy[1])
    elev = dem.elevation
    rows, cols = elev.shape
    # Fixed sampling step to avoid visible changes across frames
    step = 10
    glColor3f(0.28, 0.53, 0.28)
    glLineWidth(1.0)

    cx, cy = center_xy
    use_radius = max(radius_m, prefetch_radius_m if prefetch_radius_m is not None else radius_m, 100.0)

    global _dem_cache, _dem_debug_logged
    reuse = False
    if _dem_cache["center"] is not None and _dem_cache["radius"] == use_radius:
        lx, ly = _dem_cache["center"]
        if (cx - lx) ** 2 + (cy - ly) ** 2 < move_threshold ** 2:
            reuse = True

    if reuse:
        row_min = _dem_cache["row_min"]
        row_max = _dem_cache["row_max"]
        col_min = _dem_cache["col_min"]
        col_max = _dem_cache["col_max"]
    else:
        corners = [
            (cx - use_radius, cy - use_radius),
            (cx - use_radius, cy + use_radius),
            (cx + use_radius, cy - use_radius),
            (cx + use_radius, cy + use_radius),
        ]
        idxs = [dem.env_to_dataset_xy(x, y) for x, y in corners]
        row_min = clamp(min(i[0] for i in idxs), 0, rows - 1)
        row_max = clamp(max(i[0] for i in idxs), 0, rows - 1)
        col_min = clamp(min(i[1] for i in idxs), 0, cols - 1)
        col_max = clamp(max(i[1] for i in idxs), 0, cols - 1)

        # Fallback to full render if the cropped window collapses (should be rare)
        if (row_max - row_min) < 2 or (col_max - col_min) < 2:
            if not _dem_debug_logged:
                print(
                    f"[draw_dem] window collapsed at ({cx:.1f},{cy:.1f}); skipping DEM draw instead of rendering full tile"
                )
                _dem_debug_logged = True
            return

        row_min, row_max = int(row_min), int(row_max)
        col_min, col_max = int(col_min), int(col_max)

        _dem_cache = {
            "center": (cx, cy),
            "radius": use_radius,
            "row_min": row_min,
            "row_max": row_max,
            "col_min": col_min,
            "col_max": col_max,
        }

    # Cap density so we never push absurd vertex counts (prevents sudden GPU stalls)
    row_span = row_max - row_min
    col_span = col_max - col_min
    max_vertices = 120_000
    est = max(1, (row_span // step) * (col_span // step))
    if est > max_vertices:
        step_cap = int(math.ceil(math.sqrt((row_span * col_span) / max_vertices)))
        step = max(step, step_cap)
        global _dem_step_cap_logged
        if not _dem_step_cap_logged:
            print(f"[draw_dem] step capped to {step} to keep vertices under {max_vertices}")
            _dem_step_cap_logged = True

    if not _dem_debug_logged:
        print(
            f"[draw_dem] rows {row_min}-{row_max} cols {col_min}-{col_max} step {step} elev {dem.min_elev:.1f}/{dem.max_elev:.1f}"
        )
        _dem_debug_logged = True

    err = glGetError()
    if err:
        print(f"[GL ERROR] during draw_dem: {err}")

    # Row-wise strips
    for r_idx in range(row_min, row_max, step):
        glBegin(GL_LINE_STRIP)
        for c_idx in range(col_min, col_max, step):
            lon, lat = dem.transform * (c_idx, r_idx)
            x_env, y_env = dem.lonlat_to_env(lon, lat)
            z_env = elev[r_idx, c_idx] * z_scale
            glVertex3f(x_env, y_env, z_env)
        glEnd()
    # Column-wise strips to restore crosshatch grid
    for c_idx in range(col_min, col_max, step):
        glBegin(GL_LINE_STRIP)
        for r_idx in range(row_min, row_max, step):
            lon, lat = dem.transform * (c_idx, r_idx)
            x_env, y_env = dem.lonlat_to_env(lon, lat)
            z_env = elev[r_idx, c_idx] * z_scale
            glVertex3f(x_env, y_env, z_env)
        glEnd()


def draw_camera_footprint(
    uav,
    target,
    fov_diag_deg,
    dem: DEM,
    aspect_ratio=16 / 9,
    color=(0.1, 0.6, 1.0),
    z_scale=1.0,
    draw: bool = True,
):
    uav_pos = np.array([uav.s.x, uav.s.y, uav.s.z], dtype=float)
    tgt_pos = np.array(target, dtype=float)
    forward = tgt_pos - uav_pos
    forward /= np.linalg.norm(forward)

    fov_diag = np.radians(fov_diag_deg)
    ar = aspect_ratio
    fov_h = 2 * np.arctan(np.tan(fov_diag / 2) * ar / np.sqrt(1 + ar ** 2))
    fov_v = 2 * np.arctan(np.tan(fov_diag / 2) * 1 / np.sqrt(1 + ar ** 2))

    world_up = np.array([0, 0, 1], dtype=float)
    right = np.cross(forward, world_up)
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1, 0, 0], dtype=float)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    up /= np.linalg.norm(up)

    tan_h, tan_v = np.tan(fov_h / 2), np.tan(fov_v / 2)
    dirs = []
    for sx in [-1, 1]:
        for sy in [-1, 1]:
            d = forward + sx * tan_h * right + sy * tan_v * up
            dirs.append(d / np.linalg.norm(d))

    corners = []
    for d in dirs:
        hit = ray_intersect_dem(uav_pos, d, dem)
        if hit is not None:
            corners.append(hit)

    if draw:
        glColor3f(1, 1, 0)
        glLineWidth(2.0)
        glEnable(GL_LINE_STIPPLE)
        glLineStipple(1, 0x0C0C)
        glBegin(GL_LINES)
        glVertex3f(*uav_pos)
        glVertex3f(*tgt_pos)
        glEnd()
        glDisable(GL_LINE_STIPPLE)

    area = None
    corners_sorted = None
    if len(corners) == 4:
        cx = np.mean([c[0] for c in corners])
        cy = np.mean([c[1] for c in corners])
        corners_sorted = sorted(corners, key=lambda c: math.atan2(c[1] - cy, c[0] - cx))
        if draw:
            glColor3f(*color)
            glLineWidth(2.0)
            glBegin(GL_LINE_LOOP)
            for c in corners_sorted:
                glVertex3f(c[0], c[1], c[2] * z_scale)
            glEnd()
        x = [c[0] for c in corners_sorted]
        y = [c[1] for c in corners_sorted]
        area = 0.5 * abs(sum(x[i] * y[(i + 1) % 4] - x[(i + 1) % 4] * y[i] for i in range(4)))
    return area, corners_sorted


def _init_font():
    global _hud_font
    if _hud_font is None:
        pygame.font.init()
        try:
            _hud_font = pygame.font.SysFont("Consolas", 15)
        except Exception:
            _hud_font = pygame.font.SysFont(None, 15)


def draw_hud(lines: List[str]):
    global _hud_tex_id, _hud_size
    _init_font()
    line_surfs = [_hud_font.render(t, True, (240, 240, 240)) for t in lines]
    if not line_surfs:
        return
    pad = 8
    w = max(s.get_width() for s in line_surfs) + pad * 2
    h = sum(s.get_height() for s in line_surfs) + pad * 2
    surf = pygame.Surface((w, h), pygame.SRCALPHA)
    y = pad
    for s in line_surfs:
        surf.blit(s, (pad, y))
        y += s.get_height()
    data = pygame.image.tostring(surf, "RGBA", True)
    tw, th = surf.get_width(), surf.get_height()

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
    if _hud_tex_id is None:
        _hud_tex_id = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, _hud_tex_id)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tw, th, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
        _hud_size = (tw, th)
    else:
        glBindTexture(GL_TEXTURE_2D, _hud_tex_id)
        if _hud_size != (tw, th):
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tw, th, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
            _hud_size = (tw, th)
        else:
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, tw, th, GL_RGBA, GL_UNSIGNED_BYTE, data)

    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    glOrtho(0, WIN_W, 0, WIN_H, -1, 1)
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_TEXTURE_2D)
    glBindTexture(GL_TEXTURE_2D, _hud_tex_id)

    x0, y0 = 10, 10
    x1, y1 = x0 + tw, y0 + th
    glColor4f(1, 1, 1, 1)
    glBegin(GL_QUADS)
    glTexCoord2f(0, 0)
    glVertex2f(x0, y0)
    glTexCoord2f(1, 0)
    glVertex2f(x1, y0)
    glTexCoord2f(1, 1)
    glVertex2f(x1, y1)
    glTexCoord2f(0, 1)
    glVertex2f(x0, y1)
    glEnd()

    glBindTexture(GL_TEXTURE_2D, 0)
    glDisable(GL_TEXTURE_2D)
    glDisable(GL_BLEND)
    glEnable(GL_DEPTH_TEST)
    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)


def draw_labels(labels: List[tuple]):
    """Draw small 2D labels at given screen coords. Each label: (x, y, text, color)."""
    if not labels:
        return
    _init_font()
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    glOrtho(0, WIN_W, 0, WIN_H, -1, 1)
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_TEXTURE_2D)

    for x, y, text, color in labels:
        surf = _hud_font.render(text, True, color)
        data = pygame.image.tostring(surf, "RGBA", True)
        tw, th = surf.get_size()
        tex_id = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, tex_id)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tw, th, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)

        x0, y0 = x, y
        x1, y1 = x0 + tw, y0 + th
        glColor4f(1, 1, 1, 1)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0)
        glVertex2f(x0, y0)
        glTexCoord2f(1, 0)
        glVertex2f(x1, y0)
        glTexCoord2f(1, 1)
        glVertex2f(x1, y1)
        glTexCoord2f(0, 1)
        glVertex2f(x0, y1)
        glEnd()

        glBindTexture(GL_TEXTURE_2D, 0)
        glDeleteTextures(int(tex_id))

    glDisable(GL_TEXTURE_2D)
    glDisable(GL_BLEND)
    glEnable(GL_DEPTH_TEST)
    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
