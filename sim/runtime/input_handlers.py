from __future__ import annotations

import pygame

from sim.config import clamp
from .runtime_utils import fire_missile, reset_all_craft, spawn_target_near_camera


def apply_continuous_input(app, keys, dt):
    active_idx = app.state.active_idx
    uav = app.state.uavs[active_idx]
    is_lah = app.state.uav_types[active_idx] == "LAH"
    with app.state.uav_locks[active_idx]:
        if not app.state.crippled[active_idx]:
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
        app.state.fov_diag += 15.0 * dt
    if keys[pygame.K_d]:
        app.state.fov_diag -= 15.0 * dt
    app.state.fov_diag = clamp(app.state.fov_diag, 1.2, 31.2)
    if app.debug_frames < 1:
        print("[sim-log] after input/keys")


def process_events(app):
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            app.running = False
        elif e.type == pygame.KEYDOWN:
            handle_keydown(app, e.key)
        elif e.type == pygame.MOUSEBUTTONDOWN:
            handle_mouse_down(app, e)
        elif e.type == pygame.MOUSEBUTTONUP:
            handle_mouse_up(app, e)
        elif e.type == pygame.MOUSEMOTION:
            handle_mouse_motion(app, e)
    if app.debug_frames < 1:
        print("[sim-log] after events loop")


def handle_keydown(app, key):
    if key in (pygame.K_ESCAPE, pygame.K_q):
        app.running = False
    elif key == pygame.K_r:
        reset_all_craft(app)
    elif key == pygame.K_f:
        app.state.fog_enabled = not app.state.fog_enabled
    elif key == pygame.K_n:
        app.state.targets_move = not app.state.targets_move
    elif key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
        with app.state.time_lock:
            app.state.time_scale["value"] = clamp(app.state.time_scale["value"] + 1.0, 0.1, 100.0)
        ts = app.state.time_scale["value"]
        print(f"[time] scale -> {ts:.1f}x (dt~{app.dt_smoothed:.4f}s, dt_sim~{app.dt_smoothed * ts:.4f}s)")
    elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
        with app.state.time_lock:
            app.state.time_scale["value"] = clamp(app.state.time_scale["value"] - 1.0, 0.1, 100.0)
        ts = app.state.time_scale["value"]
        print(f"[time] scale -> {ts:.1f}x (dt~{app.dt_smoothed:.4f}s, dt_sim~{app.dt_smoothed * ts:.4f}s)")
    elif key == pygame.K_g:
        spawn_target_near_camera(app)
    elif key == pygame.K_m:
        fire_missile(app)
    elif key == pygame.K_o:
        app.state.threat_kill_disabled = True
        print("[threat] attacks disabled (kill switch 'O').")
    elif key == pygame.K_1:
        app.state.active_idx = 0
    elif key == pygame.K_2 and len(app.state.uavs) > 1:
        app.state.active_idx = 1
    elif key == pygame.K_3 and len(app.state.uavs) > 2:
        app.state.active_idx = 2
    elif key == pygame.K_4 and len(app.state.uavs) > 3:
        app.state.active_idx = 3
    elif key == pygame.K_5 and len(app.state.uavs) > 4:
        app.state.active_idx = 4
    elif key == pygame.K_6 and len(app.state.uavs) > 5:
        app.state.active_idx = 5


def handle_mouse_down(app, event):
    if event.button == 3:
        app.orbit = True
        app.last = event.pos
    elif event.button == 2:
        app.pan = True
        app.last = event.pos
    elif event.button == 4:
        app.cam.distance = max(50.0, app.cam.distance * 0.9)
    elif event.button == 5:
        app.cam.distance = min(8000.0, app.cam.distance * 1.1)


def handle_mouse_up(app, event):
    if event.button == 3:
        app.orbit = False
    elif event.button == 2:
        app.pan = False


def handle_mouse_motion(app, event):
    if app.orbit:
        dx = event.pos[0] - app.last[0]
        dy = event.pos[1] - app.last[1]
        app.cam.yaw += dx * 0.3
        app.cam.pitch = clamp(app.cam.pitch - dy * 0.3, -89.0, 89.0)
        app.last = event.pos
    elif app.pan:
        dx = event.pos[0] - app.last[0]
        dy = event.pos[1] - app.last[1]
        pan_scale = app.cam.distance * 0.002
        app.cam.target[0] -= dx * pan_scale
        app.cam.target[1] += dy * pan_scale
        app.last = event.pos
