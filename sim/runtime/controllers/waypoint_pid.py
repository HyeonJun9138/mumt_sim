from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

from sim.config import clamp, wrap_deg


@dataclass
class PIDGains:
    yaw: float = 0.9
    yaw_i: float = 0.0
    pitch_rate: float = 1.4
    pitch_damp: float = 1.0
    alt_pitch: float = 0.03
    alt_i: float = 0.0
    throttle: float = 0.1
    throttle_alt: float = 0.0008
    throttle_i: float = 0.0
    lookahead_m: float = 120.0
    freeze_yaw_dist: float = 50.0
    freeze_alt_ratio: float = 0.5


# Tuned gains taken from test/pid_tuned_gains.json (kept here so we do not rely on test assets at runtime).
DEFAULT_TUNED_GAINS = PIDGains(
    yaw=0.7034928181628536,
    yaw_i=0.024126824285276113,
    pitch_rate=3.2,
    pitch_damp=0.2,
    alt_pitch=0.09,
    alt_i=0.2,
    throttle=0.02,
    throttle_alt=0.0025000000000000005,
    throttle_i=0.026169406198635482,
    lookahead_m=196.375,
    freeze_yaw_dist=62.604277298027895,
    freeze_alt_ratio=0.5,
)


@dataclass
class WaypointTarget:
    pos: Tuple[float, float, float]
    speed: float | None = None
    filming: dict | None = None
    wp_id: int | None = None
    hover_time: float | None = None
    loiter: dict | None = None


def load_pid_gains(path: str | Path | None = None, fallback: PIDGains | None = None) -> PIDGains:
    """
    Load PID gains from a JSON file. Falls back to tuned defaults if the file is missing or malformed.
    Accepts either a flat dict or {"gains": {...}} format.
    """
    fallback = DEFAULT_TUNED_GAINS if fallback is None else fallback
    if path is None:
        return fallback

    p = Path(path)
    if not p.exists():
        return fallback

    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        if isinstance(data, dict) and "gains" in data and isinstance(data["gains"], dict):
            data = data["gains"]
        if not isinstance(data, dict):
            return fallback

        # Merge with fallback so missing keys still get sane defaults.
        merged = {**fallback.__dict__, **{k: data[k] for k in PIDGains.__dataclass_fields__ if k in data}}
        return PIDGains(**merged)
    except Exception as e:  # pragma: no cover - defensive log
        print(f"[pid] failed to load gains from {p}: {e}")
        return fallback


class WaypointPIDController:
    """
    Lightweight PID-based waypoint follower for the simple UAV model.
    Updates uav.cmd_* fields directly; call update(dt) each frame.
    """

    def __init__(
        self,
        uav,
        waypoints: Sequence[WaypointTarget] | Iterable[Tuple[float, float, float]],
        *,
        gains: PIDGains | None = None,
        speed_target: float = 90.0,
        pos_tol: float = 30.0,
        name: str | None = None,
        ground_clearance: float = 60.0,
        allow_hover: bool = False,
    ):
        self.uav = uav
        self.gains = gains or DEFAULT_TUNED_GAINS
        self.speed_target = speed_target
        self.pos_tol = pos_tol
        self.name = name or "uav"
        self.ground_clearance = ground_clearance
        self.allow_hover = allow_hover

        self.targets: List[WaypointTarget] = []
        for wp in waypoints:
            if isinstance(wp, WaypointTarget):
                self.targets.append(wp)
            elif isinstance(wp, dict) and "pos" in wp:
                pos = wp["pos"]
                filming = wp.get("filming")
                wp_id = wp.get("wp_id")
                hover_time = wp.get("hover_time")
                loiter = wp.get("loiter")
                if isinstance(pos, (list, tuple)) and len(pos) == 3:
                    self.targets.append(
                        WaypointTarget(
                            pos=(float(pos[0]), float(pos[1]), float(pos[2])),
                            speed=wp.get("speed"),
                            filming=filming,
                            wp_id=int(wp_id) if wp_id is not None else None,
                            hover_time=float(hover_time) if hover_time is not None else None,
                            loiter=loiter,
                        )
                    )
            elif isinstance(wp, (list, tuple)) and len(wp) == 3:
                self.targets.append(WaypointTarget(pos=(float(wp[0]), float(wp[1]), float(wp[2]))))

        self.curr_idx = 0
        self.finished = False

        # Integrator state
        self.yaw_int = 0.0
        self.alt_int = 0.0
        self.speed_int = 0.0
        self.yaw_int_max = 100.0
        self.alt_int_max = 10.0
        self.speed_int_max = 10.0
        self.hover_timer = 0.0
        self.is_hovering = False
        self.is_loitering = False
        self.loiter_timer = 0.0
        self.loiter_center = (0.0, 0.0, 0.0)
        self.loiter_radius = 0.0
        self.loiter_speed = 0.0
        self.loiter_dir = 1.0
        self.loiter_angle = 0.0

    def _heading_to_target(self, dx: float, dy: float) -> float:
        return wrap_deg(math.degrees(math.atan2(-dy, dx)))

    def _advance_wp(self):
        self.curr_idx += 1
        self.yaw_int = 0.0
        self.alt_int = 0.0
        self.speed_int = 0.0
        self.is_hovering = False
        self.hover_timer = 0.0
        self.is_loitering = False
        self.loiter_timer = 0.0
        if self.curr_idx >= len(self.targets):
            self.finished = True
            self._apply_hold()
            print(f"[pid-autopilot:{self.name}] waypoint mission complete.")

    def _apply_hold(self):
        """Level and gently slow down when there is nothing left to fly."""
        self.uav.cmd_yaw_rate = 0.0
        self.uav.cmd_pitch_rate = -self.uav.s.pitch * 1.0
        self.uav.cmd_roll_rate = -self.uav.s.roll * 1.5
        self.uav.cmd_throttle = -0.2

    def current_target(self) -> WaypointTarget | None:
        if self.finished or not self.targets:
            return None
        if self.curr_idx >= len(self.targets):
            return None
        return self.targets[self.curr_idx]

    def update(self, dt: float, dem=None, wall_dt: float | None = None) -> bool:
        if self.finished or not self.targets:
            return False

        # wall_dt is real-world seconds (unscaled); if not provided, fall back to dt.
        if wall_dt is None:
            wall_dt = dt

        dt = clamp(dt, 0.001, 0.05)
        if self.curr_idx >= len(self.targets):
            self.finished = True
            self._apply_hold()
            return False

        target = self.targets[self.curr_idx]
        tx, ty, tz = target.pos

        # Keep target altitude above terrain for safety.
        if dem is not None:
            try:
                ground_at_target = dem.get_height(tx, ty)
                tz = max(tz, ground_at_target + self.ground_clearance)
            except Exception:
                pass

        dx = tx - self.uav.s.x
        dy = ty - self.uav.s.y
        dz = tz - self.uav.s.z
        dist_xy = math.hypot(dx, dy)

        uav = self.uav
        gains = self.gains

        target_hover = float(target.hover_time) if (self.allow_hover and target and target.hover_time) else 0.0
        # If already hovering (only for allow_hover airframes like LAH), keep decrementing timer even if we drift slightly.
        if self.allow_hover and self.is_hovering and target_hover > 0.0:
            self.hover_timer = max(0.0, self.hover_timer - wall_dt)
            if self.hover_timer <= 0.0:
                self.is_hovering = False
                self._advance_wp()
                return not self.finished
            # Hold attitude/altitude while hovering.
            self.uav.s.u = 0.0  # force zero forward speed during hover
            uav.cmd_throttle = 0.0  # keep forward accel neutral
            uav.cmd_yaw_rate = clamp(-uav.s.r * 0.5, -uav.p.max_yaw_rate_dps, uav.p.max_yaw_rate_dps)
            alt_err = (tz - uav.s.z)
            uav.cmd_pitch_rate = clamp(alt_err * gains.pitch_rate, -uav.p.max_pitch_rate_dps, uav.p.max_pitch_rate_dps)
            uav.cmd_roll_rate = clamp(-uav.s.roll * 1.5, -uav.p.max_roll_rate_dps, uav.p.max_roll_rate_dps)
            return True

        # If already loitering, steer to moving point on the loiter circle.
        if self.is_loitering:
            self.loiter_timer = max(0.0, self.loiter_timer - wall_dt)
            if self.loiter_timer <= 0.0:
                self.is_loitering = False
                self._advance_wp()
                return not self.finished
            radius = max(1.0, self.loiter_radius)
            speed = max(0.0, self.loiter_speed if self.loiter_speed > 0 else (target.speed or self.speed_target))
            ang_rate = speed / radius
            self.loiter_angle += self.loiter_dir * ang_rate * dt
            tx = self.loiter_center[0] + radius * math.cos(self.loiter_angle)
            ty = self.loiter_center[1] + radius * math.sin(self.loiter_angle)
            tz = self.loiter_center[2]
            dx = tx - uav.s.x
            dy = ty - uav.s.y
            dz = tz - uav.s.z
            dist_xy = math.hypot(dx, dy)

        if not self.is_loitering and dist_xy < self.pos_tol and abs(dz) < self.pos_tol * 0.6:
            loiter_prop = target.loiter if isinstance(target.loiter, dict) else None
            loiter_time = 0.0
            if loiter_prop:
                try:
                    loiter_time = float(loiter_prop.get("time", 0.0) or 0.0)
                except Exception:
                    loiter_time = 0.0
            if loiter_time > 0.0 and loiter_prop:
                # Start loiter: set circle params and reuse main control loop.
                self.is_loitering = True
                self.loiter_timer = loiter_time
                self.loiter_center = (tx, ty, tz)
                try:
                    self.loiter_radius = float(loiter_prop.get("radius", 0.0) or 0.0)
                except Exception:
                    self.loiter_radius = 0.0
                try:
                    self.loiter_speed = float(loiter_prop.get("speed", 0.0) or (target.speed or self.speed_target))
                except Exception:
                    self.loiter_speed = target.speed or self.speed_target
                direction = loiter_prop.get("direction", 1)
                self.loiter_dir = -1.0 if direction == 1 else 1.0
                # Initialize angle from current heading to center to avoid jump.
                self.loiter_angle = math.atan2(uav.s.y - ty, uav.s.x - tx)
                # Immediately compute first loiter target point.
                radius = max(1.0, self.loiter_radius)
                speed = max(0.0, self.loiter_speed if self.loiter_speed > 0 else (target.speed or self.speed_target))
                ang_rate = speed / radius
                self.loiter_angle += self.loiter_dir * ang_rate * dt
                tx = self.loiter_center[0] + radius * math.cos(self.loiter_angle)
                ty = self.loiter_center[1] + radius * math.sin(self.loiter_angle)
                tz = self.loiter_center[2]
                dx = tx - uav.s.x
                dy = ty - uav.s.y
                dz = tz - uav.s.z
                dist_xy = math.hypot(dx, dy)
            if self.allow_hover and target_hover > 0.0:
                if not self.is_hovering:
                    self.hover_timer = target_hover
                    self.is_hovering = True
                else:
                    self.hover_timer = max(0.0, self.hover_timer - wall_dt)
                if self.hover_timer <= 0.0:
                    # Hover time elapsed; advance to next WP.
                    self.is_hovering = False
                    self._advance_wp()
                    return not self.finished
                # Hold attitude/altitude while hovering.
                self.uav.s.u = 0.0  # force zero forward speed during hover
                uav.cmd_throttle = 0.0  # keep forward accel neutral
                uav.cmd_yaw_rate = clamp(-uav.s.r * 0.5, -uav.p.max_yaw_rate_dps, uav.p.max_yaw_rate_dps)
                alt_err = (tz - uav.s.z)
                uav.cmd_pitch_rate = clamp(alt_err * gains.pitch_rate, -uav.p.max_pitch_rate_dps, uav.p.max_pitch_rate_dps)
                uav.cmd_roll_rate = clamp(-uav.s.roll * 1.5, -uav.p.max_roll_rate_dps, uav.p.max_roll_rate_dps)
                return True
            self._advance_wp()
            return not self.finished

        # Lookahead for smoother turns
        if dist_xy > 1e-3 and gains.lookahead_m > 0.0:
            lx = uav.s.x + dx / dist_xy * gains.lookahead_m
            ly = uav.s.y + dy / dist_xy * gains.lookahead_m
            desired_yaw = self._heading_to_target(lx - uav.s.x, ly - uav.s.y)
        else:
            desired_yaw = self._heading_to_target(dx, dy)

        freeze_yaw = dist_xy < gains.freeze_yaw_dist and abs(dz) > self.pos_tol * gains.freeze_alt_ratio
        if freeze_yaw:
            uav.cmd_yaw_rate = clamp(-uav.s.r * 0.5, -uav.p.max_yaw_rate_dps, uav.p.max_yaw_rate_dps)
        else:
            yaw_err = ((desired_yaw - uav.s.yaw + 540.0) % 360.0) - 180.0
            yaw_err = clamp(yaw_err, -60.0, 60.0)
            self.yaw_int = clamp(self.yaw_int + yaw_err * dt, -self.yaw_int_max, self.yaw_int_max)
            uav.cmd_yaw_rate = clamp(
                yaw_err * gains.yaw + self.yaw_int * gains.yaw_i,
                -uav.p.max_yaw_rate_dps,
                uav.p.max_yaw_rate_dps,
            )

        # Altitude -> pitch target -> pitch-rate loop
        desired_pitch = clamp(
            dz * gains.alt_pitch,
            -uav.p.pitch_limit_deg * 0.7,
            uav.p.pitch_limit_deg * 0.7,
        )
        self.alt_int = clamp(self.alt_int + dz * dt, -self.alt_int_max, self.alt_int_max)
        pitch_err = desired_pitch + self.alt_int * gains.alt_i - uav.s.pitch
        pitch_cmd = pitch_err * gains.pitch_rate - uav.s.q * gains.pitch_damp
        uav.cmd_pitch_rate = clamp(pitch_cmd, -uav.p.max_pitch_rate_dps, uav.p.max_pitch_rate_dps)

        # Roll damping
        uav.cmd_roll_rate = clamp(-uav.s.roll * 1.5, -uav.p.max_roll_rate_dps, uav.p.max_roll_rate_dps)

        # Throttle speed hold + altitude bias
        target_speed = target.speed if target.speed is not None else self.speed_target
        local_speed_target = target_speed * clamp(dist_xy / 300.0, 0.5, 1.0)
        speed_err = local_speed_target - uav.s.u
        self.speed_int = clamp(self.speed_int + speed_err * dt, -self.speed_int_max, self.speed_int_max)
        alt_bias = dz * gains.throttle_alt
        uav.cmd_throttle = clamp(
            speed_err * gains.throttle + self.speed_int * gains.throttle_i + alt_bias,
            -1.0,
            1.0,
        )

        return True
