"""Simple tracking/orbit controller for a single UAV to follow a target."""

from __future__ import annotations

import math

from sim.config import clamp, wrap_deg


class TrackingController:
    def __init__(self, target_idx: int, orbit_radius: float = 390.0, target_speed: float = 70.0):
        self.target_idx = target_idx
        self.orbit_radius = orbit_radius
        self.target_speed = target_speed
        self.active = True

        self.KP_YAW = 1.4
        self.KP_PITCH = 0.5
        self.KP_ROLL = 1.8
        self.KP_THROTTLE = 0.25

    def update(self, uav, target, dem, dt: float) -> bool:
        """
        Returns False if tracking should end (target lost).
        """
        if target is None or not getattr(target, "alive", True):
            return False

        # Target position and ground altitude
        tx, ty, tz = target.x, target.y, target.z
        ground_z = dem.get_height(uav.s.x, uav.s.y)
        # Keep current altitude; just ensure a small safety margin above terrain.
        desired_z = max(uav.s.z, ground_z + 80.0)

        dx = tx - uav.s.x
        dy = ty - uav.s.y
        dist_2d = math.hypot(dx, dy)

        # Always point nose toward target (no orbit offset).
        bearing_deg = wrap_deg(math.degrees(math.atan2(-dy, dx)))
        desired_yaw = bearing_deg

        # Control: yaw to desired, roll to support turn, pitch/alt hold, speed hold.
        yaw_err = ((desired_yaw - uav.s.yaw + 540.0) % 360.0) - 180.0
        uav.cmd_yaw_rate = clamp(yaw_err * self.KP_YAW, -uav.p.max_yaw_rate_dps, uav.p.max_yaw_rate_dps)

        # Roll target based on yaw rate demand to tighten orbit a bit
        target_roll_deg = clamp(uav.cmd_yaw_rate * 0.8, -uav.p.roll_limit_deg * 0.7, uav.p.roll_limit_deg * 0.7)
        roll_err = target_roll_deg - uav.s.roll
        uav.cmd_roll_rate = clamp(
            roll_err * self.KP_ROLL, -uav.p.max_roll_rate_dps, uav.p.max_roll_rate_dps
        )

        alt_err = desired_z - uav.s.z
        uav.cmd_pitch_rate = clamp(
            alt_err * self.KP_PITCH, -uav.p.max_pitch_rate_dps, uav.p.max_pitch_rate_dps
        )

        speed_err = self.target_speed - uav.s.u
        uav.cmd_throttle = clamp(speed_err * self.KP_THROTTLE, -1.0, 1.0)

        return True
