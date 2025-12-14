"""
Air-Defense Threat Model · Detection & Kill Probability
Based on '대공위협 모델링.pdf'
"""
from __future__ import annotations
from dataclasses import dataclass, field
import math
import random
from enum import Enum, auto


class WeaponType(Enum):
    GUN = auto()
    MISSILE = auto()


@dataclass
class RadarParams:
    P_fa: float = 1e-6
    a: float = 1.0
    sigma: float = 1.0
    n: float = 1.5
    t_ref: float = 5.0  # [s]


@dataclass
class WeaponParams:
    a_range: float = 3500.0  # max range [m]
    b_slope: float = 2.0
    omega: float = 0.8
    weapon_type: WeaponType = WeaponType.GUN
    t_fire: float = 3.0  # missile lock-on → fire
    reload: float = 6.0  # reload time


@dataclass
class ThreatState:
    detected: bool = False
    t_exposed: float = 0.0  # accumulated exposure time
    ammo: int | None = None  # None -> infinite


@dataclass
class AirDefenseThreat:
    radar: RadarParams
    weapon: WeaponParams
    state: ThreatState = field(default_factory=ThreatState)

    def detection_prob(self, range_m: float, los: bool, dt: float) -> float:
        """Calculate detection probability; updates detection state."""
        if not los:
            self.state.detected = False
            self.state.t_exposed = 0.0
            return 0.0

        self.state.t_exposed += dt
        x = self.radar.n * (self.state.t_exposed / self.radar.t_ref - 1.0)
        x = min(x, 80.0)
        f_texposed = math.exp(x)

        pd = (f_texposed * self.radar.P_fa) / (self.radar.a * self.radar.sigma + range_m ** 4)
        pd = max(0.0, min(pd, 1.0))

        self.state.detected = (pd >= 1.0) or (random.random() < pd)
        return pd

    def kill_prob(self, range_m: float, dt: float) -> float:
        """Kill probability; assumes detection already true."""
        if not self.state.detected:
            return 0.0

        if self.weapon.weapon_type is WeaponType.MISSILE:
            if self.state.t_exposed < self.weapon.t_fire:
                return 0.0
            if self.state.ammo == 0:
                return 0.0
            if self.state.ammo is not None:
                self.state.ammo -= 1

        pn = 1.0 / (1.0 + (range_m / self.weapon.a_range) ** (2 * self.weapon.b_slope))
        pk = pn * self.weapon.omega * dt / self.weapon.t_fire
        pk = max(0.0, min(pk, 1.0))
        return pk
