"""
Static presets for the auto-mission generation area and start reference points.

Kept in a dedicated package so other generators can import without hardcoding.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Tuple


@dataclass(frozen=True)
class LatLon:
    latitude: float
    longitude: float

    def as_tuple(self) -> Tuple[float, float]:
        return self.latitude, self.longitude

    def to_dict(self) -> dict:
        return {"latitude": self.latitude, "longitude": self.longitude}


@dataclass(frozen=True)
class ScenarioArea:
    """Axis-aligned square/rectangle defined by southwest and northeast corners."""

    southwest: LatLon
    northeast: LatLon

    @property
    def corners(self) -> Tuple[LatLon, LatLon, LatLon, LatLon]:
        sw = self.southwest
        ne = self.northeast
        nw = LatLon(latitude=ne.latitude, longitude=sw.longitude)
        se = LatLon(latitude=sw.latitude, longitude=ne.longitude)
        return (sw, nw, ne, se)

    def to_area_lat_lon_list(self) -> List[dict]:
        """Return list of dicts suitable for areaLatLonList serialization."""
        return [pt.to_dict() for pt in self.corners]


# Default auto-mission generation area (square) from the spec sheet.
AUTO_MISSION_AREA = ScenarioArea(
    # southwest=LatLon(latitude=38.0737530, longitude=127.2508390), # 3배
    # northeast=LatLon(latitude=38.1837900, longitude=127.3851820), # 3배
    southwest=LatLon(latitude=38.0370740, longitude=127.2060580), # 5배
    northeast=LatLon(latitude=38.2204690, longitude=127.4299630), # 5배

)

# Pre-seeded start reference points used when selecting TakeOver locations.
# 외곽 중심으로 분포 (5배 확장 영역 경계 부근)
START_REFERENCE_POINTS: Tuple[LatLon, ...] = (
    # 서쪽 라인 (남→북)
    LatLon(38.0450000, 127.2120000),
    LatLon(38.1000000, 127.2120000),
    LatLon(38.1550000, 127.2120000),
    LatLon(38.2100000, 127.2120000),
    # 동쪽 라인 (남→북)
    LatLon(38.0450000, 127.4240000),
    LatLon(38.1000000, 127.4240000),
    LatLon(38.1550000, 127.4240000),
    LatLon(38.2100000, 127.4240000),
    # 남쪽 라인 (서→동)
    LatLon(38.0420000, 127.2300000),
    LatLon(38.0420000, 127.3200000),
    LatLon(38.0420000, 127.4100000),
    # 북쪽 라인 (서→동)
    LatLon(38.2150000, 127.2300000),
    LatLon(38.2150000, 127.3200000),
    LatLon(38.2150000, 127.4100000),
)


def all_start_points_dicts() -> List[dict]:
    return [p.to_dict() for p in START_REFERENCE_POINTS]


def iter_start_points(shuffle: bool = False, rng=None) -> Iterable[LatLon]:
    """
    Yield start reference points. Optionally shuffle with a provided RNG.

    Args:
        shuffle: If True, iterate in random order.
        rng: Optional random.Random instance for deterministic shuffling.
    """
    if not shuffle:
        return iter(START_REFERENCE_POINTS)
    import random

    seq: List[LatLon] = list(START_REFERENCE_POINTS)
    (rng or random).shuffle(seq)
    return iter(seq)
