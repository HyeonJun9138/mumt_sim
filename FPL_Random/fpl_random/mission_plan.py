from __future__ import annotations

"""
InputMissionPlan generator (line -> area -> line) with simple geometric constraints.

Key rules:
- timestamp is ms since 2000-01-01 UTC.
- inputMissionPackageID increments from .mission_plan_state.json; filenames use LAHMUMT-IMP-####.json.
- inputMissionPackageType is random 1~5.
- mainSensor uses EO/IR weights; defaults to EO only (adjust MAIN_SENSOR_WEIGHTS to restore 80/20).
- availableAircraftList always includes manned 1,2,3 and unmanned 4,5,6.
- Missions are generated as [line, area, line]. Each line's polyline is built from 0.5~2 km legs with heading change limits and no overlaps with other missions.
- Distances use area edges (not centers): line end to area edge and area edge to next line start are kept between EDGE_GAP_MIN_M and EDGE_GAP_MAX_M. Lines and areas avoid intersecting or overlapping.
"""

import json
import math
import random
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

if __name__ == "__main__" and __package__ is None:
    # Allow running as a script without installing the package.
    import sys

    sys.path.append(str(Path(__file__).resolve().parent.parent))
    __package__ = "fpl_random"

from .areas import AUTO_MISSION_AREA, LatLon
from . import paths
from .utils import now_ms_2000, offset_lat_lon

PROJECT_ROOT = Path(__file__).resolve().parent.parent
FILE_PREFIX = "LAHMUMT-IMP-"

# Tunables
# 선 임무 구간 길이 (한 레그 당)
LINE_SEGMENT_MIN_M = 500.0       # 최소(m)
LINE_SEGMENT_MAX_M = 2000.0      # 최대(m)
LINE_POINT_COUNT_RANGE = (3, 6)  # 선 임무 경로 점 개수 범위

# 기체당 통로 폭: 200~500m (50m 스텝). UAV 수에 비례.
PER_AIRCRAFT_WIDTH_MIN_M = 200.0
PER_AIRCRAFT_WIDTH_MAX_M = 500.0
WIDTH_STEP_M = 50.0

# 면 임무(사각형) 한 변 길이 범위 (더 작은 면도 허용)
AREA_SIDE_MIN_M = 1000.0
AREA_SIDE_MAX_M = 2000.0

# 선/면 간 최소·최대 이격 (값이 클수록 더 멀리 떨어짐)
EDGE_GAP_MIN_M = 700.0
EDGE_GAP_MAX_M = 1600.0

BORDER_MARGIN_M = 400.0    # 자동 임무 영역 경계에서 안쪽으로 확보할 여유(m)
MAX_GEN_ATTEMPTS = 120     # 전체 미션 생성 재시도 횟수
SEGMENT_ATTEMPTS = 120     # 선 구간(레이그) 생성 재시도 횟수
RECT_ATTEMPTS = 160        # 면/출입점 생성 재시도 횟수

HEADING_DELTA_MAX_DEG = 30.0     # 선 구간 사이 최대 회전각(±). 값 감소→급회전 억제
CONTINUE_HEADING_MAX_DEG = 30.0  # 선·출구 등 진행 방향 일관성 허용치
FORWARD_ALIGN_DEG = 30.0         # 면 배치가 현재 진행 방향과 벌어질 수 있는 최대각(±)

# TakeOver 기준 첫 선 시작점 반경 (이보다 가까우면 배치 안 함)
START_OFFSET_MIN_M = 1500.0
START_OFFSET_MAX_M = 2500.0

# Adjust EO/IR probabilities here. Default = EO only; set to {1: 0.8, 2: 0.2} for 80/20 EO/IR.
MAIN_SENSOR_WEIGHTS = {1: 1.0, 2: 0.0}

# Manned + unmanned pool (availableAircraftList). Width scaling uses UAV count only.
AIRCRAFT_IDS = (1, 2, 3, 4, 5, 6)
UAV_IDS = (4, 5, 6)

# Local projection reference to keep distance math stable.
REF_LAT = (AUTO_MISSION_AREA.southwest.latitude + AUTO_MISSION_AREA.northeast.latitude) / 2.0
REF_LON = (AUTO_MISSION_AREA.southwest.longitude + AUTO_MISSION_AREA.northeast.longitude) / 2.0


def _db_dir() -> Path:
    return paths.db_root() / "InputMissionPlan"


def _bounds_with_margin(margin_m: float) -> Tuple[float, float, float, float]:
    sw_lat, sw_lon = offset_lat_lon(AUTO_MISSION_AREA.southwest.latitude, AUTO_MISSION_AREA.southwest.longitude, margin_m, margin_m)
    ne_lat, ne_lon = offset_lat_lon(AUTO_MISSION_AREA.northeast.latitude, AUTO_MISSION_AREA.northeast.longitude, -margin_m, -margin_m)
    return sw_lat, sw_lon, ne_lat, ne_lon


def _inside_bounds(pt: LatLon, bounds: Tuple[float, float, float, float]) -> bool:
    sw_lat, sw_lon, ne_lat, ne_lon = bounds
    return sw_lat <= pt.latitude <= ne_lat and sw_lon <= pt.longitude <= ne_lon


def _to_xy(pt: LatLon) -> Tuple[float, float]:
    east = (pt.longitude - REF_LON) * 111_320.0 * math.cos(math.radians(REF_LAT))
    north = (pt.latitude - REF_LAT) * 111_320.0
    return east, north


def _distance_m(p1: LatLon, p2: LatLon) -> float:
    e1, n1 = _to_xy(p1)
    e2, n2 = _to_xy(p2)
    return math.hypot(e2 - e1, n2 - n1)


def _bearing_deg(p1: LatLon, p2: LatLon) -> float:
    e1, n1 = _to_xy(p1)
    e2, n2 = _to_xy(p2)
    ang = math.degrees(math.atan2(e2 - e1, n2 - n1))
    return (ang + 360.0) % 360.0


def _move(point: LatLon, distance_m: float, bearing_deg: float) -> LatLon:
    rad = math.radians(bearing_deg)
    east = distance_m * math.sin(rad)
    north = distance_m * math.cos(rad)
    lat, lon = offset_lat_lon(point.latitude, point.longitude, east, north)
    return LatLon(lat, lon)


def _rect_inside_bounds(center: LatLon, width_m: float, height_m: float, bounds: Tuple[float, float, float, float]) -> bool:
    half_w = width_m / 2.0
    half_h = height_m / 2.0
    sw_lat, sw_lon = offset_lat_lon(center.latitude, center.longitude, -half_w, -half_h)
    ne_lat, ne_lon = offset_lat_lon(center.latitude, center.longitude, half_w, half_h)
    swb_lat, swb_lon, neb_lat, neb_lon = bounds
    return swb_lat <= sw_lat and swb_lon <= sw_lon and ne_lat <= neb_lat and ne_lon <= neb_lon


def _rect_corners(center: LatLon, width_m: float, height_m: float) -> List[LatLon]:
    half_w = width_m / 2.0
    half_h = height_m / 2.0
    offsets = (
        (-half_w, -half_h),
        (-half_w, half_h),
        (half_w, half_h),
        (half_w, -half_h),
    )
    corners: List[LatLon] = []
    for east, north in offsets:
        lat, lon = offset_lat_lon(center.latitude, center.longitude, east, north)
        corners.append(LatLon(lat, lon))
    return corners


def _segments_from_points(points: Sequence[LatLon]) -> List[Tuple[LatLon, LatLon]]:
    return [(points[i], points[i + 1]) for i in range(len(points) - 1)]


def _rect_edges(corners: Sequence[LatLon]) -> List[Tuple[LatLon, LatLon]]:
    return list(zip(corners, corners[1:] + corners[:1]))


def _heading_delta(a: float, b: float) -> float:
    """Smallest absolute difference between two bearings in degrees."""
    return abs(((b - a + 180.0) % 360.0) - 180.0)


def _quadrant_from_heading(heading: float) -> str:
    """Map heading to one of N/E/S/W quadrants."""
    h = heading % 360.0
    if 45.0 <= h < 135.0:
        return "E"
    if 135.0 <= h < 225.0:
        return "S"
    if 225.0 <= h < 315.0:
        return "W"
    return "N"


def _orient(a: LatLon, b: LatLon, c: LatLon) -> float:
    ae, an = _to_xy(a)
    be, bn = _to_xy(b)
    ce, cn = _to_xy(c)
    return (be - ae) * (cn - an) - (bn - an) * (ce - ae)


def _segments_intersect(a1: LatLon, a2: LatLon, b1: LatLon, b2: LatLon) -> bool:
    o1 = _orient(a1, a2, b1)
    o2 = _orient(a1, a2, b2)
    o3 = _orient(b1, b2, a1)
    o4 = _orient(b1, b2, a2)
    if (o1 == 0 and _point_on_segment(b1, a1, a2)) or (o2 == 0 and _point_on_segment(b2, a1, a2)):
        return True
    if (o3 == 0 and _point_on_segment(a1, b1, b2)) or (o4 == 0 and _point_on_segment(a2, b1, b2)):
        return True
    return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)


def _point_on_segment(p: LatLon, a: LatLon, b: LatLon) -> bool:
    e, n = _to_xy(p)
    ae, an = _to_xy(a)
    be, bn = _to_xy(b)
    return min(ae, be) - 1e-6 <= e <= max(ae, be) + 1e-6 and min(an, bn) - 1e-6 <= n <= max(an, bn) + 1e-6 and abs(_orient(a, b, p)) < 1e-6


def _point_segment_distance_m(p: LatLon, a: LatLon, b: LatLon) -> float:
    pe, pn = _to_xy(p)
    ae, an = _to_xy(a)
    be, bn = _to_xy(b)
    ab_e = be - ae
    ab_n = bn - an
    ab2 = ab_e * ab_e + ab_n * ab_n
    if ab2 == 0.0:
        return math.hypot(pe - ae, pn - an)
    t = ((pe - ae) * ab_e + (pn - an) * ab_n) / ab2
    t = max(0.0, min(1.0, t))
    proj_e = ae + ab_e * t
    proj_n = an + ab_n * t
    return math.hypot(pe - proj_e, pn - proj_n)


def _segment_distance_m(a1: LatLon, a2: LatLon, b1: LatLon, b2: LatLon) -> float:
    if _segments_intersect(a1, a2, b1, b2):
        return 0.0
    return min(
        _point_segment_distance_m(a1, b1, b2),
        _point_segment_distance_m(a2, b1, b2),
        _point_segment_distance_m(b1, a1, a2),
        _point_segment_distance_m(b2, a1, a2),
    )


def _segment_intersects_list(seg: Tuple[LatLon, LatLon], others: Sequence[Tuple[LatLon, LatLon]], skip_shared_endpoint: bool = False) -> bool:
    for o1, o2 in others:
        if skip_shared_endpoint and _shares_endpoint(seg, (o1, o2)):
            continue
        if _segments_intersect(seg[0], seg[1], o1, o2):
            return True
    return False


def _segment_min_distance(seg: Tuple[LatLon, LatLon], others: Sequence[Tuple[LatLon, LatLon]], skip_shared_endpoint: bool = False) -> float:
    mind = float("inf")
    for o1, o2 in others:
        if skip_shared_endpoint and _shares_endpoint(seg, (o1, o2)):
            continue
        mind = min(mind, _segment_distance_m(seg[0], seg[1], o1, o2))
        if mind == 0.0:
            return 0.0
    return mind


def _shares_endpoint(seg1: Tuple[LatLon, LatLon], seg2: Tuple[LatLon, LatLon], tol_m: float = 0.1) -> bool:
    return (
        _distance_m(seg1[0], seg2[0]) < tol_m
        or _distance_m(seg1[0], seg2[1]) < tol_m
        or _distance_m(seg1[1], seg2[0]) < tol_m
        or _distance_m(seg1[1], seg2[1]) < tol_m
    )


def _min_distance_polyline_edges(points: Sequence[LatLon], edges: Sequence[Tuple[LatLon, LatLon]]) -> float:
    mind = float("inf")
    for s in _segments_from_points(points):
        mind = min(mind, _segment_min_distance(s, edges))
        if mind == 0.0:
            return 0.0
    return mind


def _min_distance_point_edges(pt: LatLon, edges: Sequence[Tuple[LatLon, LatLon]]) -> float:
    return min(_point_segment_distance_m(pt, e1, e2) for e1, e2 in edges)


def _min_distance_point_polyline(pt: LatLon, points: Sequence[LatLon]) -> float:
    return min(_point_segment_distance_m(pt, s1, s2) for s1, s2 in _segments_from_points(points))


def _polyline_intersects_segments(points: Sequence[LatLon], segments: Sequence[Tuple[LatLon, LatLon]]) -> bool:
    for s in _segments_from_points(points):
        if _segment_intersects_list(s, segments):
            return True
    return False


def _pick_main_sensor(rng: random.Random) -> int:
    weights = [(sensor, weight) for sensor, weight in MAIN_SENSOR_WEIGHTS.items() if weight > 0]
    if not weights:
        return 1
    total = sum(weight for _, weight in weights)
    pick = rng.uniform(0, total)
    upto = 0.0
    for sensor, weight in weights:
        upto += weight
        if pick <= upto:
            return sensor
    return weights[-1][0]


def _line_width(rng: random.Random, aircraft_count: int) -> float:
    """Width scales by aircraft count; each aircraft contributes 200~500m in 50m steps."""
    steps = int((PER_AIRCRAFT_WIDTH_MAX_M - PER_AIRCRAFT_WIDTH_MIN_M) / WIDTH_STEP_M)
    per_aircraft = PER_AIRCRAFT_WIDTH_MIN_M + WIDTH_STEP_M * rng.randint(0, steps)
    total = per_aircraft * max(1, aircraft_count)
    return float(total)


def _generate_line_path(
    start: LatLon,
    bounds: Tuple[float, float, float, float],
    rng: random.Random,
    avoid_segments: Sequence[Tuple[LatLon, LatLon]],
    min_gap_m: float,
    heading: Optional[float] = None,
    aircraft_count: int = 1,
) -> Tuple[Optional[List[LatLon]], Optional[float]]:
    point_count = rng.randint(LINE_POINT_COUNT_RANGE[0], LINE_POINT_COUNT_RANGE[1])
    width = _line_width(rng, aircraft_count)
    pts: List[LatLon] = [start]
    heading_cur = rng.uniform(0.0, 360.0) if heading is None else heading
    segments: List[Tuple[LatLon, LatLon]] = []
    for _ in range(point_count - 1):
        success = False
        for _ in range(SEGMENT_ATTEMPTS):
            delta = rng.uniform(-HEADING_DELTA_MAX_DEG, HEADING_DELTA_MAX_DEG)
            heading_new = (heading_cur + delta) % 360.0
            distance = rng.uniform(LINE_SEGMENT_MIN_M, LINE_SEGMENT_MAX_M)
            candidate = _move(pts[-1], distance, heading_new)
            if not _inside_bounds(candidate, bounds):
                continue
            new_seg = (pts[-1], candidate)
            if _segment_intersects_list(new_seg, segments, skip_shared_endpoint=True):
                continue
            if avoid_segments and _segment_intersects_list(new_seg, avoid_segments):
                continue
            if avoid_segments and _segment_min_distance(new_seg, avoid_segments) < min_gap_m:
                continue
            segments.append(new_seg)
            pts.append(candidate)
            heading_cur = heading_new
            success = True
            break
        if not success:
            return None, None
    return pts, width


def _place_area_near_point(
    ref_point: LatLon,
    bounds: Tuple[float, float, float, float],
    rng: random.Random,
    line_points: Sequence[LatLon],
    heading_ref: float,
) -> Optional[Tuple[LatLon, float, float, List[LatLon]]]:
    line_segments = _segments_from_points(line_points)
    preferred_dir = _quadrant_from_heading(heading_ref)
    dir_cycle = {
        "N": ("N", "E", "W", "S"),
        "S": ("S", "E", "W", "N"),
        "E": ("E", "N", "S", "W"),
        "W": ("W", "N", "S", "E"),
    }[preferred_dir]
    for attempt in range(RECT_ATTEMPTS):
        width = rng.uniform(AREA_SIDE_MIN_M, AREA_SIDE_MAX_M)
        height = rng.uniform(AREA_SIDE_MIN_M, AREA_SIDE_MAX_M)
        gap = rng.uniform(EDGE_GAP_MIN_M, EDGE_GAP_MAX_M)
        # Try forward-facing directions first; fallback to others later.
        if attempt < RECT_ATTEMPTS // 2:
            direction = dir_cycle[attempt % len(dir_cycle)]
        else:
            direction = rng.choice(dir_cycle)
        if direction in ("E", "W"):
            east_offset = (width / 2.0 + gap) * (1 if direction == "E" else -1)
            north_offset = rng.uniform(-height / 2.0, height / 2.0)
        else:
            north_offset = (height / 2.0 + gap) * (1 if direction == "N" else -1)
            east_offset = rng.uniform(-width / 2.0, width / 2.0)
        lat, lon = offset_lat_lon(ref_point.latitude, ref_point.longitude, east_offset, north_offset)
        center = LatLon(lat, lon)
        if not _rect_inside_bounds(center, width, height, bounds):
            continue
        bearing_to_center = _bearing_deg(ref_point, center)
        if _heading_delta(heading_ref, bearing_to_center) > FORWARD_ALIGN_DEG:
            continue
        corners = _rect_corners(center, width, height)
        edges = _rect_edges(corners)
        if _polyline_intersects_segments(line_points, edges):
            continue
        min_dist_line = _min_distance_polyline_edges(line_points, edges)
        if min_dist_line < EDGE_GAP_MIN_M:
            continue
        dist_end = _min_distance_point_edges(ref_point, edges)
        if not (EDGE_GAP_MIN_M <= dist_end <= EDGE_GAP_MAX_M):
            continue
        return center, width, height, corners
    return None


def _point_off_rect(
    center: LatLon,
    width_m: float,
    height_m: float,
    corners: Sequence[LatLon],
    bounds: Tuple[float, float, float, float],
    rng: random.Random,
    avoid_segments: Sequence[Tuple[LatLon, LatLon]],
    heading_ref: Optional[float] = None,
    preferred_direction: Optional[str] = None,
    force_direction: bool = False,
) -> Optional[LatLon]:
    edges = _rect_edges(corners)
    for _ in range(RECT_ATTEMPTS):
        gap = rng.uniform(EDGE_GAP_MIN_M, EDGE_GAP_MAX_M)
        if preferred_direction and (force_direction or _ < RECT_ATTEMPTS // 2):
            direction = preferred_direction
        else:
            direction = rng.choice(("E", "W", "N", "S"))
        if direction in ("E", "W"):
            east_offset = (width_m / 2.0 + gap) * (1 if direction == "E" else -1)
            north_offset = rng.uniform(-height_m / 2.0, height_m / 2.0)
        else:
            north_offset = (height_m / 2.0 + gap) * (1 if direction == "N" else -1)
            east_offset = rng.uniform(-width_m / 2.0, width_m / 2.0)
        lat, lon = offset_lat_lon(center.latitude, center.longitude, east_offset, north_offset)
        pt = LatLon(lat, lon)
        if not _inside_bounds(pt, bounds):
            continue
        if heading_ref is not None and _heading_delta(heading_ref, _bearing_deg(center, pt)) > CONTINUE_HEADING_MAX_DEG:
            continue
        dist_rect = _min_distance_point_edges(pt, edges)
        if not (EDGE_GAP_MIN_M <= dist_rect <= EDGE_GAP_MAX_M):
            continue
        if avoid_segments and _min_distance_point_segments(pt, avoid_segments) < EDGE_GAP_MIN_M:
            continue
        return pt
    return None


def _min_distance_point_segments(pt: LatLon, segments: Sequence[Tuple[LatLon, LatLon]]) -> float:
    return min(_point_segment_distance_m(pt, s1, s2) for s1, s2 in segments)


def _next_ids() -> Tuple[int, int]:
    current = _existing_max_seq()
    seq = current + 1
    return seq, seq


def _existing_max_seq() -> int:
    dir_path = _db_dir()
    if not dir_path.exists():
        return 0
    max_seq = 0
    for p in dir_path.glob(f"{FILE_PREFIX}*.json"):
        stem = p.stem
        if stem.startswith(FILE_PREFIX):
            suffix = stem[len(FILE_PREFIX) :]
            if suffix.isdigit():
                max_seq = max(max_seq, int(suffix))
    return max_seq


def _bootstrap_state_if_needed() -> None:
    # Previously used a state file; now sequence is derived solely from existing files.
    return None


def _line_mission(mission_id: int, points: List[LatLon], width: float) -> Dict:
    return {
        "inputMissionID": mission_id,
        "inputMissionType": 1,
        "isDone": False,
        "missionDetail": {
            "coordinateList": None,
            "lineList": [
                {
                    "width": width,
                    "coordinateList": [
                        {"latitude": p.latitude, "longitude": p.longitude, "altitude": 0} for p in points
                    ],
                }
            ],
            "areaList": None,
        },
    }


def _area_mission(mission_id: int, coords: List[LatLon]) -> Dict:
    return {
        "inputMissionID": mission_id,
        "inputMissionType": 2,
        "isDone": False,
        "missionDetail": {
            "coordinateList": None,
            "lineList": None,
            "areaList": [
                {
                    "isHole": False,
                    "coordinateList": [
                        {"latitude": p.latitude, "longitude": p.longitude, "altitude": 0.0} for p in coords
                    ],
                }
            ],
        },
    }


def _start_from_anchor(
    anchor: LatLon,
    bounds: Tuple[float, float, float, float],
    rng: random.Random,
    heading_hint: Optional[float],
    max_start_offset_m: float,
) -> Optional[LatLon]:
    """
    Choose a start point near anchor, nudged along heading_hint if provided.
    """
    if heading_hint is None:
        return _random_point(bounds, rng)
    for _ in range(RECT_ATTEMPTS):
        offset_m = rng.uniform(START_OFFSET_MIN_M, min(START_OFFSET_MAX_M, max_start_offset_m))
        # Allow some lateral wiggle so it's not overly colinear with heading.
        hdg = heading_hint + rng.uniform(-25.0, 25.0)
        cand = _move(anchor, offset_m, hdg)
        if _inside_bounds(cand, bounds):
            return cand
    # Fallback: radial pick around anchor but respecting min offset
    return _random_point_near(bounds, anchor, rng, max_start_offset_m, min_offset_m=START_OFFSET_MIN_M)


def _build_missions(
    rng: random.Random,
    bounds: Tuple[float, float, float, float],
    anchor: Optional[LatLon] = None,
    heading_hint: Optional[float] = None,
    max_start_offset_m: float = 2500.0,
) -> Tuple[List[LatLon], float, List[LatLon], List[LatLon], float]:
    aircraft_count = len(UAV_IDS)
    for _ in range(MAX_GEN_ATTEMPTS):
        if anchor:
            start = _start_from_anchor(anchor, bounds, rng, heading_hint, max_start_offset_m) or _random_point(bounds, rng)
        else:
            start = _random_point(bounds, rng)
        line1, width1 = _generate_line_path(
            start,
            bounds,
            rng,
            avoid_segments=[],
            min_gap_m=0.0,
            heading=heading_hint,
            aircraft_count=aircraft_count,
        )
        if not line1:
            continue
        heading_ref = _bearing_deg(line1[-2], line1[-1]) if len(line1) >= 2 else rng.uniform(0.0, 360.0)
        area_params = _place_area_near_point(line1[-1], bounds, rng, line1, heading_ref)
        if not area_params:
            continue
        area_center, area_w, area_h, area_corners = area_params
        area_edges = _rect_edges(area_corners)
        # Choose the opposite edge of the one closest to the incoming line end to keep handoff cleaner.
        edge_dirs = ("W", "N", "E", "S")
        dist_by_edge = [_min_distance_point_edges(line1[-1], [edge]) for edge in area_edges]
        nearest_idx = dist_by_edge.index(min(dist_by_edge))
        preferred_dir = edge_dirs[(nearest_idx + 2) % 4]
        line1_segments = _segments_from_points(line1)
        line2_start = _point_off_rect(
            area_center,
            area_w,
            area_h,
            area_corners,
            bounds,
            rng,
            avoid_segments=line1_segments,
            heading_ref=heading_ref,
            preferred_direction=preferred_dir,
            force_direction=True,
        )
        if not line2_start:
            continue
        avoid_for_line2 = list(line1_segments) + area_edges
        line2, width2 = _generate_line_path(
            line2_start,
            bounds,
            rng,
            avoid_segments=avoid_for_line2,
            min_gap_m=EDGE_GAP_MIN_M,
            heading=_bearing_deg(area_center, line2_start),
            aircraft_count=aircraft_count,
        )
        if not line2:
            continue
        if _min_distance_polyline_edges(line2, area_edges) < EDGE_GAP_MIN_M:
            continue
        return line1, width1, area_corners, line2, width2
    raise RuntimeError("Failed to generate missions within attempt budget")


def _random_point(bounds: Tuple[float, float, float, float], rng: random.Random) -> LatLon:
    sw_lat, sw_lon, ne_lat, ne_lon = bounds
    lat = rng.uniform(sw_lat, ne_lat)
    lon = rng.uniform(sw_lon, ne_lon)
    return LatLon(lat, lon)


def _random_point_near(
    bounds: Tuple[float, float, float, float],
    anchor: LatLon,
    rng: random.Random,
    max_offset_m: float,
    min_offset_m: float = 0.0,
) -> Optional[LatLon]:
    """
    Pick a point near an anchor (within max_offset_m, outside min_offset_m) that still respects bounds.
    Returns None if no suitable point is found.
    """
    if max_offset_m <= 0:
        return None
    for _ in range(RECT_ATTEMPTS):
        dist = rng.uniform(min_offset_m, max_offset_m)
        bearing = rng.uniform(0.0, 360.0)
        east = dist * math.sin(math.radians(bearing))
        north = dist * math.cos(math.radians(bearing))
        lat, lon = offset_lat_lon(anchor.latitude, anchor.longitude, east, north)
        pt = LatLon(lat, lon)
        if _inside_bounds(pt, bounds):
            return pt
    return None


def generate(
    seed: Optional[int] = None,
    *,
    anchor: Optional[LatLon] = None,
    heading_hint: Optional[float] = None,
    max_start_offset_m: float = 2500.0,
) -> Dict:
    """
    Build an InputMissionPlan payload with a line-area-line structure.

    Args:
        seed: Optional random seed.
        anchor: Optional starting bias (e.g., takeOver point). First line start stays near this point when possible.
        heading_hint: Optional initial heading in degrees for the first leg.
        max_start_offset_m: Max distance from anchor when selecting the start point.
    """
    # If anchor is given but no heading_hint, default to northbound (triangle tip points north).
    if anchor is not None and heading_hint is None:
        heading_hint = 0.0
    _bootstrap_state_if_needed()
    pkg_id, file_seq = _next_ids()
    rng = random.Random(seed)
    bounds = _bounds_with_margin(BORDER_MARGIN_M)
    line1, width1, area_coords, line2, width2 = _build_missions(
        rng,
        bounds,
        anchor=anchor,
        heading_hint=heading_hint,
        max_start_offset_m=max_start_offset_m,
    )

    payload = {
        "timestamp": now_ms_2000(),
        "inputMissionPackageID": pkg_id,
        "inputMissionPackageType": rng.randint(1, 5),
        "mainSensor": _pick_main_sensor(rng),
        "availableAircraftList": [{"aircraftID": i} for i in AIRCRAFT_IDS],
        "inputMissionList": [
            _line_mission(1, line1, width1),
            _area_mission(2, area_coords),
            _line_mission(3, line2, width2),
        ],
        "_meta": {"fileSeq": file_seq, "seed": seed},
    }
    return payload


def save(payload: Dict) -> Path:
    dir_path = _db_dir()
    dir_path.mkdir(parents=True, exist_ok=True)
    meta = payload.get("_meta", {})
    file_seq = meta.get("fileSeq") or payload.get("inputMissionPackageID", 1)
    path = dir_path / f"{FILE_PREFIX}{int(file_seq):04d}.json"
    payload = dict(payload)
    payload.pop("_meta", None)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return path


if __name__ == "__main__":
    obj = generate()
    out = save(obj)
    print(f"generated: {out}")
