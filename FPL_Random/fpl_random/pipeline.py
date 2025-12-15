from __future__ import annotations

"""
One-shot scenario generation that ties together:
- FlightReferenceInfo (takeOver/handOver/prohibited area)
- InputMissionPlan (line -> area -> line), biased near takeOver points
- A simple target list (one static target per generated mission)

Usage:
    from fpl_random.pipeline import generate_sequence
    result = generate_sequence(seed=123)
    # result["paths"] holds saved file paths
"""

import math
import random
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

if __name__ == "__main__" and __package__ is None:
    # Allow running as a script without installing the package.
    import sys

    sys.path.append(str(Path(__file__).resolve().parent.parent))
    __package__ = "fpl_random"

from .areas import LatLon
from . import flight_ref, mission_plan, paths
from .utils import now_ms_2000, offset_lat_lon

PROJECT_ROOT = Path(__file__).resolve().parent.parent
TARGET_FILE_PREFIX = "LAHMUMT-TGT-"


def _target_dir() -> Path:
    return paths.db_root() / "TargetInfo"
TARGETS_PER_MISSION_RANGE = (1, 3)
TARGET_MIN_SEP_M = 300.0
LINE_LATERAL_OFFSET_M = 120.0


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle bearing from (lat1, lon1) to (lat2, lon2) in degrees."""
    lat1_r, lat2_r = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2_r)
    y = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon)
    ang = math.degrees(math.atan2(x, y))
    return (ang + 360.0) % 360.0


def _distance_m(p1: LatLon, p2: LatLon) -> float:
    """Equirectangular approximation; good enough for local spacing checks."""
    mean_lat = math.radians((p1.latitude + p2.latitude) / 2.0)
    dlat = (p2.latitude - p1.latitude) * 111_320.0
    dlon = (p2.longitude - p1.longitude) * 111_320.0 * math.cos(mean_lat)
    return math.hypot(dlat, dlon)


def _far_enough(existing: Sequence[LatLon], cand: LatLon, min_dist_m: float) -> bool:
    return all(_distance_m(pt, cand) >= min_dist_m for pt in existing)


def _anchor_from_flight_reference(ref_payload: Dict) -> Tuple[Optional[LatLon], Optional[float]]:
    """
    Pick a takeOver point (anchor) and derive a heading hint from its handOver pair.
    Prefers the first UAV entry. Falls back to None on missing data.
    """
    takeovers = ref_payload.get("takeOverInfoList") or []
    if not takeovers:
        return None, None

    handover_map = {
        entry.get("aircraftID"): entry.get("coordinate")
        for entry in ref_payload.get("handOverInfoList") or []
        if isinstance(entry, dict)
    }

    anchor_entry = takeovers[0]
    coord = anchor_entry.get("coordinate") or {}
    anchor = LatLon(coord.get("latitude", 0.0), coord.get("longitude", 0.0))

    heading = None
    handover_coord = handover_map.get(anchor_entry.get("aircraftID"))
    if handover_coord and all(k in handover_coord for k in ("latitude", "longitude")):
        heading = _bearing_deg(
            coord.get("latitude", 0.0),
            coord.get("longitude", 0.0),
            handover_coord["latitude"],
            handover_coord["longitude"],
        )
    return anchor, heading


def _collect_line_coords(mission: Dict) -> List[LatLon]:
    detail = mission.get("missionDetail") or {}
    lines = detail.get("lineList") or []
    if not lines:
        return []
    coords = lines[0].get("coordinateList") or []
    out: List[LatLon] = []
    for c in coords:
        try:
            out.append(LatLon(float(c["latitude"]), float(c["longitude"])))
        except Exception:
            continue
    return out


def _collect_area_coords(mission: Dict) -> List[LatLon]:
    detail = mission.get("missionDetail") or {}
    areas = detail.get("areaList") or []
    if not areas:
        return []
    coords = areas[0].get("coordinateList") or []
    out: List[LatLon] = []
    for c in coords:
        try:
            out.append(LatLon(float(c["latitude"]), float(c["longitude"])))
        except Exception:
            continue
    return out


def _point_in_poly(pt: LatLon, poly: Sequence[LatLon]) -> bool:
    """Ray casting; works for simple polygons (including rectangles)."""
    x, y = pt.longitude, pt.latitude
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i - 1].longitude, poly[i - 1].latitude
        x2, y2 = poly[i].longitude, poly[i].latitude
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1):
            inside = not inside
    return inside


def _pick_point_on_line(coords: Sequence[LatLon], rng: random.Random) -> Optional[LatLon]:
    if len(coords) < 2:
        return None
    idx = rng.randint(0, len(coords) - 2)
    a, b = coords[idx], coords[idx + 1]
    t = rng.uniform(0.2, 0.8)
    lat = a.latitude + (b.latitude - a.latitude) * t
    lon = a.longitude + (b.longitude - a.longitude) * t
    # 측면으로 약간 이동시켜 겹치지 않게 (좌/우 랜덤)
    bearing = _bearing_deg(a.latitude, a.longitude, b.latitude, b.longitude)
    lateral = rng.uniform(-LINE_LATERAL_OFFSET_M, LINE_LATERAL_OFFSET_M)
    if abs(lateral) > 1e-6:
        side_hdg = (bearing + (90.0 if lateral >= 0 else -90.0)) % 360.0
        lat, lon = offset_lat_lon(lat, lon, lateral, 0.0)
    return LatLon(lat, lon)


def _pick_point_in_area(coords: Sequence[LatLon], rng: random.Random) -> Optional[LatLon]:
    if not coords:
        return None
    lats = [c.latitude for c in coords]
    lons = [c.longitude for c in coords]
    lat_min, lat_max = min(lats), max(lats)
    lon_min, lon_max = min(lons), max(lons)
    for _ in range(80):
        lat = rng.uniform(lat_min, lat_max)
        lon = rng.uniform(lon_min, lon_max)
        candidate = LatLon(lat, lon)
        if _point_in_poly(candidate, coords):
            return candidate
    # Fallback to centroid if random search fails (should not happen for rectangles)
    return LatLon(sum(lats) / len(lats), sum(lons) / len(lons))


def _build_targets(cmpk_payload: Dict, rng: random.Random, allowed_types: Sequence[int]) -> List[Dict]:
    targets: List[Dict] = []
    type_pool = [int(t) for t in allowed_types] or [1, 2]

    for mission in cmpk_payload.get("inputMissionList") or []:
        mtype = mission.get("inputMissionType")
        coords: List[LatLon] = []
        mission_targets: List[LatLon] = []
        num_targets = rng.randint(TARGETS_PER_MISSION_RANGE[0], TARGETS_PER_MISSION_RANGE[1])
        # 이미 생성된 타겟들의 위치를 LatLon으로 변환해 중복/근접 체크
        existing_pts = [
            LatLon(t["location"]["latitude"], t["location"]["longitude"])
            for t in targets
            if isinstance(t, dict) and "location" in t and "latitude" in t["location"]
        ]
        if mtype == 1:
            coords = _collect_line_coords(mission)
            for _ in range(40):
                if len(mission_targets) >= num_targets:
                    break
                pt = _pick_point_on_line(coords, rng)
                if not pt:
                    continue
                if not _far_enough(mission_targets + existing_pts, pt, TARGET_MIN_SEP_M):
                    continue
                mission_targets.append(pt)
        elif mtype == 2:
            coords = _collect_area_coords(mission)
            for _ in range(60):
                if len(mission_targets) >= num_targets:
                    break
                pt = _pick_point_in_area(coords, rng)
                if not pt:
                    continue
                if not _far_enough(mission_targets + existing_pts, pt, TARGET_MIN_SEP_M):
                    continue
                mission_targets.append(pt)
        else:
            continue

        if not mission_targets:
            continue

        for pt in mission_targets:
            tgt = {
                "targetID": len(targets) + 1,
                "targetType": rng.choice(type_pool),
                "inputMissionID": mission.get("inputMissionID"),
                "location": {"latitude": pt.latitude, "longitude": pt.longitude, "altitude": 0},
                "path": [
                    {"latitude": pt.latitude, "longitude": pt.longitude, "altitude": 0}
                ],  # static path placeholder
            }
            targets.append(tgt)
    return targets


def save_targets(payload: Dict, file_seq: Optional[int] = None) -> Path:
    """
    Persist target payload under database/TargetInfo.
    """
    target_dir = _target_dir()
    target_dir.mkdir(parents=True, exist_ok=True)
    meta = payload.get("_meta", {})
    seq = file_seq or meta.get("fileSeq") or payload.get("inputMissionPackageID", 1)
    path = target_dir / f"{TARGET_FILE_PREFIX}{int(seq):04d}.json"
    payload = dict(payload)
    payload.pop("_meta", None)
    with path.open("w", encoding="utf-8") as fh:
        import json

        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return path


def generate_sequence(
    seed: Optional[int] = None,
    *,
    max_start_offset_m: float = 2500.0,
    target_types: Sequence[int] = (1, 2),
    save: bool = True,
) -> Dict:
    """
    Run flight reference + mission plan + target generation in one call.

    Args:
        seed: Master seed for reproducibility.
        max_start_offset_m: Clamp how far the first line start can drift from the chosen takeOver anchor.
        target_types: Allowed targetType pool (defaults to {1, 2}).
        save: When True, persist all payloads to disk.
    """
    master_rng = random.Random(seed)
    ref_seed = master_rng.randint(0, 1_000_000_000)
    cmpk_seed = master_rng.randint(0, 1_000_000_000)
    target_seed = master_rng.randint(0, 1_000_000_000)
    target_rng = random.Random(target_seed)

    ref_payload = flight_ref.generate(seed=ref_seed)
    anchor, heading = _anchor_from_flight_reference(ref_payload)
    cmpk_payload = mission_plan.generate(
        seed=cmpk_seed,
        anchor=anchor,
        heading_hint=heading,
        max_start_offset_m=max_start_offset_m,
    )

    targets = _build_targets(cmpk_payload, target_rng, allowed_types=target_types)
    target_payload = {
        "timestamp": now_ms_2000(),
        "inputMissionPackageID": cmpk_payload.get("inputMissionPackageID"),
        "missionReferencePackageID": ref_payload.get("missionReferencePackageID"),
        "targetList": targets,
        "_meta": {
            "fileSeq": cmpk_payload.get("_meta", {}).get("fileSeq") or cmpk_payload.get("inputMissionPackageID"),
            "seed": seed,
            "seeds": {"flight_reference": ref_seed, "mission_plan": cmpk_seed, "targets": target_seed},
            "anchor": anchor.as_tuple() if anchor else None,
            "heading_hint": heading,
        },
    }

    paths = {"flight_reference": None, "input_mission_plan": None, "targets": None}
    if save:
        paths["flight_reference"] = str(flight_ref.save(ref_payload))
        paths["input_mission_plan"] = str(mission_plan.save(cmpk_payload))
        paths["targets"] = str(save_targets(target_payload))

    return {
        "flight_reference": ref_payload,
        "input_mission_plan": cmpk_payload,
        "targets": target_payload,
        "paths": paths,
    }


if __name__ == "__main__":
    result = generate_sequence()
    print("flight_reference:", result["paths"]["flight_reference"])
    print("input_mission_plan:", result["paths"]["input_mission_plan"])
    print("targets:", result["paths"]["targets"])
