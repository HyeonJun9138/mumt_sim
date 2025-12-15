from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

from . import paths
from .utils import now_ms_2000


def _imp_dir() -> Path:
    return paths.db_root() / "IndividualMissionPlan"


def _fp_dir() -> Path:
    return paths.db_root() / "FlightPath"


def _max_numeric_stem(dir_path: Path) -> int:
    max_id = 0
    if not dir_path.exists():
        return max_id
    for p in dir_path.glob("*.json"):
        stem = p.stem
        if stem.isdigit():
            try:
                num = int(stem)
                max_id = max(max_id, num)
            except Exception:
                continue
    return max_id


def _max_individual_id(dir_path: Path) -> int:
    max_id = 0
    if not dir_path.exists():
        return max_id
    for p in dir_path.glob("*.json"):
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
        except Exception:
            continue
        for im in data.get("individualMissionList", []) or []:
            try:
                max_id = max(max_id, int(im.get("individualMissionID", 0)))
            except Exception:
                continue
    return max_id


def _choose_aircraft(cmpk_payload: Dict) -> int:
    for entry in cmpk_payload.get("availableAircraftList") or []:
        try:
            return int(entry.get("aircraftID"))
        except Exception:
            continue
    return 1


def _mission_coords(mission: Dict) -> Tuple[List[Dict], str]:
    mtype = mission.get("inputMissionType")
    if mtype == 1:
        lines = (mission.get("missionDetail") or {}).get("lineList") or []
        coords = (lines[0].get("coordinateList") if lines else []) or []
        return coords, "line"
    if mtype == 2:
        areas = (mission.get("missionDetail") or {}).get("areaList") or []
        coords = (areas[0].get("coordinateList") if areas else []) or []
        return coords, "area"
    return [], "unknown"


def build_individual_and_paths(
    cmpk_payload: Dict,
    *,
    cruise_alt_m: float = 1000.0,
    cruise_speed_mps: float = 40.0,
) -> Tuple[Dict, List[Dict]]:
    """
    입력임무(0201)를 기반으로 간단한 0302/0304 형태를 생성한다.
    """
    imp_dir = _imp_dir()
    fp_dir = _fp_dir()
    pkg_id = _max_numeric_stem(imp_dir) + 1
    base_ind_id = max(_max_individual_id(imp_dir), pkg_id * 1000)
    base_path_id = _max_numeric_stem(fp_dir)
    aircraft_id = _choose_aircraft(cmpk_payload)

    imp_list: List[Dict] = []
    path_list: List[Dict] = []
    mission_list = cmpk_payload.get("inputMissionList") or []

    for idx, mission in enumerate(mission_list, start=1):
        coords, mkind = _mission_coords(mission)
        if not coords:
            continue
        mission_id = mission.get("inputMissionID", idx)
        individual_id = base_ind_id + idx
        path_id = base_path_id + idx
        first_coord = dict(coords[0])
        first_coord.setdefault("altitude", cruise_alt_m)

        if mkind == "line":
            indiv_type = 7
            pattern_type = 10
        elif mkind == "area":
            indiv_type = 9
            pattern_type = 12
        else:
            indiv_type = 7
            pattern_type = 10

        imp_list.append(
            {
                "individualMissionID": individual_id,
                "isDone": False,
                "relatedMission": {"relatedMissionType": 1, "inputMissionID": mission_id, "priorMissionID": 0},
                "individualMissionInfo": {
                    "individualMissionType": indiv_type,
                    "patternType": pattern_type,
                    "autoZoomIn": False,
                    "coordinateList": coords,
                },
                "pathID": path_id,
            }
        )

        wp_list = [
            {
                "waypointID": path_id * 10 + 1,
                "coordinate": first_coord,
                "speed": cruise_speed_mps,
                "eta": 0,
                "ecf": 1.0,
                "nextWaypointID": 0,
                "waypointPassType": 1,
                "filmingProperty": {
                    "fieldOfView": 10.0,
                    "sensorType": cmpk_payload.get("mainSensor", 1),
                    "operationMode": 1,
                    "coordinateOrientation": {"coordinate": first_coord},
                },
            }
        ]

        path_list.append(
            {
                "timestamp": now_ms_2000(),
                "Source": "MMR",
                "pathID": path_id,
                "aircraftID": aircraft_id,
                "isFormationFlight": False,
                "waypointList": wp_list,
                "lahWaypointList": wp_list,
            }
        )

    imp_payload = {
        "timestamp": now_ms_2000(),
        "Source": "MMR",
        "individualMissionPackageID": pkg_id,
        "aircraftID": aircraft_id,
        "individualMissionList": imp_list,
    }

    return imp_payload, path_list


def save_individual(payload: Dict) -> Path:
    dir_path = _imp_dir()
    dir_path.mkdir(parents=True, exist_ok=True)
    pkg_id = int(payload.get("individualMissionPackageID", 1))
    path = dir_path / f"{pkg_id:09d}.json"
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return path


def save_flight_path(payload: Dict) -> Path:
    dir_path = _fp_dir()
    dir_path.mkdir(parents=True, exist_ok=True)
    path_id = int(payload.get("pathID", 1))
    path = dir_path / f"{path_id:09d}.json"
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return path
