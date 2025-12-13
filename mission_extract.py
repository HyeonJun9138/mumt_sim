"""
Mission path extractor (waypoints + filming properties).

Reads a scenario folder under mission/ and reports, per aircraft:
  - individual missions with pathID
  - waypoints from the referenced FlightPath files
  - filming properties attached to each waypoint (FOV/sensor/operationMode, etc.)

Usage:
    python mission_extract.py mission/Scenario_YYYY.../SBCx
"""

import json
from pathlib import Path
from typing import Any, Dict


def load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def find_single_json(dir_path: Path) -> Path:
    files = sorted(dir_path.glob("*.json"))
    if not files:
        raise FileNotFoundError(f"No JSON found in {dir_path}")
    return files[0]


def extract_scenario(sbc_path: Path) -> Dict[str, Any]:
    mission_plan_file = find_single_json(sbc_path / "MissionPlan")
    mission_plan = load_json(mission_plan_file)

    aircraft_map = {
        entry["aircraftID"]: entry["individualMissionPackageID"]
        for entry in mission_plan.get("aircraftList", [])
    }

    indy_dir = sbc_path / "IndividualMissionPlan"
    path_dir = sbc_path / "FlightPath"

    aircraft_data = {}
    for ac_id, pkg_id in aircraft_map.items():
        pkg_file = indy_dir / f"{pkg_id}.json"
        pkg = load_json(pkg_file)
        missions = []
        for item in pkg.get("individualMissionList", []):
            info = item.get("individualMissionInfo", {})
            missions.append(
                {
                    "individualMissionID": item.get("individualMissionID"),
                    "individualMissionType": info.get("individualMissionType"),
                    "patternType": info.get("patternType"),
                    "autoZoomIn": info.get("autoZoomIn"),
                    "coordinateList": info.get("coordinateList", []),
                    "lineList": info.get("lineList", []),
                    "areaList": info.get("areaList", []),
                    "pathID": item.get("pathID"),
                }
            )
        # attach path files referenced
        paths = {}
        for m in missions:
            pid = m.get("pathID")
            if pid and pid not in paths:
                pfile = path_dir / f"{pid}.json"
                if pfile.exists():
                    paths[pid] = load_json(pfile)
        aircraft_data[ac_id] = {
            "individualMissionPackageID": pkg_id,
            "missions": missions,
            "paths": paths,
        }

    return {
        "missionPlan": mission_plan,
        "aircraftMap": aircraft_map,
        "aircraftData": aircraft_data,
    }


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Extract mission data for flight/imagery commands")
    parser.add_argument("scenario_path", type=Path, help="Path to SBCx folder (e.g., mission/Scenario.../SBC3)")
    args = parser.parse_args()

    data = extract_scenario(args.scenario_path)

    print(f"Scenario: {args.scenario_path}")
    for ac_id, pkg_id in data["aircraftMap"].items():
        ac_info = data["aircraftData"][ac_id]
        print(f"\nAircraft {ac_id} (pkg {pkg_id})")
        for m in ac_info["missions"]:
            pid = m["pathID"]
            print(
                f"  missionID={m['individualMissionID']} type={m['individualMissionType']} "
                f"pattern={m['patternType']} autoZoom={m['autoZoomIn']} pathID={pid}"
            )
            if not pid or pid not in ac_info["paths"]:
                print("    (no path data found)")
                continue
            path = ac_info["paths"][pid]
            wps = path.get("waypointList") or path.get("lahWaypointList") or []
            print(f"    waypoints: {len(wps)}")
            for wp in wps:
                coord = wp.get("coordinate", {})
                filming = wp.get("filmingProperty", {})
                fov = filming.get("fieldOfView")
                op_mode = filming.get("operationMode")
                sensor = filming.get("sensorType")
                # optional gimbal or orientation
                gimbal = filming.get("aircraftFixed", {})
                orient = filming.get("coordinateOrientation", {}).get("coordinate", {})
                line_search = filming.get("lineSearch", {}).get("coordinateList")
                print(
                    f"      WP {wp.get('waypointID')} "
                    f"lat={coord.get('latitude')} lon={coord.get('longitude')} alt={coord.get('altitude')} "
                    f"speed={wp.get('speed')} passType={wp.get('waypointPassType')}"
                )
                if filming:
                    print(
                        f"        filming: fov={fov} sensor={sensor} opMode={op_mode} "
                        f"gimbal(pitch={gimbal.get('gimbalPitch')},yaw={gimbal.get('gimbalYaw')})"
                    )
                    if orient:
                        print(
                            f"        orient target: lat={orient.get('latitude')} lon={orient.get('longitude')} alt={orient.get('altitude')}"
                        )
                    if line_search:
                        print(f"        lineSearch coords: {len(line_search)} pts")


if __name__ == "__main__":
    main()
