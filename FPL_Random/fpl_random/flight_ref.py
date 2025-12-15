from __future__ import annotations

"""
FlightReferenceInfo 생성기.

규칙 요약:
- 파일명: LAHMUMT-SCN-0001.json 처럼 4자리 오름차순.
- missionReferencePackageID: 1부터 오름차순.
- timestamp/inputTimestamp: 2000-01-01 UTC 기준 ms.
- takeOverInfoList: UAV ID 4,5,6을 정삼각형(변 150m, 북향)으로 배치. 기준점은 시작 참조점 중 랜덤 선택.
- handOverInfoList: 각 UAV 시작점에서 동/서 랜덤 방향으로 300m 이동.
- rtbCoordinateList: handOver에서 선택한 방향의 반대편으로 300m 이동.
- flightAreaList: 자동임무 생성 구역 그대로, 고도 0~5000.
- prohibitedAreaList: 자동임무 생성 구역 바깥 서쪽에 200~400m 떨어진 위치에 반경 120m 오각형, 고도 0~5000.
"""

import json
import math
import random
from pathlib import Path
from typing import Dict, List, Tuple

if __name__ == "__main__" and __package__ is None:
    # 패키지 외부 실행 시 상위 경로를 sys.path에 추가해 relative import 오류 방지
    import sys

    sys.path.append(str(Path(__file__).resolve().parent.parent))
    __package__ = "fpl_random"

from .areas import AUTO_MISSION_AREA, START_REFERENCE_POINTS, LatLon
from . import paths
from .utils import now_ms_2000, offset_lat_lon

PROJECT_ROOT = Path(__file__).resolve().parent.parent
FILE_PREFIX = "LAHMUMT-SCN-"

# 상수 정의
UAV_IDS = (4, 5, 6)
SIDE_M = 150.0
HANDOVER_OFFSET_M = 300.0
RTB_OFFSET_M = 300.0
PROHIBITED_OFFSET_MIN = 200.0
PROHIBITED_OFFSET_MAX = 400.0
PROHIBITED_RADIUS_M = 250.0  # 반경 키워서 금지구역 크기 확대
DEFAULT_ALT_M = 1000.0


def _db_dir() -> Path:
    return paths.db_root() / "MissionReferenceInfo"


def _next_ids() -> Tuple[int, int]:
    """(missionReferencePackageID, file_seq)를 반환."""
    current = _existing_max_seq()
    seq = current + 1
    return seq, seq


def _existing_max_seq() -> int:
    dir_path = _db_dir()
    if not dir_path.exists():
        return 0
    max_seq = 0
    for p in dir_path.glob(f"{FILE_PREFIX}*.json"):
        try:
            num = int(p.stem.split("-")[-1])
            if num > max_seq:
                max_seq = num
        except Exception:
            continue
    return max_seq


def _bootstrap_state_if_needed() -> None:
    # State file no longer used; IDs derive from existing filenames.
    return None


def _triangle_vertices(center: LatLon) -> Tuple[LatLon, LatLon, LatLon]:
    """
    centroid를 기준으로 북향 정삼각형(변=SIDE_M) 좌표를 계산.
    좌표계: x=동(+), y=북(+)
    """
    h = math.sqrt(3) / 2 * SIDE_M
    v1 = (0.0, 2 * h / 3)  # 북쪽 꼭짓점
    v2 = (-SIDE_M / 2, -h / 3)
    v3 = (SIDE_M / 2, -h / 3)
    verts = []
    for east, north in (v1, v2, v3):
        lat, lon = offset_lat_lon(center.latitude, center.longitude, east, north)
        verts.append(LatLon(lat, lon))
    return tuple(verts)  # type: ignore


def _handover_point(start: LatLon, east_first: bool) -> LatLon:
    east_m = HANDOVER_OFFSET_M if east_first else -HANDOVER_OFFSET_M
    lat, lon = offset_lat_lon(start.latitude, start.longitude, east_m, 0.0)
    return LatLon(lat, lon)


def _rtb_point(start: LatLon, east_first: bool) -> LatLon:
    east_m = -RTB_OFFSET_M if east_first else RTB_OFFSET_M
    lat, lon = offset_lat_lon(start.latitude, start.longitude, east_m, 0.0)
    return LatLon(lat, lon)


def _prohibited_pentagon(rng: random.Random) -> List[Dict[str, float]]:
    sw = AUTO_MISSION_AREA.southwest
    ne = AUTO_MISSION_AREA.northeast
    mid_lat = (sw.latitude + ne.latitude) / 2
    mid_lon = (sw.longitude + ne.longitude) / 2

    # 구역 서쪽으로 200~400m + 절반 폭만큼 이동시켜 FlightArea와 겹치지 않도록 함.
    width_m = _lon_span_m(sw.latitude, ne.latitude, sw.longitude, ne.longitude)
    offset_base = width_m / 2 + rng.uniform(PROHIBITED_OFFSET_MIN, PROHIBITED_OFFSET_MAX)
    center_lat, center_lon = offset_lat_lon(mid_lat, mid_lon, -offset_base, 0.0)

    pts: List[Dict[str, float]] = []
    start_bearing = rng.uniform(0, 360)
    for i in range(5):
        bearing = math.radians(start_bearing + i * 72.0)
        east = PROHIBITED_RADIUS_M * math.sin(bearing)
        north = PROHIBITED_RADIUS_M * math.cos(bearing)
        lat, lon = offset_lat_lon(center_lat, center_lon, east, north)
        pts.append({"latitude": lat, "longitude": lon})
    return pts


def _lon_span_m(lat0: float, lat1: float, lon0: float, lon1: float) -> float:
    mid_lat = (lat0 + lat1) / 2
    lon_diff = abs(lon1 - lon0)
    return lon_diff * 111_320.0 * math.cos(math.radians(mid_lat))


def generate(seed: int | None = None) -> Dict:
    """
    FlightReferenceInfo JSON 객체 생성.
    """
    _bootstrap_state_if_needed()
    mission_id, file_seq = _next_ids()
    rng = random.Random(seed)

    base_point = rng.choice(START_REFERENCE_POINTS)
    uav_points = _triangle_vertices(base_point)

    # 전체 편대 기준으로 좌/우 랜덤 선택 (모든 UAV 동일 방향)
    handover_left = rng.choice([True, False])  # True=서쪽(좌), False=동쪽(우)

    # UAV IDs와 좌표 매핑
    take_over = []
    hand_over = []
    rtb_points = []
    for aircraft_id, pt in zip(UAV_IDS, uav_points):
        hand_pt = _handover_point(pt, east_first=not handover_left)  # east_first False→서쪽, True→동쪽
        rtb_pt = _rtb_point(pt, east_first=not handover_left)        # RTB는 반대 방향(함수 내부에서 부호 반전)
        take_over.append(
            {"aircraftID": aircraft_id, "coordinate": {"latitude": pt.latitude, "longitude": pt.longitude, "altitude": DEFAULT_ALT_M}}
        )
        hand_over.append(
            {"aircraftID": aircraft_id, "coordinate": {"latitude": hand_pt.latitude, "longitude": hand_pt.longitude, "altitude": DEFAULT_ALT_M}}
        )
        rtb_points.append({"latitude": rtb_pt.latitude, "longitude": rtb_pt.longitude, "altitude": DEFAULT_ALT_M})

    # Flight area
    flight_area = {
        "flightAreaID": 1,
        "areaLatLonList": AUTO_MISSION_AREA.to_area_lat_lon_list(),
        "altitudeLimits": {"lowerLimit": 0, "upperLimit": 5000},
    }

    # Prohibited area
    prohibited_area = {
        "prohibitedAreaID": 1,
        "areaLatLonList": _prohibited_pentagon(rng),
        "altitudeLimits": {"lowerLimit": 0, "upperLimit": 5000},
    }

    timestamp = now_ms_2000()

    return {
        "timestamp": timestamp,
        "missionReferencePackageID": mission_id,
        "inputTimestamp": timestamp,
        "takeOverInfoList": take_over,
        "handOverInfoList": hand_over,
        "rtbCoordinateList": rtb_points,
        "flightAreaList": [flight_area],
        "prohibitedAreaList": [prohibited_area],
        "_meta": {"fileSeq": file_seq, "seed": seed},
    }


def save(payload: Dict) -> Path:
    """생성된 객체를 파일로 저장하고 경로를 반환."""
    dir_path = _db_dir()
    dir_path.mkdir(parents=True, exist_ok=True)
    meta = payload.get("_meta", {})
    file_seq = meta.get("fileSeq") or payload.get("missionReferencePackageID", 1)
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
