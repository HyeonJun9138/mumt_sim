from __future__ import annotations

"""
GUI 없이 전체 파이프라인을 한 번에 실행:
1) fpl_random.pipeline.generate_sequence 로 0201/0203/TGT 생성
2) AnS.run_divide_and_pattern → 0302 (IMP) 생성
3) AnS.build_mission_plan_0301 → 0301 생성
4) d0303.build_flight_plans / d0304.build_lah_flight_plans_fixed → 0303/0304 생성
5) database 하위 MissionPlan, IndividualMissionPlan, FlightPath 에 저장

※ 알고리즘·ID 규칙은 main_MP.py / AnS 모듈과 동일. 시각화만 제거.
"""

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Tuple

from fpl_random import paths as fpl_paths


REPO_ROOT = Path(__file__).resolve().parents[1]  # .../LAHMUMT25


def _inject_paths() -> None:
    """main_MP와 동일한 모듈 경로 추가."""
    for p in [
        REPO_ROOT,
        REPO_ROOT / "MP",
        REPO_ROOT / "MP" / "modules",
        REPO_ROOT / "MP" / "modules" / "mission_planning",
        REPO_ROOT / "MP" / "modules" / "mission_planning" / "MissionPlanner",
    ]:
        p = str(p)
        if p not in sys.path:
            sys.path.insert(0, p)


def _ensure_dirs(*dirs: Path) -> None:
    for d in dirs:
        d.mkdir(parents=True, exist_ok=True)


def _load_imp_missions(imp_paths: List[Path]) -> List[Dict]:
    """IMP 패키지 리스트를 통합해 main_MP 의 self.missions 형태로 반환."""
    missions: List[Dict] = []
    for imp in imp_paths:
        pkg = json.loads(imp.read_text(encoding="utf-8"))
        aid = pkg.get("aircraftID")
        for im in pkg.get("individualMissionList", []):
            im["aircraftID"] = aid
            missions.append(im)
    return missions


def _normalize_path_ids(pkgs: List[Path]) -> None:
    """
    IMP 내부 pathID를 항공기별 범위(100/200/300/400/500/600M대)와
    중복 없는 값으로 정규화한다. main_MP + id_allocator 규칙과 동일.
    """
    from MP.modules.mission_planning.MissionPlanner.data_def.id_allocator import next_path_id

    bounds = {
        1: (100_000_001, 200_000_000),
        2: (200_000_001, 300_000_000),
        3: (300_000_001, 400_000_000),
        4: (400_000_001, 500_000_000),
        5: (500_000_001, 600_000_000),
        6: (600_000_001, 700_000_000),
    }

    for imp_path in pkgs:
        pkg = json.loads(imp_path.read_text(encoding="utf-8"))
        aid = int(pkg.get("aircraftID", 0) or 0)
        if aid not in bounds:
            continue
        lo, hi = bounds[aid]
        seen: set[int] = set()
        changed = False
        for im in pkg.get("individualMissionList", []):
            pid = int(im.get("pathID", 0))
            if pid in seen or not (lo <= pid < hi):
                pid = next_path_id(aid)
                while pid in seen:
                    pid = next_path_id(aid)
                im["pathID"] = pid
                changed = True
            seen.add(pid)
        if changed:
            imp_path.write_text(json.dumps(pkg, ensure_ascii=False, indent=2), encoding="utf-8")


def _save_outputs(
    mp_path: Path,
    imp_paths: List[Path],
    fp_0303: List[Dict],
    fp_0304: List[Dict],
    db_root: Path,
) -> Tuple[Path, List[Path], List[Path]]:
    """main_MP._save_all_missions 과 동일하게 DB 폴더에 저장."""
    dir_mp = db_root / "MissionPlan"
    dir_imp = db_root / "IndividualMissionPlan"
    dir_fp = db_root / "FlightPath"
    _ensure_dirs(dir_mp, dir_imp, dir_fp)

    saved_imp: List[Path] = []
    saved_fp: List[Path] = []

    # 0301
    mp_obj = json.loads(mp_path.read_text(encoding="utf-8"))
    mp_id = mp_obj.get("missionPlanID") or mp_obj.get("MissionPlanID")
    mp_out = dir_mp / f"{mp_id}.json"
    mp_out.write_text(json.dumps(mp_obj, ensure_ascii=False, indent=2), encoding="utf-8")

    # 0302
    for p in imp_paths:
        pkg = json.loads(p.read_text(encoding="utf-8"))
        imp_id = pkg.get("individualMissionPackageID") or pkg.get("individualMissionPlanPackageID")
        out = dir_imp / f"{imp_id}.json"
        out.write_text(json.dumps(pkg, ensure_ascii=False, indent=2), encoding="utf-8")
        saved_imp.append(out)

    # 0303/0304 (모두 FlightPath 폴더)
    for fp in fp_0303 + fp_0304:
        pid = fp.get("pathID")
        out = dir_fp / f"{pid}.json"
        out.write_text(json.dumps(fp, ensure_ascii=False, indent=2), encoding="utf-8")
        saved_fp.append(out)

    return mp_out, saved_imp, saved_fp


def run_once(seed: int | None, db_root: Path) -> Dict:
    """단일 생성 실행."""
    _inject_paths()
    # Force downstream modules to honor the provided DB root (input/target/ref + outputs).
    fpl_paths.set_db_root(db_root)

    # 지연 import (path 설정 이후)
    from fpl_random import pipeline as sg_pipeline
    from MP.modules.mission_planning.MissionPlanner.AnS import mission_pipeline
    from MP.modules.mission_planning.MissionPlanner import corridor_planner as cp
    from MP.modules.mission_planning.MissionPlanner.data_def import d0303, d0304
    from MP.modules.mission_planning.MissionPlanner.data_def.id_allocator import next_path_id

    # 1) 0201/0203/TGT
    # 1) 0201/0203/TGT (실패 시 시드 바꿔 재시도)
    gen = None
    attempts = 0
    while gen is None:
        try_seed = seed if seed is not None else None
        try:
            gen = sg_pipeline.generate_sequence(seed=try_seed, save=True)
        except Exception as e:
            attempts += 1
            if attempts >= 5:
                raise
            # 시드가 없거나 실패했으면 새로운 시드로 재시도
            seed = None
            continue
    cmpk_path = Path(gen["paths"]["input_mission_plan"])
    mrpk_path = Path(gen["paths"]["flight_reference"])

    # 2) divide & pattern (0302)
    out_dir = db_root / "mission_output_cli"
    out_dir.mkdir(parents=True, exist_ok=True)
    imp_paths_raw = mission_pipeline.run_divide_and_pattern(
        cmpk_path=str(cmpk_path),
        ref_path=str(mrpk_path),
        out_dir=str(out_dir),
        log=lambda *_: None,  # 화면 로그 생략 (cp949 문제 방지)
    )
    imp_paths = [Path(p) for p in imp_paths_raw]

    # pathID 정규화 (항공기별 범위/중복 제거)
    _normalize_path_ids(imp_paths)

    # 3) 0301 MissionPlan
    mp_tmp = out_dir / "MissionPlan_tmp.json"
    mission_pipeline.build_mission_plan_0301(
        cmpk_path=str(cmpk_path),
        mrpk_path=str(mrpk_path),
        imp_paths=[str(p) for p in imp_paths],
        mp_out_path=str(mp_tmp),
        mission_plan_id=None,
    )

    # 4) 0303 / 0304
    missions = _load_imp_missions(imp_paths)
    wp_alloc = d0303._WPAllocator()
    fp_0303 = d0303.build_flight_plans(missions, wp_alloc, cruise_speed=40.0, turn_step_deg=15.0)
    fp_0304 = d0304.build_lah_flight_plans_fixed(missions, cruise_speed=40.0, wp_alloc=wp_alloc)

    # 5) 저장
    mp_out, saved_imp, saved_fp = _save_outputs(mp_tmp, imp_paths, fp_0303, fp_0304, db_root)

    return {
        "0201": str(cmpk_path),
        "0203": str(mrpk_path),
        "TGT": gen["paths"]["targets"],
        "0301": str(mp_out),
        "0302": [str(p) for p in saved_imp],
        "0303/0304": [str(p) for p in saved_fp],
    }


def main():
    parser = argparse.ArgumentParser(description="GUI 없이 0201~0304 전체 생성")
    parser.add_argument("--count", type=int, default=1, help="생성 횟수")
    parser.add_argument("--seed", type=int, default=None, help="기본 시드 (없으면 무작위)")
    parser.add_argument(
        "--db-root",
        type=str,
        default=str(fpl_paths.db_root()),
        help="DB 루트 (MissionPlan/IndividualMissionPlan/FlightPath 상위)",
    )
    args = parser.parse_args()

    db_root = Path(args.db_root).resolve()

    results = []
    for i in range(args.count):
        seed = None if args.seed is None else args.seed + i
        results.append(run_once(seed=seed, db_root=db_root))

    print("=== 생성 완료 ===")
    for idx, r in enumerate(results, 1):
        print(f"[{idx}] 0201={r['0201']} | 0203={r['0203']} | TGT={r['TGT']}")
        print(f"     0301={r['0301']}")
        print(f"     0302={r['0302']}")
        print(f"     0303/0304={r['0303/0304']}")


if __name__ == "__main__":
    main()
