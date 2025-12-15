"""Automate one or many simulation episodes end-to-end (generator -> sim -> logs)."""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
FPL_ROOT = REPO_ROOT / "FPL_Random"
if str(FPL_ROOT) not in sys.path:
    sys.path.insert(0, str(FPL_ROOT))

from FPL_Random.run_generator import run_once as generate_once
from FPL_Random.fpl_random import paths as fpl_paths


def _next_episode_dir(base: Path) -> tuple[str, Path]:
    base.mkdir(parents=True, exist_ok=True)
    max_idx = -1
    for p in base.iterdir():
        if not p.is_dir():
            continue
        m = re.match(r"EP-(\d+)(?:@)?$", p.name)
        if m:
            try:
                max_idx = max(max_idx, int(m.group(1)))
            except ValueError:
                continue
    idx = max_idx + 1
    def _tag(i: int) -> str:
        return f"EP-{i:06d}"
    while idx < 1_000_0000:
        tag = _tag(idx)
        candidate = base / tag
        try:
            candidate.mkdir()
            return tag, candidate
        except FileExistsError:
            idx += 1
            continue
    raise RuntimeError("No available episode IDs.")


def _build_episode_paths(ep_dir: Path, tag: str) -> tuple[Path, Path, Path]:
    db_dir = ep_dir / f"database_{tag}"
    log0401 = ep_dir / f"0401_{tag}"
    log0402 = ep_dir / f"0402_{tag}"
    for d in (db_dir, log0401, log0402):
        d.mkdir(parents=True, exist_ok=True)
    return db_dir, log0401, log0402


def _run_sim(db_root: Path, log0401: Path, log0402: Path, mission_name: str, sim_seconds: float, time_scale: float, exit_on_done: bool, sim_timeout: float, headless: bool) -> None:
    env = os.environ.copy()
    env.update(
        {
            "PYGAME_HIDE_SUPPORT_PROMPT": "1",
            "FPL_DB_ROOT": str(db_root),
            "SIM_DB_ROOT": str(db_root),
            "SIM_LOG_0401_DIR": str(log0401),
            "SIM_LOG_0402_DIR": str(log0402),
            "SIM_AUTOSTART": "1",
            "SIM_EXIT_AFTER_SEC": f"{sim_seconds}",
            "SIM_EXIT_ON_MISSION_DONE": "1" if exit_on_done else "0",
            "SIM_TIME_SCALE": f"{time_scale}",
            "SIM_MISSION_NAME": mission_name,
        }
    )
    if headless:
        env["SIM_HEADLESS"] = "1"
        env.setdefault("SDL_VIDEODRIVER", "dummy")
    cmd = [sys.executable, str(REPO_ROOT / "main.py")]
    subprocess.run(cmd, cwd=str(REPO_ROOT), env=env, check=True, timeout=sim_timeout)


def run_episode(ep_idx: int, args) -> dict:
    tag, ep_dir = _next_episode_dir(args.out_dir)
    db_root, log0401, log0402 = _build_episode_paths(ep_dir, tag)

    seed = args.seed + ep_idx if args.seed is not None else None
    print(f"[{tag}] generating missions (seed={seed}) -> {db_root}")
    fpl_paths.set_db_root(db_root)
    gen_result = generate_once(seed=seed, db_root=db_root)

    mission_name = f"{tag}_{int(time.time())}"
    sim_timeout = args.sim_timeout if args.sim_timeout is not None else args.sim_seconds + 30.0
    print(
        f"[{tag}] launching sim (time_scale={args.time_scale}, auto-exit={args.sim_seconds}s, exit_on_done={args.exit_on_done})"
    )
    _run_sim(
        db_root=db_root,
        log0401=log0401,
        log0402=log0402,
        mission_name=mission_name,
        sim_seconds=args.sim_seconds,
        time_scale=args.time_scale,
        exit_on_done=args.exit_on_done,
        sim_timeout=sim_timeout,
        headless=args.headless,
    )

    return {
        "tag": tag,
        "episode_dir": str(ep_dir),
        "db_root": str(db_root),
        "log0401_dir": str(log0401),
        "log0402_dir": str(log0402),
        "generator_outputs": gen_result,
    }


def parse_args():
    parser = argparse.ArgumentParser(description="Run generator + simulator episodes automatically.")
    parser.add_argument("--count", type=int, default=1, help="Number of episodes to run.")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=REPO_ROOT / "episodes",
        help="Root directory to store EP-xxxxx@ folders.",
    )
    parser.add_argument("--seed", type=int, default=None, help="Base seed (episode idx added).")
    parser.add_argument("--sim-seconds", type=float, default=120.0, help="Wall-clock seconds before auto-exit.")
    parser.add_argument(
        "--time-scale", type=float, default=2.0, help="Initial simulator time scale (SIM_TIME_SCALE)."
    )
    parser.add_argument(
        "--exit-on-done",
        action="store_true",
        default=True,
        help="Exit early when all missions complete.",
    )
    parser.add_argument(
        "--no-exit-on-done",
        dest="exit_on_done",
        action="store_false",
        help="Keep running until sim-seconds even if missions finish.",
    )
    parser.add_argument(
        "--sim-timeout",
        type=float,
        default=None,
        help="Subprocess timeout safeguard (defaults to sim-seconds + 30s).",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run simulator without opening a window (sets SIM_HEADLESS).",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    results = []
    for i in range(args.count):
        try:
            results.append(run_episode(i, args))
        except subprocess.TimeoutExpired:
            print(f"[episode {i}] simulator timed out.")
        except subprocess.CalledProcessError as e:
            print(f"[episode {i}] simulator failed: {e}")
        except Exception as e:
            print(f"[episode {i}] failed: {e}")
    print("=== episodes complete ===")
    for r in results:
        print(f"{r['tag']}: db_root={r['db_root']} | logs={r['log0401_dir']}, {r['log0402_dir']}")


if __name__ == "__main__":
    main()
