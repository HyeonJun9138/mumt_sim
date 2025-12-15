"""Launch multiple simulator windows in parallel processes."""

import os
from pathlib import Path
import subprocess
import sys
import time


INSTANCE_COUNT = 6
TARGET_SCRIPT = "main.py"


def launch_instance(idx: int, script_path: Path) -> subprocess.Popen:
    env = os.environ.copy()
    env["SIM_INSTANCE_NAME"] = f"UAV SIM #{idx + 1}"
    print(f"[launcher] starting instance {idx + 1} with caption '{env['SIM_INSTANCE_NAME']}'")
    return subprocess.Popen(
        [sys.executable, str(script_path)],
        cwd=str(script_path.parent),
        env=env,
    )


def main() -> None:
    script_path = Path(__file__).resolve().parent / TARGET_SCRIPT
    if not script_path.exists():
        raise FileNotFoundError(f"target script not found: {script_path}")

    processes: list[subprocess.Popen] = []
    for i in range(INSTANCE_COUNT):
        processes.append(launch_instance(i, script_path))

    try:
        while processes:
            for proc in list(processes):
                ret = proc.poll()
                if ret is not None:
                    print(f"[launcher] instance pid={proc.pid} exited with code {ret}")
                    processes.remove(proc)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("[launcher] interrupted, terminating instances...")
    finally:
        for proc in processes:
            if proc.poll() is None:
                proc.terminate()
        for proc in processes:
            try:
                if proc.poll() is None:
                    proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()


if __name__ == "__main__":
    main()
