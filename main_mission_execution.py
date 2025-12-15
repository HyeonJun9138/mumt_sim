import os
from multiprocessing import freeze_support
from pathlib import Path

# Hide pygame banner in child processes spawned on Windows.
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

from sim.runtime.app import SimulationApp


def main():
    """Entry point for mission execution runner."""
    freeze_support()
    app = SimulationApp()
    # Default mission package (EP-000001); auto-pick first .json in MissionReferenceInfo.
    mission_root = Path("log") / "EP-000001" / "database_EP_000001"
    ref_dir = mission_root / "MissionReferenceInfo"
    ref_files = sorted(ref_dir.glob("*.json"))
    if ref_files:
        app.mission_reference_path = ref_files[0]
    app.flight_path_root = mission_root / "FlightPath"
    app.enable_flight_paths = True
    app.run()


if __name__ == "__main__":
    main()
