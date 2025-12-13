from pathlib import Path

# Window / world settings
WIN_W, WIN_H = 1280, 800
FPS = 60
WORLD_HALF = 2500.0
RENDER_RADIUS_M = 2000.0  # visible draw radius around UAV
DEM_PREFETCH_RADIUS_M = 2500.0  # DEM window to fetch/draw
DEM_CACHE_MOVE_THRESHOLD_M = 500.0  # recompute DEM window after moving this far

# Paths
REPO_ROOT = Path(__file__).resolve().parent.parent
MAP_DIR = REPO_ROOT / "resources" / "map"
DEFAULT_DEM_NAME = "n37_e127_1arc_v3.tif"
DEM_FILE = MAP_DIR / DEFAULT_DEM_NAME

if not DEM_FILE.exists():
    tif_files = sorted(MAP_DIR.glob("*.tif"))
    if tif_files:
        DEM_FILE = tif_files[0]
    else:
        raise FileNotFoundError(f"No DEM .tif found in {MAP_DIR}")

# Rendering defaults
DEFAULT_FOV_DIAG = 10.0
FOG_COLOR = (0.06, 0.07, 0.09, 1.0)
FOG_DENSITY = 0.0008
CLEAR_COLOR = (0.06, 0.07, 0.09, 1.0)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_deg(a: float) -> float:
    a %= 360.0
    if a < 0:
        a += 360.0
    return a
