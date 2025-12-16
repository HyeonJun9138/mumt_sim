from pathlib import Path

# Window / world settings
WIN_W, WIN_H = 1280, 800
FPS = 60
WORLD_HALF = 2500.0
# Increase the visible bubble so mission targets/paths that are several km away
# from the active aircraft still render in the 3D view.
RENDER_RADIUS_M = 15000.0  # visible draw radius around UAV
DEM_PREFETCH_RADIUS_M = 18000.0  # DEM window to fetch/draw
DEM_CACHE_MOVE_THRESHOLD_M = 500.0  # recompute DEM window after moving this far

# Paths
REPO_ROOT = Path(__file__).resolve().parent.parent
MAP_DIR = REPO_ROOT / "resources" / "map"
# Start near mission area (128E) to reduce tile thrash when spawning far east.
DEFAULT_DEM_NAME = "n37_e128_1arc_v3.tif"
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
FOG_DENSITY = 0.0005
CLEAR_COLOR = (0.06, 0.07, 0.09, 1.0)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_deg(a: float) -> float:
    a %= 360.0
    if a < 0:
        a += 360.0
    return a
