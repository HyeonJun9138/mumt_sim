from pathlib import Path

# Window / world settings
WIN_W, WIN_H = 1280, 800
FPS = 60
WORLD_HALF = 2500.0

# Paths
REPO_ROOT = Path(__file__).resolve().parents[2]
DEM_FILE = REPO_ROOT / "YEOJU.img"

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
