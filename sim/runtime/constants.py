from sim.config import REPO_ROOT
from sim.entities.threat import AirDefenseThreat, RadarParams, WeaponParams, WeaponType

# (x, y) in meters; edit this list to pre-place multiple targets
TARGET_SPAWNS = [
    (-200.0, -150.0),
    (250.0, 180.0),
]
TARGET_ROAM_RADIUS_M = 250.0  # each target roams within ~500m diameter around its spawn
TARGET_SPEED_RANGE = (2.0, 7.0)  # m/s

# Threat defaults (moderate)
THREAT_RADAR = RadarParams()  # P_fa=1e-6, n=1.5, t_ref=5s ...
THREAT_WEAPON = WeaponParams(
    weapon_type=WeaponType.MISSILE,
    a_range=3500.0,
    b_slope=2.0,
    omega=0.8,
    t_fire=3.0,
)
THREAT_DEFAULT = AirDefenseThreat(radar=THREAT_RADAR, weapon=THREAT_WEAPON)

# Initial offsets used to spread the aircraft out on spawn/reset
INITIAL_OFFSETS = [
    (-60.0, -60.0),
    (0.0, -60.0),
    (60.0, -60.0),
    (-60.0, 60.0),
    (0.0, 60.0),
    (60.0, 60.0),
]

SCENARIO_PATH = REPO_ROOT / "mission" / "Scenario_2025-12-14T013406" / "SBC3"
