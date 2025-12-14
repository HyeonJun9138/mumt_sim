import math
import numpy as np
import rasterio
from pathlib import Path
from typing import Optional, List, Tuple

from sim.config import WORLD_HALF, clamp, MAP_DIR


class DEM:
    def __init__(self, path):
        path = Path(path)
        root_dir = path.parent if path.is_file() else Path(path)
        tif_paths = sorted(root_dir.glob("*.tif"))
        if not tif_paths:
            raise FileNotFoundError(f"No DEM .tif found in {root_dir}")

        self.tiles: List[Tuple[rasterio.DatasetReader, tuple]] = []
        for p in tif_paths:
            ds = rasterio.open(p)
            self.tiles.append((ds, ds.bounds))

        # pick initial tile (given path if present, else first)
        init_idx = 0
        for i, (ds, _) in enumerate(self.tiles):
            if ds.name == str(path):
                init_idx = i
                break

        # Global reference origin/scale (fixed)
        ref_ds = self.tiles[init_idx][0]
        ref_bounds = ref_ds.bounds
        self.ref_lon = 0.5 * (ref_bounds.left + ref_bounds.right)
        self.ref_lat = 0.5 * (ref_bounds.bottom + ref_bounds.top)
        self.m_per_deg_lat = 111320.0
        self.m_per_deg_lon = math.cos(math.radians(self.ref_lat)) * 111320.0
        self.scale_x = self.m_per_deg_lon
        self.scale_y = self.m_per_deg_lat

        self.active_idx = init_idx
        self._set_active_tile(init_idx)

    def _set_active_tile(self, idx: int):
        self.active_idx = idx
        self.dataset = self.tiles[idx][0]
        self.elevation = self.dataset.read(1).astype(np.float32)
        self.transform = self.dataset.transform
        self.min_elev = float(np.nanmin(self.elevation))
        self.max_elev = float(np.nanmax(self.elevation))
        self._bounds = self.dataset.bounds
        self.xmin, self.ymin, self.xmax, self.ymax = self._bounds

        # Local ENU world in meters centered on the DEM footprint (1:1 scale)
        corners = [
            (self.xmin, self.ymin),
            (self.xmin, self.ymax),
            (self.xmax, self.ymin),
            (self.xmax, self.ymax),
        ]
        env_corners = [self.lonlat_to_env(lon, lat) for lon, lat in corners]
        xs, ys = zip(*env_corners)
        self.env_bounds = (min(xs), max(xs), min(ys), max(ys))

    def _find_tile_idx(self, lon: float, lat: float) -> Optional[int]:
        for i, (_, b) in enumerate(self.tiles):
            xmin, ymin, xmax, ymax = b
            if xmin <= lon <= xmax and ymin <= lat <= ymax:
                return i
        return None

    def ensure_tile_for_env(self, x_env: float, y_env: float):
        lon, lat = self.env_to_lonlat(x_env, y_env)
        idx = self._find_tile_idx(lon, lat)
        if idx is not None and idx != self.active_idx:
            self._set_active_tile(idx)

    def env_to_dataset_xy(self, x_env, y_env):
        lon, lat = self.env_to_lonlat(x_env, y_env)
        col, row = ~self.transform * (lon, lat)
        col = int(np.clip(col, 0, self.elevation.shape[1] - 1))
        row = int(np.clip(row, 0, self.elevation.shape[0] - 1))
        return row, col

    def env_to_lonlat(self, x_env: float, y_env: float):
        lon = self.ref_lon + x_env / self.scale_x
        lat = self.ref_lat + y_env / self.scale_y
        return lon, lat

    def lonlat_to_env(self, lon: float, lat: float):
        x_env = (lon - self.ref_lon) * self.scale_x
        y_env = (lat - self.ref_lat) * self.scale_y
        return x_env, y_env

    def get_env_bounds(self):
        return self.env_bounds

    def get_height(self, x_env, y_env):
        self.ensure_tile_for_env(x_env, y_env)
        r, c = self.env_to_dataset_xy(x_env, y_env)
        return float(self.elevation[r, c])


def ray_intersect_dem(uav_pos, d, dem: DEM, max_dist=5000.0, step=5.0):
    p = np.array(uav_pos, dtype=float)
    dist = 0.0
    while dist < max_dist:
        p += d * step
        dist += step
        ground_z = dem.get_height(p[0], p[1])
        if p[2] <= ground_z:
            p_back = p - d * step
            for alpha in np.linspace(0, 1, 10):
                test = p_back + alpha * (p - p_back)
                if test[2] <= dem.get_height(test[0], test[1]):
                    return np.array([test[0], test[1], dem.get_height(test[0], test[1])])
            return np.array([p[0], p[1], ground_z])
    return None


def check_los(uav_pos, tgt_pos, dem: DEM, step=5.0):
    dir_vec = tgt_pos - uav_pos
    dist_total = np.linalg.norm(dir_vec)
    if dist_total < 1e-6:
        return True
    d = dir_vec / dist_total
    p = np.array(uav_pos, dtype=float)
    traveled = 0.0
    while traveled < dist_total:
        p += d * step
        traveled += step
        ground_z = dem.get_height(p[0], p[1])
        if p[2] <= ground_z:
            return False
    return True
