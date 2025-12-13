import math
import numpy as np
import rasterio
from typing import Optional

from sim.config import WORLD_HALF, clamp


class DEM:
    def __init__(self, path):
        self.dataset = rasterio.open(path)
        self.elevation = self.dataset.read(1).astype(np.float32)
        self.transform = self.dataset.transform
        self.min_elev = float(np.nanmin(self.elevation))
        self.max_elev = float(np.nanmax(self.elevation))
        self._bounds = self.dataset.bounds
        self.xmin, self.ymin, self.xmax, self.ymax = self._bounds

        # Local ENU world in meters centered on the DEM footprint (1:1 scale)
        self.lon0 = 0.5 * (self.xmin + self.xmax)
        self.lat0 = 0.5 * (self.ymin + self.ymax)
        self.m_per_deg_lat = 111320.0
        self.m_per_deg_lon = math.cos(math.radians(self.lat0)) * 111320.0
        self.width_m = (self.xmax - self.xmin) * self.m_per_deg_lon
        self.height_m = (self.ymax - self.ymin) * self.m_per_deg_lat
        self.env_xmin, self.env_xmax = -0.5 * self.width_m, 0.5 * self.width_m
        self.env_ymin, self.env_ymax = -0.5 * self.height_m, 0.5 * self.height_m
        # 1:1 scaling (meters <-> env units)
        self.scale_x = self.m_per_deg_lon
        self.scale_y = self.m_per_deg_lat
        self.env_bounds = (self.env_xmin, self.env_xmax, self.env_ymin, self.env_ymax)

    def env_to_dataset_xy(self, x_env, y_env):
        lon, lat = self.env_to_lonlat(x_env, y_env)
        col, row = ~self.transform * (lon, lat)
        col = int(np.clip(col, 0, self.elevation.shape[1] - 1))
        row = int(np.clip(row, 0, self.elevation.shape[0] - 1))
        return row, col

    def env_to_lonlat(self, x_env: float, y_env: float):
        lon = self.lon0 + x_env / self.scale_x
        lat = self.lat0 + y_env / self.scale_y
        return lon, lat

    def lonlat_to_env(self, lon: float, lat: float):
        x_env = (lon - self.lon0) * self.scale_x
        y_env = (lat - self.lat0) * self.scale_y
        return x_env, y_env

    def get_env_bounds(self):
        return self.env_bounds

    def get_height(self, x_env, y_env):
        r, c = self.env_to_dataset_xy(x_env, y_env)
        return float(self.elevation[r, c])


def ray_intersect_dem(uav_pos, d, dem: DEM, max_dist=5000.0, step=10.0):
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


def check_los(uav_pos, tgt_pos, dem: DEM, step=10.0):
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
