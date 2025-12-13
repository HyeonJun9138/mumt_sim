import numpy as np
import rasterio
from typing import Optional

from config import WORLD_HALF, clamp


class DEM:
    def __init__(self, path):
        self.dataset = rasterio.open(path)
        self.elevation = self.dataset.read(1).astype(np.float32)
        self.transform = self.dataset.transform
        self._bounds = self.dataset.bounds
        self.xmin, self.ymin, self.xmax, self.ymax = self._bounds
        self.env_xmin, self.env_xmax = -WORLD_HALF, WORLD_HALF
        self.env_ymin, self.env_ymax = -WORLD_HALF, WORLD_HALF
        self.scale_x = (self.env_xmax - self.env_xmin) / (self.xmax - self.xmin)
        self.scale_y = (self.env_ymax - self.env_ymin) / (self.ymax - self.ymin)

    def env_to_dataset_xy(self, x_env, y_env):
        x = self.xmin + (x_env - self.env_xmin) / self.scale_x
        y = self.ymin + (y_env - self.env_ymin) / self.scale_y
        col, row = ~self.transform * (x, y)
        col = int(np.clip(col, 0, self.elevation.shape[1] - 1))
        row = int(np.clip(row, 0, self.elevation.shape[0] - 1))
        return row, col

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
