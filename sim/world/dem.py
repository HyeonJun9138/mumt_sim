import math
import numpy as np
import rasterio
from rasterio.errors import RasterioIOError
from rasterio.transform import from_origin, Affine
import threading
from pathlib import Path
from typing import Optional, List, Tuple
from multiprocessing import shared_memory

from sim.config import WORLD_HALF, clamp, MAP_DIR


class DEM:
    def __init__(self, path=None, *, shared: dict | None = None):
        """
        If 'shared' metadata is provided, attach to an existing shared-memory DEM
        buffer instead of loading tiles from disk. This avoids per-worker raster
        loads when multiple processes run in parallel.
        """
        if shared is not None:
            self._init_from_shared(shared)
            return

        path = Path(path)
        root_dir = path.parent if path.is_file() else Path(path)
        tif_paths = sorted(root_dir.glob("*.tif"))
        if not tif_paths:
            raise FileNotFoundError(f"No DEM .tif found in {root_dir}")

        self.tiles: List[Tuple[rasterio.DatasetReader, tuple]] = []
        self.bad_tiles: set[int] = set()
        self.synthetic = False
        self._lock = threading.Lock()
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

        # Whole-world env bounds across all tiles (for clamping/logging)
        env_bounds = []
        for _, b in self.tiles:
            xmin, ymin, xmax, ymax = b
            corners = [(xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax)]
            env_corners = [self.lonlat_to_env(lon, lat) for lon, lat in corners]
            xs, ys = zip(*env_corners)
            env_bounds.append((min(xs), max(xs), min(ys), max(ys)))
        xs_all = [b[0] for b in env_bounds] + [b[1] for b in env_bounds]
        ys_all = [b[2] for b in env_bounds] + [b[3] for b in env_bounds]
        self.world_env_bounds = (min(xs_all), max(xs_all), min(ys_all), max(ys_all))

        self.active_idx = init_idx
        ok = self._set_active_tile(init_idx)
        if not ok:
            fallback_idx = None
            for i in range(len(self.tiles)):
                if i in self.bad_tiles or i == init_idx:
                    continue
                if self._set_active_tile(i):
                    fallback_idx = i
                    print(
                        f"[DEM] initial tile failed ({self.tiles[init_idx][0].name}); using fallback {self.tiles[i][0].name}"
                    )
                    break
            if fallback_idx is None:
                self._build_flat_dem()
                print(
                    f"[DEM] WARNING: Failed to read any DEM tile (tried {len(self.tiles)} file(s); first failure: {self.tiles[init_idx][0].name}). Using synthetic flat DEM."
                )

    def _init_from_shared(self, shared: dict):
        name = shared.get("name")
        shape = tuple(shared.get("shape", ()))
        dtype = np.dtype(shared.get("dtype", "float32"))
        self.shared_mem = shared_memory.SharedMemory(name=name)
        self.elevation = np.ndarray(shape=shape, dtype=dtype, buffer=self.shared_mem.buf)
        self._lock = threading.Lock()
        self.tiles = []
        self.bad_tiles = set()
        self.synthetic = shared.get("synthetic", False)
        self.ref_lon = shared.get("ref_lon", 0.0)
        self.ref_lat = shared.get("ref_lat", 0.0)
        self.m_per_deg_lon = shared.get("scale_x", 111320.0)
        self.m_per_deg_lat = shared.get("scale_y", 111320.0)
        self.scale_x = self.m_per_deg_lon
        self.scale_y = self.m_per_deg_lat
        self._bounds = tuple(shared.get("bounds", (0.0, 0.0, 0.0, 0.0)))
        self.xmin, self.ymin, self.xmax, self.ymax = self._bounds
        self.env_bounds = tuple(shared.get("env_bounds", (0.0, 0.0, 0.0, 0.0)))
        self.world_env_bounds = tuple(shared.get("world_env_bounds", self.env_bounds))
        self.active_idx = shared.get("active_idx", 0)
        transform_params = shared.get("transform")
        self.transform = Affine(*transform_params) if transform_params else None
        self.min_elev = float(shared.get("min_elev", np.nanmin(self.elevation)))
        self.max_elev = float(shared.get("max_elev", np.nanmax(self.elevation)))

    def _set_active_tile(self, idx: int) -> bool:
        ds = self.tiles[idx][0]
        try:
            elevation = ds.read(1).astype(np.float32)
        except RasterioIOError as e:
            self.bad_tiles.add(idx)
            print(f"[DEM] failed to read tile {ds.name}: {e}")
            return False

        self.active_idx = idx
        self.dataset = ds
        self.elevation = elevation
        self.transform = ds.transform
        self.min_elev = float(np.nanmin(self.elevation))
        self.max_elev = float(np.nanmax(self.elevation))
        self._bounds = ds.bounds
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
        self.synthetic = False
        return True

    def _build_flat_dem(self):
        """Create a synthetic flat DEM when no tiles are readable."""
        self.synthetic = True
        self.bad_tiles = set(range(len(self.tiles)))
        self.ref_lon = 127.5
        self.ref_lat = 37.5
        self.m_per_deg_lat = 111320.0
        self.m_per_deg_lon = math.cos(math.radians(self.ref_lat)) * 111320.0
        self.scale_x = self.m_per_deg_lon
        self.scale_y = self.m_per_deg_lat

        size = 512
        pix_deg = 1.0 / 3600.0  # ~30m per pixel
        lon_ul = self.ref_lon - (size * pix_deg) / 2.0
        lat_ul = self.ref_lat + (size * pix_deg) / 2.0
        self.transform = from_origin(lon_ul, lat_ul, pix_deg, pix_deg)
        self.elevation = np.zeros((size, size), dtype=np.float32)
        self.min_elev = 0.0
        self.max_elev = 0.0
        self._bounds = (lon_ul, lat_ul - size * pix_deg, lon_ul + size * pix_deg, lat_ul)
        self.xmin, self.ymin, self.xmax, self.ymax = self._bounds
        corners = [
            (self.xmin, self.ymin),
            (self.xmin, self.ymax),
            (self.xmax, self.ymin),
            (self.xmax, self.ymax),
        ]
        env_corners = [self.lonlat_to_env(lon, lat) for lon, lat in corners]
        xs, ys = zip(*env_corners)
        self.env_bounds = (min(xs), max(xs), min(ys), max(ys))
        self.active_idx = -1
        self.dataset = None

    def _find_tile_idx(self, lon: float, lat: float) -> Optional[int]:
        if self.synthetic:
            xmin, ymin, xmax, ymax = self._bounds
            return 0 if (xmin <= lon <= xmax and ymin <= lat <= ymax) else None
        for i, (_, b) in enumerate(self.tiles):
            xmin, ymin, xmax, ymax = b
            if xmin <= lon <= xmax and ymin <= lat <= ymax:
                return i
        return None

    def ensure_tile_for_env(self, x_env: float, y_env: float):
        lon, lat = self.env_to_lonlat(x_env, y_env)
        idx = self._find_tile_idx(lon, lat)
        if idx is None or idx == self.active_idx or idx in self.bad_tiles:
            return
        with self._lock:
            # double-check after acquiring lock to avoid redundant switches
            if idx == self.active_idx or idx in self.bad_tiles:
                return
            ok = self._set_active_tile(idx)
            if not ok:
                # stay on previous tile and skip bad tile in future
                return

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

    def get_world_env_bounds(self):
        return self.world_env_bounds

    def get_height(self, x_env, y_env):
        # serialize tile switches to avoid thrash across threads when agents are spread out
        with self._lock:
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
