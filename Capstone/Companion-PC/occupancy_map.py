import math
import threading
import numpy as np

CELL = 0.25
SIZE = 160                    # 40m x 40m grid, origin at the centre
ORIGIN = SIZE // 2
FOV_DEG = 82.6                # Tello. Check the Unity camera FOV for the sim
N_RAYS = 32
MAX_RANGE = 4.0
BAND_Y0, BAND_Y1 = 0.30, 0.50     # image rows used, keep clear of the floor
L_FREE, L_OCC, L_MIN, L_MAX = -0.4, 0.85, -4.0, 4.0

# Your hand readings: depth value -> metres. The 4.0 end is a guess. Re-measure on the sim and the real drone.
CAL_VALUE = [2.0, 3.5, 4.5, 5.5]
CAL_METRES = [4.0, 2.0, 1.0, 0.15]


def depth_to_metres(v):
    return float(np.interp(v, CAL_VALUE, CAL_METRES))


class OccupancyMap:
    def __init__(self):
        self.logodds = np.zeros((SIZE, SIZE), np.float32)
        self.seen = np.zeros((SIZE, SIZE), bool)
        self.lock = threading.Lock()

    def _idx(self, x, z):
        return int(round(x / CELL)) + ORIGIN, int(round(z / CELL)) + ORIGIN

    def _ok(self, ix, iz):
        return 0 <= ix < SIZE and 0 <= iz < SIZE

    def update(self, depth_map, x, z, heading_deg):
        h, w = depth_map.shape
        band = depth_map[int(h * BAND_Y0):int(h * BAND_Y1), :]
        cols = np.array_split(np.arange(w), N_RAYS)
        with self.lock:
            for i, c in enumerate(cols):
                v = float(np.percentile(band[:, c], 85))     # closest part of the column
                r = depth_to_metres(v)
                hit = r < MAX_RANGE
                r = min(r, MAX_RANGE)
                bearing = math.radians(heading_deg + ((i + 0.5) / N_RAYS - 0.5) * FOV_DEG)
                dx, dz = math.sin(bearing), math.cos(bearing)
                for s in range(int(r / CELL)):
                    ix, iz = self._idx(x + dx * s * CELL, z + dz * s * CELL)
                    if self._ok(ix, iz):
                        self.logodds[iz, ix] = max(L_MIN, self.logodds[iz, ix] + L_FREE)
                        self.seen[iz, ix] = True
                if hit:
                    ix, iz = self._idx(x + dx * r, z + dz * r)
                    if self._ok(ix, iz):
                        self.logodds[iz, ix] = min(L_MAX, self.logodds[iz, ix] + L_OCC)
                        self.seen[iz, ix] = True

    def frontiers(self):
        with self.lock:
            free = self.logodds < -0.5
            pad = np.pad(~self.seen, 1, constant_values=False)
            near_unknown = (pad[:-2, 1:-1] | pad[2:, 1:-1] |
                            pad[1:-1, :-2] | pad[1:-1, 2:])
            zs, xs = np.nonzero(free & near_unknown)
        return [((xi - ORIGIN) * CELL, (zi - ORIGIN) * CELL) for xi, zi in zip(xs, zs)]

    def nearest_frontier(self, x, z, min_dist=0.8):
        best = None
        for fx, fz in self.frontiers():
            d = math.hypot(fx - x, fz - z)
            if d >= min_dist and (best is None or d < best[0]):
                best = (d, fx, fz)
        if best is None:
            return None
        d, fx, fz = best
        return math.degrees(math.atan2(fx - x, fz - z)) % 360, d

    def stats(self):
        with self.lock:
            free_cells = int((self.logodds < -0.5).sum())
        return {"explored_m2": round(free_cells * CELL * CELL, 1),
                "frontiers": len(self.frontiers())}

    def export(self, max_cells=2500):
        with self.lock:
            zs, xs = np.nonzero(self.logodds > 1.0)
        pts = [[round((xi - ORIGIN) * CELL, 2), round((zi - ORIGIN) * CELL, 2)]
               for xi, zi in zip(xs, zs)]
        return pts[:max_cells]

    def save(self, path="room_map.npz"):
        with self.lock:
            np.savez(path, logodds=self.logodds, seen=self.seen)