"""Log-odds occupancy map accumulated from lidar returns.

Sim-only, and a development aid rather than a model of anything the firmware
does today: there is no mapping code on the robot yet. It integrates against
whichever pose it is handed, and Runner hands it the *fused estimate*, not
ground truth -- so the map smears by exactly as much as the estimator drifts.
Comparing it against the ground-truth occupancy in the viewer is the quickest
read on how good the pose estimate actually is.
"""

import math
from array import array

from .plant import LIDAR_MOUNTS
from .world import GridGeometry

# Per-observation log-odds updates. Occupied evidence is weighted heavier than
# free evidence so a single clean return outweighs the free cells the ray
# swept getting there.
LOG_ODDS_FREE = -0.4
LOG_ODDS_OCCUPIED = 0.85

# Saturation. Without a ceiling a long stare at one wall makes a cell
# impossible to re-label when the estimate drifts and the wall "moves".
LOG_ODDS_LIMIT = 5.0


class Mapper(GridGeometry):
    def __init__(self, width, height, mm_per_pixel, origin_px, x_axis="up"):
        super().__init__(width, height, mm_per_pixel, origin_px, x_axis)
        self.log_odds = array("f", [0.0]) * (width * height)

    @classmethod
    def matching(cls, world):
        """A map sharing a world's geometry exactly, so the two overlay."""
        return cls(
            world.width,
            world.height,
            world.mm_per_pixel,
            world.origin_px,
            world.x_axis,
        )

    def clear(self):
        self.log_odds = array("f", [0.0]) * (self.width * self.height)

    def log_odds_at(self, px, py):
        if not self.in_bounds(px, py):
            return 0.0
        return self.log_odds[py * self.width + px]

    def _bump(self, px, py, delta):
        if not self.in_bounds(px, py):
            return
        i = py * self.width + px
        v = self.log_odds[i] + delta
        if v > LOG_ODDS_LIMIT:
            v = LOG_ODDS_LIMIT
        elif v < -LOG_ODDS_LIMIT:
            v = -LOG_ODDS_LIMIT
        self.log_odds[i] = v

    def integrate(self, pose, readings, max_range_mm):
        """Fold one set of lidar readings into the map.

        pose: the robot pose to integrate against -- pass the estimate, not
              ground truth, if you want the map to show estimator error.
        readings: {sensor name: range in mm}
        max_range_mm: readings at or above this saturated, so they carry no
              evidence of an obstacle -- only that the swept cells are free.
        """
        for name, distance in readings.items():
            mount = LIDAR_MOUNTS.get(name)
            if mount is None:
                continue

            c, s = math.cos(pose.theta), math.sin(pose.theta)
            ox = pose.x + mount.x * c - mount.y * s
            oy = pose.y + mount.x * s + mount.y * c
            heading = pose.theta + mount.theta

            saturated = distance >= max_range_mm
            span = min(distance, max_range_mm)

            dx, dy = math.cos(heading), math.sin(heading)
            step = self.mm_per_pixel * 0.5

            last_cell = None
            travelled = 0.0
            while travelled < span:
                px, py = self.world_to_px(ox + dx * travelled, oy + dy * travelled)
                cell = (int(math.floor(px)), int(math.floor(py)))
                if cell != last_cell:
                    self._bump(cell[0], cell[1], LOG_ODDS_FREE)
                    last_cell = cell
                travelled += step

            if not saturated:
                px, py = self.world_to_px(ox + dx * span, oy + dy * span)
                self._bump(int(math.floor(px)), int(math.floor(py)), LOG_ODDS_OCCUPIED)

    def to_rle(self, threshold=0.0):
        """Same wire format as World.to_rle(): alternating run lengths in
        row-major order, starting with a clear run."""
        from .world import _rle

        return _rle(self.log_odds, lambda v: v > threshold)
