"""Occupancy-grid world model, image loading and raycasting.

Sim-only: the firmware has no counterpart. An occupancy grid rather than a
cell-edge wall bitmask because the real maze carries geometry a bitmask cannot
express -- chamfered 45 degree corners and free-standing obstacles. Rays march
the grid by DDA, which handles all of it without special cases.

Frame: +x forward, +y left, theta CCW, matching the firmware.
"""

import json
import math
import pathlib

from . import png

# World +x direction expressed in image space as (column step, row step).
# Image rows increase downward. World +y is 90 degrees CCW from +x as seen
# on screen, i.e. ey = (ex_row, -ex_col).
_X_AXES = {
    "up": (0.0, -1.0),
    "down": (0.0, 1.0),
    "right": (1.0, 0.0),
    "left": (-1.0, 0.0),
}


class GridGeometry:
    """Pixel/world mapping shared by World and Mapper so the ground-truth grid
    and the robot-built map stay in exact correspondence."""

    def __init__(self, width, height, mm_per_pixel, origin_px, x_axis="up"):
        if x_axis not in _X_AXES:
            raise ValueError(f"x_axis must be one of {sorted(_X_AXES)}, got {x_axis!r}")
        self.width = width
        self.height = height
        self.mm_per_pixel = float(mm_per_pixel)
        self.origin_px = (float(origin_px[0]), float(origin_px[1]))
        self.x_axis = x_axis
        self._ex = _X_AXES[x_axis]
        self._ey = (self._ex[1], -self._ex[0])

    def world_to_px(self, x_mm, y_mm):
        ox, oy = self.origin_px
        ex, ey = self._ex, self._ey
        s = self.mm_per_pixel
        return (
            ox + (x_mm * ex[0] + y_mm * ey[0]) / s,
            oy + (x_mm * ex[1] + y_mm * ey[1]) / s,
        )

    def px_to_world(self, px, py):
        ox, oy = self.origin_px
        ex, ey = self._ex, self._ey
        s = self.mm_per_pixel
        dx, dy = px - ox, py - oy
        return ((dx * ex[0] + dy * ex[1]) * s, (dx * ey[0] + dy * ey[1]) * s)

    def direction_to_px(self, theta):
        """Unit direction in pixel space for a world heading."""
        c, s = math.cos(theta), math.sin(theta)
        ex, ey = self._ex, self._ey
        return (c * ex[0] + s * ey[0], c * ex[1] + s * ey[1])

    def in_bounds(self, px, py):
        return 0 <= px < self.width and 0 <= py < self.height

    def bounds_mm(self):
        """World-space bounding box of the grid as (min_x, min_y, max_x, max_y)."""
        corners = [
            self.px_to_world(0, 0),
            self.px_to_world(self.width, 0),
            self.px_to_world(0, self.height),
            self.px_to_world(self.width, self.height),
        ]
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        return (min(xs), min(ys), max(xs), max(ys))


def _rle(values, is_set):
    """Run-length encode a sequence into alternating run lengths, always
    starting with a run of unset cells (possibly zero-length)."""
    runs = []
    current = False
    count = 0
    for v in values:
        bit = is_set(v)
        if bit == current:
            count += 1
        else:
            runs.append(count)
            current = bit
            count = 1
    runs.append(count)
    if runs and current is False and len(runs) == 1:
        return runs
    return runs


class World(GridGeometry):
    """Ground-truth occupancy. grid[y * width + x] is 1 for occupied."""

    def __init__(self, grid, width, height, mm_per_pixel, origin_px, x_axis="up"):
        super().__init__(width, height, mm_per_pixel, origin_px, x_axis)
        self.grid = grid

    # --- construction ---------------------------------------------------

    @classmethod
    def blank(cls, width_mm, height_mm, mm_per_pixel):
        """An empty arena centred on the world origin."""
        width = max(1, int(round(width_mm / mm_per_pixel)))
        height = max(1, int(round(height_mm / mm_per_pixel)))
        return cls(
            bytearray(width * height),
            width,
            height,
            mm_per_pixel,
            (width / 2.0, height / 2.0),
            "up",
        )

    @classmethod
    def from_image(cls, image_path, sidecar_path=None):
        """Load a world from an image: white is clear, black is occupied.

        A JSON sidecar beside the image supplies the pixel-to-millimetre
        mapping. Missing sidecar falls back to 5 mm/px with the origin at the
        bottom-left and world +x pointing up the image.
        """
        image_path = pathlib.Path(image_path)
        img = png.read(image_path)

        cfg = {}
        if sidecar_path is None:
            sidecar_path = image_path.with_suffix(".json")
        sidecar_path = pathlib.Path(sidecar_path)
        if sidecar_path.exists():
            cfg = json.loads(sidecar_path.read_text())

        mm_per_pixel = float(cfg.get("mm_per_pixel", 5.0))
        threshold = int(cfg.get("threshold", 128))
        x_axis = cfg.get("x_axis", "up")
        origin_px = cfg.get("origin_px", [0, img.height - 1])

        grid = bytearray(img.width * img.height)
        for y in range(img.height):
            row = y * img.width
            for x in range(img.width):
                if img.grey(x, y) < threshold:
                    grid[row + x] = 1

        return cls(grid, img.width, img.height, mm_per_pixel, origin_px, x_axis)

    # --- queries --------------------------------------------------------

    def occupied_px(self, px, py):
        """Outside the grid counts as occupied so the arena is bounded."""
        if not self.in_bounds(px, py):
            return True
        return self.grid[py * self.width + px] != 0

    def occupied_at(self, x_mm, y_mm):
        px, py = self.world_to_px(x_mm, y_mm)
        return self.occupied_px(int(math.floor(px)), int(math.floor(py)))

    def raycast(self, x_mm, y_mm, theta, max_range_mm):
        """Distance in mm from (x_mm, y_mm) along `theta` to the first occupied
        cell, or max_range_mm if nothing is hit first. Amanatides-Woo DDA over
        the grid, run in pixel space and converted back to mm at the end."""
        px, py = self.world_to_px(x_mm, y_mm)
        dx, dy = self.direction_to_px(theta)

        cx = int(math.floor(px))
        cy = int(math.floor(py))

        if self.occupied_px(cx, cy):
            return 0.0

        max_t_px = max_range_mm / self.mm_per_pixel

        step_x = 1 if dx > 0 else -1
        step_y = 1 if dy > 0 else -1

        if dx == 0.0:
            t_max_x = math.inf
            t_delta_x = math.inf
        else:
            next_x = cx + (1 if dx > 0 else 0)
            t_max_x = (next_x - px) / dx
            t_delta_x = abs(1.0 / dx)

        if dy == 0.0:
            t_max_y = math.inf
            t_delta_y = math.inf
        else:
            next_y = cy + (1 if dy > 0 else 0)
            t_max_y = (next_y - py) / dy
            t_delta_y = abs(1.0 / dy)

        while True:
            if t_max_x < t_max_y:
                t = t_max_x
                if t > max_t_px:
                    return max_range_mm
                cx += step_x
                t_max_x += t_delta_x
            else:
                t = t_max_y
                if t > max_t_px:
                    return max_range_mm
                cy += step_y
                t_max_y += t_delta_y

            if self.occupied_px(cx, cy):
                return t * self.mm_per_pixel

    # --- wire format ----------------------------------------------------

    def to_rle(self):
        """Run lengths of alternating clear/occupied cells in row-major order,
        starting with a clear run. Sums to width * height."""
        return _rle(self.grid, lambda v: v != 0)


class MapWorld(World):
    """Ground truth from an obstacle ``Map`` -- the same type, and normally the
    same instance, the firmware's LidarObserver casts into.

    Ranges come from ``Map.cast``, i.e. exact analytic geometry, not from the
    grid: a 5 mm DDA would put 2.5 mm of quantisation into every reading before
    the sensor model got its hands on it, which is the same order as the error
    the observer exists to correct. The occupancy grid is still built, because
    the viewer draws it and Mapper overlays against it, but nothing reads it
    for a range.

    Passing a *different* map to the observer than to this world is the point
    of keeping them separate: it is how you ask what a stale or mis-exported
    map does to the fix.
    """

    def __init__(self, map, mm_per_pixel=5.0, margin_mm=200.0, max_range_mm=None):
        from .types import Vec2D

        self.map = map
        self._Vec2D = Vec2D
        # 2 m is well past the VL6180X's 300 mm ceiling, so no reading is ever
        # limited by it; it only bounds the work a cast into open floor does.
        self.max_range_mm = 2000.0 if max_range_mm is None else float(max_range_mm)

        x0, y0, x1, y1 = map.bounds()
        x0 -= margin_mm
        y0 -= margin_mm
        x1 += margin_mm
        y1 += margin_mm

        width = max(1, int(math.ceil((x1 - x0) / mm_per_pixel)))
        height = max(1, int(math.ceil((y1 - y0) / mm_per_pixel)))

        # World +x runs right and +y runs up the image, so world (x0, y0) is the
        # bottom-left *corner* of the grid -- pixel (0, height), the outside
        # edge of the last row, not its centre.
        super().__init__(
            bytearray(width * height),
            width,
            height,
            mm_per_pixel,
            (-x0 / mm_per_pixel, height + y0 / mm_per_pixel),
            "right",
        )
        self._rasterise()

    def _rasterise(self):
        """Fill the occupancy grid by point-testing each cell centre against
        every obstacle's own solid region. For display and for Mapper only."""
        from .types import CircularObstacle, Vec2D, WallObstacle

        for o in self.map.obstacles:
            r = o.boundingRadius()
            px0, py0 = self.world_to_px(o.centre.x - r, o.centre.y - r)
            px1, py1 = self.world_to_px(o.centre.x + r, o.centre.y + r)
            lo_x, hi_x = sorted((int(math.floor(px0)), int(math.ceil(px1))))
            lo_y, hi_y = sorted((int(math.floor(py0)), int(math.ceil(py1))))

            for py in range(max(0, lo_y), min(self.height, hi_y + 1)):
                for px in range(max(0, lo_x), min(self.width, hi_x + 1)):
                    wx, wy = self.px_to_world(px + 0.5, py + 0.5)
                    if self._inside(o, Vec2D(wx, wy), CircularObstacle, WallObstacle):
                        self.grid[py * self.width + px] = 1

    @staticmethod
    def _inside(obstacle, point, CircularObstacle, WallObstacle):
        d = point - obstacle.centre
        form = obstacle.form
        if isinstance(form, CircularObstacle):
            return d.x * d.x + d.y * d.y <= form.radius * form.radius
        along_x, along_y = math.cos(form.alpha), math.sin(form.alpha)
        u = d.x * along_x + d.y * along_y
        v = -d.x * along_y + d.y * along_x
        return abs(u) <= 0.5 * form.length and abs(v) <= 0.5 * form.thickness

    def raycast(self, x_mm, y_mm, theta, max_range_mm):
        """Exact range to the nearest obstacle surface, through Map.cast."""
        Vec2D = self._Vec2D
        limit = min(max_range_mm, self.max_range_mm)
        hit = self.map.cast(
            Vec2D(x_mm, y_mm),
            Vec2D(math.cos(theta), math.sin(theta)),
            limit,
        )
        return hit.distance if hit.valid else max_range_mm

    def occupied_at(self, x_mm, y_mm):
        """Against the true geometry, not the raster -- so a collision check is
        not off by half a grid cell."""
        from .types import CircularObstacle, Vec2D, WallObstacle

        p = Vec2D(x_mm, y_mm)
        return any(
            self._inside(o, p, CircularObstacle, WallObstacle)
            for o in self.map.obstacles
        )
