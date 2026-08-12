"""MapWorld: ground truth built from an obstacle Map.

The property that matters is that ranges come from the analytic cast, not from
the raster -- a grid fine enough to hide the difference would be slow, and a
grid coarse enough to be fast puts millimetres of quantisation into the readings
the observer is trying to resolve to millimetres.
"""

import math
import unittest

from ..constants import MAZE_WALL_THICKNESS
from ..types import CircularObstacle, Map, Obstacle, Vec2D, WallObstacle
from ..world import MapWorld


def wall_at(x, alpha=math.pi / 2, length=1000.0):
    return Obstacle(WallObstacle(length, MAZE_WALL_THICKNESS, alpha), Vec2D(x, 0.0))


class TestMapWorld(unittest.TestCase):
    def setUp(self):
        self.map = Map([wall_at(500.0)])
        self.w = MapWorld(self.map, mm_per_pixel=5.0)

    def test_range_is_exact_not_quantised_to_the_grid(self):
        """The face is at 500 - 6 = 494 mm, which is not a multiple of the 5 mm
        raster. A DDA over the grid could not return it."""
        d = self.w.raycast(0.0, 0.0, 0.0, 2000.0)
        self.assertAlmostEqual(d, 494.0, places=3)

    def test_range_from_an_offset_origin(self):
        d = self.w.raycast(100.0, 0.0, 0.0, 2000.0)
        self.assertAlmostEqual(d, 394.0, places=3)

    def test_oblique_range(self):
        a = 0.3
        d = self.w.raycast(0.0, 0.0, a, 2000.0)
        self.assertAlmostEqual(d, 494.0 / math.cos(a), places=3)

    def test_a_miss_returns_max_range(self):
        self.assertAlmostEqual(self.w.raycast(0.0, 0.0, math.pi, 800.0), 800.0)

    def test_max_range_is_respected(self):
        self.assertAlmostEqual(self.w.raycast(0.0, 0.0, 0.0, 100.0), 100.0)

    def test_occupied_at_uses_true_geometry(self):
        self.assertTrue(self.w.occupied_at(500.0, 0.0))
        self.assertTrue(self.w.occupied_at(495.0, 0.0))  # inside the 12 mm panel
        self.assertFalse(self.w.occupied_at(480.0, 0.0))
        self.assertFalse(self.w.occupied_at(500.0, 900.0))  # past the panel's end

    def test_the_raster_covers_the_obstacle(self):
        """The grid is for the viewer and for Mapper to overlay against, so it
        still has to be in the right place even though nothing ranges off it."""
        px, py = self.w.world_to_px(500.0, 0.0)
        self.assertTrue(self.w.occupied_px(int(px), int(py)))

    def test_the_raster_is_clear_where_the_map_is(self):
        px, py = self.w.world_to_px(0.0, 0.0)
        self.assertFalse(self.w.occupied_px(int(px), int(py)))

    def test_bounds_contain_the_map_with_margin(self):
        x0, y0, x1, y1 = self.w.bounds_mm()
        mx0, my0, mx1, my1 = self.map.bounds()
        self.assertLess(x0, mx0)
        self.assertGreater(x1, mx1)
        self.assertLess(y0, my0)
        self.assertGreater(y1, my1)

    def test_rle_covers_every_cell(self):
        self.assertEqual(sum(self.w.to_rle()), self.w.width * self.w.height)

    def test_a_circle_rasterises_and_ranges(self):
        m = Map([Obstacle(CircularObstacle(50.0), Vec2D(300.0, 0.0))])
        w = MapWorld(m, mm_per_pixel=5.0)
        self.assertAlmostEqual(w.raycast(0.0, 0.0, 0.0, 2000.0), 250.0, places=3)
        px, py = w.world_to_px(300.0, 0.0)
        self.assertTrue(w.occupied_px(int(px), int(py)))

    def test_an_empty_map_still_builds(self):
        w = MapWorld(Map([]), mm_per_pixel=20.0)
        self.assertGreater(w.width, 0)
        self.assertAlmostEqual(w.raycast(0.0, 0.0, 0.0, 500.0), 500.0)


class TestMapWorldAgainstTheSensorChain(unittest.TestCase):
    """MapWorld -> Plant.range_mm -> LIDAR: what the observer actually sees."""

    def test_the_sensor_model_quantises_what_the_world_reports(self):
        from ..lidar import L, LIDAR
        from ..plant import LIDAR_MOUNTS, Plant, PlantConfig
        from ..types import Pose

        w = MapWorld(Map([wall_at(500.0)]), mm_per_pixel=5.0)
        p = Plant(w, PlantConfig(lidar_noise_mm=0.0), Pose(0.0, 0.0, 0.0))
        lidar = LIDAR(p)
        lidar.update(0.0)

        # 494 mm to the face, less the 57 mm front mount, is past the VL6180X's
        # 300 mm ceiling -- which reads as "no target", i.e. MAX_DIST.
        self.assertEqual(lidar.getReading(LIDAR.Front), L.MAX_DIST)

        # Close enough to be in range, and quantised to the 2 mm X2 scale.
        p._set_pose(Pose(300.0, 0.0, 0.0))
        lidar.reset()
        lidar.update(0.0)
        expected = 494.0 - 300.0 - LIDAR_MOUNTS["front"].x
        reading = lidar.getReading(LIDAR.Front)
        self.assertLess(abs(reading - expected), L.SCALE_MM)
        self.assertEqual(reading % L.SCALE_MM, 0)


if __name__ == "__main__":
    unittest.main()
