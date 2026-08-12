import math
import unittest

from ..world import World


def fill_wall_at_x(w, x_mm):
    """Fill the whole image row corresponding to world x == x_mm.

    With x_axis="up", constant world x is a constant image *row*, not a
    column -- world x maps to the pixel row axis and world y to the column
    axis. The resulting wall is perpendicular to +x.
    """
    _, py = w.world_to_px(x_mm, 0.0)
    py = int(py)
    for px in range(w.width):
        w.grid[py * w.width + px] = 1


class TestRaycast(unittest.TestCase):
    def setUp(self):
        # 1000 x 1000 mm arena at 5 mm/px -> 200 x 200 grid, all clear
        self.w = World.blank(1000.0, 1000.0, 5.0)

    def test_clear_ray_returns_max_range(self):
        self.assertAlmostEqual(self.w.raycast(0.0, 0.0, 0.0, 300.0), 300.0, places=6)

    def test_hits_wall_ahead(self):
        fill_wall_at_x(self.w, 200.0)
        d = self.w.raycast(0.0, 0.0, 0.0, 300.0)
        self.assertAlmostEqual(d, 200.0, delta=5.0)  # within one cell

    def test_wall_behind_is_not_hit(self):
        fill_wall_at_x(self.w, -200.0)
        self.assertAlmostEqual(self.w.raycast(0.0, 0.0, 0.0, 300.0), 300.0, places=6)

    def test_diagonal_ray_distance(self):
        fill_wall_at_x(self.w, 200.0)
        # 45 degrees: reaches the same wall at sqrt(2) * 200
        d = self.w.raycast(0.0, 0.0, math.pi / 4, 400.0)
        self.assertAlmostEqual(d, 200.0 * math.sqrt(2), delta=8.0)

    def test_outside_grid_counts_as_occupied(self):
        # arena is bounded: a ray leaving the grid terminates rather than
        # reporting max range through open space
        d = self.w.raycast(0.0, 0.0, 0.0, 5000.0)
        self.assertLess(d, 5000.0)
        self.assertAlmostEqual(d, 500.0, delta=6.0)  # half of a 1000 mm arena

    def test_occupied_at_matches_grid(self):
        fill_wall_at_x(self.w, 200.0)
        self.assertTrue(self.w.occupied_at(200.0, 0.0))
        self.assertFalse(self.w.occupied_at(100.0, 0.0))

    def test_ray_starting_inside_a_wall_returns_zero(self):
        fill_wall_at_x(self.w, 0.0)
        self.assertEqual(self.w.raycast(0.0, 0.0, 0.0, 300.0), 0.0)

    def test_backward_ray_hits_wall_behind(self):
        fill_wall_at_x(self.w, -200.0)
        d = self.w.raycast(0.0, 0.0, math.pi, 300.0)
        self.assertAlmostEqual(d, 200.0, delta=5.0)

    def test_sideways_ray_uses_y_axis(self):
        # a wall perpendicular to +y, hit by a ray pointing along +y
        px, _ = self.w.world_to_px(0.0, 150.0)
        px = int(px)
        for py in range(self.w.height):
            self.w.grid[py * self.w.width + px] = 1
        d = self.w.raycast(0.0, 0.0, math.pi / 2, 300.0)
        self.assertAlmostEqual(d, 150.0, delta=5.0)


class TestCoordinateMapping(unittest.TestCase):
    def setUp(self):
        self.w = World.blank(1000.0, 1000.0, 5.0)

    def test_roundtrip(self):
        for x, y in [(0.0, 0.0), (123.0, -45.0), (-200.0, 310.0)]:
            px, py = self.w.world_to_px(x, y)
            bx, by = self.w.px_to_world(px, py)
            self.assertAlmostEqual(bx, x, places=3)
            self.assertAlmostEqual(by, y, places=3)

    def test_plus_x_is_up_in_image(self):
        _, py0 = self.w.world_to_px(0.0, 0.0)
        _, py1 = self.w.world_to_px(100.0, 0.0)
        self.assertLess(py1, py0)  # +x -> decreasing pixel row

    def test_plus_y_is_left_in_image(self):
        px0, _ = self.w.world_to_px(0.0, 0.0)
        px1, _ = self.w.world_to_px(0.0, 100.0)
        self.assertLess(px1, px0)  # +y is left of +x when +x points up

    def test_bounds_cover_the_arena(self):
        min_x, min_y, max_x, max_y = self.w.bounds_mm()
        self.assertAlmostEqual(max_x - min_x, 1000.0, delta=1.0)
        self.assertAlmostEqual(max_y - min_y, 1000.0, delta=1.0)


class TestRle(unittest.TestCase):
    def test_roundtrip(self):
        w = World.blank(100.0, 100.0, 5.0)
        w.grid[0] = 1
        w.grid[1] = 1
        runs = w.to_rle()
        self.assertEqual(sum(runs), w.width * w.height)
        self.assertEqual(runs[0], 0)  # encoding always starts with a 0-run
        self.assertEqual(runs[1], 2)

    def test_all_clear_is_single_run(self):
        w = World.blank(100.0, 100.0, 5.0)
        runs = w.to_rle()
        self.assertEqual(sum(runs), w.width * w.height)
        self.assertEqual(runs[0], w.width * w.height)


if __name__ == "__main__":
    unittest.main()
