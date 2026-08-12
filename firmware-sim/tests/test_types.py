import math
import unittest

from ..constants import MAZE_CELL_SIZE
from ..types import Segment, Vec2D, wrapAngle


class TestWrapAngle(unittest.TestCase):
    def test_wraps_above_pi(self):
        self.assertAlmostEqual(wrapAngle(3 * math.pi / 2), -math.pi / 2, places=6)

    def test_wraps_below_negative_pi(self):
        self.assertAlmostEqual(wrapAngle(-3 * math.pi / 2), math.pi / 2, places=6)

    def test_non_finite_returns_zero(self):
        # firmware guards this so a NaN cannot poison downstream control maths
        self.assertEqual(wrapAngle(float("nan")), 0.0)
        self.assertEqual(wrapAngle(float("inf")), 0.0)


class TestSegmentLine(unittest.TestCase):
    def setUp(self):
        self.s = Segment(Vec2D(0, 0), Vec2D(100, 0))

    def test_progress_midpoint(self):
        self.assertAlmostEqual(self.s.progress(Vec2D(50, 0)), 0.5, places=6)

    def test_progress_past_end_exceeds_one(self):
        self.assertGreater(self.s.progress(Vec2D(120, 0)), 1.0)

    def test_lateral_distance_is_perpendicular_offset(self):
        self.assertAlmostEqual(self.s.lateralDistance(Vec2D(50, 7)), 7.0, places=6)


class TestSegmentArc(unittest.TestCase):
    def setUp(self):
        # quarter circle radius 25, left turn, from (0,0) to (25,25); centre (0,25)
        self.s = Segment(Vec2D(0, 0), Vec2D(25, 25), 1.0 / 25.0, Segment.Direction.Left)

    def test_centre(self):
        self.assertAlmostEqual(self.s.c.x, 0.0, places=4)
        self.assertAlmostEqual(self.s.c.y, 25.0, places=4)

    def test_progress_halfway_is_half(self):
        r = 25.0
        mid = Vec2D(r * math.sin(math.pi / 4), 25.0 - r * math.cos(math.pi / 4))
        self.assertAlmostEqual(self.s.progress(mid), 0.5, places=4)

    def test_behind_start_reads_negative_not_past_end(self):
        # the firmware comment calls this case out explicitly: a point a few mm
        # behind the arc start must read slightly negative, never ">1", or the
        # planner's advance loop skips the whole arc
        behind = Vec2D(-5.0, 0.5)
        self.assertLess(self.s.progress(behind), 0.0)

    def test_infeasible_radius_degenerates_to_midpoint(self):
        # radius smaller than half the chord: discriminant clamps to 0
        s = Segment(Vec2D(0, 0), Vec2D(100, 0), 1.0 / 10.0, Segment.Direction.Left)
        self.assertAlmostEqual(s.c.x, 50.0, places=4)
        self.assertAlmostEqual(s.c.y, 0.0, places=4)


class TestConstants(unittest.TestCase):
    def test_cell_size_matches_firmware(self):
        self.assertEqual(MAZE_CELL_SIZE, 180.0)


if __name__ == "__main__":
    unittest.main()
