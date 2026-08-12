"""The obstacle/map layer of types.py.

The casts are checked against the closed forms from notes/Lidar Maths that live
alongside them (``d``, ``phi``, ``d_0``), not against numbers copied out of a
previous run: the two are independent computations of the same geometry, so
agreeing is evidence and disagreeing localises the fault.
"""

import math
import unittest

from ..constants import MAZE_POST_RADIUS, MAZE_WALL_THICKNESS
from ..types import (
    CircularObstacle,
    Map,
    Obstacle,
    Vec2D,
    WallObstacle,
    incidenceAngle,
    perp,
)

EAST = Vec2D(1.0, 0.0)
NORTH = Vec2D(0.0, 1.0)


class TestPerp(unittest.TestCase):
    def test_quarter_turn_ccw(self):
        p = perp(EAST)
        self.assertAlmostEqual(p.x, 0.0)
        self.assertAlmostEqual(p.y, 1.0)


class TestIncidenceAngle(unittest.TestCase):
    def test_square_on_is_zero(self):
        # beam east, surface normal pointing back west at the sensor
        self.assertAlmostEqual(incidenceAngle(Vec2D(-1.0, 0.0), EAST), 0.0, places=6)

    def test_grazing_is_a_quarter_turn(self):
        self.assertAlmostEqual(
            incidenceAngle(Vec2D(0.0, -1.0), EAST), math.pi / 2, places=6
        )

    def test_stays_accurate_just_off_square(self):
        """acos of the dot product would read a 0.03 rad angle as 0.010 here;
        atan2 is well behaved, which is why types.h uses it."""
        a = 0.03
        outward = Vec2D(-math.cos(a), -math.sin(a))
        self.assertAlmostEqual(incidenceAngle(outward, EAST), a, places=5)


class TestCircularObstacle(unittest.TestCase):
    def setUp(self):
        self.c = CircularObstacle(20.0)

    def test_cast_hits_the_near_surface(self):
        # centre 100 mm east, radius 20 -> the near face is at 80
        hit = self.c.cast(Vec2D(100.0, 0.0), EAST)
        self.assertTrue(hit.valid)
        self.assertAlmostEqual(hit.distance, 80.0, places=4)

    def test_near_root_not_far_root(self):
        """The derivation's plus root is where the beam would *leave* the
        obstacle; a lidar stops at the first surface."""
        near = self.c.cast(Vec2D(100.0, 0.0), EAST).distance
        self.assertAlmostEqual(self.c.dFar(100.0, 0.0), 120.0, places=4)
        self.assertLess(near, self.c.dFar(100.0, 0.0))

    def test_normal_points_back_at_the_sensor(self):
        hit = self.c.cast(Vec2D(100.0, 0.0), EAST)
        self.assertAlmostEqual(hit.normal.x, -1.0, places=4)
        self.assertAlmostEqual(hit.normal.y, 0.0, places=4)

    def test_misses_when_the_beam_passes_by(self):
        self.assertFalse(self.c.cast(Vec2D(100.0, 0.0), NORTH).valid)

    def test_misses_when_behind_the_sensor(self):
        self.assertFalse(self.c.cast(Vec2D(-100.0, 0.0), EAST).valid)

    def test_sensor_inside_the_obstacle_is_not_a_hit(self):
        self.assertFalse(self.c.cast(Vec2D(5.0, 0.0), EAST).valid)

    def test_phi_inverts_d(self):
        d_0, phi = 100.0, 0.1
        d = self.c.d(d_0, phi)
        self.assertAlmostEqual(self.c.phi(d_0, d), phi, places=4)

    def test_d0_inverts_d(self):
        d_0, phi = 100.0, 0.1
        d = self.c.d(d_0, phi)
        self.assertAlmostEqual(self.c.d_0(d, phi), d_0, places=3)

    def test_cast_agrees_with_the_closed_form_off_axis(self):
        to_centre = Vec2D(100.0, 12.0)
        hit = self.c.cast(to_centre, EAST)
        self.assertTrue(hit.valid)
        d_0 = math.hypot(to_centre.x, to_centre.y)
        phi = math.atan2(to_centre.y, to_centre.x)
        self.assertAlmostEqual(hit.distance, self.c.d(d_0, phi), places=4)


class TestWallObstacle(unittest.TestCase):
    def setUp(self):
        # a 180 mm panel lying along y (alpha = pi/2), i.e. facing east/west
        self.w = WallObstacle(180.0, MAZE_WALL_THICKNESS, math.pi / 2)

    def test_cast_stops_at_the_near_face(self):
        hit = self.w.cast(Vec2D(100.0, 0.0), EAST)
        self.assertTrue(hit.valid)
        self.assertAlmostEqual(hit.distance, 100.0 - MAZE_WALL_THICKNESS / 2, places=4)

    def test_the_face_chosen_is_the_one_the_sensor_is_on(self):
        east_side = self.w.cast(Vec2D(-100.0, 0.0), Vec2D(-1.0, 0.0))
        self.assertTrue(east_side.valid)
        self.assertAlmostEqual(east_side.normal.x, 1.0, places=4)

    def test_oblique_range_is_the_perpendicular_over_cos(self):
        angle = 0.4
        beam = Vec2D(math.cos(angle), math.sin(angle))
        hit = self.w.cast(Vec2D(100.0, 0.0), beam)
        self.assertTrue(hit.valid)
        perpendicular = 100.0 - MAZE_WALL_THICKNESS / 2
        self.assertAlmostEqual(hit.distance, perpendicular / math.cos(angle), places=4)

    def test_ends_are_not_capped(self):
        """The two short ends are deliberately left out: a post's own circle
        covers that corner more accurately than a square cap would."""
        # aimed past the end of a 180 mm panel
        self.assertFalse(self.w.cast(Vec2D(100.0, 200.0), EAST).valid)

    def test_a_beam_along_the_face_misses(self):
        self.assertFalse(self.w.cast(Vec2D(100.0, 0.0), NORTH).valid)

    def test_phi_inverts_d(self):
        d_0, phi, alpha = 100.0, 0.15, 0.2
        d = self.w.d(d_0, phi, alpha)
        self.assertAlmostEqual(self.w.phi(d_0, d, alpha), phi, places=4)

    def test_d0_inverts_d(self):
        d_0, phi, alpha = 100.0, 0.15, 0.2
        d = self.w.d(d_0, phi, alpha)
        self.assertAlmostEqual(self.w.d_0(d, phi, alpha), d_0, places=3)


class TestMap(unittest.TestCase):
    def setUp(self):
        self.m = Map(
            [
                Obstacle(CircularObstacle(MAZE_POST_RADIUS), Vec2D(200.0, 0.0)),
                Obstacle(CircularObstacle(MAZE_POST_RADIUS), Vec2D(400.0, 0.0)),
                Obstacle(
                    WallObstacle(180.0, MAZE_WALL_THICKNESS, math.pi / 2),
                    Vec2D(1000.0, 0.0),
                ),
            ]
        )

    def test_cast_returns_the_nearest(self):
        hit = self.m.cast(Vec2D(0.0, 0.0), EAST, 2000.0)
        self.assertTrue(hit.valid)
        self.assertEqual(hit.index, 0)
        self.assertAlmostEqual(hit.distance, 200.0 - MAZE_POST_RADIUS, places=3)

    def test_max_range_bounds_the_search(self):
        self.assertFalse(self.m.cast(Vec2D(0.0, 0.0), EAST, 100.0).valid)

    def test_candidates_cull_by_bounding_circle(self):
        near = self.m.candidates(Vec2D(0.0, 0.0), 250.0, 24)
        self.assertEqual(near, [0])

    def test_candidates_respect_the_cap(self):
        self.assertEqual(len(self.m.candidates(Vec2D(0.0, 0.0), 5000.0, 2)), 2)

    def test_restricted_cast_matches_a_full_one(self):
        """The broad phase is an optimisation, so it must not change answers --
        which is exactly why an overflowing candidate list falls back."""
        idx = self.m.candidates(Vec2D(0.0, 0.0), 400.0, 24)
        full = self.m.cast(Vec2D(0.0, 0.0), EAST, 2000.0)
        subset = self.m.cast(Vec2D(0.0, 0.0), EAST, 2000.0, idx)
        self.assertAlmostEqual(full.distance, subset.distance, places=6)
        self.assertEqual(full.index, subset.index)

    def test_bounds_cover_every_obstacle(self):
        x0, y0, x1, y1 = self.m.bounds()
        self.assertLessEqual(x0, 200.0 - MAZE_POST_RADIUS)
        self.assertGreaterEqual(x1, 1000.0 + MAZE_WALL_THICKNESS / 2)
        self.assertLessEqual(y0, -90.0)
        self.assertGreaterEqual(y1, 90.0)

    def test_an_empty_map_never_hits(self):
        self.assertFalse(Map([]).cast(Vec2D(0.0, 0.0), EAST, 1000.0).valid)


class TestImpliedHeadingError(unittest.TestCase):
    def test_zero_when_the_reading_is_what_was_predicted(self):
        o = Obstacle(CircularObstacle(20.0), Vec2D(100.0, 0.0))
        origin = Vec2D(0.0, 0.0)
        hit = o.cast(origin, EAST)
        self.assertAlmostEqual(
            o.impliedHeadingError(hit, origin, EAST, hit.distance), 0.0, places=3
        )

    def test_negative_for_an_impossible_reading(self):
        o = Obstacle(CircularObstacle(20.0), Vec2D(100.0, 0.0))
        origin = Vec2D(0.0, 0.0)
        hit = o.cast(origin, EAST)
        self.assertLess(o.impliedHeadingError(hit, origin, EAST, 5000.0), 0.0)


if __name__ == "__main__":
    unittest.main()
