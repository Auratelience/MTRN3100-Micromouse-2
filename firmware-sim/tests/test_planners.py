import math
import unittest

from ..constants import MAXIMUM_FORWARD_VELOCITY, MAZE_CELL_SIZE
from ..planners import (
    DistancePlanner,
    HeadingPlanner,
    MotionPlanner,
    PosePlanner,
    PSPlanner,
)
from ..types import Pose, Segment, Vec2D


class TestPSPlannerGrid(unittest.TestCase):
    def test_forwards_from_each_direction(self):
        D = PSPlanner.Direction
        cases = [(D.North, (1, 0)), (D.East, (0, -1)), (D.South, (-1, 0)), (D.West, (0, 1))]
        for d, (dx, dy) in cases:
            g = PSPlanner.forwards(PSPlanner.GridPose(0, 0, d))
            self.assertEqual((g.x, g.y, g.direction), (dx, dy, d))

    def test_left_cycles_directions(self):
        D = PSPlanner.Direction
        g = PSPlanner.GridPose(0, 0, D.North)
        for expected in (D.West, D.South, D.East, D.North):
            g = PSPlanner.left(g)
            self.assertEqual(g.direction, expected)

    def test_right_cycles_directions(self):
        D = PSPlanner.Direction
        g = PSPlanner.GridPose(0, 0, D.North)
        for expected in (D.East, D.South, D.West, D.North):
            g = PSPlanner.right(g)
            self.assertEqual(g.direction, expected)

    def test_turns_do_not_move_the_cell(self):
        g = PSPlanner.GridPose(3, 4, PSPlanner.Direction.North)
        for turned in (PSPlanner.left(g), PSPlanner.right(g)):
            self.assertEqual((turned.x, turned.y), (3, 4))

    def test_add_instructions_builds_expected_sequence(self):
        p = PSPlanner(10.0, 5.0)
        p.setStart(PSPlanner.GridPose(0, 0, PSPlanner.Direction.North))
        p.addInstructions("ff")
        self.assertEqual(
            [(g.x, g.y) for g in p.instructions[:3]], [(0, 0), (1, 0), (2, 0)]
        )

    def test_grid_to_world_scales_by_cell_size(self):
        w = PSPlanner.gridToWorld(PSPlanner.GridPose(2, -1, PSPlanner.Direction.North))
        self.assertAlmostEqual(w.x, 2 * MAZE_CELL_SIZE)
        self.assertAlmostEqual(w.y, -1 * MAZE_CELL_SIZE)
        self.assertAlmostEqual(w.theta, 0.0)

    def test_grid_to_world_heading_for_each_direction(self):
        D = PSPlanner.Direction
        expected = {
            D.North: 0.0,
            D.West: math.pi / 2,
            D.South: math.pi,
            D.East: -math.pi / 2,
        }
        for d, theta in expected.items():
            w = PSPlanner.gridToWorld(PSPlanner.GridPose(0, 0, d))
            self.assertAlmostEqual(abs(w.theta), abs(theta), places=5)

    def test_world_to_grid_roundtrip(self):
        D = PSPlanner.Direction
        g = PSPlanner.GridPose(2, -3, D.East)
        back = PSPlanner.worldToGrid(PSPlanner.gridToWorld(g))
        self.assertEqual((back.x, back.y, back.direction), (2, -3, D.East))


class TestPosePlanner(unittest.TestCase):
    def test_heading_only_skips_seek_and_rotates_in_place(self):
        pp = PosePlanner(10.0, 5.0)
        pp.setHeadingOnly(True)
        pp.setTarget(Pose(0.0, 0.0, math.pi / 2))
        out = pp.update(Pose(500.0, 500.0, 0.0), 0.001)  # far from target
        self.assertEqual(out.v, 0.0)  # never seeks
        self.assertGreater(out.omega, 0.0)

    def test_seek_drives_toward_target(self):
        pp = PosePlanner(10.0, 5.0)
        pp.setHeadingOnly(False)
        pp.setTarget(Pose(180.0, 0.0, 0.0))
        out = pp.update(Pose(0.0, 0.0, 0.0), 0.001)
        self.assertGreater(out.v, 0.0)

    def test_seek_velocity_clamped_to_max(self):
        pp = PosePlanner(10.0, 5.0)
        pp.setTarget(Pose(10_000.0, 0.0, 0.0))
        self.assertLessEqual(pp.update(Pose(0, 0, 0), 0.001).v, MAXIMUM_FORWARD_VELOCITY)

    def test_done_after_reaching_pose(self):
        pp = PosePlanner(10.0, 5.0)
        pp.setTarget(Pose(0.0, 0.0, 0.0))
        pp.update(Pose(0.0, 0.0, 0.0), 0.001)
        self.assertTrue(pp.done())

    def test_seek_hands_over_to_align_within_tolerance(self):
        pp = PosePlanner(10.0, 5.0)
        pp.setTarget(Pose(0.0, 0.0, math.pi / 2))
        out = pp.update(Pose(0.5, 0.0, 0.0), 0.001)  # inside positionTolerance
        self.assertEqual(out.v, 0.0)
        self.assertGreater(out.omega, 0.0)


class TestMotionPlanner(unittest.TestCase):
    def test_waits_with_no_segments(self):
        mp = MotionPlanner(10.0, 0.06)
        out = mp.update(Pose(0, 0, 0), 0.001)
        self.assertEqual((out.v, out.omega), (0.0, 0.0))

    def test_advances_past_completed_segment(self):
        mp = MotionPlanner(10.0, 0.06)
        mp.appendSegment(Segment(Vec2D(0, 0), Vec2D(100, 0)))
        mp.appendSegment(Segment(Vec2D(100, 0), Vec2D(200, 0)))
        mp.update(Pose(150.0, 0.0, 0.0), 0.001)
        self.assertEqual(mp.idx(), 1)

    def test_returns_to_wait_when_path_completed(self):
        mp = MotionPlanner(10.0, 0.06)
        mp.appendSegment(Segment(Vec2D(0, 0), Vec2D(100, 0)))
        mp.update(Pose(200.0, 0.0, 0.0), 0.001)
        self.assertEqual(mp.s(), MotionPlanner.State.Wait)

    def test_steers_back_toward_the_line(self):
        mp = MotionPlanner(10.0, 0.06)
        mp.appendSegment(Segment(Vec2D(0, 0), Vec2D(1000, 0)))
        left_of_line = mp.update(Pose(100.0, 20.0, 0.0), 0.001)
        right_of_line = mp.update(Pose(100.0, -20.0, 0.0), 0.001)
        self.assertLess(left_of_line.omega, right_of_line.omega)

    def test_arc_segment_applies_feedforward_curvature(self):
        mp = MotionPlanner(10.0, 0.06)
        mp.appendSegment(
            Segment(Vec2D(0, 0), Vec2D(25, 25), 1.0 / 25.0, Segment.Direction.Left)
        )
        # sitting exactly on the arc start facing the tangent: the only
        # non-zero term left is the curvature feedforward
        out = mp.update(Pose(0.0, 0.0, 0.0), 0.001)
        self.assertGreater(out.omega, 0.0)


class TestHeadingPlanner(unittest.TestCase):
    def test_stops_within_tolerance(self):
        hp = HeadingPlanner(5.0)
        hp.setTarget(0.0)
        self.assertEqual(hp.update(Pose(0, 0, 0.001), 0.001).omega, 0.0)

    def test_rotates_toward_target(self):
        hp = HeadingPlanner(5.0)
        hp.setTarget(math.pi / 2)
        self.assertGreater(hp.update(Pose(0, 0, 0.0), 0.001).omega, 0.0)

    def test_takes_the_short_way_round(self):
        hp = HeadingPlanner(5.0)
        hp.setTarget(-math.pi + 0.1)
        self.assertGreater(hp.update(Pose(0, 0, math.pi - 0.1), 0.001).omega, 0.0)


class TestDistancePlanner(unittest.TestCase):
    def test_stops_within_tolerance(self):
        dp = DistancePlanner(3.0, 0.06)
        dp.setTarget(200.0)
        self.assertEqual(dp.update(Pose(-200.0, 0, 0), 0.001).v, 0.0)

    def test_velocity_clamped_to_max(self):
        dp = DistancePlanner(3.0, 0.06)
        dp.setTarget(0.0)
        self.assertLessEqual(
            dp.update(Pose(-10_000.0, 0, 0), 0.001).v, MAXIMUM_FORWARD_VELOCITY
        )


if __name__ == "__main__":
    unittest.main()
