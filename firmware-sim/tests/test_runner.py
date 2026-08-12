import json
import math
import unittest

from ..plant import PlantConfig
from ..runner import Runner
from ..scenarios import SCENARIOS
from ..world import World


def _runner(name, world=None, **kw):
    return Runner(
        SCENARIOS[name](),
        world if world is not None else World.blank(8000.0, 8000.0, 10.0),
        plant_config=PlantConfig(**kw),
    )


class TestRunner(unittest.TestCase):
    def test_step_returns_a_frame(self):
        f = _runner("task34").step()
        self.assertIsNotNone(f.true_pose)
        self.assertIsNotNone(f.est_pose)

    def test_time_advances_by_loop_period(self):
        r = _runner("task34")
        r.step()
        first = r.t
        r.step()
        self.assertAlmostEqual(r.t - first, 1.0 / 1000.0, places=9)

    def test_trails_accumulate(self):
        r = _runner("task34")
        for _ in range(100):
            r.step()
        self.assertEqual(len(r.true_trail), len(r.est_trail))
        self.assertGreater(len(r.true_trail), 1)

    def test_frame_carries_lidar_readings(self):
        f = _runner("task34").step()
        self.assertEqual(set(f.readings), {"front", "left", "right"})

    def test_estimate_starts_at_the_true_pose(self):
        r = _runner("task34")
        self.assertAlmostEqual(r.position_error(), 0.0, places=6)

    def test_estimate_diverges_from_truth_with_wheel_miscalibration(self):
        # the whole point of the rewrite: the planner does not see ground truth
        r = _runner("task34", gyro_bias=0.0, gyro_noise=0.0, enc_scale_error_left=1.05)
        r.run_until_done(20.0)
        self.assertGreater(r.position_error(), 1.0)

    def test_divergence_scales_with_the_miscalibration(self):
        small = _runner("task34", gyro_bias=0.0, gyro_noise=0.0, enc_scale_error_left=1.02)
        large = _runner("task34", gyro_bias=0.0, gyro_noise=0.0, enc_scale_error_left=1.05)
        small.run_until_done(20.0)
        large.run_until_done(20.0)
        self.assertGreater(large.position_error(), small.position_error())

    def test_constant_gyro_bias_is_calibrated_out(self):
        # ImuObserver.init() averages the bias at rest and subtracts it, so a
        # constant offset costs nothing. Wheel scale error is the real enemy.
        r = _runner("task34", gyro_bias=0.05, gyro_noise=0.0)
        r.run_until_done(20.0)
        self.assertLess(r.position_error(), 1.0)

    def test_gyro_noise_averages_out(self):
        r = _runner("task34", gyro_bias=0.0, gyro_noise=0.02)
        r.run_until_done(20.0)
        self.assertLess(r.position_error(), 1.0)

    def test_instruction_string_lands_on_the_expected_cell(self):
        # "lfrfrflf" from (0,0,North) walks to grid (2,0) == world (360, 0)
        r = _runner("task34", gyro_bias=0.0, gyro_noise=0.0)
        self.assertTrue(r.run_until_done(30.0))
        self.assertAlmostEqual(r.true_pose.x, 360.0, delta=15.0)
        self.assertAlmostEqual(r.true_pose.y, 0.0, delta=15.0)

    def test_ideal_sensors_keep_heading_estimate_close_to_truth(self):
        r = _runner("task33", gyro_bias=0.0, gyro_noise=0.0)
        r.run_until_done(20.0)
        self.assertLess(abs(r.true_pose.theta - r.est_pose.theta), 0.15)

    def test_heading_scenario_reaches_target(self):
        r = _runner("task33", gyro_bias=0.0, gyro_noise=0.0)
        r.run_until_done(20.0)
        self.assertAlmostEqual(r.est_pose.theta, math.pi / 2, delta=0.1)

    def test_reset_restores_start_state(self):
        r = _runner("task34")
        for _ in range(500):
            r.step()
        r.reset()
        self.assertEqual(r.t, 0.0)
        self.assertEqual(len(r.true_trail), 1)
        self.assertAlmostEqual(r.true_pose.x, 0.0, places=6)

    def test_every_scenario_constructs_and_steps(self):
        for name in SCENARIOS:
            with self.subTest(scenario=name):
                _runner(name).step()

    def test_geometry_is_serialisable(self):
        for name in SCENARIOS:
            with self.subTest(scenario=name):
                json.dumps(_runner(name).geometry())

    def test_geometry_kinds_match_planners(self):
        self.assertEqual(_runner("task31").geometry()["kind"], "segments")
        self.assertEqual(_runner("task32").geometry()["kind"], "distance")
        self.assertEqual(_runner("task33").geometry()["kind"], "heading")
        self.assertEqual(_runner("task34").geometry()["kind"], "grid")

    def test_ps_planner_geometry_has_every_instruction(self):
        g = _runner("task34").geometry()
        # setStart + 8 instructions
        self.assertEqual(len(g["targets"]), 9)

    def test_map_accumulates_during_a_run(self):
        r = _runner("task34")
        for _ in range(200):
            r.step()
        self.assertTrue(any(v != 0.0 for v in r.mapper.log_odds))

    def test_idle_planner_marks_the_run_done(self):
        r = _runner("task33", gyro_bias=0.0, gyro_noise=0.0)
        self.assertTrue(r.run_until_done(20.0))
        self.assertTrue(r.done())


class TestLidarObserverWiring(unittest.TestCase):
    def test_task32_registers_the_front_lidar_as_its_pose_source(self):
        r = _runner("task32", gyro_bias=0.0, gyro_noise=0.0)
        self.assertEqual(len(r.sf.poseSources), 1)
        self.assertIs(r.sf.poseSources[0].observer, r.hw.fl_obsv)

    def test_the_prior_delegate_is_the_fused_pose(self):
        """setup() wires lidar_obsv.setPrior(fusedPose) once sf exists, because
        SensorFusion never hands its pose sources the estimate they correct."""
        from ..maze_header import load_map
        from ..scenarios import planned
        from ..types import Pose, Segment, Vec2D
        from ..world import MapWorld

        m = load_map()
        s = planned([Segment(Vec2D(0, 0), Vec2D(100, 0))], map=m)
        r = Runner(s, MapWorld(m, mm_per_pixel=20.0))

        r.sf.set(Pose(11.0, 22.0, 0.5))
        self.assertAlmostEqual(r.hw.lidar_obsv.prior_func().x, 11.0)

    def test_a_scenario_with_no_map_still_runs(self):
        """A None map means the observer has nothing to cast into, so every
        solve returns the prior -- dead reckoning, and no crash."""
        r = _runner("task31", gyro_bias=0.0, gyro_noise=0.0)
        self.assertIsNone(r.scenario.map)
        for _ in range(500):
            r.step()
        self.assertEqual(r.hw.lidar_obsv.beams(), 0)


if __name__ == "__main__":
    unittest.main()
