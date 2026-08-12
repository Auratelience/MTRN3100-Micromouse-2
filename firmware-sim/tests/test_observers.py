import unittest

from ..constants import AXLE_LEN, MAXIMUM_WHEEL_PWM, WHEEL_RADIUS
from ..kinematics import Kinematics
from ..lidar import LIDAR
from ..observers import (
    ImuObserver,
    LidarObserver,
    ModelObserver,
    SimIMU,
    SimMotor,
    WheelObserver,
)
from ..plant import Plant, PlantConfig
from ..types import Pose, Velocity
from ..world import World


def _plant(**kw):
    return Plant(World.blank(4000.0, 4000.0, 5.0), PlantConfig(**kw), Pose(0, 0, 0))


def _run_wheel_observer(pwm_left, pwm_right, steps=200, dt=0.001):
    """Drive the plant open-loop and collect the observer's estimates.

    Returns (mean_v, mean_omega, samples). Single-sample estimates are not
    meaningful at 1 kHz -- see TestWheelObserverQuantisation below -- so
    assertions here are on the mean.
    """
    p = _plant(pwm_deadband=0.0, motor_tau=0.0)
    k = Kinematics(WHEEL_RADIUS, AXLE_LEN)
    obs = WheelObserver(SimMotor(p, "left"), SimMotor(p, "right"), k)
    p.set_pwm(pwm_left, pwm_right)
    samples = []
    for _ in range(steps):
        p.step(dt)
        obs.update(dt)
        samples.append(obs.estimate())
    mean_v = sum(s.v for s in samples) / len(samples)
    mean_omega = sum(s.omega for s in samples) / len(samples)
    return mean_v, mean_omega, samples


class TestWheelObserver(unittest.TestCase):
    def test_estimates_forward_velocity_from_encoders(self):
        mean_v, mean_omega, _ = _run_wheel_observer(
            MAXIMUM_WHEEL_PWM, MAXIMUM_WHEEL_PWM
        )
        self.assertGreater(mean_v, 300.0)
        self.assertAlmostEqual(mean_omega, 0.0, delta=0.5)

    def test_estimates_rotation_from_encoders(self):
        mean_v, mean_omega, _ = _run_wheel_observer(
            -MAXIMUM_WHEEL_PWM, MAXIMUM_WHEEL_PWM
        )
        self.assertGreater(mean_omega, 0.0)
        self.assertAlmostEqual(mean_v, 0.0, delta=5.0)


class TestWheelObserverQuantisation(unittest.TestCase):
    """Encoder quantisation is a first-class part of what this sim exists to
    show. At 1 kHz a wheel at full speed advances only ~2.7 encoder counts per
    tick, so the per-tick velocity estimate lands on a coarse ladder. The
    firmware does not filter this, so neither does the port."""

    def test_single_sample_velocity_is_visibly_noisy(self):
        _, _, samples = _run_wheel_observer(MAXIMUM_WHEEL_PWM, MAXIMUM_WHEEL_PWM)
        spread = max(s.v for s in samples) - min(s.v for s in samples)
        self.assertGreater(spread, 20.0)

    def test_straight_driving_shows_spurious_rotation(self):
        # left and right carry different ENC_SCALE values (6.09 vs 6.24 rad
        # per rev), so their quantisation ladders differ and a perfectly
        # straight run still reads as turning tick to tick
        _, _, samples = _run_wheel_observer(MAXIMUM_WHEEL_PWM, MAXIMUM_WHEEL_PWM)
        self.assertGreater(max(abs(s.omega) for s in samples), 1.0)

    def test_noise_shrinks_at_a_slower_loop_rate(self):
        _, _, fast = _run_wheel_observer(
            MAXIMUM_WHEEL_PWM, MAXIMUM_WHEEL_PWM, steps=200, dt=0.001
        )
        _, _, slow = _run_wheel_observer(
            MAXIMUM_WHEEL_PWM, MAXIMUM_WHEEL_PWM, steps=200, dt=0.010
        )
        fast_spread = max(s.v for s in fast) - min(s.v for s in fast)
        slow_spread = max(s.v for s in slow) - min(s.v for s in slow)
        self.assertLess(slow_spread, fast_spread)

    def test_ready_always_true(self):
        p = _plant()
        k = Kinematics(WHEEL_RADIUS, AXLE_LEN)
        self.assertTrue(
            WheelObserver(SimMotor(p, "left"), SimMotor(p, "right"), k).ready()
        )


class TestImuObserver(unittest.TestCase):
    def test_init_measures_and_removes_bias(self):
        p = _plant(gyro_bias=0.05, gyro_noise=0.0)
        obs = ImuObserver(SimIMU(p))
        obs.init()
        obs.update(0.001)
        self.assertAlmostEqual(obs.estimate().omega, 0.0, delta=0.005)

    def test_not_ready_before_init(self):
        self.assertFalse(ImuObserver(SimIMU(_plant())).ready())

    def test_estimate_is_zero_before_init(self):
        obs = ImuObserver(SimIMU(_plant(gyro_bias=0.5)))
        obs.update(0.001)
        self.assertEqual(obs.estimate().omega, 0.0)

    def test_reports_rotation_after_init(self):
        p = _plant(gyro_bias=0.0, gyro_noise=0.0, pwm_deadband=0.0, motor_tau=0.0)
        obs = ImuObserver(SimIMU(p))
        obs.init()
        p.set_pwm(-MAXIMUM_WHEEL_PWM, MAXIMUM_WHEEL_PWM)
        p.step(0.001)
        obs.update(0.001)
        self.assertGreater(obs.estimate().omega, 0.0)

    def test_noise_averages_out_over_the_init_window(self):
        p = _plant(gyro_bias=0.05, gyro_noise=0.02, seed=3)
        obs = ImuObserver(SimIMU(p))
        obs.init()
        self.assertAlmostEqual(obs.gyro_z_drift, 0.05, delta=0.005)


class TestModelObserver(unittest.TestCase):
    def test_dead_reckons_straight_line(self):
        obs = ModelObserver(lambda: Velocity(100.0, 0.0))
        for _ in range(1000):
            obs.update(0.001)
        self.assertAlmostEqual(obs.estimate().x, 100.0, delta=0.5)
        self.assertAlmostEqual(obs.estimate().y, 0.0, places=4)

    def test_dead_reckons_rotation(self):
        obs = ModelObserver(lambda: Velocity(0.0, 1.0))
        for _ in range(1000):
            obs.update(0.001)
        self.assertAlmostEqual(obs.estimate().theta, 1.0, delta=0.01)

    def test_set_overrides_pose(self):
        obs = ModelObserver(lambda: Velocity(0.0, 0.0))
        obs.set(Pose(5.0, 6.0, 0.7))
        self.assertAlmostEqual(obs.estimate().x, 5.0)

    def test_pose_func_overrides_internal_state(self):
        obs = ModelObserver(lambda: Velocity(0.0, 0.0), lambda: Pose(9.0, 0.0, 0.0))
        obs.update(0.001)
        self.assertAlmostEqual(obs.estimate().x, 9.0)


class TestLidarObserver(unittest.TestCase):
    def test_always_ready(self):
        """SensorFusion tests ready() before update(), so an observer that
        reports itself unready is never updated again and can never come back.
        A cycle with nothing to say returns the prior instead."""
        self.assertTrue(LidarObserver(LIDAR(_plant())).ready())

    def test_no_map_returns_the_prior_unchanged(self):
        obs = LidarObserver(LIDAR(_plant()))
        obs.set(Pose(1.0, 2.0, 3.0))
        obs.update(0.001)
        self.assertAlmostEqual(obs.estimate().x, 1.0)
        self.assertEqual(obs.beams(), 0)

    def test_prior_delegate_overrides_internal_state(self):
        obs = LidarObserver(LIDAR(_plant()), prior=lambda: Pose(5.0, 6.0, 0.0))
        obs.set(Pose(1.0, 2.0, 3.0))
        obs.update(0.001)
        self.assertAlmostEqual(obs.estimate().x, 5.0)

    def test_solves_a_pose_error_against_a_known_map(self):
        """The whole point of the observer: put the robot somewhere the prior
        does not say it is, and the beams should pull the estimate back."""
        from ..scenarios import task32_bench

        world, true_start = task32_bench()
        plant = Plant(world, PlantConfig(lidar_noise_mm=0.0), true_start)
        lidar = LIDAR(plant)

        # The prior is 12 mm short of where the robot really is.
        prior = Pose(true_start.x - 12.0, true_start.y, true_start.theta)
        obs = LidarObserver(lidar, world.map, prior=lambda: prior)

        lidar.advance(1.0)
        lidar.update()
        obs.update(1.0)

        self.assertGreater(obs.beams(), 0)
        # closer to the truth than the prior was, and by most of the 12 mm
        self.assertLess(abs(obs.estimate().x - true_start.x), 4.0)

    def test_a_saturated_beam_is_not_an_equation(self):
        """MIN_DIST and MAX_DIST both mean 'no target', not a range. An empty
        arena saturates every beam, so no equation survives gating."""
        from ..types import Map

        obs = LidarObserver(LIDAR(_plant()), Map([]))
        obs.set(Pose(0.0, 0.0, 0.0))
        obs.update(1.0)
        self.assertEqual(obs.beams(), 0)

    def test_rate_limited_to_the_sensor_period(self):
        """The VL6180X free-runs at LIDAR_CONTINUOUS_PERIOD_MS; re-solving at
        loop rate would just re-fit the same three numbers."""
        from ..scenarios import task32_bench

        world, true_start = task32_bench()
        lidar = LIDAR(Plant(world, PlantConfig(lidar_noise_mm=0.0), true_start))
        obs = LidarObserver(lidar, world.map, prior=lambda: true_start)

        obs.update(1.0)  # first call always samples
        solved = obs.beams()
        obs.update(0.001)  # 1 ms later, well inside the 10 ms period
        self.assertGreater(solved, 0)
        self.assertEqual(obs.beams(), 0)


if __name__ == "__main__":
    unittest.main()
