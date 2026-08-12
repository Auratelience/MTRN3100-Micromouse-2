import math
import unittest

from ..constants import MAXIMUM_WHEEL_ANGULAR_VELOCITY, MAXIMUM_WHEEL_PWM
from ..plant import LIDAR_MOUNTS, Plant, PlantConfig
from ..types import Pose
from ..world import World

from .test_world import fill_wall_at_x


def _plant(**kw):
    return Plant(World.blank(4000.0, 4000.0, 5.0), PlantConfig(**kw), Pose(0.0, 0.0, 0.0))


def _wall_at(x_mm, size_mm=2000.0):
    w = World.blank(size_mm, size_mm, 5.0)
    fill_wall_at_x(w, x_mm)
    return w


class TestMotorModel(unittest.TestCase):
    def test_zero_pwm_stays_still(self):
        p = _plant()
        p.set_pwm(0, 0)
        for _ in range(100):
            p.step(0.001)
        self.assertAlmostEqual(p.pose.x, 0.0, places=6)

    def test_deadband_blocks_small_pwm(self):
        p = _plant(pwm_deadband=20.0)
        p.set_pwm(10, 10)
        for _ in range(500):
            p.step(0.001)
        self.assertAlmostEqual(p.pose.x, 0.0, places=6)

    def test_full_pwm_approaches_max_wheel_speed(self):
        p = _plant(motor_tau=0.060, pwm_deadband=0.0)
        p.set_pwm(MAXIMUM_WHEEL_PWM, MAXIMUM_WHEEL_PWM)
        for _ in range(1000):  # 1 s, ~16 time constants
            p.step(0.001)
        self.assertAlmostEqual(
            p.wheel_speed_left, MAXIMUM_WHEEL_ANGULAR_VELOCITY, delta=0.1
        )

    def test_first_order_lag_reaches_63pc_at_tau(self):
        p = _plant(motor_tau=0.100, pwm_deadband=0.0)
        p.set_pwm(MAXIMUM_WHEEL_PWM, MAXIMUM_WHEEL_PWM)
        for _ in range(100):  # 0.1 s == tau
            p.step(0.001)
        self.assertAlmostEqual(
            p.wheel_speed_left / MAXIMUM_WHEEL_ANGULAR_VELOCITY, 0.632, delta=0.02
        )

    def test_differential_pwm_turns_ccw(self):
        p = _plant(pwm_deadband=0.0)
        p.set_pwm(-MAXIMUM_WHEEL_PWM, MAXIMUM_WHEEL_PWM)
        for _ in range(200):
            p.step(0.001)
        self.assertGreater(p.pose.theta, 0.0)
        self.assertAlmostEqual(p.pose.x, 0.0, places=3)

    def test_reverse_pwm_drives_backward(self):
        p = _plant(pwm_deadband=0.0, motor_tau=0.0)
        p.set_pwm(-MAXIMUM_WHEEL_PWM, -MAXIMUM_WHEEL_PWM)
        for _ in range(100):
            p.step(0.001)
        self.assertLess(p.pose.x, 0.0)


class TestEncoders(unittest.TestCase):
    def test_counts_are_integers_and_quantised(self):
        p = _plant(pwm_deadband=0.0)
        p.set_pwm(80, 80)
        p.step(0.0005)
        self.assertIsInstance(p.count_left(), int)

    def test_one_true_revolution_reads_two_pi_after_calibration(self):
        # plant emits counts at the real (miscalibrated) rate; ENC_SCALE_*
        # corrects it, so a true revolution must read 2*pi
        p = _plant(pwm_deadband=0.0)
        p._set_true_wheel_angles(2 * math.pi, 2 * math.pi)
        self.assertAlmostEqual(p.angular_displacement_left(), 2 * math.pi, delta=0.02)
        self.assertAlmostEqual(p.angular_displacement_right(), 2 * math.pi, delta=0.02)

    def test_scale_error_knob_breaks_calibration(self):
        p = _plant(pwm_deadband=0.0, enc_scale_error_left=1.10)
        p._set_true_wheel_angles(2 * math.pi, 2 * math.pi)
        self.assertGreater(abs(p.angular_displacement_left() - 2 * math.pi), 0.3)

    def test_counts_increase_as_the_wheel_turns(self):
        p = _plant(pwm_deadband=0.0, motor_tau=0.0)
        p.set_pwm(MAXIMUM_WHEEL_PWM, MAXIMUM_WHEEL_PWM)
        before = p.count_left()
        for _ in range(100):
            p.step(0.001)
        self.assertGreater(p.count_left(), before)


class TestImu(unittest.TestCase):
    def test_gyro_reports_bias_when_stationary(self):
        p = _plant(gyro_bias=0.02, gyro_noise=0.0)
        self.assertAlmostEqual(p.gyro_z(), 0.02, places=6)

    def test_gyro_is_reproducible_for_a_given_seed(self):
        a = _plant(gyro_noise=0.01, seed=7)
        b = _plant(gyro_noise=0.01, seed=7)
        self.assertEqual(
            [a.gyro_z() for _ in range(5)], [b.gyro_z() for _ in range(5)]
        )

    def test_gyro_tracks_true_rotation(self):
        p = _plant(gyro_bias=0.0, gyro_noise=0.0, pwm_deadband=0.0, motor_tau=0.0)
        p.set_pwm(-MAXIMUM_WHEEL_PWM, MAXIMUM_WHEEL_PWM)
        p.step(0.001)
        self.assertGreater(p.gyro_z(), 0.0)


class TestLidar(unittest.TestCase):
    def test_mounts_have_full_poses(self):
        for name in ("front", "left", "right"):
            m = LIDAR_MOUNTS[name]
            self.assertIsInstance(m.x, float)
            self.assertIsInstance(m.y, float)
            self.assertIsInstance(m.theta, float)

    def test_front_range_shortens_as_robot_approaches_wall(self):
        p = Plant(_wall_at(500.0), PlantConfig(lidar_noise_mm=0.0), Pose(0.0, 0.0, 0.0))
        far = p.range_mm("front")
        p._set_pose(Pose(200.0, 0.0, 0.0))
        self.assertLess(p.range_mm("front"), far)

    def test_mount_offset_is_applied(self):
        # front mount sits +40 mm ahead of centre, so it reads 40 mm less
        p = Plant(_wall_at(400.0), PlantConfig(lidar_noise_mm=0.0), Pose(0.0, 0.0, 0.0))
        self.assertAlmostEqual(
            p.range_mm("front"), 400.0 - LIDAR_MOUNTS["front"].x, delta=6.0
        )

    def test_mount_bearing_is_applied(self):
        # left sensor looks along +y, so a wall perpendicular to +x is not
        # what it sees; it should read the far arena boundary instead
        p = Plant(_wall_at(400.0), PlantConfig(lidar_noise_mm=0.0), Pose(0.0, 0.0, 0.0))
        self.assertGreater(p.range_mm("left"), p.range_mm("front"))

    def test_rotating_the_robot_rotates_the_mounts(self):
        p = Plant(_wall_at(400.0), PlantConfig(lidar_noise_mm=0.0), Pose(0.0, 0.0, 0.0))
        front_facing_wall = p.range_mm("front")
        p._set_pose(Pose(0.0, 0.0, math.pi / 2))  # now the right sensor faces it
        self.assertAlmostEqual(p.range_mm("right"), front_facing_wall, delta=45.0)


if __name__ == "__main__":
    unittest.main()
