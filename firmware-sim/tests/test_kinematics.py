import unittest

from ..constants import AXLE_LEN, MAXIMUM_WHEEL_ANGULAR_VELOCITY, WHEEL_RADIUS
from ..kinematics import Kinematics
from ..types import Velocity, WheelVelocities


class TestKinematics(unittest.TestCase):
    def setUp(self):
        self.k = Kinematics(WHEEL_RADIUS, AXLE_LEN)

    def test_fk_straight(self):
        v = self.k.fk.velocity(WheelVelocities(10.0, 10.0))
        self.assertAlmostEqual(v.v, WHEEL_RADIUS * 10.0, places=4)
        self.assertAlmostEqual(v.omega, 0.0, places=6)

    def test_fk_spin_in_place(self):
        v = self.k.fk.velocity(WheelVelocities(-10.0, 10.0))
        self.assertAlmostEqual(v.v, 0.0, places=6)
        self.assertGreater(v.omega, 0.0)  # +omega is CCW = left wheel back

    def test_ik_roundtrip(self):
        wv = self.k.ik.velocityRaw(Velocity(200.0, 1.5))
        v = self.k.fk.velocity(wv)
        self.assertAlmostEqual(v.v, 200.0, places=3)
        self.assertAlmostEqual(v.omega, 1.5, places=5)

    def test_ik_clamps_on_magnitude(self):
        wv = self.k.ik.velocity(Velocity(10_000.0, 0.0))
        self.assertLessEqual(
            max(abs(wv.left), abs(wv.right)), MAXIMUM_WHEEL_ANGULAR_VELOCITY + 1e-6
        )

    def test_ik_clamps_large_negative_wheel_speeds(self):
        # firmware clamps on fabsf so fast reverse / sharp turns scale too
        wv = self.k.ik.velocity(Velocity(0.0, -100.0))
        self.assertLessEqual(
            max(abs(wv.left), abs(wv.right)), MAXIMUM_WHEEL_ANGULAR_VELOCITY + 1e-6
        )

    def test_ik_clamp_preserves_ratio(self):
        raw = self.k.ik.velocityRaw(Velocity(1000.0, 2.0))
        cl = self.k.ik.velocity(Velocity(1000.0, 2.0))
        self.assertAlmostEqual(cl.left / cl.right, raw.left / raw.right, places=5)

    def test_velocity_raw_does_not_clamp(self):
        wv = self.k.ik.velocityRaw(Velocity(10_000.0, 0.0))
        self.assertGreater(wv.left, MAXIMUM_WHEEL_ANGULAR_VELOCITY)


if __name__ == "__main__":
    unittest.main()
