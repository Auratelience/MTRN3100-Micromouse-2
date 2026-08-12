import unittest

from ..constants import (
    AXLE_LEN,
    MAXIMUM_WHEEL_PWM,
    PID_SATURATION_ABSOLUTE,
    WHEEL_RADIUS,
)
from ..control import PID, MotionController
from ..kinematics import Kinematics
from ..observers import SimMotor
from ..plant import Plant, PlantConfig
from ..types import Pose, Velocity
from ..world import World


class TestPid(unittest.TestCase):
    def test_proportional_output(self):
        self.assertAlmostEqual(
            PID(2.0, 0.0, 0.0).step(10.0, 4.0, 0.001), 12.0, places=5
        )

    def test_integral_accumulates(self):
        pid = PID(0.0, 1.0, 0.0)
        for _ in range(1000):
            pid.step(1.0, 0.0, 0.001)
        self.assertAlmostEqual(pid.step(1.0, 0.0, 0.001), 1.0, delta=0.01)

    def test_output_saturates(self):
        self.assertEqual(
            PID(1e9, 0.0, 0.0).step(1.0, 0.0, 0.001), PID_SATURATION_ABSOLUTE
        )

    def test_output_saturates_negative(self):
        self.assertEqual(
            PID(1e9, 0.0, 0.0).step(-1.0, 0.0, 0.001), -PID_SATURATION_ABSOLUTE
        )

    def test_non_finite_input_returns_zero(self):
        self.assertEqual(PID(1.0, 1.0, 1.0).step(float("nan"), 0.0, 0.001), 0.0)
        self.assertEqual(PID(1.0, 1.0, 1.0).step(1.0, float("inf"), 0.001), 0.0)

    def test_integral_is_clamped(self):
        pid = PID(0.0, 1.0, 0.0)
        for _ in range(100_000):
            pid.step(1000.0, 0.0, 0.01)
        self.assertLessEqual(pid.integral, PID_SATURATION_ABSOLUTE)

    def test_reset_clears_integral(self):
        pid = PID(0.0, 1.0, 0.0)
        for _ in range(1000):
            pid.step(1.0, 0.0, 0.001)
        pid.reset()
        self.assertAlmostEqual(pid.step(1.0, 0.0, 0.001), 0.001, delta=0.001)

    def test_derivative_zero_on_first_step(self):
        # justReset seeds prevError so the first derivative is not a spike
        self.assertAlmostEqual(PID(0.0, 0.0, 1.0).step(10.0, 0.0, 0.001), 0.0, places=5)


class TestMotionController(unittest.TestCase):
    def setUp(self):
        self.k = Kinematics(WHEEL_RADIUS, AXLE_LEN)
        self.plant = Plant(
            World.blank(4000.0, 4000.0, 5.0),
            PlantConfig(pwm_deadband=0.0),
            Pose(0, 0, 0),
        )
        self.mc = MotionController(
            SimMotor(self.plant, "left"),
            SimMotor(self.plant, "right"),
            self.k,
            20.0,
            3.0,
            0.0,
        )

    def test_zero_desired_stops_motors(self):
        self.mc.update(Velocity(200.0, 0.0), Velocity(0.0, 0.0), 0.001)
        self.mc.update(Velocity(0.0, 0.0), Velocity(0.0, 0.0), 0.001)
        self.assertEqual((self.plant.pwm_left, self.plant.pwm_right), (0, 0))

    def test_zero_desired_clears_the_integrator(self):
        for _ in range(500):
            self.mc.update(Velocity(200.0, 0.0), Velocity(0.0, 0.0), 0.001)
        self.mc.update(Velocity(0.0, 0.0), Velocity(0.0, 0.0), 0.001)
        self.assertEqual(self.mc.leftPID.integral, 0.0)

    def test_drives_forward_when_behind_setpoint(self):
        self.mc.update(Velocity(200.0, 0.0), Velocity(0.0, 0.0), 0.001)
        self.assertGreater(self.plant.pwm_left, 0)
        self.assertGreater(self.plant.pwm_right, 0)

    def test_pwm_clamped_to_max(self):
        self.mc.update(Velocity(10_000.0, 0.0), Velocity(0.0, 0.0), 0.001)
        self.assertLessEqual(self.plant.pwm_left, MAXIMUM_WHEEL_PWM)

    def test_turn_command_drives_wheels_opposite(self):
        self.mc.update(Velocity(0.0, 3.0), Velocity(0.0, 0.0), 0.001)
        self.assertLess(self.plant.pwm_left, 0)
        self.assertGreater(self.plant.pwm_right, 0)

    def _run_closed_loop(self, desired, steps, dt=0.001):
        for _ in range(steps):
            self.plant.step(dt)
            measured = self.k.fk.velocity(self.plant.wheel_velocities())
            self.mc.update(desired, measured, dt)
        return self.k.fk.velocity(self.plant.wheel_velocities())

    def test_closed_loop_converges_to_commanded_velocity(self):
        final = self._run_closed_loop(Velocity(200.0, 0.0), 30_000)
        self.assertAlmostEqual(final.v, 200.0, delta=10.0)

    def test_closed_loop_converges_to_commanded_rotation(self):
        final = self._run_closed_loop(Velocity(0.0, 2.0), 30_000)
        self.assertAlmostEqual(final.omega, 2.0, delta=0.2)

    def test_closed_loop_is_stable_and_does_not_overshoot(self):
        # ki is small enough that the loop approaches from below throughout;
        # no oscillation to worry about at these gains
        peak = 0.0
        for _ in range(30_000):
            self.plant.step(0.001)
            measured = self.k.fk.velocity(self.plant.wheel_velocities())
            peak = max(peak, measured.v)
            self.mc.update(Velocity(200.0, 0.0), measured, 0.001)
        self.assertLessEqual(peak, 205.0)


class TestSlowIntegralAction(unittest.TestCase):
    """ki = 3 at a 1 kHz loop is slow enough to matter.

    Steady state at 200 mm/s needs the integral term to supply most of a PWM
    of ~130, which at ki = 3 means winding the integrator to ~43. Starting
    from zero that takes tens of seconds -- far longer than any single move in
    a maze run. In practice the inner velocity loop therefore operates on
    proportional action alone and settles well short of the commanded
    velocity. This is a property of the firmware's gains, not of the sim.
    """

    def setUp(self):
        self.k = Kinematics(WHEEL_RADIUS, AXLE_LEN)
        self.plant = Plant(
            World.blank(40_000.0, 40_000.0, 20.0),
            PlantConfig(pwm_deadband=0.0),
            Pose(0, 0, 0),
        )
        self.mc = MotionController(
            SimMotor(self.plant, "left"),
            SimMotor(self.plant, "right"),
            self.k,
            20.0,
            3.0,
            0.0,
        )

    def _velocity_after(self, seconds, desired=Velocity(200.0, 0.0)):
        for _ in range(int(seconds * 1000)):
            self.plant.step(0.001)
            measured = self.k.fk.velocity(self.plant.wheel_velocities())
            self.mc.update(desired, measured, 0.001)
        return self.k.fk.velocity(self.plant.wheel_velocities()).v

    def test_still_well_short_of_setpoint_after_three_seconds(self):
        v = self._velocity_after(3.0)
        self.assertLess(v, 0.85 * 200.0)
        self.assertGreater(v, 0.60 * 200.0)  # proportional action alone

    def test_keeps_climbing_as_the_integrator_winds_up(self):
        at_3s = self._velocity_after(3.0)
        at_10s = self._velocity_after(7.0)
        self.assertGreater(at_10s, at_3s)


if __name__ == "__main__":
    unittest.main()
