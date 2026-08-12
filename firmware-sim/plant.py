"""Ground-truth robot: the physical thing the firmware code is trying to
control, and the only place true state exists.

Sim-only. Everything the ported firmware modules see comes out of this class
through the sensor accessors -- quantised encoder counts, a biased and noisy
gyro, ranged lidar -- never through `pose`. `pose` is for the visualiser and
for tests.
"""

import math
import random
from dataclasses import dataclass

from .constants import (
    AXLE_LEN,
    ENC_CPR,
    ENC_SCALE_LEFT,
    ENC_SCALE_RIGHT,
    LIDAR_MOUNT_FRONT_THETA,
    LIDAR_MOUNT_FRONT_X,
    LIDAR_MOUNT_FRONT_Y,
    LIDAR_MOUNT_LEFT_THETA,
    LIDAR_MOUNT_LEFT_X,
    LIDAR_MOUNT_LEFT_Y,
    LIDAR_MOUNT_RIGHT_THETA,
    LIDAR_MOUNT_RIGHT_X,
    LIDAR_MOUNT_RIGHT_Y,
    MAXIMUM_WHEEL_ANGULAR_VELOCITY,
    MAXIMUM_WHEEL_PWM,
    TWO_PI,
    WHEEL_RADIUS,
)
from .types import Pose, WheelVelocities

# Sensor mount poses relative to the robot centre: offset AND bearing, so a
# sensor that is not aimed radially outward from its mounting point is still
# represented correctly.
#
# These are constants.h's measured LIDAR_MOUNT_* values, keyed by name rather
# than by LIDAR sensor index because the plant's raycast interface is by name.
# LidarObserver.MOUNTS is the same three poses in index order; both read the
# same constants, so the observer's model of where the sensors are and the
# plant's model of where they are cannot drift apart.
LIDAR_MOUNTS = {
    "front": Pose(LIDAR_MOUNT_FRONT_X, LIDAR_MOUNT_FRONT_Y, LIDAR_MOUNT_FRONT_THETA),
    "left": Pose(LIDAR_MOUNT_LEFT_X, LIDAR_MOUNT_LEFT_Y, LIDAR_MOUNT_LEFT_THETA),
    "right": Pose(LIDAR_MOUNT_RIGHT_X, LIDAR_MOUNT_RIGHT_Y, LIDAR_MOUNT_RIGHT_THETA),
}

# How far a simulated ray is allowed to travel before the plant gives up. The
# VL6180X's own 300 mm ceiling is applied in lidar.py, not here: the plant
# reports true geometry and the sensor model clips it.
MAX_RAYCAST_MM = 2000.0


@dataclass
class PlantConfig:
    # First-order velocity lag. 60 ms is the value at which the old sim_ps.py
    # reproduced the observed turn failure. Not measured from a step response.
    motor_tau: float = 0.060
    # PWM below which the wheel does not turn. Not measured.
    pwm_deadband: float = 20.0
    # 1.0 means ENC_SCALE_* is exactly right. Perturb to test the estimator's
    # sensitivity to a stale wheel calibration.
    enc_scale_error_left: float = 1.0
    enc_scale_error_right: float = 1.0
    # rad/s of constant gyro offset, which ImuObserver.init() must measure out
    gyro_bias: float = 0.01
    gyro_noise: float = 0.005
    lidar_noise_mm: float = 1.5
    seed: int = 0


class Plant:
    def __init__(self, world, config=None, start_pose=None):
        self.world = world
        self.config = config if config is not None else PlantConfig()
        self._start_pose = start_pose if start_pose is not None else Pose(0.0, 0.0, 0.0)
        self.rng = random.Random(self.config.seed)
        self.reset()

    def reset(self):
        s = self._start_pose
        self.pose = Pose(s.x, s.y, s.theta)
        self.wheel_speed_left = 0.0
        self.wheel_speed_right = 0.0
        self.wheel_angle_left = 0.0
        self.wheel_angle_right = 0.0
        self.pwm_left = 0
        self.pwm_right = 0
        self.rng = random.Random(self.config.seed)

    # --- actuation ------------------------------------------------------

    def set_pwm(self, left, right):
        self.pwm_left = int(left)
        self.pwm_right = int(right)

    def set_pwm_side(self, side, value):
        if side == "left":
            self.pwm_left = int(value)
        else:
            self.pwm_right = int(value)

    def _target_speed(self, pwm):
        """PWM -> steady-state wheel angular velocity, through the deadband."""
        deadband = self.config.pwm_deadband
        magnitude = abs(pwm)
        if magnitude <= deadband:
            return 0.0
        span = MAXIMUM_WHEEL_PWM - deadband
        if span <= 0:
            return 0.0
        duty = min(1.0, (magnitude - deadband) / span)
        return math.copysign(duty * MAXIMUM_WHEEL_ANGULAR_VELOCITY, pwm)

    def step(self, dt):
        """Advance ground truth by dt seconds."""
        tau = self.config.motor_tau
        target_l = self._target_speed(self.pwm_left)
        target_r = self._target_speed(self.pwm_right)

        if tau <= 0.0:
            self.wheel_speed_left = target_l
            self.wheel_speed_right = target_r
        else:
            k = dt / tau
            if k > 1.0:
                k = 1.0
            self.wheel_speed_left += (target_l - self.wheel_speed_left) * k
            self.wheel_speed_right += (target_r - self.wheel_speed_right) * k

        self.wheel_angle_left += self.wheel_speed_left * dt
        self.wheel_angle_right += self.wheel_speed_right * dt

        v = (WHEEL_RADIUS / 2.0) * (self.wheel_speed_left + self.wheel_speed_right)
        omega = (WHEEL_RADIUS / AXLE_LEN) * (
            self.wheel_speed_right - self.wheel_speed_left
        )

        self.pose.x += v * math.cos(self.pose.theta) * dt
        self.pose.y += v * math.sin(self.pose.theta) * dt
        self.pose.theta += omega * dt

        self._true_omega = omega
        self._true_v = v

    def wheel_velocities(self):
        return WheelVelocities(self.wheel_speed_left, self.wheel_speed_right)

    # --- encoders -------------------------------------------------------
    #
    # A true wheel revolution produces ENC_RAD_PER_REV_* radians of *raw*
    # reading on hardware, i.e. fewer counts than a perfect 2 pi would imply.
    # ENC_SCALE_* corrects that, so counts per true radian is
    # ENC_CPR / (ENC_SCALE * 2 pi). With the error knob at 1.0 a true
    # revolution therefore reads back as exactly 2 pi.

    def _counts(self, angle, scale, error):
        return int(angle * ENC_CPR / (scale * TWO_PI) * error)

    def count_left(self):
        return self._counts(
            self.wheel_angle_left, ENC_SCALE_LEFT, self.config.enc_scale_error_left
        )

    def count_right(self):
        return self._counts(
            self.wheel_angle_right, ENC_SCALE_RIGHT, self.config.enc_scale_error_right
        )

    def angular_displacement_left(self):
        """What Motor::angularDisplacement() returns on hardware (rad)."""
        return ENC_SCALE_LEFT * TWO_PI * self.count_left() / ENC_CPR

    def angular_displacement_right(self):
        return ENC_SCALE_RIGHT * TWO_PI * self.count_right() / ENC_CPR

    # --- IMU ------------------------------------------------------------

    def gyro_z(self):
        omega = getattr(self, "_true_omega", 0.0)
        noise = (
            self.rng.gauss(0.0, self.config.gyro_noise)
            if self.config.gyro_noise > 0.0
            else 0.0
        )
        return omega + self.config.gyro_bias + noise

    def accel_x(self):
        # The firmware's ImuObserver has this path commented out as "too
        # unreliable for use". Present for parity, reports nothing useful.
        return 0.0

    def accel_y(self):
        return 0.0

    # --- lidar ----------------------------------------------------------

    def mount_pose(self, sensor):
        """World-frame pose of a sensor mount."""
        m = LIDAR_MOUNTS[sensor]
        c, s = math.cos(self.pose.theta), math.sin(self.pose.theta)
        return Pose(
            self.pose.x + m.x * c - m.y * s,
            self.pose.y + m.x * s + m.y * c,
            self.pose.theta + m.theta,
        )

    def range_mm(self, sensor, max_range_mm=MAX_RAYCAST_MM):
        """True distance from a sensor mount to the nearest obstacle, plus
        range noise. Quantisation and the 300 mm ceiling belong to the sensor
        model in lidar.py, not here."""
        mp = self.mount_pose(sensor)
        d = self.world.raycast(mp.x, mp.y, mp.theta, max_range_mm)
        if self.config.lidar_noise_mm > 0.0 and d < max_range_mm:
            d += self.rng.gauss(0.0, self.config.lidar_noise_mm)
            if d < 0.0:
                d = 0.0
        return d

    # --- test seams -----------------------------------------------------
    #
    # Underscore-prefixed: these reach past the actuation path to place the
    # robot or its wheels directly. For tests and for Runner.reset() only.

    def _set_pose(self, pose):
        self.pose = Pose(pose.x, pose.y, pose.theta)

    def _set_true_wheel_angles(self, left, right):
        self.wheel_angle_left = left
        self.wheel_angle_right = right
