"""Mirrors micromouse/control.h."""

import math

from .constants import MAXIMUM_WHEEL_PWM, PID_SATURATION_ABSOLUTE


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prevError = 0.0
        self.justReset = True

    def step(self, setpoint, measurement, dt):
        # Bail on any non-finite input so a stray NaN/Inf can't permanently
        # poison the integrator or flow through round() into an int cast.
        if not (
            math.isfinite(setpoint) and math.isfinite(measurement) and math.isfinite(dt)
        ):
            return 0.0

        error = setpoint - measurement
        if self.justReset:
            self.prevError = error
            self.justReset = False

        if self.ki != 0:
            self.integral += error * dt
        if self.integral > PID_SATURATION_ABSOLUTE:
            self.integral = PID_SATURATION_ABSOLUTE
        if self.integral < -PID_SATURATION_ABSOLUTE:
            self.integral = -PID_SATURATION_ABSOLUTE

        derivative = 0.0
        if self.kd != 0:
            derivative = (error - self.prevError) / dt
        self.prevError = error

        result = self.kp * error + self.ki * self.integral + self.kd * derivative
        if result >= PID_SATURATION_ABSOLUTE:
            return PID_SATURATION_ABSOLUTE
        if result <= -PID_SATURATION_ABSOLUTE:
            return -PID_SATURATION_ABSOLUTE
        return result

    def reset(self):
        self.integral = 0.0
        self.prevError = 0.0
        self.justReset = True


class MotionController:
    def __init__(self, leftMotor, rightMotor, kinematics, kp, ki, kd):
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.kinematics = kinematics
        self.leftPID = PID(kp, ki, kd)
        self.rightPID = PID(kp, ki, kd)

    def update(self, desired, current, dt):
        # A zero desired velocity means "stop", not "regulate to zero with a
        # wound-up integrator still driving the motors". Cut the motors and
        # clear PID state so a finished plan (planner Wait -> {0, 0}) brings
        # the robot to rest instead of coasting on the stale integral term.
        if desired.v == 0.0 and desired.omega == 0.0:
            self.reset()
            self.leftMotor.stop()
            self.rightMotor.stop()
            return

        targetWV = self.kinematics.ik.velocity(desired)
        currentWV = self.kinematics.ik.velocityRaw(current)

        left = int(round(self.leftPID.step(targetWV.left, currentWV.left, dt)))
        right = int(round(self.rightPID.step(targetWV.right, currentWV.right, dt)))

        self.leftMotor.move(self._clamp(left))
        self.rightMotor.move(self._clamp(right))

    def reset(self):
        self.leftPID.reset()
        self.rightPID.reset()

    @staticmethod
    def _clamp(x):
        if x > MAXIMUM_WHEEL_PWM:
            return MAXIMUM_WHEEL_PWM
        if x < -MAXIMUM_WHEEL_PWM:
            return -MAXIMUM_WHEEL_PWM
        return x
