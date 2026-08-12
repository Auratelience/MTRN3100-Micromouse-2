"""Mirrors micromouse/kinematics.h."""

from .constants import MAXIMUM_WHEEL_ANGULAR_VELOCITY
from .types import Velocity, WheelVelocities


class Kinematics:
    def __init__(self, radius, length):
        self.r = radius
        self.l = length
        self.ik = Kinematics.IK(self)
        self.fk = Kinematics.FK(self)

    class FK:
        def __init__(self, k):
            self.k = k

        def velocity(self, wv):
            k = self.k
            return Velocity(
                (k.r / 2) * (wv.left + wv.right),
                (k.r / k.l) * (wv.right - wv.left),
            )

    class IK:
        def __init__(self, k):
            self.k = k

        def velocityRaw(self, v):
            """Unclamped body-velocity -> wheel-speed conversion. Use for a
            *measured* velocity: saturating a noise spike here would distort
            the tracking error the controller sees."""
            k = self.k
            return WheelVelocities(
                (1.0 / k.r) * (v.v - v.omega * k.l / 2.0),
                (1.0 / k.r) * (v.v + v.omega * k.l / 2.0),
            )

        def velocity(self, v):
            """As velocityRaw(), but scaled so neither wheel exceeds
            MAXIMUM_WHEEL_ANGULAR_VELOCITY. Use for command/target
            velocities."""
            wv = self.velocityRaw(v)
            # Clamp on magnitude so large negative wheel speeds (fast reverse /
            # sharp turns) are scaled too, not just positive ones.
            exceeder = max(abs(wv.left), abs(wv.right))
            if exceeder > MAXIMUM_WHEEL_ANGULAR_VELOCITY:
                scale_factor = MAXIMUM_WHEEL_ANGULAR_VELOCITY / exceeder
                wv.left *= scale_factor
                wv.right *= scale_factor
            return wv
