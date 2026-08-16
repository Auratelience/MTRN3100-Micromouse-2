"""Mirrors micromouse/sensorFusion.h."""

from dataclasses import dataclass, field

from .constants import (
    SENSOR_FUSION_MAX_POSE_OBSERVERS,
    SENSOR_FUSION_MAX_VELOCITY_OBSERVERS,
)
from .observers import ModelObserver
from .types import Pose, Velocity, wrapAngle


@dataclass
class ObserverVTrust:
    vTrust: float
    omegaTrust: float


@dataclass
class ObserverPTrust:
    xTrust: float
    yTrust: float
    thetaTrust: float


class FusionWeights:
    PoseCorrectionGain = 0.2

    DefaultPTrust = ObserverPTrust(1, 1, 1)
    ThetaPTrust = ObserverPTrust(0, 0, 1)
    XYPTrust = ObserverPTrust(1, 1, 0)
    XPTrust = ObserverPTrust(1, 0, 0)
    YPTrust = ObserverPTrust(0, 1, 0)

    DefaultVTrust = ObserverVTrust(1, 1)
    VVTrust = ObserverVTrust(1, 0)
    OmegaVTrust = ObserverVTrust(0, 1)


@dataclass
class VelocitySource:
    observer: object
    trust: ObserverVTrust = field(
        default_factory=lambda: ObserverVTrust(1, 1)
    )


@dataclass
class PoseSource:
    observer: object
    trust: ObserverPTrust = field(
        default_factory=lambda: ObserverPTrust(1, 1, 1)
    )


class _Estimate:
    """Mirrors the nested SensorFusion::estimate accessor object."""

    def __init__(self, sf):
        self.sf = sf

    def pose(self):
        return self.sf.fusedPose

    def velocity(self):
        return self.sf.fusedVelocity


class SensorFusion:
    """Minimum 1 velocity observer and 0 pose observers."""

    def __init__(
        self,
        velocitySrcs,
        poseSrcs=(),
        poseCorrectionGain=FusionWeights.PoseCorrectionGain,
    ):
        # etl::vector bounds are fixed capacity; the C++ constructor clamps
        # with etl::min and silently drops the overflow. Same here.
        self.velocitySources = list(velocitySrcs)[:SENSOR_FUSION_MAX_VELOCITY_OBSERVERS]
        self.poseSources = list(poseSrcs)[:SENSOR_FUSION_MAX_POSE_OBSERVERS]

        self.fusedVelocity = Velocity(0.0, 0.0)
        self.fusedPose = Pose(0.0, 0.0, 0.0)
        self.poseCorrectionGain = poseCorrectionGain

        self.modelObserver = ModelObserver(self._getFusedVelocity)
        self.estimate = _Estimate(self)

    # C++ overloads set(Pose) and set(Velocity); Python dispatches on type.
    def set(self, value):
        if isinstance(value, Pose):
            self._setPose(value)
        elif isinstance(value, Velocity):
            self._setVelocity(value)
        else:
            raise TypeError(f"set() takes a Pose or a Velocity, got {type(value)!r}")

    def _setPose(self, p):
        self.fusedPose = Pose(p.x, p.y, p.theta)
        self.modelObserver.set(p)
        for src in self.poseSources:
            src.observer.set(p)

    def _setVelocity(self, v):
        self.fusedVelocity = Velocity(v.v, v.omega)
        for src in self.velocitySources:
            if src.observer.ready():
                src.observer.set(v)

    def update(self, dt):
        # Order matters: fuse velocity, dead-reckon from it, then correct.
        # Applying the correction after modelObserver.update() and feeding it
        # back with modelObserver.set() is what makes a correction persist
        # rather than being overwritten on the next tick.
        for src in self.velocitySources:
            src.observer.update(dt)

        self.fusedVelocity = self._fuseVelocity()
        self.modelObserver.update(dt)
        self.fusedPose = self.modelObserver.estimate()

        if self.poseSources:
            any_ready = False
            for src in self.poseSources:
                if src.observer.ready():
                    src.observer.update(dt)
                    any_ready = True
            if any_ready:
                self.fusedPose = self._fusePose(self.fusedPose)
                self.modelObserver.set(self.fusedPose)

    # --- internals ------------------------------------------------------

    def _getFusedVelocity(self):
        return self.fusedVelocity

    def _fuseVelocity(self):
        vWeightTotal = omegaWeightTotal = 0.0
        vTotal = omegaTotal = 0.0

        for src in self.velocitySources:
            if not src.observer.ready():
                continue

            velocityEstimate = src.observer.estimate()
            t = src.trust

            vWeightTotal += t.vTrust
            omegaWeightTotal += t.omegaTrust
            vTotal += t.vTrust * velocityEstimate.v
            omegaTotal += t.omegaTrust * velocityEstimate.omega

        return Velocity(
            self.fusedVelocity.v if vWeightTotal <= 0.0 else vTotal / vWeightTotal,
            self.fusedVelocity.omega
            if omegaWeightTotal <= 0.0
            else omegaTotal / omegaWeightTotal,
        )

    def _fusePose(self, dead_reckoned):
        # All six totals start at zero, so the ModelObserver gets no vote of its
        # own and the weighted mean is over the pose sources alone. The result
        # is then applied as a *correction* to dead reckoning rather than as a
        # replacement for it: each axis moves poseCorrectionGain of the way from
        # where dead reckoning says the robot is to where the sources say it is.
        #
        # An axis no source has an opinion on keeps dead reckoning untouched,
        # which is what the `<= 0.0` guards are for. That is not a rare edge
        # case -- the .ino runs its lidar source at FusionWeights::XYPTrust
        # (1, 1, 0), so dthetaWeightTotal is zero every tick and theta is pure
        # dead reckoning by construction.
        #
        # This is the corrected form. Earlier revisions of sensorFusion.h seeded
        # the three weights at 1.0 "to represent the Model_Observer's trust"
        # while leaving the numerators at zero, so the model's vote for x and y
        # was a vote for the *origin*: a source agreeing exactly with dead
        # reckoning still dragged the estimate toward (0, 0) by g/(1+t) of the
        # remaining distance every tick, pinning the position within
        # milliseconds at 1 kHz. theta escaped it because its numerator
        # accumulates deltas, and the model's own delta really is zero.
        xTotal = yTotal = dthetaTotal = 0.0
        xWeightTotal = yWeightTotal = dthetaWeightTotal = 0.0

        for src in self.poseSources:
            if not src.observer.ready():
                continue

            correction = src.observer.estimate()
            t = src.trust

            dtheta = wrapAngle(correction.theta - dead_reckoned.theta)

            xWeightTotal += t.xTrust
            yWeightTotal += t.yTrust
            dthetaWeightTotal += t.thetaTrust

            xTotal += t.xTrust * correction.x
            yTotal += t.yTrust * correction.y
            dthetaTotal += t.thetaTrust * dtheta

        g = self.poseCorrectionGain

        return Pose(
            dead_reckoned.x
            if xWeightTotal <= 0.0
            else dead_reckoned.x + g * (xTotal / xWeightTotal - dead_reckoned.x),
            dead_reckoned.y
            if yWeightTotal <= 0.0
            else dead_reckoned.y + g * (yTotal / yWeightTotal - dead_reckoned.y),
            wrapAngle(
                dead_reckoned.theta
                if dthetaWeightTotal <= 0.0
                else dead_reckoned.theta + g * dthetaTotal / dthetaWeightTotal
            ),
        )
