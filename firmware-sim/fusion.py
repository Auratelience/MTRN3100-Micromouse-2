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
        seedPoseMeanWithModel=False,
    ):
        # etl::vector bounds are fixed capacity; the C++ constructor clamps
        # with etl::min and silently drops the overflow. Same here.
        self.velocitySources = list(velocitySrcs)[:SENSOR_FUSION_MAX_VELOCITY_OBSERVERS]
        self.poseSources = list(poseSrcs)[:SENSOR_FUSION_MAX_POSE_OBSERVERS]

        self.fusedVelocity = Velocity(0.0, 0.0)
        self.fusedPose = Pose(0.0, 0.0, 0.0)
        self.poseCorrectionGain = poseCorrectionGain
        # See _fusePose(). False mirrors sensorFusion.h exactly, which is this
        # module's job; True is the one-line change that makes the weighted mean
        # mean what its own comment says.
        self.seedPoseMeanWithModel = seedPoseMeanWithModel

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
        # All three weights start at 1.0 "to represent the Model_Observer's
        # trust", as sensorFusion.h has it. For theta that is consistent: the
        # numerator accumulates *deltas*, and the model's own delta really is
        # zero, so the mean is an honest blend that applies 1/(1+t) of the
        # observer's correction.
        #
        # For x and y it is not. Those numerators accumulate *absolute*
        # positions, and the model contributes nothing to them -- so its vote is
        # a vote for the origin rather than for where dead reckoning says the
        # robot is. With one source at trust t the mean comes out
        # t*correction/(1+t), and a source agreeing exactly with dead reckoning
        # still drags the estimate toward (0, 0) by g/(1+t) of the remaining
        # distance every tick: 17% per control loop at the .ino's t=0.2, g=0.2.
        # At 1 kHz the position is pinned at the origin within milliseconds,
        # whatever the robot does.
        #
        # Mirrored rather than quietly corrected: predicting what the firmware
        # does is the whole job of this file. seedPoseMeanWithModel gives x and
        # y the matching dead-reckoned term, which is what the header's own
        # comment describes. theta is untouched by it -- there is nothing there
        # to fix. Run the sim both ways before changing the header.
        if self.seedPoseMeanWithModel:
            xTotal, yTotal = dead_reckoned.x, dead_reckoned.y
        else:
            xTotal = yTotal = 0.0
        dthetaTotal = 0.0
        xWeightTotal = yWeightTotal = dthetaWeightTotal = 1.0

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
