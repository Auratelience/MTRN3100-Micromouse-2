import unittest

from ..fusion import (
    FusionWeights,
    ObserverPTrust,
    ObserverVTrust,
    PoseSource,
    SensorFusion,
    VelocitySource,
)
from ..observers import ObserverP, ObserverV
from ..types import Pose, Velocity


class FakeV(ObserverV):
    def __init__(self, v, omega, ready=True):
        self._v = Velocity(v, omega)
        self._ready = ready

    def estimate(self):
        return self._v

    def set(self, v):
        pass

    def update(self, dt):
        pass

    def ready(self):
        return self._ready


class FakeP(ObserverP):
    def __init__(self, pose, ready=True):
        self._p = pose
        self._ready = ready

    def estimate(self):
        return self._p

    def set(self, p):
        self._p = p

    def update(self, dt):
        pass

    def ready(self):
        return self._ready


class TestVelocityFusion(unittest.TestCase):
    def test_trust_weights_select_per_channel(self):
        # wheel trusted for v, imu trusted for omega -- the TASK 3.4 config
        sf = SensorFusion(
            [
                VelocitySource(FakeV(100.0, 9.0), ObserverVTrust(1.0, 0.0)),
                VelocitySource(FakeV(0.0, 2.0), FusionWeights.OmegaVTrust),
            ]
        )
        sf.update(0.001)
        self.assertAlmostEqual(sf.estimate.velocity().v, 100.0, places=4)
        self.assertAlmostEqual(sf.estimate.velocity().omega, 2.0, places=4)

    def test_weighted_average_of_two_sources(self):
        sf = SensorFusion(
            [
                VelocitySource(FakeV(100.0, 0.0), ObserverVTrust(1.0, 1.0)),
                VelocitySource(FakeV(200.0, 0.0), ObserverVTrust(3.0, 1.0)),
            ]
        )
        sf.update(0.001)
        self.assertAlmostEqual(sf.estimate.velocity().v, 175.0, places=4)

    def test_unready_observer_is_skipped(self):
        sf = SensorFusion(
            [
                VelocitySource(FakeV(100.0, 0.0), ObserverVTrust(1.0, 1.0)),
                VelocitySource(FakeV(999.0, 0.0), ObserverVTrust(1.0, 1.0)),
            ]
        )
        sf.velocitySources[1].observer._ready = False
        sf.update(0.001)
        self.assertAlmostEqual(sf.estimate.velocity().v, 100.0, places=4)

    def test_all_unready_holds_previous_velocity(self):
        sf = SensorFusion([VelocitySource(FakeV(100.0, 0.0))])
        sf.update(0.001)
        sf.velocitySources[0].observer._ready = False
        sf.update(0.001)
        self.assertAlmostEqual(sf.estimate.velocity().v, 100.0, places=4)

    def test_velocity_source_capacity_is_bounded(self):
        srcs = [VelocitySource(FakeV(1.0, 0.0)) for _ in range(10)]
        sf = SensorFusion(srcs)
        self.assertLessEqual(len(sf.velocitySources), 4)


class TestPoseIntegration(unittest.TestCase):
    def test_dead_reckons_when_no_pose_sources(self):
        sf = SensorFusion([VelocitySource(FakeV(100.0, 0.0))])
        for _ in range(1000):
            sf.update(0.001)
        self.assertAlmostEqual(sf.estimate.pose().x, 100.0, delta=0.5)

    def test_pose_source_pulls_estimate_by_correction_gain(self):
        sf = SensorFusion(
            [VelocitySource(FakeV(0.0, 0.0))],
            [PoseSource(FakeP(Pose(100.0, 0.0, 0.0)), FusionWeights.XPTrust)],
            poseCorrectionGain=0.5,
        )
        sf.update(0.001)
        # The mean is over the pose sources alone -- the model gets no vote --
        # so one source at any nonzero trust means the mean is just its estimate,
        # 100, and the gain moves dead reckoning half way there. See _fusePose.
        self.assertAlmostEqual(sf.estimate.pose().x, 50.0, delta=0.5)

    def test_the_mean_is_scale_invariant_in_the_trusts(self):
        """With the weights no longer seeded at 1.0, only the *ratio* of the
        trusts survives into the weighted mean. A lone source at trust 0.2 and
        the same source at trust 1.0 must land in the same place -- which is
        why the .ino moving from (0.2, 0.2, 0.1) to XYPTrust changed nothing."""
        for trust in (ObserverPTrust(0.2, 0.0, 0.0), FusionWeights.XPTrust):
            sf = SensorFusion(
                [VelocitySource(FakeV(0.0, 0.0))],
                [PoseSource(FakeP(Pose(100.0, 0.0, 0.0)), trust)],
                poseCorrectionGain=0.5,
            )
            sf.update(0.001)
            self.assertAlmostEqual(sf.estimate.pose().x, 50.0, delta=0.5)

    def test_a_source_that_agrees_is_a_no_op(self):
        """The regression the header fix bought. A pose source reporting exactly
        what dead reckoning already says must leave the estimate alone. Under the
        old arithmetic -- weights seeded at 1.0, numerators at 0 -- the model's
        vote was a vote for the *origin*, and this decayed to 0 within a few
        dozen ticks whatever the robot was doing."""
        agreeing = FakeP(Pose(100.0, 0.0, 0.0))
        sf = SensorFusion(
            [VelocitySource(FakeV(0.0, 0.0))],
            [PoseSource(agreeing, FusionWeights.XYPTrust)],
        )
        sf.set(Pose(100.0, 0.0, 0.0))
        for _ in range(50):
            agreeing._p = Pose(sf.estimate.pose().x, 0.0, 0.0)  # always agrees
            sf.update(0.001)
        self.assertAlmostEqual(sf.estimate.pose().x, 100.0, delta=0.01)

    def test_an_axis_with_no_trust_keeps_dead_reckoning(self):
        """XYPTrust is (1, 1, 0), so dthetaWeightTotal is zero every tick and
        the `<= 0.0` guard hands theta straight back. This is the .ino's live
        configuration, not an edge case: heading there is pure dead reckoning."""
        fake = FakeP(Pose(0.0, 0.0, 1.0))
        sf = SensorFusion(
            [VelocitySource(FakeV(0.0, 0.0))],
            [PoseSource(fake, FusionWeights.XYPTrust)],
            poseCorrectionGain=1.0,
        )
        sf.set(Pose(0.0, 0.0, 0.5))
        fake._p = Pose(0.0, 0.0, 1.0)  # disagrees by 0.5 rad
        sf.update(0.001)
        self.assertAlmostEqual(sf.estimate.pose().theta, 0.5, places=4)

    def test_pose_correction_persists_across_ticks(self):
        # modelObserver.set() after fusePose() is what makes this hold
        sf = SensorFusion(
            [VelocitySource(FakeV(0.0, 0.0))],
            [PoseSource(FakeP(Pose(100.0, 0.0, 0.0)), FusionWeights.XPTrust)],
            poseCorrectionGain=0.5,
        )
        sf.update(0.001)
        sf.update(0.001)
        # 0 -> 50 -> 50 + 0.5 * (100 - 50)
        self.assertAlmostEqual(sf.estimate.pose().x, 75.0, delta=0.5)

    def test_unready_pose_source_applies_no_correction(self):
        sf = SensorFusion(
            [VelocitySource(FakeV(0.0, 0.0))],
            [PoseSource(FakeP(Pose(100.0, 0.0, 0.0), ready=False), FusionWeights.XPTrust)],
            poseCorrectionGain=0.5,
        )
        sf.update(0.001)
        self.assertAlmostEqual(sf.estimate.pose().x, 0.0, places=4)

    def test_set_pose_propagates_to_pose_observers(self):
        # SensorFusion::set(Pose) re-datums every pose source, so an observer
        # cannot disagree with the pose it was just handed
        fake = FakeP(Pose(99.0, 0.0, 0.0))
        sf = SensorFusion(
            [VelocitySource(FakeV(0.0, 0.0))],
            [PoseSource(fake, FusionWeights.XPTrust)],
        )
        sf.set(Pose(7.0, 0.0, 0.0))
        self.assertAlmostEqual(fake.estimate().x, 7.0)

    def test_theta_trust_corrects_through_the_wrap(self):
        fake = FakeP(Pose(0.0, 0.0, 0.0))
        sf = SensorFusion(
            [VelocitySource(FakeV(0.0, 0.0))],
            [PoseSource(fake, FusionWeights.ThetaPTrust)],
            poseCorrectionGain=1.0,
        )
        sf.set(Pose(0.0, 0.0, 3.0))
        # set() just re-datumed the observer; make it disagree again
        fake._p = Pose(0.0, 0.0, -3.0)
        sf.update(0.001)
        # 3.0 -> -3.0 is a 0.283 rad step through pi, not a 6 rad swing, and at
        # gain 1.0 all of it lands: the model no longer holds half the weight.
        # 3.0 + 0.283 wraps back to -3.0.
        self.assertAlmostEqual(sf.estimate.pose().theta, -3.0, delta=0.01)

    def test_set_pose_resets_model_observer(self):
        sf = SensorFusion([VelocitySource(FakeV(0.0, 0.0))])
        sf.set(Pose(7.0, 8.0, 0.5))
        sf.update(0.001)
        self.assertAlmostEqual(sf.estimate.pose().x, 7.0, places=4)
        self.assertAlmostEqual(sf.estimate.pose().theta, 0.5, places=4)

    def test_set_rejects_other_types(self):
        sf = SensorFusion([VelocitySource(FakeV(0.0, 0.0))])
        with self.assertRaises(TypeError):
            sf.set("not a pose")


if __name__ == "__main__":
    unittest.main()
