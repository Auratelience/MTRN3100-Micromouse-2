import math
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
        # The header's arithmetic, not the intended one: the mean's denominator
        # carries the model's weight of 1 but its numerator carries no
        # dead-reckoned term, so 100 at trust 1 averages to 100/2 = 50, and the
        # gain then moves 0 half way to that. See _fusePose.
        self.assertAlmostEqual(sf.estimate.pose().x, 25.0, delta=0.5)

    def test_seeding_the_mean_makes_the_gain_mean_what_it_says(self):
        sf = SensorFusion(
            [VelocitySource(FakeV(0.0, 0.0))],
            [PoseSource(FakeP(Pose(100.0, 0.0, 0.0)), FusionWeights.XPTrust)],
            poseCorrectionGain=0.5,
            seedPoseMeanWithModel=True,
        )
        sf.update(0.001)
        # dead reckoning says x=0, observer says 100, both at weight 1 -> the
        # mean is 50, and gain 0.5 moves 0 half way to it
        self.assertAlmostEqual(sf.estimate.pose().x, 25.0, delta=0.5)

    def test_a_source_that_agrees_still_drags_the_estimate_to_the_origin(self):
        """The header bug, pinned. A pose source reporting exactly what dead
        reckoning already says should be a no-op; instead it decays the estimate
        toward (0, 0, 0) every single tick, because the model's vote in the
        weighted mean is a vote for zero."""
        agreeing = FakeP(Pose(100.0, 0.0, 0.0))
        sf = SensorFusion(
            [VelocitySource(FakeV(0.0, 0.0))],
            [PoseSource(agreeing, ObserverPTrust(0.2, 0.2, 0.1))],
        )
        sf.set(Pose(100.0, 0.0, 0.0))
        for _ in range(50):
            agreeing._p = Pose(sf.estimate.pose().x, 0.0, 0.0)  # always agrees
            sf.update(0.001)
        self.assertLess(sf.estimate.pose().x, 1.0)

        # ...and does not, once the numerator is seeded to match.
        agreeing = FakeP(Pose(100.0, 0.0, 0.0))
        fixed = SensorFusion(
            [VelocitySource(FakeV(0.0, 0.0))],
            [PoseSource(agreeing, ObserverPTrust(0.2, 0.2, 0.1))],
            seedPoseMeanWithModel=True,
        )
        fixed.set(Pose(100.0, 0.0, 0.0))
        for _ in range(50):
            agreeing._p = Pose(fixed.estimate.pose().x, 0.0, 0.0)
            fixed.update(0.001)
        self.assertAlmostEqual(fixed.estimate.pose().x, 100.0, delta=0.01)

    def test_pose_correction_persists_across_ticks(self):
        # modelObserver.set() after fusePose() is what makes this hold
        sf = SensorFusion(
            [VelocitySource(FakeV(0.0, 0.0))],
            [PoseSource(FakeP(Pose(100.0, 0.0, 0.0)), FusionWeights.XPTrust)],
            poseCorrectionGain=0.5,
        )
        sf.update(0.001)
        sf.update(0.001)
        # 0 -> 25 -> 25 + 0.5 * (50 - 25)
        self.assertAlmostEqual(sf.estimate.pose().x, 37.5, delta=0.5)

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
        # 3.0 -> -3.0 is a 0.283 rad step through pi, not a 6 rad swing. Only
        # half of it lands, for the same reason as the x tests above: the
        # weighted mean's denominator counts the model, the numerator does not.
        self.assertAlmostEqual(sf.estimate.pose().theta, math.pi, delta=0.01)

    def test_seeding_the_mean_leaves_theta_alone(self):
        """theta's numerator accumulates deltas, and the model's own delta is
        zero, so its vote at weight 1 is already the right one. The x/y fix has
        nothing to do there and must not change the answer."""
        for seeded in (False, True):
            fake = FakeP(Pose(0.0, 0.0, 0.0))
            sf = SensorFusion(
                [VelocitySource(FakeV(0.0, 0.0))],
                [PoseSource(fake, FusionWeights.ThetaPTrust)],
                poseCorrectionGain=1.0,
                seedPoseMeanWithModel=seeded,
            )
            sf.set(Pose(0.0, 0.0, 3.0))
            fake._p = Pose(0.0, 0.0, -3.0)
            sf.update(0.001)
            # half of the 0.283 rad step, the model holding the other half
            self.assertAlmostEqual(sf.estimate.pose().theta, math.pi, delta=0.01)

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
