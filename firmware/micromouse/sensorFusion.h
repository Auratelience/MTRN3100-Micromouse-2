// Sensor Fusion
//
// Zimmy Levi z5587840

#pragma once

#include <Embedded_Template_Library.h>
#include <etl/vector.h>
#include <etl/span.h>
#include <etl/algorithm.h>

#include "types.h"
#include "observers.h"
#include "constants.h"

#pragma GCC push_options
#pragma GCC optimize("O2")

struct ObserverVTrust {
    float vTrust;
    float omegaTrust;
};

struct ObserverPTrust {
    float xTrust;
    float yTrust;
    float thetaTrust;
};

namespace FusionWeights {
// How much of a pose observer's correction is taken per fusion cycle, per
// axis. Split because the two axes are now corrected by different sensors with
// very different standing: position comes from three lidar beams against a map,
// heading from a gyro integrated on its own clock.
//
// Position stays low. A lidar fix is a least-squares solve against a map that
// may be wrong -- MazeWallMap holds only the walls found so far -- and taking
// a fifth of it per cycle averages several solves before the estimate has
// moved far, so one bad association drags nothing very far.
//
// Heading is high, because there is nothing to average against. ImuObserver is
// the only source weighted for theta, and the dead reckoning it corrects is
// wheel rotation through an AXLE_LEN that was never calibrated for turning --
// the worse of the two, not a second opinion. At 0.8 the fused heading is
// effectively the gyro's, with the wheels contributing only a transient, which
// is the intent.
//
// Both are per cycle, not per second, so the time constant they imply scales
// with the loop rate: 0.8 at ~300 Hz settles in about 4 ms, 0.2 in about 17 ms.
constexpr float PositionCorrectionGain = 0.2f;
constexpr float ThetaCorrectionGain    = 0.8f;

    constexpr ObserverPTrust DefaultPTrust = ObserverPTrust{1, 1, 1};
    constexpr ObserverPTrust ThetaPTrust   = ObserverPTrust{0, 0, 1};
    constexpr ObserverPTrust XYPTrust      = ObserverPTrust{1, 1, 0};
    constexpr ObserverPTrust XPTrust       = ObserverPTrust{1, 0, 0};
    constexpr ObserverPTrust YPTrust       = ObserverPTrust{0, 1, 0};

    constexpr ObserverVTrust DefaultVTrust = ObserverVTrust{1, 1};
    constexpr ObserverVTrust VVTrust       = ObserverVTrust{1, 0};
    constexpr ObserverVTrust OmegaVTrust   = ObserverVTrust{0, 1};
}

// Combines velocity and pose observers to provide combined estimates
struct VelocitySource {
    ObserverV* observer;
    ObserverVTrust trust = FusionWeights::DefaultVTrust;
};

struct PoseSource {
    ObserverP* observer;
    ObserverPTrust trust = FusionWeights::DefaultPTrust;
};

class SensorFusion {
    public:

    class estimate {
        public:

        estimate(const SensorFusion& sf) : sf(sf) {}

        Pose pose() const {
            return sf.fusedPose;
        }

        Velocity velocity() const {
            return sf.fusedVelocity;
        }

        private:

        const SensorFusion& sf;
    };

    // Minimum 1 velocity observer and 0 pose observers
    SensorFusion(
        etl::span<const VelocitySource> velocitySrcs,
        etl::span<const PoseSource> poseSrcs = {},
        float positionCorrectionGain         = FusionWeights::PositionCorrectionGain,
        float thetaCorrectionGain            = FusionWeights::ThetaCorrectionGain
    ) :
        estimate(*this),
        modelObserver(
            ModelObserver::VelocityFunc::create<SensorFusion, &SensorFusion::getFusedVelocity>(
                *this
            )
        ),
        fusedVelocity({0, 0}),
        fusedPose({0, 0, 0}),
        positionCorrectionGain(positionCorrectionGain),
        thetaCorrectionGain(thetaCorrectionGain) {
        const size_t nv = etl::min(velocitySrcs.size(), velocitySources.max_size());
        for (size_t i = 0; i < nv; ++i)
            velocitySources.push_back(velocitySrcs[i]);

        const size_t np = etl::min(poseSrcs.size(), poseSources.max_size());
        for (size_t i = 0; i < np; ++i)
            poseSources.push_back(poseSrcs[i]);
    }

    const estimate estimate;

    void set(Pose p) {
        fusedPose = p;
        modelObserver.set(p);
        for (auto& src : poseSources)
            src.observer->set(p);
    }

    void set(Velocity v) {
        fusedVelocity = v;
        for (auto& src : velocitySources) {
            if (src.observer->ready()) src.observer->set(v);
        }
    }

    void update(float dt) {
        for (auto& src : velocitySources)
            src.observer->update(dt);

        fusedVelocity = fuseVelocity();
        modelObserver.update(dt);
        fusedPose = modelObserver.estimate();

        if (!poseSources.empty()) {
            bool any_ready = false;
            for (auto& src : poseSources) {
                if (src.observer->ready()) {
                    src.observer->update(dt);
                    any_ready = true;
                }
            }
            if (any_ready) {
                fusedPose = fusePose(fusedPose);
                modelObserver.set(fusedPose);
            }
        }
    }

    private:

    Velocity getFusedVelocity() const {
        return fusedVelocity;
    }

    Velocity fuseVelocity() const {
        float vWeightTotal = 0.0f, omegaWeightTotal = 0.0f;
        float vTotal = 0.0f, omegaTotal = 0.0f;

        for (const auto& src : velocitySources) {
            if (!src.observer->ready()) continue;

            Velocity velocityEstimate = src.observer->estimate();
            const ObserverVTrust& t   = src.trust;

            vWeightTotal += t.vTrust;
            omegaWeightTotal += t.omegaTrust;
            vTotal += t.vTrust * velocityEstimate.v;
            omegaTotal += t.omegaTrust * velocityEstimate.omega;
        }

        return Velocity{(vWeightTotal <= 0.0f ? fusedVelocity.v : vTotal / vWeightTotal),
            (omegaWeightTotal <= 0.0f ? fusedVelocity.omega : omegaTotal / omegaWeightTotal)};
    }

    Pose fusePose(const Pose &dead_reckoned) const {
        float xWeightTotal = 0.0f, yWeightTotal = 0.0f, dthetaWeightTotal = 0.0f;
        float xTotal = 0.0f, yTotal = 0.0f, dthetaTotal = 0.0f;

        for (const auto& src : poseSources) {
            if (!src.observer->ready()) continue;

            Pose correction         = src.observer->estimate();
            const ObserverPTrust& t = src.trust;

            float dtheta = wrapAngle(correction.theta - dead_reckoned.theta);

            xWeightTotal += t.xTrust;
            yWeightTotal += t.yTrust;
            dthetaWeightTotal += t.thetaTrust;

            xTotal += t.xTrust * correction.x;
            yTotal += t.yTrust * correction.y;
            dthetaTotal += t.thetaTrust * dtheta;
        }

        return Pose{
            .x =
                (xWeightTotal <= 0.0f
                        ? dead_reckoned.x
                        : dead_reckoned.x +
                              positionCorrectionGain * (xTotal / xWeightTotal - dead_reckoned.x)),
            .y =
                (yWeightTotal <= 0.0f
                        ? dead_reckoned.y
                        : dead_reckoned.y +
                              positionCorrectionGain * (yTotal / yWeightTotal - dead_reckoned.y)),
            .theta = wrapAngle(
                dthetaWeightTotal <= 0.0f
                    ? dead_reckoned.theta
                    : dead_reckoned.theta + thetaCorrectionGain * dthetaTotal / dthetaWeightTotal
            ),
        };
    }

    etl::vector<VelocitySource, SENSOR_FUSION_MAX_VELOCITY_OBSERVERS> velocitySources;
    etl::vector<PoseSource, SENSOR_FUSION_MAX_POSE_OBSERVERS> poseSources;

    ModelObserver modelObserver;

    Velocity fusedVelocity;
    Pose fusedPose;
    float positionCorrectionGain;
    float thetaCorrectionGain;
};

#pragma GCC pop_options
