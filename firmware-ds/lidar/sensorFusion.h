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
    constexpr float PoseCorrectionGain = 0.2f;

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
        float poseCorrectionGain             = FusionWeights::PoseCorrectionGain
    ) :
        estimate(*this),
        modelObserver(
            ModelObserver::VelocityFunc::create<SensorFusion, &SensorFusion::getFusedVelocity>(
                *this
            )
        ),
        fusedVelocity({0, 0}),
        fusedPose({0, 0, 0}),
        poseCorrectionGain(poseCorrectionGain) {
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
        // initialised at 1.0f to represent the Model_Observer's trust.
        float xWeightTotal = 1.0f, yWeightTotal = 1.0f, dthetaWeightTotal = 1.0f;
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

        // Coefficient of how much observers affect dead reckoning

        return Pose{
            .x =
                (xWeightTotal <= 0.0f
                        ? dead_reckoned.x
                        : dead_reckoned.x +
                              poseCorrectionGain * (xTotal / xWeightTotal - dead_reckoned.x)),
            .y =
                (yWeightTotal <= 0.0f
                        ? dead_reckoned.y
                        : dead_reckoned.y +
                              poseCorrectionGain * (yTotal / yWeightTotal - dead_reckoned.y)),
            .theta = wrapAngle(
                dthetaWeightTotal <= 0.0f
                    ? dead_reckoned.theta
                    : dead_reckoned.theta + poseCorrectionGain * dthetaTotal / dthetaWeightTotal
            ),
        };
    }

    etl::vector<VelocitySource, SENSOR_FUSION_MAX_VELOCITY_OBSERVERS> velocitySources;
    etl::vector<PoseSource, SENSOR_FUSION_MAX_POSE_OBSERVERS> poseSources;

    ModelObserver modelObserver;

    Velocity fusedVelocity;
    Pose fusedPose;
    float poseCorrectionGain;
};

#pragma GCC pop_options
