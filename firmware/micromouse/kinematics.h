// Kinematics
//
// Zimmy Levi z5587840

#pragma once

#include "constants.h"
#include "types.h"
#include <algorithm>

#pragma GCC push_options
#pragma GCC optimize("O2")

class Kinematics {
    public:

    Kinematics(float radius, float length) : r(radius), l(length), ik(*this), fk(*this) {}

    class FK {
        public:

        FK(const Kinematics& k) : k(k) {}

        Velocity velocity(const WheelVelocities& wv) const {
            return Velocity((k.r / 2) * (wv.left + wv.right), (k.r / k.l) * (wv.right - wv.left));
        }

        private:

        const Kinematics& k;
    };

    class IK {
        public:

        IK(const Kinematics& k) : k(k) {}

        WheelVelocities velocityRaw(const Velocity& v) const {
            return {(1.0f / k.r) * (v.v - v.omega * k.l / 2.0f),
                (1.0f / k.r) * (v.v + v.omega * k.l / 2.0f)};
        }

        WheelVelocities velocity(const Velocity& v) const {
            WheelVelocities wv = velocityRaw(v);
            float exceeder = std::max(fabsf(wv.left), fabsf(wv.right));
            if (exceeder > MAXIMUM_WHEEL_ANGULAR_VELOCITY) {
                float scale_factor = MAXIMUM_WHEEL_ANGULAR_VELOCITY / exceeder;
                wv.left *= scale_factor;
                wv.right *= scale_factor;
            }

            return wv;
        }

        private:

        const Kinematics& k;
    };

    const IK ik;
    const FK fk;

    private:

    float r;
    float l;
};

#pragma GCC pop_options
