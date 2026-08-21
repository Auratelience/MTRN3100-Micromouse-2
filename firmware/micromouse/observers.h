// Observers
//
// Zimmy Levi z5587840

#pragma once

#include <Arduino.h>

#include <Embedded_Template_Library.h>
#include <etl/delegate.h>

#include "imu.h"
#include "constants.h"
#include "kinematics.h"
#include "lidar.h"
#include "motor.h"
#include "types.h"
#include "imu.h"
#include "lidar.h"

#pragma GCC push_options
#pragma GCC optimize("O2")

// Produces pose estimates
class ObserverP {
    public:

    // Gets the pose estimate
    virtual Pose estimate() = 0;

    // Sets the Observer's reference to a known Pose
    virtual void set(Pose p) = 0;

    // Updates the observer
    virtual void update(float dt) = 0;

    // Should return false if an estimate is not yet ready
    virtual bool ready() = 0;
};

// Produces velocity estimates
class ObserverV {
    public:

    // dt is in seconds
    virtual Velocity estimate()   = 0;
    virtual void set(Velocity v)  = 0;
    virtual void update(float dt) = 0;
    virtual bool ready()          = 0;
};

class WheelObserver : public ObserverV {
    public:

    WheelObserver(BaseMotor& left, BaseMotor& right, Kinematics& model) :
        left(left),
        right(right),
        model(model),
        left_prev_rad(left.angularDisplacement()),
        right_prev_rad(right.angularDisplacement()),
        velocity({0, 0}) {}

    Velocity estimate() override {
        return velocity;
    }

    // No-op since there is nothing to set.
    void set(Velocity v) override {}

    void update(float dt) override {
        float left_curr_rad  = left.angularDisplacement();
        float right_curr_rad = right.angularDisplacement();

        // Note that dt is assumed to be in seconds, not millis
        WheelVelocities vw = {
            (left_curr_rad - left_prev_rad) / dt, (right_curr_rad - right_prev_rad) / dt};

        left_prev_rad  = left_curr_rad;
        right_prev_rad = right_curr_rad;

        velocity = model.fk.velocity(vw);
    }

    // Can calculate whenever
    bool ready() override {
        return true;
    }

    // Re-seeds the deltas from where the wheels are now.
    //
    // The wizard is driven by turning the wheels by hand, so between this
    // object's construction and the first control tick they may have moved a
    // long way. Odometry integrates deltas, so the absolute counts do not
    // matter and there is nothing to zero -- only the marks need moving.
    void reset() {
        left_prev_rad  = left.angularDisplacement();
        right_prev_rad = right.angularDisplacement();
        velocity       = {0, 0};
    }

    private:

    BaseMotor& left;
    BaseMotor& right;

    Kinematics& model;

    float left_prev_rad;
    float right_prev_rad;

    Velocity velocity;
};

// Heading from the gyro, integrated on the sensor's own clock.
//
// A pose observer rather than a velocity one, and the distinction is the point
// of the class. A velocity observer hands SensorFusion an omega, which
// ModelObserver multiplies by the control loop's dt -- and those two describe
// different intervals the moment the loop jitters. The OLED framebuffer
// transfer is about 23 ms during which the gyro is not read at all, so a short
// counter-rotation landing inside one is never sampled and its angle is simply
// gone, while the same jerk landing just before one is held across the fat dt
// and counted several times over. Either way the estimate comes out short of a
// jerk, which is the failure this replaces.
//
// Reading the FIFO removes the coupling instead of tuning it. The sensor
// samples into its own 1024-byte buffer at a fixed rate straight through the
// stall, so the batch drained afterwards holds every sample, each covering
// exactly IMU::samplePeriod() seconds. Integrating is then a sum, and the loop
// dt never enters it -- update()'s parameter is deliberately unused.
//
// Coming in through the pose side is what keeps that exact. As a velocity
// observer this could report dtheta/dt and let ModelObserver cancel the dt
// back out, but fuseVelocity would average that against the wheel observer's
// omega first and only a fraction of the batch would survive. A pose
// observer's theta is weighted whole.
//
// Register with FusionWeights::ThetaPTrust. The x and y in the returned Pose
// are not estimates of anything.
class ImuObserver : public ObserverP {
    public:

    ImuObserver(IMU& imu) :
        imu(imu),
        theta(0),
        gyro_z_bias_raw(0),
        lost_batches(0),
        obsv_init(false) {}

    // Call after imu.init() succeeds. Measures the gyro's zero-rate output at
    // rest, in raw LSB, off the same FIFO stream update() integrates -- so the
    // number cancels sample for sample rather than approximately.
    //
    // Blocking, and the robot must be still for it. A hand on the chassis here
    // is written into the bias and comes back as drift for the rest of the run.
    void init() {
        delay(IMU_STARTUP_SETTLE_MS);
        imu.resetFifo(); // drop the settling transient

        int32_t sum  = 0;
        uint32_t n   = 0;
        const unsigned long until = millis() + IMU_CALIBRATION_MS;

        // Drained in pieces rather than once at the end: the window is free to
        // be as long as averaging wants only so long as no single wait comes
        // near the FIFO's depth.
        while (static_cast<long>(millis() - until) < 0) {
            delay(IMU_CALIBRATION_DRAIN_MS);
            const IMU::GyroBatch batch = imu.drainGyroZ();
            if (batch.lost) {
                // A hole in the window biases the mean by whatever was
                // happening when the samples went. Start the average over
                // rather than calibrate against it.
                sum = 0;
                n   = 0;
                continue;
            }
            sum += batch.sum;
            n += batch.count;
        }

        if (n == 0) return; // ready() stays false and setup() reports it

        gyro_z_bias_raw = static_cast<float>(sum) / static_cast<float>(n);
        obsv_init       = true;
    }

    // The dt is unused, which is the whole idea: the batch carries its own.
    void update(float) override {
        if (!obsv_init) return;

        const IMU::GyroBatch batch = imu.drainGyroZ();
        if (batch.lost) ++lost_batches;

        ++updates;
        samples_seen += batch.count;

        if (batch.count == 0) return; // nothing new this cycle; theta stands

        // Bias removed per sample, not per batch. The batch is a sum over count
        // samples, so it carries count times the zero-rate offset.
        const float dtheta =
            (static_cast<float>(batch.sum) -
                static_cast<float>(batch.count) * gyro_z_bias_raw) *
            imu.gyroRadPerLsb() * imu.samplePeriod();

        theta = wrapAngle(theta + dtheta);
    }

    // x and y are placeholders; ThetaPTrust weights them to nothing.
    Pose estimate() override {
        if (!obsv_init) return Pose{0, 0, 0};
        return Pose{0, 0, theta};
    }

    void set(Pose p) override {
        theta = wrapAngle(p.theta);
    }

    bool ready() override {
        return obsv_init;
    }

    // DIAGNOSTIC: batches thrown away because the FIFO overflowed or lost
    // alignment. Each one is heading that was measured and then dropped, so a
    // non-zero count here is the estimate silently going short.
    unsigned long lostBatches() const {
        return lost_batches;
    }

    // DIAGNOSTIC: mean FIFO samples per update(), and so a direct read on
    // whether CONTROL_LOOP_NOMINAL_HZ -- which IMU_SAMPLE_RATE_DIVIDER is
    // derived from -- matches the loop this is actually running in.
    //
    // Want a little over 1. Below 1 the sample rate is under the loop rate and
    // some cycles see an empty batch, so theta updates in steps of the sample
    // period; well above 1 the loop is slower than the constant claims and the
    // rate could come down. Neither costs accuracy -- the batch integrates
    // exactly either way -- so this is a tuning readout, not a fault light.
    float samplesPerUpdate() const {
        if (updates == 0) return 0.0f;
        return static_cast<float>(samples_seen) / static_cast<float>(updates);
    }

    private:

    IMU& imu;
    float theta;
    float gyro_z_bias_raw;
    unsigned long lost_batches;
    unsigned long samples_seen = 0;
    unsigned long updates      = 0;
    bool obsv_init;
};

class FrontLidarObserver : public ObserverP {
    public:

    FrontLidarObserver(LIDAR& lidar) : lidar(lidar), pose({0, 0, 0}) {}

    void update(float dt) override {
        lidar.update();
        pose.x = -lidar.getReading(LIDAR::Front);
    }

    void set(Pose p) override {
        pose = p;
    }

    Pose estimate() override {
        return pose;
    }

    bool ready() override {
        return true;
    }

    private:

    LIDAR& lidar;
    Pose pose;
};

// LLM-generated Code
// Pose from the lidar, by matching what the beams should read against what
// they do read.
//
// Every solve starts from a prior -- the dead-reckoned pose SensorFusion is
// about to correct -- casts the three beams into the map from there, and forms
// a residual per beam:
//
//     r = measured - expected
//
// Each residual is one equation in the pose error. To first order
//
//     r = J . delta,   J = [dd/dx, dd/dy, dd/dtheta]
//
// so three beams give three equations in three unknowns, solved as a damped
// least squares (Levenberg-Marquardt) step against the prior.
//
// Three beams are rarely three independent equations, and the prior is what
// makes that ordinary rather than special. A robot halfway down a corridor
// with its front beam past the sensor's 300 mm ceiling knows nothing about
// where it sits along that corridor; with the front beam square on a wall
// ahead, the two side beams see parallel walls and their Jacobian rows come
// out antiparallel, so y and theta are constrained only in combination. Each
// axis therefore carries one extra equation saying it should stay where dead
// reckoning put it, weighted by how far dead reckoning is trusted to be out --
// tightly in heading, loosely in position. Unobservable directions then sit on
// the prior, and observable ones are still free to move.
//
// The Jacobian comes out of the hit normal. The hit sits on a surface, so
// n . (s + d b - h) = 0 holds through any small change of pose;
// differentiating,
//
//     dd = -[n . ds + d (n . db)] / (n . b)
//
// with ds/dx = (1,0), ds/dy = (0,1), ds/dtheta = perp(s - p) -- the mount
// offset swinging about the robot centre -- and db/dtheta = perp(b). For a
// panel that reduces to the d tan(alpha) the derivation gives; stating it
// through the normal covers both obstacle types with one expression.
//
// Register it with FusionWeights::XYPTrust unless heading is wanted from the
// lidar. Where a beam is square on to a wall, rotating the robot changes its
// range only at second order -- 49 mm through 0.05 rad is 0.06 mm, below what
// the reading can even carry -- so in a corridor y and theta are constrained
// only in combination, and a lateral correction leaks about 0.01 rad of
// rotation into the estimate. That heading is worse than the gyro's, and
// XYPTrust drops it. Off square, where heading really is visible, DefaultPTrust
// recovers about half a 0.06 rad error per sample.
//
// Wiring: the prior comes from a delegate, the same indirection ModelObserver
// uses, because SensorFusion never hands its pose sources the estimate they
// are correcting. In the sketch:
//
//     MazeWallMap<MAZE_SIZE_MAX> wallMap(runner.mapper);    // discovered
//     LidarObserver<MazeWallMap<MAZE_SIZE_MAX>> lidar_obsv(lidar, wallMap);
//     // or, against the exported map:
//     // LidarObserver<Map<MAZE_OBSTACLE_COUNT>> lidar_obsv(lidar, MAZE_MAP);
//     const std::array<PoseSource, 1> obs_p = {{{&lidar_obsv, FusionWeights::XYPTrust}}};
//     SensorFusion sf(obs_v, obs_p);
//
//     Pose fusedPose() { return sf.estimate.pose(); }
//     // in setup(), once sf exists:
//     lidar_obsv.setPrior(decltype(lidar_obsv)::PoseFunc::create<fusedPose>());
//
// Left unwired it falls back to its own last estimate, which is only useful
// for bench testing a single solve.
// Templated on the map type, not on an obstacle count. Anything offering
// cast() and candidates() with Map's signatures will do, which is what lets
// the same observer localise against the exported map, or against
// MazeWallMap -- the walls the robot has discovered for itself -- where
// there is no exported map to have.
template <typename MapT>
class LidarObserver : public ObserverP {
    public:

    using PoseFunc = etl::delegate<Pose()>;

    LidarObserver(LIDAR& lidar, const MapT& map, PoseFunc prior = {}) :
        lidar(lidar),
        map(map),
        prior_func(prior),
        pose({0, 0, 0}),
        candidate_count(0),
        candidate_overflow(false),
        beams_used(0),
        last_sample_ms(0) {}

    void setPrior(PoseFunc p) {
        prior_func = p;
    }

    void set(Pose p) override {
        pose = p;
    }

    Pose estimate() override {
        return pose;
    }

    // Always ready. SensorFusion tests ready() before calling update(), so an
    // observer that reports itself unready is never updated again and can
    // never come back; a cycle with nothing to say returns the prior instead,
    // which fusePose() folds in as a zero correction.
    bool ready() override {
        return true;
    }

    // Beams that survived gating in the last solve. Zero means the estimate is
    // the prior, unchanged.
    uint8_t beams() const {
        return beams_used;
    }

    void update(float dt) override {
        const Pose prior = prior_func ? prior_func() : pose;
        pose             = prior;
        beams_used       = 0;

        // The VL6180X is free-running at LIDAR_CONTINUOUS_PERIOD_MS, so
        // re-reading it at loop rate spends I2C on repeats of the same value
        // and re-solves against them.
        const unsigned long now = millis();
        if (now - last_sample_ms < LIDAR_CONTINUOUS_PERIOD_MS) return;
        last_sample_ms = now;
        lidar.update();

        // Broad phase once per solve, around the prior: the iterations below
        // move the pose by millimetres, far less than the margin in
        // LIDAR_OBSERVER_SEARCH_RADIUS_MM.
        //
        // A full list may have been truncated, and a dropped obstacle could be
        // the nearest one, so that case gives up the broad phase and casts
        // against everything. Slower, never wrong.
        candidate_count = map.candidates(Vec2D{prior.x, prior.y}, LIDAR_OBSERVER_SEARCH_RADIUS_MM, candidate_idx);
        candidate_overflow = (candidate_count == LIDAR_OBSERVER_MAX_CANDIDATES);

        Pose p = prior;
        for (uint8_t iteration = 0; iteration < LIDAR_OBSERVER_ITERATIONS; ++iteration) {
            // Normal equations, H = sum J J^T and g = sum J r, accumulated
            // over the beams that pass gating.
            float H[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
            float g[3]    = {0, 0, 0};
            beams_used    = 0;

            for (uint8_t s = 0; s < LIDAR::COUNT; ++s) {
                float J[3];
                float r;
                if (!beamEquation(p, s, J, r)) continue;

                for (uint8_t i = 0; i < 3; ++i) {
                    g[i] += J[i] * r;
                    for (uint8_t j = 0; j < 3; ++j)
                        H[i][j] += J[i] * J[j];
                }
                ++beams_used;
            }

            if (beams_used == 0) {
                // Nothing to correct with. Hand back the prior so the fusion
                // sees a zero correction rather than a stale absolute pose.
                // Losing every beam on a later iteration means a step of a few
                // mm walked the solve off its own surfaces, so the whole solve
                // is abandoned rather than half-applied.
                pose = prior;
                return;
            }

            // Fold in the prior. Each axis gets one more equation, weighted by
            // 1/sigma^2, saying it should stay where dead reckoning put it --
            // which both makes H invertible whatever the beams did and settles
            // how a correction is split across a direction they cannot see.
            // The right hand side is the prior's own residual, zero on the
            // first iteration and non-zero once the pose has moved.
            const float w[3] = {
                1.0f / (LIDAR_OBSERVER_PRIOR_SIGMA_MM * LIDAR_OBSERVER_PRIOR_SIGMA_MM),
                1.0f / (LIDAR_OBSERVER_PRIOR_SIGMA_MM * LIDAR_OBSERVER_PRIOR_SIGMA_MM),
                1.0f / (LIDAR_OBSERVER_PRIOR_SIGMA_RAD * LIDAR_OBSERVER_PRIOR_SIGMA_RAD)
            };
            const float prior_residual[3] = {
                prior.x - p.x, prior.y - p.y, wrapAngle(prior.theta - p.theta)
            };

            for (uint8_t i = 0; i < 3; ++i) {
                H[i][i] = H[i][i] * (1.0f + LIDAR_OBSERVER_LAMBDA) + w[i];
                g[i] += w[i] * prior_residual[i];
            }

            float delta[3];
            if (!solve3(H, g, delta)) break;

            delta[0] = clampAbs(delta[0], LIDAR_OBSERVER_MAX_STEP_MM);
            delta[1] = clampAbs(delta[1], LIDAR_OBSERVER_MAX_STEP_MM);
            delta[2] = clampAbs(delta[2], LIDAR_OBSERVER_MAX_STEP_RAD);

            p.x += delta[0];
            p.y += delta[1];
            p.theta = wrapAngle(p.theta + delta[2]);

            if (fabsf(delta[0]) < LIDAR_OBSERVER_STEP_TOL_MM &&
                fabsf(delta[1]) < LIDAR_OBSERVER_STEP_TOL_MM &&
                fabsf(delta[2]) < LIDAR_OBSERVER_STEP_TOL_RAD)
                break;
        }

        pose = p;
    }

    private:

    struct Mount {
        float x;
        float y;
        float theta;
    };

    // Indexed by LIDAR::Sensors.
    static constexpr Mount MOUNTS[LIDAR::COUNT] = {
        {LIDAR_MOUNT_FRONT_X, LIDAR_MOUNT_FRONT_Y, LIDAR_MOUNT_FRONT_THETA},
        {LIDAR_MOUNT_LEFT_X, LIDAR_MOUNT_LEFT_Y, LIDAR_MOUNT_LEFT_THETA},
        {LIDAR_MOUNT_RIGHT_X, LIDAR_MOUNT_RIGHT_Y, LIDAR_MOUNT_RIGHT_THETA}
    };

    // One beam's contribution: its Jacobian row and residual, or false if the
    // reading carries nothing worth folding in.
    bool beamEquation(const Pose& p, uint8_t sensor, float J[3], float& r) const {
        const float measured =
            static_cast<float>(lidar.getReading(static_cast<LIDAR::Sensors>(sensor)));

        // Both clamps mean "no target", not "target at 0 mm" or "at 300 mm" --
        // see LidarSensor::read(). A saturated beam only says the wall is
        // somewhere beyond the ceiling, which is not an equation.
        if (measured <= static_cast<float>(L::MIN_DIST) ||
            measured >= static_cast<float>(L::MAX_DIST))
            return false;

        const Mount& mount = MOUNTS[sensor];
        const float c      = trig::xcos(p.theta);
        const float s      = trig::xsin(p.theta);

        // Mount offset in world axes, kept separate because it doubles as
        // ds/dtheta once rotated a further quarter turn.
        const Vec2D offset  = {mount.x * c - mount.y * s, mount.x * s + mount.y * c};
        const Vec2D origin  = Vec2D{p.x, p.y} + offset;
        const float heading = p.theta + mount.theta;
        const Vec2D beam    = {trig::xcos(heading), trig::xsin(heading)};

        // A little past the sensor's ceiling, so a prediction that lands just
        // beyond it is still available to be rejected by the residual gate
        // rather than silently becoming a miss.
        const RayHit hit = map.cast(
            origin,
            beam,
            static_cast<float>(L::MAX_DIST) + LIDAR_OBSERVER_MAX_RESIDUAL_MM,
            candidate_overflow ? nullptr : candidate_idx.data(),
            candidate_count
        );
        if (!hit.valid) return false;

        const float nb = dot(hit.normal, beam); // negative: the beam runs into the face
        if (-nb < LIDAR_OBSERVER_MIN_INCIDENCE_COS) return false; // grazing

        // Association. A residual this large is a beam looking at something
        // the map does not hold, or at a different surface entirely, and
        // folding it in would drag the pose rather than fix it.
        //
        // Obstacle::impliedHeadingError() is the derivation's phi inverse and
        // looks like the sharper test, but it is not one to gate on: it asks
        // how far the beam would have to swing to explain the reading, and a
        // beam pointed square at a wall barely changes range when it swings
        // (dd/dphi = d tan(alpha), which is zero at alpha = 0). A 6 mm
        // translation error therefore implies half a radian of rotation, and
        // gating on it throws away precisely the beams that say the most about
        // position. The residual and the incidence cover association on their
        // own; the pose error is what the least squares below is for.
        r = measured - hit.distance;
        if (fabsf(r) > LIDAR_OBSERVER_MAX_RESIDUAL_MM) return false;

        J[0] = -hit.normal.x / nb;
        J[1] = -hit.normal.y / nb;
        J[2] = -(dot(hit.normal, perp(offset)) + hit.distance * dot(hit.normal, perp(beam))) / nb;
        return true;
    }

    // H x = g, by the adjugate. Damping has already put a floor under every
    // diagonal, including the rows no beam constrains, so the determinant only
    // vanishes here if H is degenerate for some other reason.
    static bool solve3(const float H[3][3], const float g[3], float x[3]) {
        const float adj[3][3] = {
            {H[1][1] * H[2][2] - H[1][2] * H[2][1],
             H[0][2] * H[2][1] - H[0][1] * H[2][2],
             H[0][1] * H[1][2] - H[0][2] * H[1][1]},
            {H[1][2] * H[2][0] - H[1][0] * H[2][2],
             H[0][0] * H[2][2] - H[0][2] * H[2][0],
             H[0][2] * H[1][0] - H[0][0] * H[1][2]},
            {H[1][0] * H[2][1] - H[1][1] * H[2][0],
             H[0][1] * H[2][0] - H[0][0] * H[2][1],
             H[0][0] * H[1][1] - H[0][1] * H[1][0]}
        };

        const float det = H[0][0] * adj[0][0] + H[0][1] * adj[1][0] + H[0][2] * adj[2][0];
        if (fabsf(det) < STD_TOL) return false;

        const float inv = 1.0f / det;
        for (uint8_t i = 0; i < 3; ++i)
            x[i] = inv * (adj[i][0] * g[0] + adj[i][1] * g[1] + adj[i][2] * g[2]);
        return true;
    }

    static float clampAbs(float v, float limit) {
        if (v > limit) return limit;
        if (v < -limit) return -limit;
        return v;
    }

    LIDAR& lidar;
    const MapT& map;
    PoseFunc prior_func;
    Pose pose;

    std::array<uint16_t, LIDAR_OBSERVER_MAX_CANDIDATES> candidate_idx;
    size_t candidate_count;
    bool candidate_overflow;
    uint8_t beams_used;
    unsigned long last_sample_ms;
};

template <typename MapT>
constexpr typename LidarObserver<MapT>::Mount LidarObserver<MapT>::MOUNTS[LIDAR::COUNT];

class ModelObserver : public ObserverP {
    public:

    using VelocityFunc = etl::delegate<Velocity()>;
    using PoseFunc     = etl::delegate<Pose()>;

    ModelObserver(VelocityFunc velocity_func, PoseFunc pose_func = {}) :
        velocity_func(velocity_func),
        pose_func(pose_func),
        previous_pose({0, 0, 0}),
        pose({0, 0, 0}) {}

    // Note that dt is assumed to be in seconds.
    Pose estimate() override {
        return pose;
    }

    void set(Pose p) override {
        pose = p;
    }

    void update(float dt) override {
        if (!pose_func) {
            previous_pose = pose;
        } else {
            previous_pose = pose_func();
        }
        Velocity current_velocity = velocity_func();
        pose = {
            .x = previous_pose.x +
                 current_velocity.v *
                 Trig<TRIG_LUT_SIZE>::xcos(previous_pose.theta) * dt,
            .y = previous_pose.y +
                 current_velocity.v *
                 Trig<TRIG_LUT_SIZE>::xsin(previous_pose.theta) * dt,
            .theta = previous_pose.theta + current_velocity.omega * dt
        };
    }

    bool ready() override {
        return true;
    }

    void setFunctionPointers(VelocityFunc vf, PoseFunc pf) {
        velocity_func = vf;
        pose_func     = pf;
    }

    private:

    Pose previous_pose;
    Pose pose;
    VelocityFunc velocity_func;
    PoseFunc pose_func;
};

#pragma GCC pop_options
