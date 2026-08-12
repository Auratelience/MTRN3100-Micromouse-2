// Planners
//
// Zimmy Levi z5587840

#pragma once

#include <math.h>
#include <array>

#include <Embedded_Template_Library.h>
#include <etl/string.h>

#include "constants.h"
#include "types.h"

#pragma GCC push_options
#pragma GCC optimize("O2")

class MotionPlanner {
    public:

    enum class State { Run, Wait };

    MotionPlanner(
        float KPHeading, float KPLateral, float cruiseVelocity = MAXIMUM_FORWARD_VELOCITY
    ) :
        KPHeading(KPHeading),
        KPLateral(KPLateral),
        cruiseVelocity(cruiseVelocity),
        state(State::Wait) {}

    bool appendSegment(const Segment& s) {
        if (pathLen >= PATH_SEGMENTS_MAX_LEN) return false;
        path[pathLen] = s;
        ++pathLen;
        state = State::Run;
        return true;
    }

    float progress(const Pose& pose) const {
        if (pathIdx >= pathLen) {
            return 1.0f;
        }

        Vec2D pos(pose.x, pose.y);
        return path[pathIdx].progress(pos);
    }

    Velocity update(const Pose& pose, float dt) {
        if (state == State::Wait) return wait();
        else if (state == State::Run) return run(pose, dt);
        else return wait();
    }

    State s() const {
        return state;
    }

    uint16_t idx() const {
        return pathIdx;
    }

    private:

    Velocity wait() const {
        return {0, 0};
    }

    Velocity run(const Pose& pose, float dt) {
        if (pathIdx >= pathLen) {
            state = State::Wait;
            return {0, 0};
        }

        Vec2D pos(pose.x, pose.y);

        while (pathIdx < pathLen && path[pathIdx].progress(pos) >= SEGMENT_ADVANCE_THRESHOLD) {
            ++pathIdx;
        }

        // Finishing the last segment lands pathIdx on pathLen, and path[pathLen]
        // is a default Segment: zero length, so omega()'s lateralError divides
        // 0 by 0 and commands NaN at cruiseVelocity for one tick before the
        // check above catches it on the next call.
        if (pathIdx >= pathLen) {
            state = State::Wait;
            return {0, 0};
        }

        const Segment& s = path[pathIdx];
        float v          = cruiseVelocity;
        float o          = omega(s, pose, v);
        return {v, o};
    }

    float KPHeading;
    float KPLateral;
    float cruiseVelocity;
    std::array<Segment, PATH_SEGMENTS_MAX_LEN> path{};
    int pathLen = 0;
    int pathIdx = 0;
    State state;

    float omega(const Segment& s, const Pose& p, float v) const {
        Vec2D pos(p.x, p.y);

        if (s.curvature <= STRAIGHT_TOLERANCE) {
            float lineAngle    = arg(s.end - s.start);
            float headingError = wrapAngle(lineAngle - p.theta);

            Vec2D line = s.end - s.start;
            Vec2D perpRight(line.y, -line.x);
            float lateralError = dot(pos - s.start, perpRight) / dist(line);

            return KPHeading * headingError + KPLateral * lateralError;
        }

        float dirScalar    = s.direction == Segment::Direction::Left ? 1.0f : -1.0f;
        Vec2D c            = s.c;
        Vec2D nearestPoint = s.lateralPoint(pos);
        Vec2D radius       = nearestPoint - c;
        float tangentAngle = arg(radius) + dirScalar * PI_TWO;
        float headingError = wrapAngle(tangentAngle - p.theta);

        float lateralError = dirScalar * s.lateralDistance(pos);

        return dirScalar * s.curvature * v + KPHeading * headingError + KPLateral * lateralError;
    }
};

class PosePlanner {
    public:

    PosePlanner(
        float KPLinear,
        float KPAngular,
        float positionTolerance = STD_DIST_TOL,
        float angleTolerance    = STD_ANG_TOL
    ) :
        KPLinear(KPLinear),
        KPAngular(KPAngular),
        positionTolerance(positionTolerance),
        angleTolerance(angleTolerance),
        state(State::Done) {}

    void setTarget(const Pose& t) {
        target = t;
        // Heading-only skips Seek and rotates in place to target.theta. Used
        // for pure rotations (turns), where the target position is the current
        // cell: seeking a point the robot has coasted past yields a near-180
        // deg heading error and a runaway spin instead of a clean turn.
        state = headingOnly ? State::Align : State::Seek;
    }

    // When enabled, the next setTarget() ignores the target position and only
    // rotates to target.theta. Toggle per segment (rotation vs translation).
    void setHeadingOnly(bool enabled) {
        headingOnly = enabled;
    }

    bool done() const {
        return state == State::Done;
    }

    Velocity update(const Pose& current, float dt) {
        switch (state) {
            case State::Seek:  return seek(current);
            case State::Align: return align(current);
            default:           return Velocity{0, 0};
        }
    }

    private:

    enum class State { Seek, Align, Done };

    float KPLinear;
    float KPAngular;
    float positionTolerance;
    float angleTolerance;
    Pose target;
    State state;
    bool headingOnly = false;

    Velocity seek(const Pose& current) {
        float dx       = target.x - current.x;
        float dy       = target.y - current.y;
        float distance = sqrtf(dx * dx + dy * dy);

        if (distance < positionTolerance) {
            state = State::Align;
            return align(current);
        }

        float angleToTarget = arg(Vec2D(dx, dy));
        float headingError  = wrapAngle(angleToTarget - current.theta);

        float v = KPLinear * distance;
        if (v > MAXIMUM_FORWARD_VELOCITY) v = MAXIMUM_FORWARD_VELOCITY;
        return Velocity{v, KPAngular * headingError};
    }

    Velocity align(const Pose& current) {
        float headingError = wrapAngle(target.theta - current.theta);

        if (fabsf(headingError) < angleTolerance) {
            state = State::Done;
            return Velocity{0, 0};
        }

        return Velocity{0, KPAngular * headingError};
    }
};

// POSE SEQUENCE PLANNER
class PSPlanner {
    public:

    enum Direction : int { North = 0, West = 1, South = 2, East = -1 };
    enum class Instruction { Forwards, Left, Right };

    struct GridPose {
        int x;
        int y;
        Direction direction;
    };

    PSPlanner(float KPLinear, float KPAngular) : pp(KPLinear, KPAngular) {}

    void setStart(GridPose g) {
        instructions[pathLen] = g;
        ++pathLen;
    }

    bool addInstructions(etl::string<MAZE_INSTRUCTION_MAX_LEN> instructions) {
        bool ok = true;
        for (const auto& c : instructions) {
            switch (c) {
                case 'f': ok = addInstruction(Instruction::Forwards); break;
                case 'r': ok = addInstruction(Instruction::Right); break;
                case 'l': ok = addInstruction(Instruction::Left); break;
            }
            if (!ok) return false;
        }
        return true;
    }

    bool addInstruction(Instruction i) {
        GridPose curr = instructions[pathLen - 1];
        GridPose next;
        switch (i) {
            case Instruction::Forwards: next = forwards(curr); break;
            case Instruction::Right:    next = right(curr); break;
            case Instruction::Left:     next = left(curr); break;
        }
        return appendGridPose(next);
    }

    Velocity update(const Pose& pose, float dt) {
        if (pathIdx >= pathLen) {
            return {0, 0};
        }
        Vec2D pos(pose.x, pose.y);

        if (pp.done()) {
            ++pathIdx;
            if (pathIdx >= pathLen) {
                return {0, 0};
            }
            const GridPose& prev = instructions[pathIdx - 1];
            const GridPose& curr = instructions[pathIdx];
            // A pure rotation keeps the same cell; only the heading changes.
            // Drive it as a heading-only move so the planner rotates in place
            // rather than seeking a point the robot may have coasted past.
            bool isTurn = (curr.x == prev.x) && (curr.y == prev.y);
            pp.setHeadingOnly(isTurn);
            pp.setTarget(gridToWorld(curr));
        }

        return pp.update(pose, dt);
    }

    static GridPose forwards(const GridPose& curr) {
        switch (curr.direction) {
            case North: return {curr.x + 1, curr.y, North};
            case East:  return {curr.x, curr.y - 1, East};
            case South: return {curr.x - 1, curr.y, South};
            case West:  return {curr.x, curr.y + 1, West};
            default:    return curr;
        }
    }

    static GridPose left(const GridPose& curr) {
        switch (curr.direction) {
            case North: return {curr.x, curr.y, West};
            case East:  return {curr.x, curr.y, North};
            case South: return {curr.x, curr.y, East};
            case West:  return {curr.x, curr.y, South};
            default:    return curr;
        }
    }

    static GridPose right(const GridPose& curr) {
        switch (curr.direction) {
            case North: return {curr.x, curr.y, East};
            case East:  return {curr.x, curr.y, South};
            case South: return {curr.x, curr.y, West};
            case West:  return {curr.x, curr.y, North};
            default:    return curr;
        }
    }

    private:

    bool appendGridPose(const GridPose& g) {
        if (pathLen >= MAZE_INSTRUCTION_MAX_LEN) return false;
        instructions[pathLen] = g;
        ++pathLen;
        return true;
    }

    static inline Vec2D gridToWorld(const Vec2D& g) {
        return MAZE_CELL_SIZE * g;
    }

    static inline Pose gridToWorld(const GridPose& g) {
        return {g.x * MAZE_CELL_SIZE, g.y * MAZE_CELL_SIZE, wrapAngle(g.direction * PI_TWO)};
    }

    static inline Direction thetaToDirection(float theta) {
        int d = roundf(theta / PI_TWO);
        switch (d) {
            case 0:  return North;
            case 1:  return West;
            case 2:  return South;
            case -1: return East;
            case -2: return North; // -π
            default: return North;
        }
    }

    static inline GridPose worldToGrid(const Pose& p) {
        return {static_cast<int>(roundf(p.x / MAZE_CELL_SIZE)),
            static_cast<int>(roundf(p.y / MAZE_CELL_SIZE)),
            thetaToDirection(p.theta)};
    }

    PosePlanner pp;
    std::array<GridPose, MAZE_INSTRUCTION_MAX_LEN> instructions{};
    int pathLen = 0;
    int pathIdx = 0;
};

class HeadingPlanner {
    public:

    HeadingPlanner(float KPAngular, float angleTolerance = STD_ANG_TOL) :
        KPAngular(KPAngular),
        angleTolerance(angleTolerance),
        targetTheta(0) {}

    void setTarget(float theta) {
        targetTheta = theta;
    }

    Velocity update(const Pose& current, float dt) {
        float headingError = wrapAngle(targetTheta - current.theta);

        if (fabsf(headingError) < angleTolerance) {
            return Velocity{0, 0};
        }

        return Velocity{0, KPAngular * headingError};
    }

    private:

    float KPAngular;
    float angleTolerance;
    float targetTheta;
};

class DistancePlanner {
    public:

    DistancePlanner(float KpDistance, float KpHeading) :
        KpDistance(KpDistance),
        KpHeading(KpHeading),
        targetDistance(0) {}

    void setTarget(float distance) {
        targetDistance = distance;
    }

    Velocity update(const Pose& current, float dt) {
        float current_distance = -current.x;
        float distance_error   = current_distance - targetDistance;

        float forward_velocity = KpDistance * distance_error;
        if (forward_velocity > MAXIMUM_FORWARD_VELOCITY)
            forward_velocity = MAXIMUM_FORWARD_VELOCITY;
        if (forward_velocity < -MAXIMUM_FORWARD_VELOCITY)
            forward_velocity = -MAXIMUM_FORWARD_VELOCITY;

        float heading_error = wrapAngle(0.0f - current.theta);
        float omega         = KpHeading * heading_error;

        return Velocity{forward_velocity, omega};
    }

    private:

    float KpDistance;
    float KpHeading;
    float targetDistance;
};

#pragma GCC pop_options
