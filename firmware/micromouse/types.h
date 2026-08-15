// Types
//
// Zimmy Levi z5587840

#pragma once

#include <math.h>
#include <array>
#include <variant>

#include "constants.h"

#pragma GCC push_options
#pragma GCC optimize("O2")

template <size_t NV>
class Trig {
    static_assert(NV > 1, "NV > 1 required.");

    public:

    // Only for v in [-1, 1]
    static float acos(float v) {
        if (v <= -1.0f) return PI;
        if (v >= 1.0f) return 0;

        bool neg = false;
        if (v < 0) {
            neg = true;
            v   = -v;
        }

        // get value from LUT
        size_t idx = static_cast<size_t>(v * NV);
        if (idx >= NV) return acosLUT[NV];
        float t = (v * NV) - idx;

        float res = acosLUT[idx] + t * (acosLUT[idx + 1] - acosLUT[idx]);
        return (neg ? PI - res : res);
    }

    static float atan2(float y, float x) {
        if (x == 0.0f) {
            if (y > 0.0f) return PI_TWO;
            if (y < 0.0f) return -PI_TWO;
            return 0.0f; // Undefined (0,0)
        }

        // Reduce inputs to absolute octant space
        float abs_x = (x < 0.0f) ? -x : x;
        float abs_y = (y < 0.0f) ? -y : y;

        float v;
        bool invert = false;
        if (abs_y > abs_x) {
            v      = abs_x / abs_y;
            invert = true;
        } else {
            v = abs_y / abs_x;
        }

        // interpolate
        float float_idx = v * static_cast<float>(NV);
        size_t idx      = static_cast<size_t>(float_idx);
        if (idx >= NV) idx = NV - 1;

        float t     = float_idx - static_cast<float>(idx);
        float angle = atanLUT[idx] + t * (atanLUT[idx + 1] - atanLUT[idx]);

        if (invert) angle = PI_TWO - angle;
        if (x < 0.0f) angle = PI - angle;
        if (y < 0.0f) angle = -angle;

        return angle;
    }

    // LUT-based sin/cos. Both share a single quarter-wave table (sinLUT,
    // covering [0, PI/2]) and recover the full circle via quadrant folding and
    // the reflection identities sin(PI/2 + r) = sin(PI/2 - r) and
    // cos(x) = sin(x + PI/2). Contract: x in [0, 2PI). For arbitrary x use
    // xsin/xcos, which fmodf-wrap into range first.
    static float sin(float x) {
        size_t q = static_cast<size_t>(x * inv_half_pi);
        float r  = x - static_cast<float>(q) * PI_TWO;
        switch (q & 3u) { // & 3 absorbs x == 2PI and tiny overshoot
            case 0:  return quarterSin(r);
            case 1:  return quarterSin(PI_TWO - r);
            case 2:  return -quarterSin(r);
            default: return -quarterSin(PI_TWO - r); // case 3
        }
    }

    static float cos(float x) {
        size_t q = static_cast<size_t>(x * inv_half_pi);
        float r  = x - static_cast<float>(q) * PI_TWO;
        switch (q & 3u) {
            case 0:  return quarterSin(PI_TWO - r);
            case 1:  return -quarterSin(r);
            case 2:  return -quarterSin(PI_TWO - r);
            default: return quarterSin(r); // case 3
        }
    }

    // Extended-range variants: accept any finite x, wrapping into [0, 2PI).
    static float xsin(float x) {
        float r = fmodf(x, TWO_PI);
        if (r < 0.0f) r += TWO_PI;
        return sin(r);
    }

    static float xcos(float x) {
        float r = fmodf(x, TWO_PI);
        if (r < 0.0f) r += TWO_PI;
        return cos(r);
    }

    // Full accuracy for floats
    // Slower than LUT-based
    class ha {
        public:

        static constexpr float cos(float x) {
            float term = 1.0f, sum = 1.0f, x2 = x * x;
            for (int i = 1; i <= sinCosIters; ++i) {
                term *= -x2 / static_cast<float>((2 * i - 1) * (2 * i));
                sum += term;
            }
            return sum;
        }

        static constexpr float sin(float x) {
            float term = x, sum = x, x2 = x * x;
            for (int i = 1; i <= sinCosIters; ++i) {
                term *= -x2 / static_cast<float>((2 * i) * (2 * i + 1));
                sum += term;
            }
            return sum;
        }

        static constexpr float acos(float x) {
            if (x >= 1.0f) return 0.0f;
            if (x <= -1.0f) return PI;
            bool negative = (x < 0.0f);
            float abs_x   = negative ? -x : x;
            float theta   = PI_TWO * (1.0f - abs_x);
            for (int i = 0; i < acosIters; ++i) {
                float f       = cos(theta) - abs_x;
                float f_prime = -sin(theta);
                if (f_prime > -STD_TOL && f_prime <= 0.0f) f_prime = -STD_TOL;
                if (f_prime < STD_TOL && f_prime >= 0.0f) f_prime = STD_TOL;
                theta = theta - (f / f_prime);
            }
            return negative ? (PI - theta) : theta;
        }

        // ONLY FOR 0 to 1!
        static constexpr float atan(float x) {
            float sum       = 0.0f;
            float current_x = x;
            float x2        = x * x;
            for (int i = 0; i < atanIters; ++i) {
                float denominator = static_cast<float>(2 * i + 1);
                float term        = current_x / denominator;
                if (i % 2 == 1) sum -= term;
                else sum += term;
                current_x *= x2;
            }
            return sum;
        }

        private:

        constexpr static uint8_t acosIters   = 5;
        constexpr static uint8_t sinCosIters = 10;
        constexpr static uint8_t atanIters   = 150;
    };

    private:

    static constexpr float acos_interval = 1.0f / static_cast<float>(NV);

    constexpr static std::array<float, NV + 1> genAcosLUT() {
        std::array<float, NV + 1> array{};
        for (size_t i = 0; i < NV; ++i) {
            array[i] = ha::acos(i * acos_interval);
        }
        array[NV] = 0;
        return array;
    }

    constexpr static std::array<float, NV + 1> acosLUT = genAcosLUT();

    static constexpr float atan_interval = 1.0f / static_cast<float>(NV);

    static constexpr std::array<float, NV + 1> genAtanLUT() {
        std::array<float, NV + 1> array{};
        for (size_t i = 0; i <= NV; ++i) {
            array[i] = ha::atan(static_cast<float>(i) * atan_interval);
        }
        return array;
    }

    constexpr static std::array<float, NV + 1> atanLUT = genAtanLUT();

    // Quarter-wave sine table over [0, PI/2], shared by sin/cos/xsin/xcos.
    static constexpr float sin_interval    = PI_TWO / static_cast<float>(NV);
    static constexpr float sin_index_scale = static_cast<float>(NV) / PI_TWO;
    static constexpr float inv_half_pi     = 1.0f / PI_TWO;

    static constexpr std::array<float, NV + 1> genSinLUT() {
        std::array<float, NV + 1> array{};
        for (size_t i = 0; i < NV; ++i) {
            array[i] = ha::sin(static_cast<float>(i) * sin_interval);
        }
        array[NV] = 1.0f; // sin(PI/2)
        return array;
    }

    static constexpr std::array<float, NV + 1> sinLUT = genSinLUT();

    // Interpolate the quarter-wave table for a in [0, PI/2].
    static float quarterSin(float a) {
        float fidx = a * sin_index_scale;
        if (fidx < 0.0f) fidx = 0.0f; // guard float rounding of PI_TWO - r
        size_t idx = static_cast<size_t>(fidx);
        if (idx >= NV) idx = NV - 1;
        float t = fidx - static_cast<float>(idx);
        return sinLUT[idx] + t * (sinLUT[idx + 1] - sinLUT[idx]);
    }
};

using trig = Trig<TRIG_LUT_SIZE>;

struct WheelVelocities {
    float left;
    float right;
};

struct Pose {
    // mm
    float x;
    // mm
    float y;
    // rad
    float theta;
};

struct Vec2D {
    enum Quadrant : uint8_t { First = 1, Second, Third, Fourth };

    float x;
    float y;

    // constexpr so a Map can be built at compile time and live in flash
    constexpr Vec2D(float x, float y) : x(x), y(y) {}

    constexpr Vec2D() : x(0), y(0) {}

    // default constructors
    Vec2D(const Vec2D&)            = default;
    Vec2D& operator=(const Vec2D&) = default;
};

// Linter hidden as this is a header-only library
// NOLINTBEGIN(misc-definitions-in-headers)

inline Vec2D operator*(const float a, const Vec2D& b) {
    return Vec2D{a * b.x, a * b.y};
}

inline Vec2D operator+(const Vec2D& a, const Vec2D& b) {
    return Vec2D{a.x + b.x, a.y + b.y};
}

inline Vec2D operator-(const Vec2D& v) {
    return Vec2D{-v.x, -v.y};
}

inline Vec2D operator-(const Vec2D& a, const Vec2D& b) {
    return Vec2D(a.x - b.x, a.y - b.y);
}

inline void operator+=(Vec2D& a, const Vec2D& b) {
    a.x += b.x;
    a.y += b.y;
}

inline float dist(const Vec2D& v) {
    return sqrtf(v.x * v.x + v.y * v.y);
}

inline float distSq(const Vec2D& v) {
    return v.x * v.x + v.y * v.y;
}

inline float arg(const Vec2D& v) {
    return Trig<TRIG_LUT_SIZE>::atan2(v.y, v.x);
}

inline Vec2D::Quadrant quadrant(const Vec2D& v) {
    if (v.x >= 0 && v.y >= 0) return Vec2D::Quadrant::First;
    else if (v.x < 0 && v.y >= 0) return Vec2D::Quadrant::Second;
    else if (v.x > 0 && v.y < 0) return Vec2D::Quadrant::Third;
    else return Vec2D::Quadrant::Fourth;
}

// Wraps an angle to [-PI, PI].
// Inf or NaN returns 0 rather than poisoning downstream control maths.
inline float wrapAngle(float a) {
    if (!isfinite(a)) return 0.0f;
    a = fmodf(a, TWO_PI);
    if (a > PI) {
        a -= TWO_PI;
    } else if (a < -PI) {
        a += TWO_PI;
    }
    return a;
}

inline float dot(const Vec2D& a, const Vec2D& b) {
    return a.x * b.x + a.y * b.y;
}

// Rotated a quarter turn CCW, which is also d/dtheta of a vector rigidly
// rotating by theta. The lidar observer's heading Jacobian is built from it.
inline Vec2D perp(const Vec2D& v) {
    return Vec2D{-v.y, v.x};
}

inline float cross(const Vec2D& a, const Vec2D& b) {
    return (a.x * b.y) - (a.y * b.x);
}

inline float angleBetween(const Vec2D& a, const Vec2D& b) {
    return Trig<TRIG_LUT_SIZE>::atan2(cross(a, b), dot(a, b));
}

inline bool operator==(const Vec2D& a, const Vec2D& b) {
    return (a.x == b.x) && (a.y == b.y);
}

bool withinTolerance(const Vec2D& a, const Vec2D& b, float tol = 0.0f) {
    return distSq(a - b) <= tol * tol;
}

struct Velocity {

    Velocity(float v, float omega) : v(v), omega(omega) {}

    float v;
    float omega;

    Vec2D vec() const;
};

Velocity operator*(const float a, const Velocity& b) {
    return Velocity(a * b.v, b.omega);
}

// NOLINTEND(misc-definitions-in-headers)

// GRID DIRECTIONS
//
// The one grid-heading convention for the firmware, shared by PSPlanner and
// MazeMapper. Anything that talks about cells talks in these terms, so a route
// produced by the mapper can be handed to the planner without a translation
// layer in between -- two conventions is one too many, and the bug it hides is
// a silent mirror-image path.
//
// Counter-clockwise, and numbered so that theta = d * PI_TWO is the world
// heading directly. That identity is why East is -1 rather than 3: it keeps
// the result inside [-PI, PI] without a wrap.
//
// Axes are the robot frame, x forward and y left, so North steps +x and West
// steps +y. A cell's world centre is (x, y) * MAZE_CELL_SIZE.
//
// The helpers below switch on the enum rather than doing modular arithmetic on
// it. That is deliberate: the values are not contiguous, so the (d + 1) & 3
// trick that works on a plain 0..3 clockwise enum is wrong here.
enum Direction : int { North = 0, West = 1, South = 2, East = -1 };

struct GridPose {
    int x;
    int y;
    Direction direction;
};

// A quarter turn CCW.
inline Direction leftOf(Direction d) {
    switch (d) {
        case North: return West;
        case West:  return South;
        case South: return East;
        case East:  return North;
    }
    return d;
}

// A quarter turn CW.
inline Direction rightOf(Direction d) {
    switch (d) {
        case North: return East;
        case East:  return South;
        case South: return West;
        case West:  return North;
    }
    return d;
}

inline Direction backOf(Direction d) {
    switch (d) {
        case North: return South;
        case South: return North;
        case East:  return West;
        case West:  return East;
    }
    return d;
}

// One cell step along d, in grid units.
inline int stepX(Direction d) {
    switch (d) {
        case North: return 1;
        case South: return -1;
        default:    return 0;
    }
}

inline int stepY(Direction d) {
    switch (d) {
        case West: return 1;
        case East: return -1;
        default:   return 0;
    }
}

// Dense 0..3 index, for callers packing a direction into an array slot or a
// bitfield. Ordering is the CCW one, so this is (d + 4) % 4 written out.
inline uint8_t directionIndex(Direction d) {
    switch (d) {
        case North: return 0;
        case West:  return 1;
        case South: return 2;
        case East:  return 3;
    }
    return 0;
}

inline Direction directionFromIndex(uint8_t i) {
    switch (i & 3) {
        case 0:  return North;
        case 1:  return West;
        case 2:  return South;
        default: return East;
    }
}

inline GridPose stepForward(const GridPose& curr) {
    return {curr.x + stepX(curr.direction), curr.y + stepY(curr.direction), curr.direction};
}

inline GridPose turnLeft(const GridPose& curr) {
    return {curr.x, curr.y, leftOf(curr.direction)};
}

inline GridPose turnRight(const GridPose& curr) {
    return {curr.x, curr.y, rightOf(curr.direction)};
}

inline float directionToTheta(Direction d) {
    return wrapAngle(d * PI_TWO);
}

// Nearest grid direction to an arbitrary heading. Wrapping first bounds the
// quotient to [-2, 2], so -PI and +PI both land on South rather than one of
// them falling through to a default.
inline Direction thetaToDirection(float theta) {
    const int d = static_cast<int>(roundf(wrapAngle(theta) / PI_TWO));
    switch (d) {
        case 0:  return North;
        case 1:  return West;
        case 2:  return South;
        case -2: return South;
        case -1: return East;
        default: return North;
    }
}

class Segment {
    public:

    enum class Direction { Left, Right };

    Vec2D start;
    Vec2D end;
    float curvature; // inverse radius
    Direction direction;
    Vec2D c = centre();

    Segment() : start(0, 0), end(0, 0), curvature(0), direction(Direction::Left) {}

    Segment(Vec2D start, Vec2D end, float curv, Direction dir) :
        start(start),
        end(end),
        curvature(curv),
        direction(dir) {}

    Segment(Vec2D start, Vec2D end) :
        start(start),
        end(end),
        curvature(0),
        direction(Direction::Left) {}

    float lateralDistance(const Vec2D& pos) const {
        if (curvature <= STRAIGHT_TOLERANCE) {
            return lateralDistanceForStraightLine(pos);
        }
        float r = 1.0f / curvature;
        return dist(pos - c) - r;
    }

    Vec2D lateralPoint(const Vec2D& pos) const {
        if (curvature <= STRAIGHT_TOLERANCE) {
            return lateralPointForStraightLine(pos);
        }

        float r  = 1.0f / curvature;
        Vec2D pc = pos - c;
        return c + (r / dist(pc)) * pc;
    }

    // Returns progress along the segment in [0, 1+], where >=1 means the
    // nearest point has passed the segment's end.
    float progress(const Vec2D& pos) const {
        if (curvature <= STRAIGHT_TOLERANCE) return lineProgress(pos);
        else return arcProgress(pos);
    }

    // Distance from the robot's projected position to the segment end, measured
    // along the path (arc length for arcs). Zero once the end has been passed.
    float remainingDistance(const Vec2D& pos) const {
        float p = progress(pos);
        if (p >= 1.0f) return 0.0f;

        if (curvature <= STRAIGHT_TOLERANCE) {
            return (1.0f - p) * dist(end - start);
        }

        Vec2D c               = centre();
        float startAngle      = arg(start - c);
        float endAngle        = arg(end - c);
        float directionScalar = direction == Direction::Left ? 1.0f : -1.0f;

        float totalSweep = endAngle - startAngle;
        while (directionScalar * totalSweep < 0)
            totalSweep += directionScalar * TWO_PI;

        float r = 1.0f / curvature;
        return (1.0f - p) * fabsf(totalSweep) * r;
    }

    private:

    Vec2D centre() const {
        Vec2D m = 0.5f * (start + end);
        if (curvature <= STRAIGHT_TOLERANCE) return m;
        return centrePreCalcRadiusAndMidpoint(1.0f / curvature, m);
    }

    // Fraction of the arc travelled, measured as angular progress from start
    // toward end in the turn direction (Left = CCW, Right = CW). The old
    // pe/se ratio used angleBetween(), which wraps to (-PI, PI]; a point just
    // *behind* the arc start (e.g. when the previous straight hands over a few
    // mm early) then read as ">1" ("past the end"), so the planner's advance
    // loop skipped the whole arc. Here a behind-start point reads slightly
    // negative instead, and sweeps up to a full turn are handled correctly.
    inline float arcProgress(const Vec2D& pos) const {
        float startAngle = arg(start - c);
        float sweep      = arcTravel(startAngle, arg(end - c));
        if (sweep <= 0.0f) return 1.0f;
        float travelled = arcTravel(startAngle, arg(pos - c));
        // Points on the unused side of the circle wrap toward TWO_PI; treat
        // those as "not started" (negative) rather than "past the end".
        if (travelled > 0.5f * (sweep + TWO_PI)) travelled -= TWO_PI;
        return travelled / sweep;
    }

    // Positive angular distance from `from` to `to` along the turn direction
    // (Left = CCW / increasing angle, Right = CW / decreasing), in [0, TWO_PI).
    inline float arcTravel(float from, float to) const {
        float delta = (direction == Direction::Left) ? (to - from) : (from - to);
        while (delta < 0.0f)
            delta += TWO_PI;
        while (delta >= TWO_PI)
            delta -= TWO_PI;
        return delta;
    }

    inline float lineProgress(const Vec2D& pos) const {
        Vec2D line         = end - start;
        Vec2D posFromStart = pos - start;
        return dot(posFromStart, line) / distSq(line);
    }

    inline Vec2D centrePreCalcRadiusAndMidpoint(float r, const Vec2D& m) const {
        Vec2D chordVector     = end - start;
        float chordLength     = dist(chordVector);
        float halfChordLength = 0.5f * chordLength;
        float directionScalar = direction == Direction::Left ? 1 : -1;
        // Guard an infeasible radius (r < half the chord): the discriminant
        // would go negative and sqrtf() would return NaN, poisoning the
        // centre. Clamp to 0 so the arc degenerates to its midpoint instead.
        float discriminant = (r * r) - (halfChordLength * halfChordLength);
        if (discriminant < 0.0f) discriminant = 0.0f;
        float h     = sqrtf(discriminant);
        float scale = (directionScalar * h) / chordLength;
        Vec2D chordVector_L90(-chordVector.y, chordVector.x);
        return m + (scale * chordVector_L90);
    }

    inline float lateralDistanceForStraightLine(const Vec2D& pos) const {
        Vec2D line         = end - start;
        Vec2D posFromStart = pos - start;

        float t = dot(posFromStart, line) / distSq(line);
        t       = t < 0.0f ? 0.0f : t;
        t       = t > 1.0f ? 1.0f : t;

        return dist(pos - (start + (t * line)));
    }

    inline Vec2D lateralPointForStraightLine(const Vec2D& pos) const {
        Vec2D line         = end - start;
        Vec2D posFromStart = pos - start;

        float t = dot(posFromStart, line) / distSq(line);
        t       = t < 0.0f ? 0.0f : t;
        t       = t > 1.0f ? 1.0f : t;

        return start + t * line;
    }
};

template <typename T, size_t Size>
class RingBuffer {
    static_assert(Size != 0, "RingBuffer size cannot be zero.");

    public:

    RingBuffer() = default;

    // Returns false if the buffer is full.
    bool push(const T& value) noexcept {
        if (full()) return false;

        buffer[head] = value;
        head         = increment(head);
        ++count;
        return true;
    }

    // Move-pushes an item onto the buffer.
    // Returns false if the buffer is full.
    bool push(T&& value) noexcept {
        if (full()) return false;

        buffer[head] = std::move(value);
        head         = increment(head);
        ++count;
        return true;
    }

    // push with overwrite
    void pushover(const T& value) noexcept {
        if (full()) {
            tail = increment(tail);
            --count;
        }

        buffer[head] = value;
        head         = increment(head);
        ++count;
    }

    // false if empty
    bool pop(T& value) noexcept {
        if (empty()) return false;

        value = std::move(buffer[tail]);
        tail  = increment(tail);
        --count;
        return true;
    }

    // nullptr if empty. does not pop
    T* front() noexcept {
        return empty() ? nullptr : &buffer[tail];
    }

    const T* front() const noexcept {
        return empty() ? nullptr : &buffer[tail];
    }

    void clear() noexcept {
        head  = 0;
        tail  = 0;
        count = 0;
    }

    bool empty() const noexcept {
        return count == 0;
    }

    bool full() const noexcept {
        return count == Size;
    }

    size_t size() const noexcept {
        return count;
    }

    size_t available() const noexcept {
        return Size - count;
    }

    static constexpr size_t capacity() noexcept {
        return Size;
    }

    private:

    static constexpr size_t increment(size_t index) noexcept {
        ++index;
        return (index == Size) ? 0 : index;
    }

    std::array<T, Size> buffer;
    size_t head  = 0;
    size_t tail  = 0;
    size_t count = 0;
};

// OBSTACLES AND MAPS
//
// The world LidarObserver measures against. A beam leaves a sensor at s
// travelling along the unit vector b, and the observer wants two things from
// whatever it lands on: the range the sensor ought to report, and the surface
// normal there, which is what turns the difference between expected and
// measured into a pose correction.
//
// Both obstacle types carry the relations from notes/Lidar Maths:
//
//   d_0    reference distance. For a wall, the range along the reference
//          direction; for a circle, the range to its centre.
//   alpha  wall only: how far the panel is tilted away from perpendicular to
//          the reference direction, so d_0 cos(alpha) is the perpendicular
//          distance from the sensor to the face.
//   phi    the angle of the beam away from the reference direction.
//   d      the range the sensor should report.
//
// The reference direction is free, and taking it to be the beam at whatever
// pose is currently being tested is what makes these relations useful to an
// observer rather than merely descriptive: phi is then that beam's heading
// error. d() predicts a reading, phi() inverts a reading into the heading
// error that would explain it, and d_0() inverts it into a range error.

// Angle between a beam and the inward normal of the surface it hit, given the
// outward normal.
//
// Through atan2 rather than acos of the dot product: an almost square-on hit
// puts that dot product just under 1, where acos has a vertical tangent and
// the LUT's linear interpolation reads a 0.03 rad angle as 0.010. Square-on is
// the common case for a maze, so the ill-conditioned form would be wrong most
// of the time. atan2 is well behaved everywhere.
inline float incidenceAngle(const Vec2D& outward, const Vec2D& beam) {
    const Vec2D inward = -outward;
    return Trig<TRIG_LUT_SIZE>::atan2(fabsf(cross(inward, beam)), dot(inward, beam));
}

// What a beam meets.
struct RayHit {
    // Range from the sensor to the surface, mm.
    float distance = 0.0f;
    // Unit surface normal at the hit, pointing back towards the sensor.
    Vec2D normal;
    // The note's alpha with the beam taken as the reference direction: the
    // angle between the beam and the inward normal. 0 is square on, PI/2 is
    // grazing.
    float incidence = 0.0f;
    // Index of the obstacle within its Map, or -1 for a miss.
    int16_t index = -1;
    bool valid    = false;
};

// A post, or a free-standing cylinder.
class CircularObstacle {
    public:

    constexpr explicit CircularObstacle(float radius) : radius(radius) {}

    // Range to the near surface. The derivation writes
    //     d = d_0 cos(phi) + sqrt(r^2 - d_0^2 sin^2(phi))
    // which is the far intersection, where the beam would leave the obstacle;
    // a lidar stops at the first surface it meets, so the minus root is the
    // one that gets measured. dFar() keeps the original.
    // Returns -1 when the beam misses, or the obstacle is behind the sensor.
    float d(float d_0, float phi) const {
        const float s = trig::xsin(phi);
        if (d_0 <= radius || fabsf(s) * d_0 > radius || trig::xcos(phi) <= 0.0f) return -1.0f;
        return d_0 * trig::xcos(phi) - sqrtf(radius * radius - d_0 * d_0 * s * s);
    }

    float dFar(float d_0, float phi) const {
        const float s = trig::xsin(phi);
        if (fabsf(s) * d_0 > radius) return -1.0f;
        return d_0 * trig::xcos(phi) + sqrtf(radius * radius - d_0 * d_0 * s * s);
    }

    // Beam angle implied by a measurement, from the law of cosines on the
    // sensor-centre-hit triangle. Unsigned -- a range alone cannot say which
    // side of the centre the beam passed. Returns -1 when d cannot have come
    // from this obstacle at this d_0 at all.
    // acosf rather than trig::acos: this is a law-of-cosines inverse, so its
    // argument sits just under 1 for exactly the small angles it is asked
    // about, and that is where the LUT's linear interpolation gives up --
    // acos has a vertical tangent at 1, and a 0.03 rad angle comes back as
    // 0.010. Nothing here is in the control loop.
    float phi(float d_0, float d) const {
        if (d <= 0.0f || fabsf(d_0 - radius) > d || d > d_0) return -1.0f;
        const float c = (d_0 * d_0 + d * d - radius * radius) / (2.0f * d_0 * d);
        return acosf(c > 1.0f ? 1.0f : (c < -1.0f ? -1.0f : c));
    }

    // Distance to the centre implied by a measurement. The derivation writes a
    // minus root, which pairs with the far intersection above; for a hit on
    // the near surface the centre lies beyond it, so the plus root is the one
    // that inverts d().
    float d_0(float d, float phi) const {
        const float s = trig::xsin(phi);
        if (d <= 0.0f || fabsf(s) * d > radius) return -1.0f;
        return d * trig::xcos(phi) + sqrtf(radius * radius - d * d * s * s);
    }

    constexpr float boundingRadius() const {
        return radius;
    }

    // to_centre is the centre relative to the sensor; beam is a unit vector.
    RayHit cast(const Vec2D& to_centre, const Vec2D& beam) const {
        RayHit hit;
        const float centre_range = dist(to_centre);
        if (centre_range <= radius) return hit; // sensor inside the obstacle

        const float range = d(centre_range, angleBetween(beam, to_centre));
        if (range < 0.0f) return hit;

        hit.distance  = range;
        hit.normal    = (1.0f / radius) * ((range * beam) - to_centre);
        hit.incidence = incidenceAngle(hit.normal, beam);
        hit.valid     = true;
        return hit;
    }

    private:

    float radius;
};

// One panel: a rectangle of `length` by `thickness`, centred on its lattice
// bond.
class WallObstacle {
    public:

    constexpr WallObstacle(float length, float thickness, float alpha) :
        length(length),
        thickness(thickness),
        alpha(alpha) {}

    // The relations below take the note's alpha directly, as `alpha_rel`. The
    // member `alpha` is the panel's heading in map coordinates and only
    // becomes the note's alpha once the beam's heading is known, which is what
    // cast() works out. Keeping them separate means these three stay pure
    // functions of the geometry the derivation describes.

    // Expected range. Returns -1 when the beam is running along the face or
    // away from it.
    float d(float d_0, float phi, float alpha_rel) const {
        const float c = trig::xcos(phi + alpha_rel);
        if (c <= STD_TOL) return -1.0f;
        return d_0 * trig::xcos(alpha_rel) / c;
    }

    // Beam angle implied by a measurement.
    //
    // The derivation writes phi = arccos(d / (d_0 cos alpha)) - alpha, but
    // d_0 cos(alpha) is the perpendicular distance to the face and no
    // measurement of it can be shorter, so that ratio is never below 1 and the
    // arccos always collapses to 0. Inverting d = d_0 cos(alpha) /
    // cos(phi + alpha) gives the ratio the other way up, which is what this
    // returns. Unsigned in the arccos branch: a single range cannot say which
    // way the beam swung. Returns -3 PI when d is short of the face.
    // acosf for the same reason as CircularObstacle::phi(): the ratio is just
    // under 1 for the small angles this is asked about, which is where the
    // LUT's acos loses most of its accuracy.
    float phi(float d_0, float d, float alpha_rel) const {
        const float perpendicular = d_0 * trig::xcos(alpha_rel);
        if (d <= 0.0f || perpendicular > d) return -3.0f * PI;
        return acosf(perpendicular / d) - alpha_rel;
    }

    // Reference distance implied by a measurement taken at beam angle phi.
    float d_0(float d, float phi, float alpha_rel) const {
        const float c = trig::xcos(alpha_rel);
        if (fabsf(c) <= STD_TOL) return -1.0f;
        return d * trig::xcos(phi + alpha_rel) / c;
    }

    // Not constexpr: the build passes -fno-builtin, so sqrtf is a library call
    // even in a constant expression.
    float boundingRadius() const {
        return 0.5f * sqrtf(length * length + thickness * thickness);
    }

    // Beam against the face the sensor is on. The two ends are left out: every
    // bond in the lattice terminates on a post, and a post's own circle covers
    // that corner more accurately than a square end cap would.
    RayHit cast(const Vec2D& to_centre, const Vec2D& beam) const {
        RayHit hit;

        const Vec2D along  = {trig::xcos(alpha), trig::xsin(alpha)};
        const Vec2D normal = perp(along);

        // Signed distance from the sensor to the panel's centre plane. Its
        // sign picks which of the two faces is the one facing the sensor.
        const float centre_offset = dot(normal, to_centre);
        const Vec2D outward       = (centre_offset >= 0.0f) ? -normal : normal;

        // d_0 cos(alpha) in the derivation: perpendicular distance from the
        // sensor to that face.
        const float perpendicular = fabsf(centre_offset) - 0.5f * thickness;
        if (perpendicular <= 0.0f) return hit; // sensor inside the panel

        // The note's alpha for this beam, and with it d = d_0 cos(alpha) /
        // cos(phi + alpha) at phi = 0.
        const float cos_alpha = -dot(outward, beam);
        if (cos_alpha <= STD_TOL) return hit; // parallel to the face, or behind it

        const float range = perpendicular / cos_alpha;
        if (fabsf(dot(along, (range * beam) - to_centre)) > 0.5f * length) return hit;

        hit.distance  = range;
        hit.normal    = outward;
        hit.incidence = incidenceAngle(outward, beam);
        hit.valid     = true;
        return hit;
    }

    private:

    float length;
    float thickness;
    float alpha; // Angle of the wall from the x-axis
};

struct Obstacle {
    std::variant<CircularObstacle, WallObstacle> form;
    Vec2D centre;

    // std::get_if rather than std::visit: visit is not constexpr in C++17 and
    // drags in more of <variant> than a two-alternative dispatch needs on a
    // build with -fno-exceptions.
    float boundingRadius() const {
        const CircularObstacle* c = std::get_if<CircularObstacle>(&form);
        return c ? c->boundingRadius() : std::get_if<WallObstacle>(&form)->boundingRadius();
    }

    RayHit cast(const Vec2D& origin, const Vec2D& beam) const {
        const Vec2D to_centre     = centre - origin;
        const CircularObstacle* c = std::get_if<CircularObstacle>(&form);
        return c ? c->cast(to_centre, beam) : std::get_if<WallObstacle>(&form)->cast(to_centre, beam);
    }

    // How far the beam would have to swing for this obstacle to return
    // `measured`, given that it returns `prior.distance` as aimed: the note's
    // phi inverse, applied to a beam in the world.
    //
    // This answers a one-degree-of-freedom question -- how much heading error
    // explains this reading, if the sensor's position is exact -- so it is a
    // diagnostic, not a gate. Square on to a wall the range hardly responds to
    // heading at all, and a few mm of position error comes back as a wildly
    // implausible half a radian. LidarObserver solves all three degrees of
    // freedom together instead; see the note in its beamEquation().
    //
    // Returns -1 when the reading is impossible for this obstacle at any angle.
    float impliedHeadingError(const RayHit& prior, const Vec2D& origin, const Vec2D& beam,
        float measured) const {
        if (!prior.valid || measured <= 0.0f) return -1.0f;

        if (const CircularObstacle* c = std::get_if<CircularObstacle>(&form)) {
            const Vec2D to_centre = centre - origin;
            const float centre_range = dist(to_centre);
            const float implied      = c->phi(centre_range, measured);
            if (implied < 0.0f) return -1.0f;
            // phi() is unsigned, so compare magnitudes: both branches are the
            // same swing, mirrored about the centre line.
            return fabsf(implied - fabsf(angleBetween(beam, to_centre)));
        }

        const WallObstacle* w = std::get_if<WallObstacle>(&form);
        const float implied   = w->phi(prior.distance, measured, prior.incidence);
        if (implied < -PI) return -1.0f;
        return fabsf(implied);
    }
};

// A fixed set of obstacles in map coordinates.
//
// Built offline: path-planning/export_map.py runs the vision pipeline over a
// photo of the maze and emits a constexpr Map, so the whole thing lives in
// flash and costs no RAM. See maze_map.h.
//
// An aggregate, deliberately. std::variant's copy constructor is not constexpr
// in the GCC 7.2 libstdc++ the Renesas core ships, so a constructor taking the
// array by reference cannot copy it in a constant expression; initialising the
// members in place sidesteps the copy entirely.
template <size_t S>
struct Map {

    std::array<Obstacle, S> obstacles;

    static constexpr size_t size() {
        return S;
    }

    constexpr const Obstacle& operator[](size_t i) const {
        return obstacles[i];
    }

    // Nearest surface along `beam` from `origin`, out to `max_range`.
    //
    // `indices`/`count` restrict the search to a subset -- see candidates().
    // Passing nullptr searches everything, which is correct but walks the
    // whole map on every call.
    RayHit cast(const Vec2D& origin, const Vec2D& beam, float max_range,
        const uint16_t* indices = nullptr, size_t count = 0) const {
        RayHit best;
        best.distance      = max_range;
        const size_t total = (indices == nullptr) ? S : count;

        for (size_t k = 0; k < total; ++k) {
            const size_t i = (indices == nullptr) ? k : indices[k];
            RayHit hit     = obstacles[i].cast(origin, beam);
            if (!hit.valid || hit.distance >= best.distance) continue;
            hit.index = static_cast<int16_t>(i);
            best      = hit;
        }
        return best;
    }

    // Indices of every obstacle whose bounding circle reaches within `radius`
    // of `centre`. Run once per solve so the casts inside the iteration only
    // walk the neighbourhood. Returns the number written; `out` is filled to
    // capacity and the rest dropped, so a caller that fills it should widen
    // its array or fall back to a full-map cast.
    template <size_t N>
    size_t candidates(const Vec2D& centre, float radius, std::array<uint16_t, N>& out) const {
        size_t n = 0;
        for (size_t i = 0; i < S && n < N; ++i) {
            const float reach = radius + obstacles[i].boundingRadius();
            if (distSq(obstacles[i].centre - centre) > reach * reach) continue;
            out[n++] = static_cast<uint16_t>(i);
        }
        return n;
    }
};

#pragma GCC pop_options
