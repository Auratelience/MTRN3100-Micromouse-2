// The maze the mapper has discovered, as castable geometry
//
// Zimmy Levi z5587840

#pragma once

#include <math.h>
#include <array>

#include "constants.h"
#include "mazeMapper.h"
#include "types.h"

#pragma GCC push_options
#pragma GCC optimize("O2")

// A Map-shaped view of MazeMapper's wall bits.
//
// maze_map.h is fitted by computer vision from a photograph of the maze, which
// is exactly what an unseen maze does not give you: the maze is unknown, and
// finding it is the job. So LidarObserver cannot localise against it. It
// localises against this instead -- the walls the robot has actually seen,
// plus the perimeter and any priors, converted to panels and posts on demand.
//
// Nothing is stored. Every obstacle is derived from a bit in the mapper, so
// this costs one reference of RAM and is never stale: a wall discovered on one
// tick is available to the next solve.
//
// Duck-typed against Map<S> rather than derived from it. LidarObserver and
// OLEDScreen need only cast(), candidates(), size(), present() and operator[],
// and templating them on the map type keeps both working with either -- the
// exported map where there is one, this one where the maze is unseen.
//
// What it gives up: a fitted map carries the real maze, tilted panels and free
// standing cylinders included, whereas this is a perfect lattice. It is
// therefore only as good as the mapper's belief, and a wall the robot has not
// seen yet is simply absent. That direction is safe rather than merely
// tolerable -- a beam landing on an undiscovered wall reads short of whatever
// this predicts behind it, and LIDAR_OBSERVER_MAX_RESIDUAL_MM drops it.
// Missing geometry costs beams; it does not invent corrections.
//
// Geometry this holds and the maze does not is the direction that could drag a
// fix, since a prediction wrong by less than that residual gate is folded in
// rather than rejected. Hence the two rules below: a panel only where the
// mapper has a wall bit, and a post only where a panel ends on it.
//
// The index space is laid out in three blocks, so an index decodes without a
// lookup table:
//
//   [0, NS)              north-south panels: NS(a, b) separates cells
//                        (a - 1, b) and (a, b), so a runs 0..N inclusive
//   [NS, NS + EW)        east-west panels: EW(a, b) separates (a, b - 1) and
//                        (a, b)
//   [NS + EW, COUNT)     lattice posts, one per corner (a, b)
template <size_t N>
class MazeWallMap {
    static constexpr size_t NS_COUNT   = (N + 1) * N;
    static constexpr size_t EW_COUNT   = N * (N + 1);
    static constexpr size_t POST_COUNT = (N + 1) * (N + 1);

    public:

    static constexpr size_t COUNT = NS_COUNT + EW_COUNT + POST_COUNT;

    explicit MazeWallMap(const MazeMapper<N>& mapper) : mapper(mapper) {}

    static constexpr size_t size() {
        return COUNT;
    }

    // Whether the slot holds anything. Map<S> is full by construction and
    // answers true; here most slots are empty, because a maze holds nowhere
    // near a wall on every boundary.
    bool present(size_t index) const {
        if (index < NS_COUNT) {
            const int a = static_cast<int>(index / N);
            const int b = static_cast<int>(index % N);
            return nsWall(a, b);
        }
        if (index < NS_COUNT + EW_COUNT) {
            const size_t i = index - NS_COUNT;
            const int a    = static_cast<int>(i / (N + 1));
            const int b    = static_cast<int>(i % (N + 1));
            return ewWall(a, b);
        }
        const size_t i = index - NS_COUNT - EW_COUNT;
        const int a    = static_cast<int>(i / (N + 1));
        const int b    = static_cast<int>(i % (N + 1));
        return postAt(a, b);
    }

    // By value, not by reference: nothing is stored, so there is no obstacle
    // to refer to. Undefined for a slot present() rejects.
    //
    // The centre comes from centreOf rather than being worked out again here.
    // The two used to carry their own copy of the same offset arithmetic, and
    // they have to agree: candidates() bounds a slot at centreOf and cast()
    // builds it here, so a slot whose obstacle sat anywhere its bounding test
    // did not would be dropped by the sweep and then hit by the ray.
    Obstacle operator[](size_t index) const {
        // A north-south panel runs along y, so alpha is a quarter turn.
        if (index < NS_COUNT) {
            return Obstacle{
                WallObstacle{MAZE_CELL_SIZE, MAZE_WALL_THICKNESS, PI_TWO}, centreOf(index)
            };
        }
        if (index < NS_COUNT + EW_COUNT) {
            return Obstacle{
                WallObstacle{MAZE_CELL_SIZE, MAZE_WALL_THICKNESS, 0.0f}, centreOf(index)
            };
        }
        return Obstacle{CircularObstacle{MAZE_POST_RADIUS}, centreOf(index)};
    }

    // Nearest surface along `beam`, out to `max_range`. Same contract as
    // Map<S>::cast, including that a null index list means search everything.
    RayHit cast(const Vec2D& origin, const Vec2D& beam, float max_range,
        const uint16_t* indices = nullptr, size_t count = 0) const {
        RayHit best;
        best.distance      = max_range;
        const size_t total = (indices == nullptr) ? COUNT : count;

        for (size_t k = 0; k < total; ++k) {
            const size_t i = (indices == nullptr) ? k : indices[k];
            if (!present(i)) continue;
            RayHit hit = (*this)[i].cast(origin, beam);
            if (!hit.valid || hit.distance >= best.distance) continue;
            hit.index = static_cast<int16_t>(i);
            best      = hit;
        }
        return best;
    }

    // Indices whose bounding circle reaches within `radius` of `centre`.
    //
    // Walks the whole index space, as Map<S>::candidates does. The bounding
    // radius is a constant per kind here, so the test is a centre and a
    // compare with no obstacle built and no sqrtf -- which is what keeps a
    // 280-slot sweep at N = 9 cheaper than the 142-obstacle one it replaces.
    template <size_t M>
    size_t candidates(const Vec2D& centre, float radius, std::array<uint16_t, M>& out) const {
        size_t n = 0;
        for (size_t i = 0; i < COUNT && n < M; ++i) {
            if (!present(i)) continue;
            const float reach = radius + boundingRadiusOf(i);
            if (distSq(centreOf(i) - centre) > reach * reach) continue;
            out[n++] = static_cast<uint16_t>(i);
        }
        return n;
    }

    private:

    const MazeMapper<N>& mapper;

    static Cell cell(int x, int y) {
        return Cell{static_cast<int8_t>(x), static_cast<int8_t>(y)};
    }

    // NS(a, b) is the boundary between cells (a - 1, b) and (a, b). At a = 0
    // the lower cell is outside the maze, so the boundary is read from the
    // inner cell's South side instead; at a = N the upper one is, and
    // hasWall(Cell{N - 1, b}, North) covers it.
    bool nsWall(int a, int b) const {
        if (b < 0 || b >= static_cast<int>(N)) return false;
        if (a <= 0) return mapper.hasWall(cell(0, b), South);
        return mapper.hasWall(cell(a - 1, b), North);
    }

    bool ewWall(int a, int b) const {
        if (a < 0 || a >= static_cast<int>(N)) return false;
        if (b <= 0) return mapper.hasWall(cell(a, 0), East);
        return mapper.hasWall(cell(a, b - 1), West);
    }

    // A post exists where at least one panel meets it.
    //
    // Not at every lattice point: the exported map counts 52 posts on a 100
    // point lattice, so claiming all of them would put geometry in open floor
    // and predict returns from nothing. Where a panel does end, the post is
    // what covers the corner -- WallObstacle::cast leaves its end caps out on
    // purpose, for exactly this reason.
    bool postAt(int a, int b) const {
        return nsWall(a, b) || nsWall(a, b - 1) || ewWall(a, b) || ewWall(a - 1, b);
    }

    static bool isPost(size_t index) {
        return index >= NS_COUNT + EW_COUNT;
    }

    // Constant per kind, so neither of the sweeps above needs an obstacle or a
    // square root to reject a slot. Held as a member rather than a function
    // local static: sqrtf is not constexpr on this build (-fno-builtin), and a
    // local static would put a guard variable check in the middle of a sweep
    // that runs 280 times a solve.
    const float panelRadius = 0.5f * sqrtf(MAZE_CELL_SIZE * MAZE_CELL_SIZE +
                                           MAZE_WALL_THICKNESS * MAZE_WALL_THICKNESS);

    float boundingRadiusOf(size_t index) const {
        return isPost(index) ? MAZE_POST_RADIUS : panelRadius;
    }

    Vec2D centreOf(size_t index) const {
        if (index < NS_COUNT) {
            const int a = static_cast<int>(index / N);
            const int b = static_cast<int>(index % N);
            const auto origin = mapper.startPosition();
            return cellToWorld(static_cast<float>(a) - 0.5f, static_cast<float>(b), origin.x, origin.y);
        }
        if (index < NS_COUNT + EW_COUNT) {
            const size_t i = index - NS_COUNT;
            const int a    = static_cast<int>(i / (N + 1));
            const int b    = static_cast<int>(i % (N + 1));
            const auto origin = mapper.startPosition();
            return cellToWorld(static_cast<float>(a), static_cast<float>(b) - 0.5f, origin.x, origin.y);
        }
        const size_t i = index - NS_COUNT - EW_COUNT;
        const int a    = static_cast<int>(i / (N + 1));
        const int b    = static_cast<int>(i % (N + 1));
        const auto origin = mapper.startPosition();
        return cellToWorld(static_cast<float>(a) - 0.5f, static_cast<float>(b) - 0.5f, origin.x, origin.y);
    }
};

#pragma GCC pop_options
