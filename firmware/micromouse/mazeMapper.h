// Maze Mapper
//
// Gokul Krishnan
// Zimmy Levi z5587840

#pragma once

#include <Arduino.h>

#include <Embedded_Template_Library.h>
#include <etl/string.h>

#include "constants.h"
#include "types.h"

#pragma GCC push_options
#pragma GCC optimize("O2")

// Frontier exploration of an unknown N x N maze, then a shortest route through
// what the exploration actually saw.
//
// The sweep always drives towards the nearest cell that still has somewhere
// worth exploring to go, and once none is left, home to the start cell. Both
// legs are the shortest route through explored cells, so the mouse never
// retraces a corridor it has already seen just because that is the way it came
// in.
//
// "Worth exploring" narrows as the map fills: once the goal has been reached,
// anywhere that provably cannot yield a shorter route to it is dropped, and the
// sweep ends the moment nothing is left that could. So the map this leaves is
// partial by design -- every cell missing from it is one that could not have
// changed the route -- and doneExploring() means the shortest route is known,
// not that every cell was seen.
//
// Cells are the shared grid convention from types.h -- (x, y) with x forward
// and y left, North stepping +x and West stepping +y -- so a route this class
// produces feeds PSPlanner with no axis or sign fixup in between. The class
// owns the map and the search and nothing else: no sensor, no motor, no pose.
// Everything is in cell units, so MAZE_CELL_SIZE never appears here.
//
// The caller drives the loop:
//
//     MazeMapper<9> mapper(Cell{0, 0}, North, Cell{4, 4});
//     mapper.begin();
//     mapper.markWall(Cell{4, 4}, North);                  // priors, if any
//     while (!mapper.doneExploring()) {
//         mapper.observe(frontWall, leftWall, rightWall);  // at the current
//         cell Direction move; if (!mapper.planMove(move)) break; if
//         (!driveOneCell(move)) break;                  // caller's problem
//         mapper.commitMove(move);                         // only once it
//         arrived
//     }
//     if (mapper.faulted()) { /* map is suspect, do not race on it */ }
//     mapper.buildShortestPathToGoal();
//
// observe() only ever sees the three sides the sensors face, so the side the
// robot came in through is never sensed. That is fine everywhere except the
// start cell, which was never entered: nobody has looked at its fourth side,
// and a cleared wall bit reads as open, so planMove would drive backwards
// through it on the first move.
//
// The caller settles that by looking rather than by assuming. Turn ninety
// degrees before the first planMove -- commitTurn to keep the map's heading
// with the robot's -- and the next observe() reads the side that was behind on
// the sensor that is now pointing at it. Unnecessary if the perimeter already
// settled it, which it has whenever the run starts against the maze edge.

struct Cell {
    int8_t x;
    int8_t y;
};

template <size_t N>
class MazeMapper {
    static_assert(N >= 2, "N >= 2 required.");
    // Cell packs a coordinate into an int8_t, and the counters below are
    // uint16_t, which caps N * N at 65535 well above this anyway.
    static_assert(N <= 127, "N <= 127 required.");

    public:

    // Which side of a cell carries a wall. Values line up with
    // directionIndex(), so wallMask() is a shift rather than another switch.
    enum WallBit : uint8_t { WallNorth = 1, WallWest = 2, WallSouth = 4, WallEast = 8 };

    MazeMapper(Cell startCell, Direction startHeading, Cell goalCell) :
        start(startCell), goal(goalCell), current(startCell),
        facing(startHeading), startFacing(startHeading) {}

    // Longest route the maze can hold, and so the size of every path buffer.
    static constexpr uint16_t MAX_CELLS = static_cast<uint16_t>(N) * static_cast<uint16_t>(N);

    // Seeds the map: everything unexplored, walls only around the perimeter.
    // Returns false if either cell is outside the maze, leaving the mapper
    // unstarted rather than half-configured -- observe() writes at the current
    // cell, so an out-of-range start would corrupt memory on the first call.
    bool begin() {
        started = false;
        if (!inside(start) || !inside(goal)) return false;

        current      = start;
        finalPathLen = 0;
        movePending  = false;
        explored     = false;
        broken       = false;
        seenCells    = 0;
        homeHops     = 0;
        homeHops0    = 0;

        for (uint8_t x = 0; x < N; ++x) {
            for (uint8_t y = 0; y < N; ++y) {
                walls[x][y]          = 0;
                openBoundaries[x][y] = 0;
                visitedCells[x][y]   = false;
                // Nothing is ruled out before the first planMove refreshes it,
                // which matters because markWall priors land between here and
                // there and the frontier search must not read a stale verdict.
                improving[x][y]      = true;
            }
        }

        // Perimeter. Each of these mirrors onto a cell outside the maze, which
        // addWall drops, so no already-initialised neighbour is touched.
        for (uint8_t i = 0; i < N; ++i) {
            const int8_t last = static_cast<int8_t>(N - 1);
            addWall(Cell{0, static_cast<int8_t>(i)}, South);
            addWall(Cell{last, static_cast<int8_t>(i)}, North);
            addWall(Cell{static_cast<int8_t>(i), 0}, East);
            addWall(Cell{static_cast<int8_t>(i), last}, West);
        }

        started = true;
        return true;
    }

    bool ready() const { return started; }
    Cell position() const { return current; }
    Direction heading() const { return facing; }
    Direction startHeading() const { return startFacing; }
    Cell startPosition() const { return start; }
    Cell goalPosition() const { return goal; }
    bool doneExploring() const { return explored; }
    bool atGoal() const { return sameCell(current, goal); }

    // Cells observe() has stood in. Kept as a running total rather than left
    // to the caller: the display and the progress meter both want it every
    // frame, and each was walking all N * N cells to get it.
    uint16_t visitedCount() const { return seenCells; }

    // True once the mapper caught its own state going inconsistent -- a commit
    // it could not apply, or a start cell it could not route back to.
    // Exploration stops when this trips, which also raises doneExploring(), so
    // the two have to be read together: doneExploring() alone cannot tell a maze
    // that was fully swept from a run that gave up part way, and the map from
    // the second is not safe to race on.
    bool faulted() const { return broken; }

    // True while the sweep is on its way back to the start cell: planMove has
    // run out of frontiers worth having and is stepping home under rule 3.
    //
    // There is no separate state for that leg -- rules 1 to 3 are all
    // exploration as far as the runner is concerned -- so this is the only way
    // to tell a mouse still discovering the maze from one that has finished and
    // is walking back. A display that could not would report EXPL for the whole
    // return trip.
    bool homing() const { return homeHops > 0; }

    // How far along the way home the sweep is, on [0, 1]. Zero unless homing(),
    // and one notch per cell arrived at, since planMove runs once per arrival.
    //
    // Measured against the distance home at the moment the leg began, not
    // against the maze, so it starts near zero and reaches one on the start
    // cell. Monotonic: rule 3 always takes a shortest route through explored
    // cells, so the remaining distance never grows within a leg.
    float homeProgress() const {
        if (homeHops == 0 || homeHops0 == 0) return 0.0f;
        if (homeHops >= homeHops0) return 0.0f;
        return 1.0f - (static_cast<float>(homeHops) / static_cast<float>(homeHops0));
    }

    // Records what the sensors see from the current cell. Walls are stored on
    // both sides of the boundary, so a wall found from one cell is known from
    // its neighbour too.
    void observe(bool frontWall, bool leftWall, bool rightWall) {
        if (!started) return;
        if (!visitedCells[current.x][current.y]) ++seenCells;
        visitedCells[current.x][current.y] = true;
        if (frontWall) addWall(current, facing);
        if (leftWall) addWall(current, leftOf(facing));
        if (rightWall) addWall(current, rightOf(facing));
    }

    // Seeds a wall the sensors will not produce: the start cell's rear, or a
    // known feature of the competition maze. Same both-sides storage as
    // observe(). False if the cell is outside the maze, or if the robot has
    // already driven through that boundary.
    bool markWall(const Cell& c, Direction d) {
        if (!started || !inside(c)) return false;
        if (isOpen(c, d)) return false;
        addWall(c, d);
        return true;
    }

    // Walls a cell in on all four sides: the way to say "this cell is not
    // there". A maze whose corners are chamfered, or whose outline is not a
    // rectangle, is seeded with these before the first observe().
    //
    // There is no separate notion of a nonexistent cell, and none is needed.
    // planMove skips a neighbour behind a wall and buildShortestPath will not
    // expand through one, so a cell walled on every side can be neither
    // entered nor planned through. False if any side was refused -- a boundary the robot
    // has already driven through outranks a prior, and a cell that is only
    // partly sealed is one the search can still walk into.
    bool sealCell(const Cell& c) {
        bool ok = markWall(c, North);
        ok      = markWall(c, South) && ok;
        ok      = markWall(c, West) && ok;
        ok      = markWall(c, East) && ok;
        return ok;
    }

    // Walled on every side and never entered, so unreachable: a cell the
    // discovered walls have closed off, which is how a cell the physical maze
    // does not have -- a chamfered corner -- shows up once the sweep has been
    // past it. There is no separate notion of a nonexistent cell, and none is
    // needed: planMove skips a neighbour behind a wall and buildShortestPath
    // will not expand through one, so such a cell can be neither entered nor
    // planned through.
    //
    // Visited cells are excluded because they cannot be sealed -- the boundary
    // the robot came in through was cleared on the way.
    bool sealedCell(const Cell& c) const {
        return !visited(c) && hasWall(c, North) && hasWall(c, South) && hasWall(c, West) &&
               hasWall(c, East);
    }

    // Picks the next move, in four rules:
    //
    //   1. An unexplored neighbour of this cell worth having -- the nearest
    //      frontier there is, reached at no travel cost. Prefers left, then
    //      straight, then right, then behind, which keeps the mouse sweeping one
    //      way through open areas instead of oscillating.
    //   2. Otherwise a step towards the nearest cell that has one.
    //   3. Otherwise -- nothing left worth exploring -- a step towards the start
    //      cell.
    //   4. Otherwise the sweep is finished, standing where it began.
    //
    // Rules 2 and 3 are shortest routes through explored cells, so a travel leg
    // costs the distance to where the mouse is going and not the length of the
    // path that happened to discover it.
    //
    // "Worth having" is refreshImproving's test, and it is what ends the sweep
    // early: once every remaining frontier is provably useless the map already
    // holds the shortest route to the goal, rules 1 and 2 find nothing, and rule
    // 3 takes the mouse home. Nothing else has to notice -- there is no separate
    // proof step, because running out of worthwhile frontiers is the proof.
    //
    // Rule 3 is nonetheless observable, through homing() and homeProgress():
    // the leg home is the one stretch of a sweep where the mouse is not
    // discovering anything, and a caller reporting on the run wants to say so.
    //
    // Does not move the mapper. Call commitMove once the robot has arrived.
    bool planMove(Direction& move) {
        movePending = false;
        if (!started) return false;

        refreshImproving();

        const Direction order[4] = {leftOf(facing), facing, rightOf(facing), backOf(facing)};
        for (uint8_t i = 0; i < 4; ++i) {
            const Direction d    = order[i];
            const Cell next = neighbour(current, d);
            if (!inside(next)) continue;
            if (hasWall(current, d)) continue;
            if (visited(next)) continue;
            if (!worthExploring(next)) continue;
            homeHops = 0;
            return setPending(move, d);
        }

        Direction step;
        uint16_t hops = 0;
        if (firstStepToward(Target::Frontier, step, hops)) {
            homeHops = 0;
            return setPending(move, step);
        }

        if (!sameCell(current, start)) {
            if (firstStepToward(Target::Start, step, hops)) {
                // The homing leg has begun, or is one cell further along. hops
                // is the distance still to run; the first one is kept as the
                // denominator, so homeProgress() is measured against the route
                // home the mouse actually faced rather than against a maze
                // dimension it may never have crossed.
                //
                // Never re-seeded once set: rules 1 and 2 clear it, so a
                // frontier the sweep changes its mind about restarts the
                // measure rather than rescaling one already in progress.
                homeHops = hops;
                if (homeHops0 == 0) homeHops0 = hops;
                return setPending(move, step);
            }

            // Cannot happen: every boundary the robot drove through is recorded
            // open, a route home only has to cross those, and clearWallBetween
            // marks them from both sides. If the search cannot find one anyway
            // the map contradicts the drive that built it, which is not a map
            // to race on -- so stop, and flag it rather than reporting a
            // finished sweep.
            homeHops = 0;
            broken   = true;
            explored = true;
            return false;
        }

        // Standing where it began, with nothing left worth exploring: the
        // sweep is over and the homing leg with it, so homing() stops
        // answering true before doneExploring() starts.
        homeHops = 0;
        explored = true;
        return false;
    }

    // Applies the move planMove returned, once the robot has physically made
    // it. Rejects anything else, so a dropped, repeated or invented move cannot
    // silently walk the mapper's position away from the robot's.
    bool commitMove(Direction move) {
        if (!movePending || move != pendingMove) return false;
        movePending = false;

        // Cannot fire off a move planMove actually returned -- it only ever
        // offers in-range cells. It is here because the robot has already
        // physically moved by this point, so failing quietly would leave the
        // map tracking a position the robot is not in.
        const Cell next = neighbour(current, move);
        if (!inside(next)) {
            broken = true;
            return false;
        }

        // The robot just drove through this boundary, so whatever the map
        // thought, there is no wall there.
        clearWallBetween(current, move);
        current = next;
        facing  = move;
        return true;
    }

    // Records a rotation on the spot, once the robot has physically made it. The
    // heading changes, the cell does not, and no boundary is touched -- turning
    // is not evidence about a wall either way.
    //
    // It has to be recorded at all because observe() reads its three arguments
    // relative to facing: a rotation the map does not know about folds the next
    // set of readings onto the wrong sides of the cell. That is also why this
    // takes the heading the robot ended on rather than which way it span.
    //
    // Nothing to reject -- any heading is a legal thing to be facing -- so this
    // is not the plan-then-commit contract commitMove is, and a pending move
    // survives it untouched: planMove's answer is a compass direction, not a
    // turn, so it means the same thing afterwards.
    bool commitTurn(Direction heading) {
        if (!started) return false;
        facing = heading;
        return true;
    }

    // Shortest route from the start cell to the goal through explored cells
    // only, so it never plans through a boundary that was never looked at.
    //
    // From the start rather than from wherever the robot is now, because a
    // finished sweep routes itself home, leaving the robot on the start cell.
    // Use buildShortestPath if the run stopped early -- on atGoal(), say -- or
    // to route from where the robot stands.
    //
    // The heading it ends on is a different question: it is whatever the last
    // step home happened to be, not the one begin() was given, so render this
    // with the toInstructions overload that takes heading() explicitly.
    bool buildShortestPathToGoal() { return buildShortestPath(start, goal); }

    uint16_t shortestPathLength() const { return finalPathLen; }

    bool shortestPathCell(uint16_t index, Cell& cell) const {
        if (index >= finalPathLen) return false;
        cell = finalPath[index];
        return true;
    }

    // Renders the shortest path as the "flrf" string PSPlanner::addInstructions
    // consumes, starting from the heading given. This is the whole handoff
    // between the two classes, and it is short only because they share a
    // convention.
    //
    // The heading is always the caller's to supply: the only one this class
    // could assume is the one begin() was given, and the robot is not on it by
    // the time a route is rendered -- neither at the end of a sweep nor part way
    // through one. Pass heading().
    //
    // False on truncation rather than a quietly shortened route.
    bool toInstructions(etl::string<MAZE_INSTRUCTION_MAX_LEN>& out, Direction fromHeading) const {
        out.clear();
        Direction d = fromHeading;

        for (uint16_t i = 1; i < finalPathLen; ++i) {
            Direction step;
            if (!directionTo(finalPath[i - 1], finalPath[i], step)) return false;
            if (!appendTurns(out, d, step)) return false;

            if (out.full()) return false;
            out += 'f';
        }
        return true;
    }

    static bool inside(const Cell& c) {
        return c.x >= 0 && c.x < static_cast<int>(N) && c.y >= 0 && c.y < static_cast<int>(N);
    }

    static bool sameCell(const Cell& a, const Cell& b) { return a.x == b.x && a.y == b.y; }

    static Cell neighbour(const Cell& c, Direction d) {
        return Cell{static_cast<int8_t>(c.x + stepX(d)), static_cast<int8_t>(c.y + stepY(d))};
    }

    // Heading from one cell to an orthogonally adjacent one. False for a
    // coincident, diagonal or distant pair instead of guessing: the result
    // feeds code that erases walls, and the unchecked version of this returned
    // West for a coincident pair and North for anything diagonal.
    static bool directionTo(const Cell& from, const Cell& to, Direction& out) {
        const int deltaX = to.x - from.x;
        const int deltaY = to.y - from.y;
        if (deltaX == 1 && deltaY == 0) { out = North; return true; }
        if (deltaX == -1 && deltaY == 0) { out = South; return true; }
        if (deltaX == 0 && deltaY == 1) { out = West; return true; }
        if (deltaX == 0 && deltaY == -1) { out = East; return true; }
        return false;
    }

    bool hasWall(const Cell& c, Direction d) const {
        if (!inside(c)) return true; // outside the maze is solid
        return (walls[c.x][c.y] & wallMask(d)) != 0;
    }

    bool visited(const Cell& c) const {
        if (!inside(c)) return false;
        return visitedCells[c.x][c.y];
    }

    private:

    static uint8_t wallMask(Direction d) {
        return static_cast<uint8_t>(1u << directionIndex(d));
    }

    // A boundary the robot has driven through. Direct evidence, and it outranks
    // anything the sensors say later: without this a single false positive --
    // the front sensor picking up a post, or the mouse sitting skew in a cell
    // it has been through before -- re-walls an opening the robot has already
    // used, and buildShortestPath then plans the long way round it or fails.
    bool isOpen(const Cell& c, Direction d) const {
        if (!inside(c)) return false;
        return (openBoundaries[c.x][c.y] & wallMask(d)) != 0;
    }

    void addWall(const Cell& c, Direction d) {
        if (!inside(c)) return;
        if (isOpen(c, d)) return;
        walls[c.x][c.y] |= wallMask(d);
        const Cell other = neighbour(c, d);
        if (inside(other)) walls[other.x][other.y] |= wallMask(backOf(d));
    }

    void clearWallBetween(const Cell& c, Direction d) {
        if (!inside(c)) return;
        walls[c.x][c.y] &= static_cast<uint8_t>(~wallMask(d));
        openBoundaries[c.x][c.y] |= wallMask(d);
        const Cell other = neighbour(c, d);
        if (inside(other)) {
            walls[other.x][other.y] &= static_cast<uint8_t>(~wallMask(backOf(d)));
            openBoundaries[other.x][other.y] |= wallMask(backOf(d));
        }
    }

    bool setPending(Direction& move, Direction d) {
        move        = d;
        pendingMove = d;
        movePending = true;
        return true;
    }

    // No route to a cell, in a distance field.
    static constexpr uint16_t Unreachable = 0xFFFF;

    // True for an unexplored cell a route shorter than the best one the map
    // already holds could still come through. Everything, until the goal has
    // been reached and there is a route to beat.
    bool worthExploring(const Cell& c) const {
        return inside(c) && improving[c.x][c.y];
    }

    // Cells from `source`, breadth-first, in cells travelled.
    //
    // throughUnknown treats every boundary not known to be a wall as open and
    // every cell as enterable, which makes the result a lower bound on the true
    // distance: no maze consistent with what has been seen so far can do better.
    // Without it expansion is limited to visited cells, giving the distance the
    // robot could actually drive today.
    void distancesFrom(const Cell& source, bool throughUnknown, uint16_t out[N][N]) const {
        for (uint8_t x = 0; x < N; ++x) {
            for (uint8_t y = 0; y < N; ++y) out[x][y] = Unreachable;
        }
        if (!inside(source)) return;
        if (!throughUnknown && !visited(source)) return;

        Cell queue[MAX_CELLS];
        uint16_t head = 0;
        uint16_t tail = 0;

        out[source.x][source.y] = 0;
        queue[tail++]           = source;

        while (head < tail) {
            const Cell u        = queue[head++];
            const uint16_t step = static_cast<uint16_t>(out[u.x][u.y] + 1);

            for (uint8_t i = 0; i < 4; ++i) {
                const Direction d = directionFromIndex(i);
                const Cell next   = neighbour(u, d);
                if (!inside(next)) continue;
                if (hasWall(u, d)) continue;
                if (!throughUnknown && !visited(next)) continue;
                if (out[next.x][next.y] != Unreachable) continue;

                out[next.x][next.y] = step;
                queue[tail++]       = next;
            }
        }
    }

    // Marks every cell a shorter route to the goal could still run through, and
    // so decides how much of the maze is left worth driving into.
    //
    // A route through cell c is at least distStart(c) + distGoal(c) long, both
    // measured optimistically, so if that sum already matches the route the map
    // holds, c cannot improve on it. Prune every such cell and the sweep stops
    // driving into regions that cannot change the answer -- and runs out of
    // frontiers entirely once no cell can, which is exactly when the known route
    // is provably the shortest one the maze has.
    //
    // Soundness rests on the bounds being lower bounds. A route shorter than the
    // known one has to leave the visited region, and the first unvisited cell it
    // enters is reachable from a visited cell through a boundary that route
    // proves is open -- so that cell is a frontier rule 1 or 2 would offer, and
    // its two bounds sum to no more than that route's length. If nothing passes
    // the test, no such route exists.
    void refreshImproving() {
        uint16_t fromStart[N][N];
        uint16_t fromGoal[N][N];

        // Visited cells only: the shortest route the robot could drive right
        // now, which is the standard everything else is measured against.
        distancesFrom(start, false, fromStart);
        const uint16_t known = fromStart[goal.x][goal.y];

        // No route to beat yet, so nothing can be ruled out. This is the whole
        // sweep up until the goal is first reached.
        if (known == Unreachable) {
            for (uint8_t x = 0; x < N; ++x) {
                for (uint8_t y = 0; y < N; ++y) improving[x][y] = true;
            }
            return;
        }

        distancesFrom(start, true, fromStart);
        distancesFrom(goal, true, fromGoal);

        for (uint8_t x = 0; x < N; ++x) {
            for (uint8_t y = 0; y < N; ++y) {
                const uint16_t toHere  = fromStart[x][y];
                const uint16_t toThere = fromGoal[x][y];
                improving[x][y]        = toHere != Unreachable && toThere != Unreachable &&
                                  static_cast<uint16_t>(toHere + toThere) < known;
            }
        }
    }

    // What a travel leg is aiming for.
    enum class Target : uint8_t { Frontier, Start };

    // True for a cell planMove would travel to. A frontier cell is one with an
    // open side onto a cell the sweep has not stood in and still wants: somewhere
    // worth exploring to go on arrival. Every visited cell has all four sides
    // settled -- observe() reads three and the boundary the robot came in through
    // was cleared on the way -- so an open side here means a real opening, not one
    // nobody looked at.
    bool isTarget(Target what, const Cell& c) const {
        if (what == Target::Start) return sameCell(c, start);

        for (uint8_t i = 0; i < 4; ++i) {
            const Direction d = directionFromIndex(i);
            const Cell next   = neighbour(c, d);
            if (!inside(next)) continue;
            if (hasWall(c, d)) continue;
            if (visited(next)) continue;
            if (worthExploring(next)) return true;
        }
        return false;
    }

    // First move of the shortest route from the current cell to the nearest cell
    // isTarget accepts. Breadth-first, so the first candidate reached is a
    // nearest one, and limited to visited cells for the same reason
    // buildShortestPath is: an unvisited cell has no wall data, and treating its
    // four unknown sides as open would route straight through them.
    //
    // Cells are tagged with the first move that leads to them rather than with a
    // predecessor, because the answer is then ready the moment the target is
    // reached -- this runs once per arrival and only ever needs the next move,
    // never the whole path. False if no target is reachable.
    //
    // `hops` comes back as the length of that route in cells, which is what
    // homeProgress() is measured in. It rides along on the same search rather
    // than costing a second one: the sweep is two bytes per cell wider for it
    // (72 bytes at N = 6, 162 at N = 9) against the ~490 this already borrows.
    //
    // uint16_t and not a byte, though a byte holds every route a maze up to
    // 16 cells a side can contain: the width that overflows is not the one that
    // runs out of stack, so the cheap guard is the wider counter rather than a
    // bound on N nobody would think to check.
    bool firstStepToward(Target what, Direction& out, uint16_t& hops) const {
        constexpr uint8_t Unseen = 0xFF;
        uint8_t firstMove[N][N];
        uint16_t depth[N][N];
        Cell queue[MAX_CELLS];
        uint16_t head = 0;
        uint16_t tail = 0;
        hops          = 0;

        for (uint8_t x = 0; x < N; ++x) {
            for (uint8_t y = 0; y < N; ++y) {
                firstMove[x][y] = Unseen;
                depth[x][y]     = 0;
            }
        }

        // Marks the cell the robot is on as seen. Never read as a move: a
        // neighbour of it takes the direction stepped to reach it, and the cell
        // itself is never a target -- planMove has already ruled out both its
        // own unexplored neighbours and standing on the start cell by the time
        // it asks.
        firstMove[current.x][current.y] = 0;
        queue[tail++]                   = current;

        while (head < tail) {
            const Cell u          = queue[head++];
            const bool fromSource = sameCell(u, current);

            for (uint8_t i = 0; i < 4; ++i) {
                const Direction d = directionFromIndex(i);
                const Cell next   = neighbour(u, d);
                if (!inside(next)) continue;
                if (hasWall(u, d)) continue;
                if (!visited(next)) continue;
                if (firstMove[next.x][next.y] != Unseen) continue;

                const uint8_t step = fromSource ? directionIndex(d) : firstMove[u.x][u.y];
                firstMove[next.x][next.y] = step;
                depth[next.x][next.y]     = static_cast<uint16_t>(depth[u.x][u.y] + 1);
                if (isTarget(what, next)) {
                    out  = directionFromIndex(step);
                    hops = depth[next.x][next.y];
                    return true;
                }
                queue[tail++] = next;
            }
        }
        return false;
    }

    public:

    // Shortest route between any two explored cells. Breadth-first, so the
    // first time the target is dequeued it is on a shortest route. Expansion is
    // limited to visited cells: an unvisited cell has no wall data, and
    // treating its four unknown sides as open would plan straight through them.
    //
    // Predecessors are stored as a direction index per cell rather than a Cell
    // per cell, which is a byte instead of two and doubles as the seen flag.
    bool buildShortestPath(const Cell& from, const Cell& to) {
        finalPathLen = 0;
        if (!started || !inside(from) || !inside(to)) return false;
        if (!visited(from) || !visited(to)) return false;

        constexpr uint8_t Unseen = 0xFF;
        uint8_t cameFrom[N][N];
        Cell queue[MAX_CELLS];
        uint16_t head = 0;
        uint16_t tail = 0;

        for (uint8_t x = 0; x < N; ++x) {
            for (uint8_t y = 0; y < N; ++y) cameFrom[x][y] = Unseen;
        }

        // The start has a predecessor slot but no predecessor. Any value other
        // than Unseen marks it seen; the backward walk stops on the cell
        // itself, so this is never followed.
        cameFrom[from.x][from.y] = 0;
        queue[tail++]            = from;
        bool found               = sameCell(from, to);

        while (head < tail && !found) {
            const Cell u = queue[head++];
            for (uint8_t i = 0; i < 4; ++i) {
                const Direction d = directionFromIndex(i);
                const Cell next       = neighbour(u, d);
                if (!inside(next)) continue;
                if (hasWall(u, d)) continue;
                if (!visited(next)) continue;
                if (cameFrom[next.x][next.y] != Unseen) continue;

                cameFrom[next.x][next.y] = directionIndex(backOf(d));
                if (sameCell(next, to)) {
                    found = true;
                    break;
                }
                queue[tail++] = next;
            }
        }

        if (!found) return false;

        // Walk back once to measure, once to fill. Cheaper than a scratch
        // buffer the size of the maze, which is what the third path-sized
        // array here used to be.
        uint16_t length = 1;
        for (Cell p = to; !sameCell(p, from); ++length) {
            p = neighbour(p, directionFromIndex(cameFrom[p.x][p.y]));
        }

        finalPathLen   = length;
        uint16_t index = length;
        for (Cell p = to; !sameCell(p, from); p = neighbour(p, directionFromIndex(cameFrom[p.x][p.y]))) {
            finalPath[--index] = p;
        }
        finalPath[0] = from;
        return true;
    }

    private:

    uint8_t walls[N][N];
    uint8_t openBoundaries[N][N];
    bool visitedCells[N][N];

    // refreshImproving's verdict, one cell at a time, rebuilt on every planMove.
    // A member rather than a local because the frontier search consults it per
    // cell, and it is a byte a cell against the two distance fields it is
    // distilled from.
    bool improving[N][N];

    Cell start;
    Cell goal{0, 0};
    Cell current{0, 0};
    Direction facing      = North;
    Direction startFacing = North;

    Cell finalPath[MAX_CELLS];
    uint16_t finalPathLen = 0;

    uint16_t seenCells    = 0;
    Direction pendingMove = North;
    bool movePending      = false;
    bool explored         = false;
    bool started          = false;
    bool broken           = false;

    // The homing leg, as cells still to run and as the cells it started with.
    // Zero in homeHops means the sweep is not homing, which is why every exit
    // from planMove that is not rule 3 clears it; homeHops0 is the denominator
    // and survives until begin().
    uint16_t homeHops  = 0;
    uint16_t homeHops0 = 0;
};

#pragma GCC pop_options
