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

// Depth-first exploration of an unknown N x N maze, then a shortest route
// through what the exploration actually saw.
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
//     mapper.markWall(Cell{0, 0}, South);                  // priors, if any
//     while (!mapper.doneExploring()) {
//         mapper.observe(frontWall, leftWall, rightWall);  // at the current cell
//         Direction move;
//         if (!mapper.planMove(move)) break;
//         if (!driveOneCell(move)) break;                  // caller's problem
//         mapper.commitMove(move);                         // only once it arrived
//     }
//     if (mapper.faulted()) { /* map is suspect, do not race on it */ }
//     mapper.buildShortestPathToGoal();
//
// observe() only ever sees the three sides the sensors face, so the side the
// robot came in through is never sensed. That is fine everywhere except the
// start cell, which was never entered: its fourth side is unknown, and
// planMove is willing to drive backwards through it on the first move. Seed it
// with markWall before the loop if it is not a perimeter wall already.
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

    struct Cell {
        int8_t x;
        int8_t y;
    };

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
        stackTop     = 0;
        finalPathLen = 0;
        movePending  = false;
        explored     = false;
        broken       = false;
        seenCells    = 0;

        for (uint8_t x = 0; x < N; ++x) {
            for (uint8_t y = 0; y < N; ++y) {
                walls[x][y]          = 0;
                openBoundaries[x][y] = 0;
                visitedCells[x][y]   = false;
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

    // True once the mapper caught its own state going inconsistent -- a
    // desynced backtrack stack, or a commit it could not apply. Exploration
    // stops when this trips, which also raises doneExploring(), so the two have
    // to be read together: doneExploring() alone cannot tell a maze that was
    // fully swept from a run that gave up part way, and the map from the second
    // is not safe to race on.
    bool faulted() const { return broken; }

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

    // Walled on every side and never entered, so unreachable: a cell sealed by
    // sealCell, or one the discovered walls have closed off. Visited cells are
    // excluded because they cannot be sealed -- the boundary the robot came in
    // through was cleared on the way.
    bool sealedCell(const Cell& c) const {
        return !visited(c) && hasWall(c, North) && hasWall(c, South) && hasWall(c, West) &&
               hasWall(c, East);
    }

    // Picks the next move: an unexplored neighbour if there is one, otherwise
    // a step back along the route in. Prefers left, then straight, then right,
    // then behind, which keeps the mouse sweeping one way through open areas
    // instead of oscillating.
    //
    // Does not move the mapper. Call commitMove once the robot has arrived.
    bool planMove(Direction& move) {
        movePending = false;
        if (!started) return false;

        const Direction order[4] = {leftOf(facing), facing, rightOf(facing), backOf(facing)};
        for (uint8_t i = 0; i < 4; ++i) {
            const Direction d    = order[i];
            const Cell next = neighbour(current, d);
            if (!inside(next)) continue;
            if (hasWall(current, d)) continue;
            if (visited(next)) continue;
            return setPending(move, d, false);
        }

        if (stackTop > 0) {
            Direction back;
            // The top of the stack is the cell this one was entered from, so
            // it is always adjacent. If it somehow is not, the stack has
            // desynced and driving the move would corrupt the map: stop
            // exploring instead, and flag it so the caller does not read the
            // halt as a finished sweep.
            if (!directionTo(current, stack[stackTop - 1], back)) {
                broken   = true;
                explored = true;
                return false;
            }
            return setPending(move, back, true);
        }

        explored = true;
        return false;
    }

    // Applies the move planMove returned, once the robot has physically made
    // it. Rejects anything else, so a dropped, repeated or invented move
    // cannot silently desync the backtrack stack.
    bool commitMove(Direction move) {
        if (!movePending || move != pendingMove) return false;
        movePending = false;

        // Neither of these can fire off a move planMove actually returned: it
        // only offers in-range cells, and the stack is bounded by the cell
        // count. They are here because the robot has already physically moved
        // by this point, so failing quietly would leave the map tracking a
        // position the robot is not in.
        const Cell next = neighbour(current, move);
        if (!inside(next)) {
            broken = true;
            return false;
        }

        if (pendingBacktrack) {
            if (stackTop == 0) {
                broken = true;
                return false;
            }
            --stackTop;
        } else if (!push(current)) {
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

    // Shortest route from the start cell to the goal through explored cells
    // only, so it never plans through a boundary that was never looked at.
    //
    // From the start rather than from wherever the robot is now, because a
    // finished sweep unwinds the backtrack stack all the way back, leaving the
    // robot on the start cell. Use buildShortestPath if the run stopped early
    // -- on atGoal(), say -- or to route home from where the robot stands.
    bool buildShortestPathToGoal() { return buildShortestPath(start, goal); }

    uint16_t shortestPathLength() const { return finalPathLen; }

    bool shortestPathCell(uint16_t index, Cell& cell) const {
        if (index >= finalPathLen) return false;
        cell = finalPath[index];
        return true;
    }

    // Renders the shortest path as the "flrf" string PSPlanner::addInstructions
    // consumes, starting from startHeading. This is the whole handoff between
    // the two classes, and it is short only because they share a convention.
    //
    // False on truncation rather than a quietly shortened route.
    //
    // The no-heading overload assumes the path starts where the robot did and
    // uses the heading begin() was given -- right for buildShortestPathToGoal,
    // wrong for a route built from anywhere else, which has to pass its own.
    bool toInstructions(etl::string<MAZE_INSTRUCTION_MAX_LEN>& out) const {
        return toInstructions(out, startFacing);
    }

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

    bool setPending(Direction& move, Direction d, bool backtrack) {
        move             = d;
        pendingMove      = d;
        pendingBacktrack = backtrack;
        movePending      = true;
        return true;
    }

    bool push(const Cell& cell) {
        if (stackTop >= MAX_CELLS) return false;
        stack[stackTop++] = cell;
        return true;
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

    Cell start;
    Cell goal{0, 0};
    Cell current{0, 0};
    Direction facing      = North;
    Direction startFacing = North;

    // Route in, one entry per cell stepped forward into, popped on the way
    // back out. Empty means the depth-first search has unwound to the start
    // and every reachable cell has been seen.
    Cell stack[MAX_CELLS];
    uint16_t stackTop = 0;

    Cell finalPath[MAX_CELLS];
    uint16_t finalPathLen = 0;

    uint16_t seenCells    = 0;
    Direction pendingMove = North;
    bool pendingBacktrack = false;
    bool movePending      = false;
    bool explored         = false;
    bool started          = false;
    bool broken           = false;
};

using mazeMapper = MazeMapper<MAZE_SIZE>;

#pragma GCC pop_options
