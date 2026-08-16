// One continuous run: explore, plan, race
//
// Zimmy Levi z5587840

#pragma once

#include <array>

#include <Arduino.h>

#include <Embedded_Template_Library.h>
#include <etl/algorithm.h>
#include <etl/span.h>
#include <etl/string.h>

#include "constants.h"
#include "lidar.h"
#include "mazeMapper.h"
#include "planners.h"
#include "types.h"

#pragma GCC push_options
#pragma GCC optimize("O2")

// Drives MazeMapper closed loop against the robot, in one pass:
//
//     Init -> Explore -> Plan -> Race -> Done
//
// Non-blocking throughout. update() is called once per control tick and hands
// back a desired velocity, so the sketch's loop() stays a wiring diagram: read
// the pose, ask the runner what to do, give it to the controller, draw.
//
// It owns the mapper, the run state and the discovered route, and borrows the
// lidar and the planner -- the planner because its gains are tuning, and
// tuning belongs in the sketch with the rest of the configuration.
//
// The one rule this class exists to enforce is MazeMapper's: planMove only
// reads and commitMove only writes, and a move planned but not driven desyncs
// the backtrack stack, which then hands a non-adjacent cell pair to the
// wall-clearing code and erases real walls. So a planned move is held here for
// exactly as long as it is in flight, and committed once, on arrival.
template <size_t N>
class MazeRunner {
    public:

    using Cell = typename MazeMapper<N>::Cell;

    enum class State : uint8_t { Init, Explore, Plan, Race, Done };

    MazeRunner(
        LIDAR& lidar,
        PSPlanner& planner,
        const Cell& startCell,
        Direction startHeading,
        const Cell& goalCell
    ) :
        lidar(lidar), planner(planner), mapper(startCell, startHeading, goalCell) {}

    // Seeds the mapper and every prior it needs, then opens exploration.
    // False -- and the run parked in Done -- if the mapper rejects the start
    // or goal cell.
    //
    // Priors have to land here, before the first observe(): markWall refuses a
    // boundary the robot has already driven through, and a corner sealed after
    // the search had planned into it would be too late anyway.
    bool begin() {
        runState = State::Done;
        if (!mapper.begin()) return false;

        const Cell start = mapper.startPosition();

        croppedCells   = sealCroppedCells();
        reachableCells = static_cast<uint16_t>(MazeMapper<N>::MAX_CELLS - croppedCells);

        // observe() only ever sees the three sides the sensors face, so the
        // side the robot came in through is never sensed -- and the start cell
        // was never entered. Without this planMove will reverse out of it on
        // the first move.
        mapper.markWall(start, backOf(mapper.startHeading()));

        moveInFlight = false;
        routeLen     = 0;
        runState     = State::Explore;
        return true;
    }

    Velocity update(const Pose& pose, float dt) {
        switch (runState) {
            case State::Explore: return explore(pose, dt);
            case State::Plan:    plan(); return Velocity{0, 0};
            case State::Race:    return race(pose, dt);
            default:             return Velocity{0, 0};
        }
    }

    State state() const { return runState; }

    // True once the run is driving a planned route rather than discovering
    // one. The sketch selects which display to draw from this.
    bool racing() const { return runState == State::Race || runState == State::Done; }

    const MazeMapper<N>& map() const { return mapper; }

    uint16_t cropped() const { return croppedCells; }

    uint16_t reachable() const { return reachableCells; }

    // The discovered route in millimetres, one point per cell, empty until
    // Plan has run.
    etl::span<const Vec2D> route() const {
        return etl::span<const Vec2D>(routePoints.data(), routeLen);
    }

    // Still under-reads whenever part of the maze is walled off, because the
    // reachable count is not known until the sweep finishes. That is why it is
    // a meter and not a completion test -- doneExploring() is the test. The
    // cropped corners are the part that is known up front, and leaving them in
    // the denominator would peg a finished sweep at 86%.
    float exploreProgress() const {
        if (reachableCells == 0) return 0.0f;
        return static_cast<float>(mapper.visitedCount()) / static_cast<float>(reachableCells);
    }

    // One notch per instruction. setStart contributes a pose of its own, so a
    // route of k instructions has len() == k + 1.
    float raceProgress() const {
        if (planner.len() <= 1) return 0.0f;
        return static_cast<float>(planner.idx()) / static_cast<float>(planner.len() - 1);
    }

    // True for a cell the physical maze does not have. Every corner is
    // chamfered, so a cell within MAZE_CORNER_CROP steps of a corner --
    // Manhattan, measured from the corner cell -- is missing. At 1 that is the
    // corner plus its two orthogonal neighbours: three cells per corner,
    // twelve in all.
    static bool croppedCell(int x, int y) {
        const int dx = etl::min<int>(x, static_cast<int>(N) - 1 - x);
        const int dy = etl::min<int>(y, static_cast<int>(N) - 1 - y);
        return (dx + dy) <= static_cast<int>(MAZE_CORNER_CROP);
    }

    private:

    LIDAR& lidar;
    PSPlanner& planner;
    MazeMapper<N> mapper;

    State runState        = State::Init;
    bool moveInFlight     = false;
    Direction pendingMove = North;

    uint16_t croppedCells   = 0;
    uint16_t reachableCells = MazeMapper<N>::MAX_CELLS;

    std::array<Vec2D, MazeMapper<N>::MAX_CELLS> routePoints;
    uint16_t routeLen = 0;

    uint16_t sealCroppedCells() {
        uint16_t n = 0;
        for (int8_t x = 0; x < static_cast<int8_t>(N); ++x) {
            for (int8_t y = 0; y < static_cast<int8_t>(N); ++y) {
                if (!croppedCell(x, y)) continue;
                mapper.sealCell(Cell{x, y});
                ++n;
            }
        }
        return n;
    }

    Velocity explore(const Pose& pose, float dt) {
        if (!moveInFlight) {
            stepExploration();
            return Velocity{0, 0};
        }

        const Velocity desired = planner.update(pose, dt);
        if (!planner.done()) return desired;

        // Only now, and exactly once: the robot has physically arrived, which
        // is the whole precondition commitMove has.
        if (!mapper.commitMove(pendingMove)) {
            Serial.println(F("COMMIT REJECTED"));
            runState = State::Done;
        } else {
            trace("commit", pendingMove);
        }
        moveInFlight = false;
        return desired;
    }

    // Reads the three lidars, folds them into the map, and starts the next
    // move. Only ever called with no move in flight.
    void stepExploration() {
        if (mapper.faulted() || mapper.doneExploring()) {
            runState = State::Plan;
            return;
        }

        // LidarObserver::update() already refreshed these inside sf.update()
        // this tick, so the readings are current and re-reading the bus would
        // only cost the control loop time.
        const bool front = lidar.getReading(LIDAR::Front) < WALL_THRESHOLD_MM;
        const bool left  = lidar.getReading(LIDAR::Left) < WALL_THRESHOLD_MM;
        const bool right = lidar.getReading(LIDAR::Right) < WALL_THRESHOLD_MM;

        mapper.observe(front, left, right);
        Serial.print(F("obs f/l/r "));
        Serial.print(front);
        Serial.print(left);
        Serial.println(right);

        Direction move;
        if (!mapper.planMove(move)) {
            runState = State::Plan;
            return;
        }

        trace("plan", move);
        if (!beginMove(move)) {
            Serial.println(F("PLANNER REJECTED THE MOVE"));
            runState = State::Done;
            return;
        }

        pendingMove  = move;
        moveInFlight = true;
    }

    // Turns the mapper's chosen heading into instructions from the heading the
    // robot is actually on, and hands them to the planner as a one-cell route.
    //
    // appendTurns is the same rule MazeMapper::toInstructions renders a whole
    // path with, which is the point of sharing it: the heading the mapper
    // tracks and the one the robot is driven onto cannot drift apart.
    bool beginMove(Direction move) {
        etl::string<MAZE_INSTRUCTION_MAX_LEN> ins;
        Direction d = mapper.heading();
        if (!appendTurns(ins, d, move)) return false;
        ins += 'f';

        const Cell c = mapper.position();
        const Cell origin = mapper.startPosition();
        if (!planner.setStart(cellToGridPose(c.x, c.y, mapper.heading(), origin.x, origin.y))) return false;
        return planner.addInstructions(ins);
    }

    // Builds the route the exploration earned and re-seeds the planner with it.
    void plan() {
        if (mapper.faulted()) {
            // A fault also raises doneExploring(), so the two have to be read
            // together. The map behind a fault is not safe to race on.
            Serial.println(F("MAPPER FAULTED -- not racing"));
            runState = State::Done;
            return;
        }

        // No arguments: a completed sweep unwinds the backtrack stack all the
        // way out, so the robot is standing on the start cell and the route
        // starts where it does.
        if (!mapper.buildShortestPathToGoal()) {
            Serial.println(F("NO ROUTE TO GOAL"));
            runState = State::Done;
            return;
        }

        etl::string<MAZE_INSTRUCTION_MAX_LEN> instructions;
        if (!mapper.toInstructions(instructions)) {
            Serial.println(F("ROUTE TOO LONG"));
            runState = State::Done;
            return;
        }

        Serial.print(F("route ("));
        Serial.print(mapper.shortestPathLength());
        Serial.print(F(" cells): "));
        Serial.println(instructions.c_str());

        // Cells to millimetres, so the display can overlay the route without
        // knowing anything about the mapper or the maze size.
        routeLen = 0;
        for (uint16_t i = 0; i < mapper.shortestPathLength(); ++i) {
            Cell c;
            if (!mapper.shortestPathCell(i, c)) break;
            const Cell origin = mapper.startPosition();
            routePoints[routeLen++] = cellToWorld(c.x, c.y, origin.x, origin.y);
        }

        const Cell s = mapper.startPosition();
        const Cell origin = mapper.startPosition();
        if (!planner.setStart(cellToGridPose(s.x, s.y, mapper.startHeading(), origin.x, origin.y)) ||
            !planner.addInstructions(instructions)) {
            Serial.println(F("PLANNER REJECTED THE ROUTE"));
            runState = State::Done;
            return;
        }

        runState = State::Race;
        Serial.println(F("racing"));
    }

    Velocity race(const Pose& pose, float dt) {
        const Velocity desired = planner.update(pose, dt);
        if (planner.done()) {
            Serial.println(F("done"));
            runState = State::Done;
        }
        return desired;
    }

    void trace(const char* what, Direction d) {
        Serial.print(what);
        Serial.print(F(" @ "));
        Serial.print(mapper.position().x);
        Serial.print(',');
        Serial.print(mapper.position().y);
        Serial.print(F(" -> "));
        Serial.println(directionChar(d));
    }
};

#pragma GCC pop_options
