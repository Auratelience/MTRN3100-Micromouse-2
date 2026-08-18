// One continuous run: explore, plan, race
//
// Zimmy Levi z5587840

#pragma once

#include <array>

#include <Arduino.h>

#include <Embedded_Template_Library.h>
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
    // boundary the robot has already driven through.
    bool begin() {
        runState = State::Done;
        if (!mapper.begin()) return false;

        const Cell start = mapper.startPosition();

        // The start cell's fourth side is the one thing the sensors cannot
        // settle from where they sit: observe() reads front, left and right, and
        // the robot never drove in through the back. So look at it -- see
        // stepExploration -- rather than assume a wall is there, which would
        // invent one in the middle of the maze on any run that does not start
        // against a boundary, and keep it for the rest of the run: observe()
        // only ever adds walls, and only a driven boundary is ever cleared.
        //
        // Unless the perimeter settled it already, which it has whenever the
        // start cell backs onto the edge of the maze.
        lookBehind = !mapper.hasWall(start, backOf(mapper.startHeading()));

        moveInFlight = false;
        turnInFlight = false;
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

    // The discovered route in millimetres, one point per cell, empty until
    // Plan has run.
    etl::span<const Vec2D> route() const {
        return etl::span<const Vec2D>(routePoints.data(), routeLen);
    }

    // Fraction of the grid the sweep has stood in. Under-reads whenever part of
    // the maze is walled off or missing, because how many cells are actually
    // reachable is not known until the sweep finishes. That is why it is a
    // meter and not a completion test -- doneExploring() is the test.
    float exploreProgress() const {
        return static_cast<float>(mapper.visitedCount()) /
               static_cast<float>(MazeMapper<N>::MAX_CELLS);
    }

    // One notch per instruction. setStart contributes a pose of its own, so a
    // route of k instructions has len() == k + 1.
    float raceProgress() const {
        if (planner.len() <= 1) return 0.0f;
        return static_cast<float>(planner.idx()) / static_cast<float>(planner.len() - 1);
    }

    private:

    LIDAR& lidar;
    PSPlanner& planner;
    MazeMapper<N> mapper;

    State runState        = State::Init;
    bool moveInFlight     = false;
    Direction pendingMove = North;

    // What is in flight is a rotation on the spot rather than a step, so it
    // commits as a turn. Only ever set for the look behind at the start.
    bool turnInFlight = false;

    // The look behind is still owed. Cleared the moment it is started, so it
    // happens once even though stepExploration runs every arrival.
    bool lookBehind = false;

    std::array<Vec2D, MazeMapper<N>::MAX_CELLS> routePoints;
    uint16_t routeLen = 0;

    Velocity explore(const Pose& pose, float dt) {
        if (!moveInFlight) {
            stepExploration();
            return Velocity{0, 0};
        }

        const Velocity desired = planner.update(pose, dt);
        if (!planner.done()) return desired;

        // Only now, and exactly once: the robot has physically arrived, which
        // is the whole precondition commitMove and commitTurn share.
        const bool committed = turnInFlight ? mapper.commitTurn(pendingMove)
                                            : mapper.commitMove(pendingMove);
        if (!committed) {
            Serial.println(F("COMMIT REJECTED"));
            runState = State::Done;
        } else {
            trace(turnInFlight ? "turned" : "commit", pendingMove);
        }
        moveInFlight = false;
        turnInFlight = false;
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

        // Before the first move is ever planned, and only then: turn to put a
        // sensor on the one side of the start cell nobody has looked at.
        //
        // Ninety degrees is enough, and left is as good as right -- after a left
        // turn the left sensor points where the back did, since turning left
        // twice is turning around. The reading lands in the observe() above on
        // the next pass, with no special case: from the new heading it is simply
        // what is on the left.
        if (lookBehind) {
            lookBehind = false;
            const Direction look = leftOf(mapper.heading());
            trace("look", look);
            if (!beginTurn(look)) {
                Serial.println(F("PLANNER REJECTED THE LOOK"));
                runState = State::Done;
                return;
            }
            pendingMove  = look;
            moveInFlight = true;
            turnInFlight = true;
            return;
        }

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

    // beginMove without the step: the turns alone, leaving the robot facing a
    // new way in the cell it is already in. Commit it with commitTurn, not
    // commitMove -- nothing has moved.
    bool beginTurn(Direction to) {
        etl::string<MAZE_INSTRUCTION_MAX_LEN> ins;
        Direction d = mapper.heading();
        if (!appendTurns(ins, d, to)) return false;

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

        // No arguments: a completed sweep routes itself home, so the robot is
        // standing on the start cell and the route starts where it does.
        if (!mapper.buildShortestPathToGoal()) {
            Serial.println(F("NO ROUTE TO GOAL"));
            runState = State::Done;
            return;
        }

        // From heading(), not startHeading(). The robot is back on the start
        // cell but not on the heading it began the run on: the last step of the
        // route home is whatever direction it came in from.
        //
        // The route survives getting this wrong -- the turns are relative and
        // setStart replays them from the same heading, so the poses land on the
        // same cells facing the same way either way. What it costs is the first
        // move: claim a heading the robot is not on and the opening rotation is
        // missing from the sequence, so instead of turning on the spot and
        // driving straight the robot slews into the first cell from whatever
        // angle it was left at. It also makes the instruction string printed
        // below a description of a run nobody made.
        etl::string<MAZE_INSTRUCTION_MAX_LEN> instructions;
        if (!mapper.toInstructions(instructions, mapper.heading())) {
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
        if (!planner.setStart(cellToGridPose(s.x, s.y, mapper.heading(), origin.x, origin.y)) ||
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
