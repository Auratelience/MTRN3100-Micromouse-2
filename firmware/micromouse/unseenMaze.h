// Unseen Maze -- explore, plan, race
//
// Zimmy Levi z5587840
//
// Included by micromouse.ino part way down the sketch rather than up with the
// other headers: everything here is built from lidar, obs_v, dt and display,
// so it has to come after them. The sketch is one translation unit, so those
// names are already in scope and this header declares no hardware of its own.
//
// The robot is given a maze size, a start cell, a start heading and a goal, all
// chosen at boot through startupUI.h, and nothing else. It explores until the
// goal is reachable, plans a route over what it found, and drives it. The
// observer localises against those discovered walls: there is no photograph of
// the maze -- finding it is the exercise -- so MazeWallMap stands in for an
// exported map. LidarObserver is templated on the map type and MazeWallMap
// offers Map's cast()/candidates(), so one observer serves both.

#pragma once

#include <array>

#include <Embedded_Template_Library.h>
#include <etl/delegate.h>

#include "constants.h"
#include "mazeMapper.h"
#include "mazeRunner.h"
#include "mazeWallMap.h"
#include "observers.h"
#include "oledScreen.h"
#include "planners.h"
#include "sensorFusion.h"
#include "types.h"

// Cells per side, and so the size everything below instantiates MazeMapper<>
// at, the same way TRIG_LUT_SIZE sizes trig. The classes are templated on it,
// so a test or a second instance can pick another size without touching a
// header.
//
// Cost grows as N^2: at 9 the mapper holds ~500 bytes, and the deepest of its
// breadth-first searches -- the frontier pruning, which wants two distance
// fields and a queue at once -- borrows ~490 bytes of stack. At 16 that is
// ~1.5 kB held and ~1.5 kB borrowed. The searches run one at a time, so the
// borrow does not stack.
//
// Five is a smaller maze than the competition deck, which is 9 cells a side --
// maze_map.h describes its 10x10 post lattice, and ten post lines bound nine
// cells, with cell centres from -180 mm to 1260 mm on both axes. Set this to 9
// to explore the full deck; nothing else has to change.
constexpr uint8_t MAZE_SIZE = 9;

using mazeMapper = MazeMapper<MAZE_SIZE>;
// The complete maze configuration: the runner is handed it at construction and
// runBegin() only has to start it.
//
// Cells are the grid convention from types.h -- (x, y) with x forward and y
// left, so North steps +x and West steps +y. Start in a corner facing North,
// goal at (2, 4).
PSPlanner psp(8.0f, 10.0f);
Cell startCell = {1, 1};
Direction startHeading = North;
Cell goalCell  = {5, 5};

MazeRunner<MAZE_SIZE> runner(lidar, psp, startCell, startHeading, goalCell);

MazeWallMap<MAZE_SIZE> wallMap(runner.mapper);

LidarObserver<MazeWallMap<MAZE_SIZE>> lidar_obsv(lidar, wallMap);

// The lidar for position, the gyro for heading, and neither for the other.
//
// XYPTrust on the lidar for the reason set out over LidarObserver: three beams
// in a corridor constrain y and theta only in combination, so a lateral offset
// comes back partly as rotation, and that heading is worse than the gyro's.
//
// ThetaPTrust on the IMU because heading is all it has -- the x and y in the
// Pose it returns are placeholders. This is the axis that had no absolute
// reference at all before: theta was dead reckoning from the fused omega, with
// nothing on the pose side weighted to correct it, so every error in that omega
// integrated for the whole run with nothing to pull it back.
const std::array<PoseSource, 2> obs_p = {{
    {&lidar_obsv, FusionWeights::XYPTrust},
    {&imu_obsv, FusionWeights::ThetaPTrust}
}};

SensorFusion sf(obs_v, obs_p, 0.1f, FusionWeights::ThetaCorrectionGain);

Pose fusedPose() { return sf.estimate.pose(); }

using runnerState = MazeRunner<MAZE_SIZE>::State;

// The phase of the run, in four letters.
//
// HOME is not one of MazeRunner's states -- the leg back to the start cell is
// rule 3 of MazeMapper::planMove and lives inside Explore -- but it is the one
// stretch where the robot is retracing rather than discovering, and reporting it
// as EXPL would leave the display claiming to explore for the whole return trip.
//
// FAULT is kept distinct from DONE deliberately. A mapper that catches its own
// state going inconsistent stops the run and reports Done like a finished sweep,
// so a display that folded the two together would show a run that gave up as a
// completed one -- which is exactly the failure this screen exists to catch.
const char* screenMode() {
    switch (runner.state()) {
        case runnerState::Init:    return "INIT";
        case runnerState::Explore: return runner.mapper.homing() ? "HOME" : "EXPL";
        case runnerState::Plan:    return "PLAN";
        case runnerState::Race:    return "EXEC";
        default:                   return runner.mapper.faulted() ? "FAULT" : "DONE";
    }
}

// What the percentage counts, which depends on what the robot is doing:
//
//   EXPL  cells of the grid stood in. Under-reads whenever part of the maze is
//         walled off, because how much is reachable is not known until the
//         sweep ends -- a meter, not a completion test.
//   HOME  distance run of the distance home when the leg began, one notch per
//         cell. Not the planner's own progress: the runner re-seeds it per cell
//         during exploration, so that figure resets every step.
//   else  poses driven of the poses in the route, one notch per instruction.
OLEDMetric screenMetric() {
    if (runner.state() == runnerState::Explore) {
        if (runner.mapper.homing()) return OLEDMetric{'P', runner.mapper.homeProgress()};
        return OLEDMetric{'E', runner.exploreProgress()};
    }
    return OLEDMetric{'P', runner.raceProgress()};
}

// The discovered map, drawn as geometry for the whole run: walls appear as
// panels the moment the mapper records them, and the robot's pose and heading
// are on screen from the first tick rather than only once it starts racing.
//
// wallMap and not runner.map() -- the same walls, but as the panels and posts
// the lidar localises against, so what is drawn is the geometry the fix was
// taken from.
OLEDScreen<MazeWallMap<MAZE_SIZE>> screen(
    display,
    wallMap,
    etl::delegate<Pose()>::create<fusedPose>(),
    etl::delegate<const char*()>::create<screenMode>(),
    etl::delegate<OLEDMetric()>::create<screenMetric>()
);

// Called from setup(), after the shared bring-up.
void runBegin() {
    if (!runner.begin()) {
        Serial.println("\b\b\b [MAZE RUNNER REJECTED START OR GOAL]");
    } else {
        Serial.println("\b\b\b [OKAY]");
    }

    // After runner.begin(), which seeds the perimeter -- the extent the map
    // pane is fitted to. Walls found later fall inside it, so one fit holds
    // for the whole run.
    Serial.print("Fitting map to display...");
    if (!screen.init()) {
        Serial.println("\b\b\b [MAP DID NOT FIT]");
    } else {
        Serial.println("\b\b\b [OKAY]");
    }
}

// Explore, plan, race. Non-blocking, and it commands zero once done, so
// nothing downstream has to special-case a stopped robot.
Velocity runUpdate(const Pose& pose, float dt) {
    return runner.update(pose, dt);
}

// One screen for every phase, so there is no renderer to select and no risk of
// starving one: OLEDDisplay::due() is consuming, and only one thing draws.
//
// The route is handed over unconditionally. It is empty until Plan has run, and
// a route shorter than two points draws nothing, so this needs no idea of which
// phase the run is in.
void runRender() {
    screen.setRoute(runner.route());
    screen.update();
}
