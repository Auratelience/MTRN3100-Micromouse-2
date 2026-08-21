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
#include "selfCheck.h"
#include "sensorFusion.h"
#include "types.h"

// Everything below is instantiated at the capacity rather than at the maze
// being run. MAZE_SIZE_MAX is how much room the buffers have; the grid actually
// in use is a runtime value the mapper carries, set by runner.configure(), so
// this binary handles any deck from MAZE_SIZE_MIN up without a recompile.
using mazeMapper = MazeMapper<MAZE_SIZE_MAX>;

PSPlanner psp(8.0f, 10.0f);

// Unconfigured until runBegin() says otherwise: a mapper with no grid refuses
// begin(), so there is no half-set-up run to trip over in between.
MazeRunner<MAZE_SIZE_MAX> runner(lidar, psp);

MazeWallMap<MAZE_SIZE_MAX> wallMap(runner.mapper);

LidarObserver<MazeWallMap<MAZE_SIZE_MAX>> lidar_obsv(lidar, wallMap);

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

using runnerState = MazeRunner<MAZE_SIZE_MAX>::State;

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
        case runnerState::Explore: return runner.homing() ? "HOME" : "EXPL";
        case runnerState::Plan:    return "PLAN";
        case runnerState::Race:    return "EXEC";
        default:                   return runner.faulted() ? "FAULT" : "DONE";
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
        if (runner.homing()) return OLEDMetric{'P', runner.homeProgress()};
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
OLEDScreen<MazeWallMap<MAZE_SIZE_MAX>> screen(
    display,
    wallMap,
    etl::delegate<Pose()>::create<fusedPose>(),
    etl::delegate<const char*()>::create<screenMode>(),
    etl::delegate<OLEDMetric()>::create<screenMetric>()
);

// Called from setup(), after the shared bring-up and after runner.configure()
// -- the wizard's size, start cell, start heading and goal are already
// latched in by the time this runs, so all that is left is to seed the
// perimeter and fit the display to it.
void runBegin() {
#ifdef MICROMOUSE_DEBUG
    runSelfChecks(runner.mapper, wallMap);
#endif

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
