// Task 4.3 -- explore, plan, race
//
// Zimmy Levi z5587840
//
// Selected by TASK in micromouse.ino, which includes this file part way down
// the sketch rather than up with the other includes: everything here is built
// from lidar, obs_v, dt and display, so it has to come after them. The sketch
// is one translation unit, so those names are already in scope and this header
// declares no hardware of its own.
//
// Its counterpart is task42.h. Both declare the same names -- lidar_obsv,
// obs_p, sf, fusedPose(), and the three task hooks below -- so setup() and
// loop() are written once and neither is wrapped in an #if.
//
// The robot is given a start cell, a start heading and a goal, and nothing
// else. It explores until the goal is reachable, plans a route over what it
// found, and drives it. The observer localises against those discovered walls:
// 4.3 has no photograph of the maze -- finding it is the exercise -- so
// MazeWallMap stands in for the exported map 4.1/4.2 uses. LidarObserver is
// templated on the map type and MazeWallMap offers Map's cast()/candidates(),
// so the same observer serves both.

#pragma once

#include <array>

#include <Embedded_Template_Library.h>
#include <etl/delegate.h>

#include "constants.h"
#include "mazeMapper.h"
#include "mazeRunner.h"
#include "mazeWallMap.h"
#include "observers.h"
#include "oled.h"
#include "oledMap.h"
#include "oledPath.h"
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
constexpr uint8_t MAZE_SIZE = 5;

using mazeMapper = MazeMapper<MAZE_SIZE>;

// The complete maze configuration: the runner is handed it at construction and
// taskBegin() only has to start it.
//
// Cells are the grid convention from types.h -- (x, y) with x forward and y
// left, so North steps +x and West steps +y. Start in a corner facing North,
// goal at (2, 4).
PSPlanner psp(8.0f, 8.0f);
mazeMapper::Cell startCell = {0, 0};
Direction startHeading     = North;
mazeMapper::Cell goalCell  = {2, 4};

MazeRunner<MAZE_SIZE> runner(
    lidar,
    psp,
    startCell,
    startHeading,
    goalCell
);

MazeWallMap<MAZE_SIZE> wallMap(runner.map());
LidarObserver<MazeWallMap<MAZE_SIZE>> lidar_obsv(lidar, wallMap);

const std::array<PoseSource, 1> obs_p = {{
    {&lidar_obsv, FusionWeights::XYPTrust}
}};

SensorFusion sf(obs_v, obs_p, 0.1);

Pose fusedPose() { return sf.estimate.pose(); }

// Delegates, so neither display depends on the runner's type.
float exploreProgress() {
    return runner.exploreProgress();
}

float raceProgress() {
    return runner.raceProgress();
}

OLEDMap<MAZE_SIZE> oledMap(display, runner.map(), etl::delegate<float()>::create<exploreProgress>());

OLEDPath<MazeWallMap<MAZE_SIZE>> oledPath(
    display,
    wallMap,
    etl::delegate<Pose()>::create<fusedPose>(),
    etl::delegate<float()>::create<raceProgress>()
);

// Kept constructed and available for bring-up, but not driven by taskRender():
// OLEDDisplay::due() is consuming, so only one renderer may draw per tick.
const std::array values = {
    OLEDValue{"x", []() { return sf.estimate.pose().x; }},
    OLEDValue{"y", []() { return sf.estimate.pose().y; }},
    OLEDValue{"th", []() { return sf.estimate.pose().theta; }},
    OLEDValue{"dt", []() { return dt; }},
    OLEDValue{"pgr", []() { return raceProgress(); }},
    OLEDValue{"sta",
              []() {
                return static_cast<float>(static_cast<uint8_t>(runner.state()));
              }},
    OLEDValue{"bms", []() { return static_cast<float>(lidar_obsv.beams()); }},
};

OLEDValues oled(display, values);

// Called from setup(), after the shared bring-up and under its "Loading
// goal..." print, which this is expected to terminate.
void taskBegin() {
    if (!runner.begin()) {
        Serial.println("\b\b\b [MAZE RUNNER REJECTED START OR GOAL]");
    } else {
        Serial.println("\b\b\b [OKAY]");
    }

    // After runner.begin(), which seeds the perimeter -- the extent the map
    // pane is fitted to. Walls found later fall inside it, so one fit holds
    // for the whole run.
    Serial.print("Fitting map to display...");
    if (!oledPath.init()) {
        Serial.println("\b\b\b [MAP DID NOT FIT]");
    } else {
        Serial.println("\b\b\b [OKAY]");
    }
}

// Explore, plan, race. Non-blocking, and it commands zero once done, so
// nothing downstream has to special-case a stopped robot.
Velocity taskUpdate(const Pose& pose, float dt) {
    return runner.update(pose, dt);
}

// One renderer per tick: OLEDDisplay::due() is consuming, so drawing two would
// starve whichever asked second.
void taskRender() {
    if (runner.racing()) {
        oledPath.setRoute(runner.route());
        oledPath.update();
    } else {
        oledMap.update();
    }
}
