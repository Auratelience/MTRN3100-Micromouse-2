# Startup UI and Runtime Maze Configuration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drop task 4.2, rename 4.3 to "Unseen Maze", and move maze size, start cell, start heading and goal from compile-time constants to a boot wizard driven by the wheel encoders and side lidars.

**Architecture:** `MazeMapper`/`MazeWallMap`/`MazeRunner` keep their template parameter `N` as an *array capacity* (16) and gain a *runtime* grid size `n`. A new blocking `startupUI.h` collects a `RunConfig` in `setup()` before the IMU calibrates, and hands it to `runner.configure()`. A `--debug` build flag gates a boot self-check and the existing loop diagnostics.

**Tech Stack:** C++11/14 header-only Arduino sketch, `arduino:renesas_uno:nanor4` (RA4M1, 32 kB SRAM), Adafruit_SSD1306 + Adafruit_GFX, Embedded Template Library (ETL). Built with `./compile.sh`.

**Spec:** `docs/superpowers/specs/2026-08-20-startup-ui-design.md`

## Global Constraints

- Firmware version string is exactly `v2.0`.
- Board is `arduino:renesas_uno:nanor4`. The only build command is `./compile.sh` (~15 s) from the repo root; it wipes `firmware/build` on purpose. Never reuse the build directory.
- clangd diagnostics in the editor are unreliable in this tree (phantom `'array' file not found` cascades). **Only `./compile.sh` decides whether something builds.**
- Header-only. Every file is part of one translation unit; the sketch has no `.cpp` files. Free functions defined in headers need `inline`, and header-only classes carry the existing `// NOLINTBEGIN(misc-definitions-in-headers)` pattern where the codebase already uses it.
- Style: 4-space indent, access specifiers indented one level (`    public:`), comments explain *why* not *what*, and existing comment blocks are preserved unless the change makes them false.
- `MAZE_SIZE_MIN = 2`, `MAZE_SIZE_MAX = 16`.
- Grid convention (`types.h:374-380`): x forward, y left. `Direction : int { North = 0, West = 1, South = 2, East = -1 }` — **East is −1**, so directions cannot be walked by integer increment.
- World frame is maze-aligned: origin at the start cell, North = theta 0.
- There is no unit test harness for this firmware. Verification is `./compile.sh`, the boot self-check, and bench testing. Do not invent a test framework.

---

### Task 1: Drop task 4.2, rename 4.3 to Unseen Maze

Pure rename and deletion, no behaviour change. Isolated first so the runtime-size work lands in a tree with one task header instead of two.

**Files:**
- Delete: `firmware/micromouse/task42.h`
- Rename: `firmware/micromouse/task43.h` -> `firmware/micromouse/unseenMaze.h`
- Modify: `firmware/micromouse/micromouse.ino:19-32` (the TASK comment block, define and guard), `:98-103` (the conditional include), `:191` (`taskBegin`), `:216` (`taskUpdate`), `:220` (`taskRender`)
- Modify: `firmware/micromouse/observers.h:324-341` (stale 4.1/4.2 comment)
- Modify: `firmware/micromouse/mazeWallMap.h:20,32` (stale 4.1/4.2 comment)

**Interfaces:**
- Consumes: nothing.
- Produces: `unseenMaze.h` declaring `runBegin()`, `Velocity runUpdate(const Pose&, float)`, `runRender()`, plus the existing `lidar_obsv`, `obs_p`, `sf`, `fusedPose()`, `runner`, `wallMap`, `screen`, and `constexpr uint8_t MAZE_SIZE = 9`.

- [ ] **Step 1: Delete `task42.h` and rename `task43.h`**

```bash
cd /Users/zimmylevi/Desktop/Uni/MTRN3100/Micromouse
git rm -q firmware/micromouse/task42.h
git mv firmware/micromouse/task43.h firmware/micromouse/unseenMaze.h
```

- [ ] **Step 2: Remove the dead code at the top of `unseenMaze.h`**

Delete the `#include <sys/wait.h>` line — a POSIX header with no business in Arduino firmware, presumably an autocomplete accident. Delete the commented-out block:

```cpp
// bool lambda = []() {
//     runner.mapper.hasWall(Cell{7, 8}, South);
//     runner.mapper.hasWall(Cell{8, 1}, South);
//     runner.mapper.hasWall(Cell{7, 0}, South);
//     runner.mapper.hasWall(Cell{0, 1}, South);
//     return true;
// }();
```

- [ ] **Step 3: Rewrite the `unseenMaze.h` header comment**

Replace the whole leading comment block (down to `#pragma once`) with:

```cpp
// Unseen Maze -- explore, plan, race
//
// Zimmy Levi z5587840
//
// Included by micromouse.ino part way down the sketch rather than up with the
// other headers: everything here is built from lidar, obs_v, dt and display, so
// it has to come after them. The sketch is one translation unit, so those names
// are already in scope and this header declares no hardware of its own.
//
// The robot is given a maze size, a start cell, a start heading and a goal, all
// chosen at boot through startupUI.h, and nothing else. It explores until the
// goal is reachable, plans a route over what it found, and drives it. The
// observer localises against those discovered walls: there is no photograph of
// the maze -- finding it is the exercise -- so MazeWallMap stands in for an
// exported map. LidarObserver is templated on the map type and MazeWallMap
// offers Map's cast()/candidates(), so one observer serves both.
```

- [ ] **Step 4: Rename the three hooks in `unseenMaze.h`**

`taskBegin` -> `runBegin`, `taskUpdate` -> `runUpdate`, `taskRender` -> `runRender`. Update the comment above `runBegin` so it reads "Called from setup(), after the shared bring-up".

- [ ] **Step 5: Strip the TASK machinery from `micromouse.ino`**

Replace lines 19-32 (from `// WHICH TASK TO BUILD` through the `#endif` of the `#error` guard) with nothing. Replace the conditional include at lines 98-103:

```cpp
// The task, included here rather than with the headers above because it builds
// on lidar, obs_v, dt and display.
#if TASK == 42
#include "task42.h"
#else
#include "task43.h"
#endif
```

with:

```cpp
// The run, included here rather than with the headers above because it builds
// on lidar, obs_v, dt and display.
#include "unseenMaze.h"
```

Then rename the three call sites: `taskBegin()` -> `runBegin()`, `taskUpdate(pose, dt)` -> `runUpdate(pose, dt)`, `taskRender()` -> `runRender()`.

- [ ] **Step 6: Fix the stale task references in comments**

In `observers.h:324-341`, the block currently reads `// MazeWallMap<MAZE_SIZE> wallMap(runner.map()); // 4.3, discovered` and `// or, for 4.1 | 4.2, against the exported map:` and "...4.2 and against MazeWallMap -- the walls the robot has discovered for itself -- for 4.3...". Reword to name the exported map and the discovered map rather than task numbers, e.g. "against the exported map, or against MazeWallMap -- the walls the robot has discovered for itself".

In `mazeWallMap.h:20`, "is exactly what task 4.3 does not have" -> "is exactly what an unseen maze does not give you". At line 32, "exported map for tasks 4.1 and 4.2, this one for 4.3" -> "the exported map where there is one, this one where the maze is unseen".

- [ ] **Step 7: Build**

```bash
cd /Users/zimmylevi/Desktop/Uni/MTRN3100/Micromouse && ./compile.sh
```

Expected: clean build, "binary -> .../micromouse.ino.bin". If the linker reports undefined `taskBegin`/`taskUpdate`/`taskRender`, a call site in `micromouse.ino` was missed.

- [ ] **Step 8: Commit**

```bash
git add -A firmware/micromouse
git commit -m "Drop task 4.2; rename 4.3 to Unseen Maze"
```

---

### Task 2: `--debug` builds, and move the loop diagnostics behind the flag

**Files:**
- Modify: `scripts/build.sh` (usage text, option loop, compile invocation)
- Modify: `firmware/micromouse/micromouse.ino:222-259` (the two `DIAGNOSTIC` blocks in `loop()`)

**Interfaces:**
- Consumes: nothing.
- Produces: the preprocessor symbol `MICROMOUSE_DEBUG` (defined to `1` only under `./compile.sh --debug`), which Tasks 4 and 5 use to gate the boot self-check.

- [ ] **Step 1: Add `--debug` to `build.sh`'s option loop**

In the `while (($#))` loop, beside the `--db` case, add:

```sh
	--debug)
		debug=1
		shift
		;;
```

Initialise it next to `db_only=0` / `flash=0`:

```sh
debug=0
```

- [ ] **Step 2: Inject the define into the compile invocation**

Immediately above the existing `mode=()` line, add:

```sh
# Extra -D flags for the sketch. compiler.cpp.extra_flags is the documented
# arduino-cli hook for this, and the build directory is wiped every run, so a
# define cannot survive into a later build that did not ask for it.
defines=()
((debug)) && defines=(--build-property "compiler.cpp.extra_flags=-DMICROMOUSE_DEBUG=1")
```

and change the compile line to include it:

```sh
arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_DIR" "${mode[@]}" "${defines[@]}" "${passthru[@]}" "$SKETCH_DIR"
```

- [ ] **Step 3: Document `--debug` in `usage()`**

Change the Usage line to `Usage: ./scripts/build.sh [target] [--db] [--debug] [--flash] [--port dev] [--help]` and add to the Options block, after `--db`:

```
  --debug     Compile with -DMICROMOUSE_DEBUG=1, which enables the boot
              self-check for the runtime maze grid and the two diagnostic
              reports in loop(). Off by default: those reports write to Serial
              from inside the control loop, which costs it milliseconds.

              Note that --db builds compile_commands.json without this define
              unless --debug is passed too, so debug-only code reads as
              inactive in the editor.
```

- [ ] **Step 4: Gate the two diagnostic blocks in `loop()`**

Wrap both existing blocks — the one printing `imu dropped ... reads` and the one printing `imu ... samples/cycle` — in a single guard. Replace the `// DIAGNOSTIC:` lead-in of the first with:

```cpp
#ifdef MICROMOUSE_DEBUG
    // Gyro read failures and bus recoveries, rate limited so the report cannot
    // itself stall the loop it is measuring. Built only under ./compile.sh
    // --debug: a periodic Serial write is a few ms of exactly the loop stall
    // the IMU FIFO path exists to stop mattering.
```

and close with `#endif` after the second block's closing brace. Keep both block bodies byte-for-byte as they are.

- [ ] **Step 5: Build both ways**

```bash
cd /Users/zimmylevi/Desktop/Uni/MTRN3100/Micromouse && ./compile.sh && ./compile.sh --debug
```

Expected: both clean. The `--debug` binary should be slightly larger — the Serial format strings are only linked in that build.

- [ ] **Step 6: Commit**

```bash
git add scripts/build.sh firmware/micromouse/micromouse.ino
git commit -m "Add --debug builds; gate loop diagnostics behind MICROMOUSE_DEBUG"
```

---

### Task 3: Collapse the repeated bring-up print in `setup()`

`setup()` repeats `Serial.print("...")` / `Serial.println("\b\b\b [OKAY]")` six times with variations. `setup()` is restructured in Task 10, so tidy it first while it still does one thing.

**Files:**
- Modify: `firmware/micromouse/micromouse.ino:110-175` (the body of `setup()`)

**Interfaces:**
- Consumes: nothing.
- Produces: `void beginStep(const char* what)` and `void endStep(bool ok, const char* failure)`, used by Task 10.

- [ ] **Step 1: Add the helpers above `setup()`**

```cpp
// Bring-up progress, as one line per step.
//
// beginStep writes "Doing the thing..." and endStep backs over the ellipsis
// with a verdict, so a step that hangs leaves its own name on the wire as the
// last thing printed. Six steps used to spell this out individually and had
// drifted apart in their failure text.
void beginStep(const char* what) {
    Serial.print(what);
    Serial.print("...");
}

void endStep(bool ok, const char* failure) {
    Serial.print("\b\b\b [");
    Serial.print(ok ? "OKAY" : failure);
    Serial.println("]");
}
```

- [ ] **Step 2: Convert the six call sites**

Each existing pair becomes a `beginStep` / `endStep`. For example the OLED step:

```cpp
    beginStep("Initialising OLED");
    if (!display.init()) {
        endStep(false, "OLED INIT FAILED");
    } else {
        drawSplash(display);
        endStep(true, "");
    }
```

Do the same for "Initialising I2C", "Initialising Motors", "Initialising IMU Observer (P)", "Initialising Lidar Observer (P)" and "Loading goal". Note the IMU step has two distinct failure texts (`MPU6050 INIT FAILED` and `IMU OBSERVER INIT FAILED`) — keep both, passing the right one to `endStep`.

- [ ] **Step 3: Build**

```bash
cd /Users/zimmylevi/Desktop/Uni/MTRN3100/Micromouse && ./compile.sh
```

Expected: clean.

- [ ] **Step 4: Commit**

```bash
git add firmware/micromouse/micromouse.ino
git commit -m "Collapse repeated bring-up print in setup()"
```

---

### Task 4: Runtime grid size in `MazeMapper`, with the boot self-check as its test

This is the highest-risk task in the plan. The self-check is written first and must fail before the implementation exists — that is this codebase's only available red-green cycle.

**Files:**
- Modify: `firmware/micromouse/constants.h` (add size bounds)
- Modify: `firmware/micromouse/mazeMapper.h:86-88` (constructor), `:91` (add `gridSize`/`cellCount`/`configure`), `:97-134` (`begin`), `:416` (`inside`), and the private data block near `:751`
- Create: `firmware/micromouse/selfCheck.h`
- Modify: `firmware/micromouse/unseenMaze.h` (include and call the self-check)

**Interfaces:**
- Consumes: `MICROMOUSE_DEBUG` from Task 2.
- Produces:
  - `constexpr uint8_t MAZE_SIZE_MIN = 2;` and `constexpr uint8_t MAZE_SIZE_MAX = 16;`
  - `MazeMapper<N>::MazeMapper()` — default constructor, no arguments
  - `bool MazeMapper<N>::configure(uint8_t gridSize, Cell start, Direction heading, Cell goal)`
  - `uint8_t MazeMapper<N>::gridSize() const`
  - `uint16_t MazeMapper<N>::cellCount() const`
  - `bool runSelfChecks()` in `selfCheck.h`, defined only under `MICROMOUSE_DEBUG`

- [ ] **Step 1: Add the size bounds to `constants.h`**

Replace the existing comment at `constants.h:95-96` ("Cells per side lives in task43.h...") with:

```cpp
// Cells per side. MAZE_SIZE_MAX is the template capacity every maze buffer is
// sized to, not the maze being run: the grid actually in use is chosen at boot
// and carried as a runtime value, so one binary handles every size in range.
//
// Cost grows as N^2. At the capacity, MazeMapper holds ~1.5 kB and its deepest
// breadth-first search -- the frontier pruning, which wants two distance fields
// and a queue at once -- borrows ~1.5 kB of stack. The searches run one at a
// time, so the borrow does not stack. That is affordable on the Nano R4's
// 32 kB, especially with task42.h's MotionPlanner instance gone.
constexpr uint8_t MAZE_SIZE_MIN = 2;
constexpr uint8_t MAZE_SIZE_MAX = 16;
```

- [ ] **Step 2: Write the self-check — the failing test**

Create `firmware/micromouse/selfCheck.h`:

```cpp
// Boot self-check for the runtime maze grid
//
// Zimmy Levi z5587840
//
// Built only under ./compile.sh --debug. The risk in making the grid size a
// runtime value is a place that still reads the template capacity N where it
// should read the selected n: the result is a mapper that explores cells the
// maze does not have, which the panel cannot show you.
//
// Runs on the live mapper rather than a private one. A second MazeMapper at the
// capacity would be ~1.5 kB of RAM to test with, and it is not needed --
// runStartupUI() reconfigures the mapper afterwards, so whatever this leaves
// behind is overwritten before the run begins.

#pragma once

#ifdef MICROMOUSE_DEBUG

#include <Arduino.h>

#include "constants.h"
#include "mazeMapper.h"
#include "mazeWallMap.h"
#include "types.h"

// Linter hidden as this is a header-only library
// NOLINTBEGIN(misc-definitions-in-headers)

inline bool checkOne(const char* what, bool ok) {
    if (!ok) {
        Serial.print(F("  FAIL: "));
        Serial.println(what);
    }
    return ok;
}

// One grid size, end to end. The mapper and wall map are the live ones.
template <size_t N, typename WallMapT>
bool selfCheckGrid(MazeMapper<N>& mapper, const WallMapT& wallMap, uint8_t n) {
    Serial.print(F("self-check n="));
    Serial.println(n);

    const int8_t last = static_cast<int8_t>(n - 1);
    bool ok           = true;

    ok = checkOne("configure rejected an in-range size",
                  mapper.configure(n, Cell{0, 0}, North, Cell{last, last})) && ok;
    ok = checkOne("begin() failed", mapper.begin()) && ok;
    ok = checkOne("gridSize() disagrees", mapper.gridSize() == n) && ok;
    ok = checkOne("cellCount() disagrees",
                  mapper.cellCount() == static_cast<uint16_t>(n) * n) && ok;

    // The perimeter is sealed exactly at the n boundary.
    ok = checkOne("no wall on the north edge", mapper.hasWall(Cell{last, 0}, North)) && ok;
    ok = checkOne("no wall on the south edge", mapper.hasWall(Cell{0, 0}, South)) && ok;
    ok = checkOne("no wall on the east edge", mapper.hasWall(Cell{0, 0}, East)) && ok;
    ok = checkOne("no wall on the west edge", mapper.hasWall(Cell{0, last}, West)) && ok;

    // ...and nowhere else. Only meaningful once there is an interior.
    if (n >= 3) {
        ok = checkOne("phantom interior wall", !mapper.hasWall(Cell{1, 1}, North)) && ok;
    }

    // A cell past the edge is outside the maze, whatever the capacity is.
    ok = checkOne("markWall accepted a cell outside n",
                  !mapper.markWall(Cell{static_cast<int8_t>(n), 0}, North)) && ok;

    // The wall map's slot space is the selected grid, not the capacity.
    const size_t expect = (static_cast<size_t>(n) + 1) * n + static_cast<size_t>(n) * (n + 1) +
                          (static_cast<size_t>(n) + 1) * (n + 1);
    ok = checkOne("wallMap.size() is not the slot count for n", wallMap.size() == expect) && ok;

    return ok;
}

template <size_t N, typename WallMapT>
bool runSelfChecks(MazeMapper<N>& mapper, const WallMapT& wallMap) {
    // A plain array, not an initializer_list: nothing else in this tree pulls
    // <initializer_list> in and a debug-only header is a poor place to start.
    const uint8_t sizes[] = {2, 5, 9, MAZE_SIZE_MAX};

    bool ok = true;
    for (uint8_t i = 0; i < sizeof(sizes) / sizeof(sizes[0]); ++i) {
        ok = selfCheckGrid(mapper, wallMap, sizes[i]) && ok;
    }

    ok = checkOne("configure accepted a size below MAZE_SIZE_MIN",
                  !mapper.configure(MAZE_SIZE_MIN - 1, Cell{0, 0}, North, Cell{1, 1})) && ok;
    ok = checkOne("configure accepted a size above the capacity",
                  !mapper.configure(MAZE_SIZE_MAX + 1, Cell{0, 0}, North, Cell{1, 1})) && ok;

    Serial.println(ok ? F("self-check PASSED") : F("self-check FAILED"));
    return ok;
}

// NOLINTEND(misc-definitions-in-headers)

#endif  // MICROMOUSE_DEBUG
```

- [ ] **Step 3: Call it, and watch the build fail**

In `unseenMaze.h`, add `#include "selfCheck.h"` to the include block, and at the top of `runBegin()`:

```cpp
#ifdef MICROMOUSE_DEBUG
    runSelfChecks(runner.mapper, wallMap);
#endif
```

Run:

```bash
cd /Users/zimmylevi/Desktop/Uni/MTRN3100/Micromouse && ./compile.sh --debug
```

Expected: **FAIL**, with errors on `mapper.configure`, `mapper.gridSize` and `mapper.cellCount` being undefined members. A plain `./compile.sh` still succeeds, because the self-check is not compiled.

- [ ] **Step 4: Give `MazeMapper` its runtime grid**

Replace the constructor at `mazeMapper.h:86-88`:

```cpp
    MazeMapper(Cell startCell, Direction startHeading, Cell goalCell) :
        start(startCell), goal(goalCell), current(startCell),
        facing(startHeading), startFacing(startHeading) {}
```

with:

```cpp
    // Unconfigured. n = 0 makes inside() false everywhere, so begin() refuses
    // and observe() cannot write, which is the right state for a mapper whose
    // maze has not been chosen yet.
    MazeMapper() = default;
```

Directly under `MAX_CELLS`, add:

```cpp
    // The grid actually in use, as opposed to MAX_CELLS, which is the capacity
    // every buffer above is sized to. Everything that asks "is this cell in the
    // maze" goes through inside(), and inside() reads this.
    uint8_t gridSize() const { return n; }

    uint16_t cellCount() const { return static_cast<uint16_t>(n) * static_cast<uint16_t>(n); }

    // The whole runtime configuration, in one call. False -- and the mapper
    // left as it was -- if the size is out of range. Start and goal are not
    // checked here: begin() already validates them against the grid, and doing
    // it in one place keeps the two from disagreeing.
    bool configure(uint8_t gridSize, Cell startCell, Direction startHeading, Cell goalCell) {
        if (gridSize < MAZE_SIZE_MIN || gridSize > N) return false;
        started     = false;
        n           = gridSize;
        start       = startCell;
        goal        = goalCell;
        current     = startCell;
        facing      = startHeading;
        startFacing = startHeading;
        return true;
    }
```

Add to the private data block near line 751:

```cpp
    // Zero until configure() runs. See the constructor.
    uint8_t n = 0;
```

- [ ] **Step 5: Point `inside()` at the runtime grid**

At `mazeMapper.h:416`, replace:

```cpp
        return c.x >= 0 && c.x < static_cast<int>(N) && c.y >= 0 && c.y < static_cast<int>(N);
```

with:

```cpp
        return c.x >= 0 && c.x < static_cast<int>(n) && c.y >= 0 && c.y < static_cast<int>(n);
```

- [ ] **Step 6: Seed the perimeter at the runtime boundary**

In `begin()`, replace the perimeter loop at lines 124-129 with:

```cpp
        // Perimeter, at the selected grid's boundary rather than the capacity's.
        // Each of these mirrors onto a cell outside the maze, which addWall
        // drops, so no already-initialised neighbour is touched.
        for (uint8_t i = 0; i < n; ++i) {
            const int8_t last = static_cast<int8_t>(n - 1);
            addWall(Cell{0, static_cast<int8_t>(i)}, South);
            addWall(Cell{last, static_cast<int8_t>(i)}, North);
            addWall(Cell{static_cast<int8_t>(i), 0}, East);
            addWall(Cell{static_cast<int8_t>(i), last}, West);
        }
```

**Leave every other loop in this file at `N`.** The clearing loop above it, and the sweeps in `distancesFrom`, `refreshImproving`, `planMove` and `buildShortestPath`, are array initialisation over the full capacity. Expansion in all of them is gated by `inside(next)`, so cells outside `n` stay `Unreachable` and are never entered. Iterating 256 cells instead of n² is a few microseconds, and it removes the entire class of "stale data outside n" bug. Add a comment on the clearing loop saying so:

```cpp
        // The whole capacity, not just the selected grid: these arrays outlive
        // any one configure(), and a cell outside n that still holds a previous
        // run's walls is the failure mode this loop exists to prevent. inside()
        // keeps the search out of that region regardless; this keeps it clean.
```

- [ ] **Step 7: Build and run the self-check**

```bash
cd /Users/zimmylevi/Desktop/Uni/MTRN3100/Micromouse && ./compile.sh --debug
```

Expected: clean. The `wallMap.size()` assertion will still fail at runtime for every n except 9 — `MazeWallMap` is still strided on the capacity, and Task 5 fixes it. Everything else should pass. Flash and read Serial at 115200 to confirm:

```bash
./compile.sh --debug --flash
```

Expected on the wire: four `self-check n=` blocks, with only `wallMap.size() is not the slot count for n` failing, and a final `self-check FAILED`.

- [ ] **Step 8: Commit**

```bash
git add firmware/micromouse
git commit -m "MazeMapper: runtime grid size with N as capacity"
```

---

### Task 5: Runtime grid size in `MazeWallMap`

The remaining self-check failure from Task 4. This is what keeps the lidar solve proportional to the maze being run: `size()` is 280 slots at n = 9 and 833 at n = 16, and `candidates()` sweeps all of them on every solve. Pinning the stride at the capacity would make a 5×5 maze pay the 833-slot sweep for the whole run.

**Files:**
- Modify: `firmware/micromouse/mazeWallMap.h:56-68` (the count constants), `:74-90` (`present`), `:100-112` (`operator[]`), `:117-131` (`cast`), `:141-151` (`candidates`), `:165-171` (`nsWall`/`ewWall`), `:188` (`isPost`), `:203-222` (`centreOf`)

**Interfaces:**
- Consumes: `MazeMapper<N>::gridSize()` from Task 4.
- Produces: `size_t MazeWallMap<N>::size() const` — **no longer `static constexpr`**. `oledScreen.h:158` and `types.h:1243` already call it on an instance, so they need no change.

- [ ] **Step 1: Replace the static counts with runtime accessors**

Replace lines 56-68:

```cpp
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
```

with:

```cpp
template <size_t N>
class MazeWallMap {
    public:

    explicit MazeWallMap(const MazeMapper<N>& mapper) : mapper(mapper) {}

    // The slot space is the grid actually being run, not the template capacity.
    // That is a speed property, not just a tidiness one: candidates() walks
    // every slot on every lidar solve, and the count is 280 at a 9 x 9 grid
    // against 833 at 16 x 16. A 5 x 5 maze has no business paying for a 16 x 16
    // sweep. Zero until the mapper is configured, which makes every sweep below
    // a no-op rather than a divide by zero.
    size_t grid() const { return mapper.gridSize(); }

    size_t nsCount() const { return (grid() + 1) * grid(); }
    size_t ewCount() const { return grid() * (grid() + 1); }
    size_t postCount() const { return (grid() + 1) * (grid() + 1); }

    size_t size() const {
        if (grid() == 0) return 0;
        return nsCount() + ewCount() + postCount();
    }
```

- [ ] **Step 2: Convert `present()`**

```cpp
    bool present(size_t index) const {
        const size_t g = grid();
        if (g == 0) return false;

        const size_t ns = nsCount();
        if (index < ns) {
            return nsWall(static_cast<int>(index / g), static_cast<int>(index % g));
        }
        if (index < ns + ewCount()) {
            const size_t i = index - ns;
            return ewWall(static_cast<int>(i / (g + 1)), static_cast<int>(i % (g + 1)));
        }
        const size_t i = index - ns - ewCount();
        return postAt(static_cast<int>(i / (g + 1)), static_cast<int>(i % (g + 1)));
    }
```

- [ ] **Step 3: Convert `operator[]`, `isPost` and `centreOf`**

In `operator[]`, replace `index < NS_COUNT` with `index < nsCount()` and `index < NS_COUNT + EW_COUNT` with `index < nsCount() + ewCount()`.

`isPost` stops being static:

```cpp
    bool isPost(size_t index) const { return index >= nsCount() + ewCount(); }
```

`centreOf` in full — the offset arithmetic is unchanged, only the stride moves
from `N` to `grid()`:

```cpp
    Vec2D centreOf(size_t index) const {
        const size_t g    = grid();
        const size_t ns   = nsCount();
        const auto origin = mapper.startPosition();

        if (index < ns) {
            const int a = static_cast<int>(index / g);
            const int b = static_cast<int>(index % g);
            return cellToWorld(
                static_cast<float>(a) - 0.5f, static_cast<float>(b), origin.x, origin.y
            );
        }
        if (index < ns + ewCount()) {
            const size_t i = index - ns;
            const int a    = static_cast<int>(i / (g + 1));
            const int b    = static_cast<int>(i % (g + 1));
            return cellToWorld(
                static_cast<float>(a), static_cast<float>(b) - 0.5f, origin.x, origin.y
            );
        }
        const size_t i = index - ns - ewCount();
        const int a    = static_cast<int>(i / (g + 1));
        const int b    = static_cast<int>(i % (g + 1));
        return cellToWorld(
            static_cast<float>(a) - 0.5f, static_cast<float>(b) - 0.5f, origin.x, origin.y
        );
    }
```

`centreOf` and `operator[]` must agree — `candidates()` bounds a slot at
`centreOf` and `cast()` builds the obstacle in `operator[]`, so a slot whose
obstacle sat anywhere its bounding test did not would be dropped by the sweep
and then hit by the ray. That is why the stride change has to land in both.

- [ ] **Step 4: Convert `cast()` and `candidates()`**

In `cast()`, `const size_t total = (indices == nullptr) ? COUNT : count;` becomes `... ? size() : count;`.

In `candidates()`, `for (size_t i = 0; i < COUNT && n < M; ++i)` becomes `for (size_t i = 0; i < size() && found < M; ++i)`. **Rename the local `n` to `found`** — there is now a runtime grid called `n` in the mapper and a local shadowing that name in a sweep is exactly the confusion this task exists to avoid. Update the `return n;` and `out[n++]` accordingly.

Also update the comment above `candidates()`: "which is what keeps a 280-slot sweep at N = 9 cheaper" -> "at a 9 x 9 grid".

- [ ] **Step 5: Bound `nsWall`/`ewWall` by the runtime grid**

```cpp
    bool nsWall(int a, int b) const {
        const int g = static_cast<int>(grid());
        if (b < 0 || b >= g) return false;
        if (a <= 0) return mapper.hasWall(cell(0, b), South);
        return mapper.hasWall(cell(a - 1, b), North);
    }

    bool ewWall(int a, int b) const {
        const int g = static_cast<int>(grid());
        if (a < 0 || a >= g) return false;
        if (b <= 0) return mapper.hasWall(cell(a, 0), East);
        return mapper.hasWall(cell(a, b - 1), West);
    }
```

Update the comment above `nsWall` — "so a runs 0..N inclusive" and "at a = N the upper one is, and `hasWall(Cell{N - 1, b}, North)` covers it" should read `n` rather than `N`. Same for the index-space comment at lines 47-54.

- [ ] **Step 6: Build and run the self-check**

```bash
cd /Users/zimmylevi/Desktop/Uni/MTRN3100/Micromouse && ./compile.sh --debug --flash
```

Expected on Serial at 115200: four `self-check n=` blocks with no `FAIL:` lines, then `self-check PASSED`.

- [ ] **Step 7: Commit**

```bash
git add firmware/micromouse/mazeWallMap.h
git commit -m "MazeWallMap: slot space follows the runtime grid"
```

---

### Task 6: `MazeRunner` configuration, and drop the corner crop

**Files:**
- Modify: `firmware/micromouse/mazeRunner.h:47-56` (constructor), `:62-89` (`begin`), `:91-108` (delete the crop block), `:135-138` (`exploreProgress`), and add pass-throughs near `:123`
- Modify: `firmware/micromouse/constants.h:8` (delete `MAZE_CORNER_CROP`)
- Modify: `firmware/micromouse/unseenMaze.h` (drop the global start/heading/goal, use the pass-throughs)
- Modify: `firmware/micromouse/mazeMapper.h` (add `RunConfig` beside `Cell`)

**Interfaces:**
- Consumes: `MazeMapper<N>::configure`, `gridSize`, `cellCount` from Task 4.
- Produces:
  - `struct RunConfig { uint8_t size; Cell start; Direction heading; Cell goal; };` in `mazeMapper.h`
  - `MazeRunner<N>::MazeRunner(LIDAR&, PSPlanner&)`
  - `bool MazeRunner<N>::configure(const RunConfig&)`
  - `bool MazeRunner<N>::homing() const`, `bool MazeRunner<N>::faulted() const`, `float MazeRunner<N>::homeProgress() const`

- [ ] **Step 1: Add `RunConfig` to `mazeMapper.h`**

It goes directly under `struct Cell` (`mazeMapper.h:68-71`), **not** in `types.h`:
`Cell` is declared here, and `types.h` is included *by* this file, so it cannot
see `Cell`. `Direction` comes from `types.h`, which this file already includes.

```cpp
// Everything about a run that is chosen rather than measured: the maze the
// robot has been put in, and where in it the job starts and ends. Filled by
// startupUI.h at boot and handed to MazeRunner::configure().
struct RunConfig {
    uint8_t   size;
    Cell      start;
    Direction heading;
    Cell      goal;
};
```

- [ ] **Step 2: Take the configuration out of the constructor**

Replace `mazeRunner.h:47-56`:

```cpp
    MazeRunner(
        LIDAR& lidar,
        PSPlanner& planner,
        const Cell& startCell,
        Direction startHeading,
        const Cell& goalCell
    ) :
        lidar(lidar), planner(planner), mapper(startCell, startHeading, goalCell) {}
```

with:

```cpp
    MazeRunner(LIDAR& lidar, PSPlanner& planner) : lidar(lidar), planner(planner) {}

    // The run's configuration, which is chosen at boot rather than compiled in.
    // False if the mapper rejects the size; the start and goal are validated by
    // begin(), which is where they have always been validated.
    bool configure(const RunConfig& cfg) {
        return mapper.configure(cfg.size, cfg.start, cfg.heading, cfg.goal);
    }
```

- [ ] **Step 3: Delete the corner crop**

Delete `mazeRunner.h:91-108` entirely — the `croppedCells` and `reachableCells` members, `croppedCell()` and `sealCroppedCells()`. Delete the two lines that call them in `begin()`:

```cpp
        croppedCells = sealCroppedCells();
        reachableCells = static_cast<uint16_t>(MazeMapper<N>::MAX_CELLS - croppedCells);
```

Delete `constexpr uint8_t MAZE_CORNER_CROP = 1;` from `constants.h:8`, and its comment if it has one. If `etl/algorithm.h`'s `etl::min` is now unused in `mazeRunner.h`, leave the include — other code in the file may use it; only remove it if `./compile.sh` warns.

Add a short note where the crop used to be, so the next reader knows it was a decision:

```cpp
    // There is no corner crop. It seeded the competition deck's chamfered
    // corners as sealed cells, which is a prior about a specific maze -- and an
    // unseen maze of a size chosen at boot gives no grounds for it. Exploration
    // discovers a walled-off corner by itself, at the cost of a few probing
    // moves; exploreProgress() already documents that it under-reads when part
    // of the maze is unreachable.
```

- [ ] **Step 4: Fix `exploreProgress()`**

Replace `mazeRunner.h:135-138`:

```cpp
    float exploreProgress() const {
        return static_cast<float>(mapper.visitedCount()) /
               static_cast<float>(MazeMapper<N>::MAX_CELLS);
    }
```

with:

```cpp
    float exploreProgress() const {
        const uint16_t cells = mapper.cellCount();
        if (cells == 0) return 0.0f;
        return static_cast<float>(mapper.visitedCount()) / static_cast<float>(cells);
    }
```

Against `MAX_CELLS` this was correct only while the template parameter *was* the grid. With it as a capacity, a 5x5 maze would have read 10% at completion.

- [ ] **Step 5: Add the mapper pass-throughs**

Directly under the public `MazeMapper<N> mapper;` member, add:

```cpp
    // The display asks these of the run, not of the mapper. They were reached
    // through runner.mapper.* from the task header, which coupled the screen to
    // the mapper's interface for three predicates.
    bool homing() const { return mapper.homing(); }
    bool faulted() const { return mapper.faulted(); }
    float homeProgress() const { return mapper.homeProgress(); }
```

- [ ] **Step 6: Update `unseenMaze.h`**

Delete the four configuration globals and the `MAZE_SIZE` constant with its comment block:

```cpp
constexpr uint8_t MAZE_SIZE = 9;
using mazeMapper = MazeMapper<MAZE_SIZE>;
Cell startCell = {1, 1};
Direction startHeading = North;
Cell goalCell  = {5, 5};
```

Replace with:

```cpp
using mazeMapper = MazeMapper<MAZE_SIZE_MAX>;
```

Change the runner construction to `MazeRunner<MAZE_SIZE_MAX> runner(lidar, psp);` and every other `MAZE_SIZE` to `MAZE_SIZE_MAX` (the `MazeWallMap<MAZE_SIZE>`, `LidarObserver<MazeWallMap<MAZE_SIZE>>` and `OLEDScreen<MazeWallMap<MAZE_SIZE>>` declarations).

In `screenMode()` and `screenMetric()`, replace `runner.mapper.homing()` -> `runner.homing()`, `runner.mapper.faulted()` -> `runner.faulted()`, `runner.mapper.homeProgress()` -> `runner.homeProgress()`.

- [ ] **Step 7: Give `runBegin()` a configuration to work with, temporarily**

Task 10 replaces this with the wizard's output. For now, so the tree builds and runs, put a literal at the top of `runBegin()` after the self-check:

```cpp
    // TEMPORARY -- replaced by runStartupUI() in the bring-up reorder.
    runner.configure(RunConfig{9, Cell{1, 1}, North, Cell{5, 5}});
```

- [ ] **Step 8: Build, and check the self-check still passes**

```bash
cd /Users/zimmylevi/Desktop/Uni/MTRN3100/Micromouse && ./compile.sh && ./compile.sh --debug --flash
```

Expected: both clean, `self-check PASSED` on the wire, and the robot behaves as it did before this plan started — same 9x9 maze, same start and goal.

- [ ] **Step 9: Commit**

```bash
git add -A firmware/micromouse
git commit -m "MazeRunner: runtime configuration; drop the corner crop"
```

---

### Task 7: UI constants and the input layer

Encoder dials and drift-compensated lidar buttons, with no drawing and no screens. Separated from the wizard so the input semantics can be reviewed on their own.

**Files:**
- Modify: `firmware/micromouse/constants.h` (UI constants block)
- Create: `firmware/micromouse/startupUI.h`

**Interfaces:**
- Consumes: `LIDAR::getReading`, `LIDAR::Sensors` from `lidar.h`; `BaseMotor::count()` from `motor.h`.
- Produces: `class UIDial` with `void begin(long count)`, `int take(long count)`, `float angle(long count) const`; `class UIButton` with `void begin(uint16_t reading, unsigned long nowMs)`, `bool update(uint16_t reading, unsigned long nowMs)`.

- [ ] **Step 1: Add the UI constants**

Append to `constants.h`, after the OLED block:

```cpp
// FIRMWARE VERSION
// Shown bottom-left on the splash. Bumped for the startup UI, which changes
// what the robot expects of its operator -- worth being able to read off the
// panel rather than off a git hash.
constexpr char FIRMWARE_VERSION[] = "v2.0";

// STARTUP UI
constexpr uint16_t UI_SPLASH_MS    = 2000;
constexpr uint16_t UI_COUNTDOWN_MS = 5000;

// Encoder detent, in raw counts. ENC_CPR / 12 is twelve clicks per wheel
// revolution, which spans the 2..16 size range in about 1.2 turns. Purely a
// feel setting: raise it for a coarser dial, lower it for a finer one.
constexpr int UI_ENCODER_DETENT_COUNTS = ENC_CPR / 12;

// Side lidar as a momentary button, measured against a baseline rather than an
// absolute distance. The sensors are mounted 35 mm off centre and a wall's
// inner face is 84 mm from a cell centre, so a side sensor reads about 49 mm to
// an adjacent wall -- there is not enough room between that and a hand for a
// fixed threshold to be reliable.
constexpr uint16_t UI_BUTTON_PRESS_DELTA_MM   = 25;
constexpr uint16_t UI_BUTTON_RELEASE_DELTA_MM = 15;
constexpr uint8_t UI_BUTTON_DEBOUNCE_SAMPLES  = 3;

// A reading that sits past PRESS_DELTA this long without being taken as a press
// is the world having changed -- the robot was moved, or a wall arrived -- and
// becomes the new baseline. Without it, carrying the robot from the bench into
// a corridor is a ~150 mm drop, six times PRESS_DELTA, and reads as a press.
constexpr uint16_t UI_BUTTON_BASELINE_ADOPT_MS = 1200;

// The countdown is the one screen during which the robot is being handled, so
// both sensors are unreliable there in a way drift compensation reduces but
// cannot eliminate: a decisive placement can still land inside the adopt
// window. Both off by default, which also means the countdown draws no button
// chrome -- self-documenting as "nothing is live, place the robot freely".
constexpr bool UI_COUNTDOWN_SKIP_ENABLED = false;
constexpr bool UI_COUNTDOWN_BACK_ENABLED = false;

// Chrome geometry. Buttons are drawn centred on the edge pixel, so GFX clips
// exactly half and a semicircle costs no more than a circle.
constexpr uint16_t UI_BLINK_MS     = 200;
constexpr int16_t UI_BUTTON_RADIUS = 6;
constexpr int16_t UI_DIAL_RADIUS   = 7;
constexpr int16_t UI_DIAL_LEFT_X   = 8;
constexpr int16_t UI_DIAL_RIGHT_X  = 119;
constexpr int16_t UI_DIAL_Y        = 55;
```

- [ ] **Step 2: Create `startupUI.h` with the input layer**

```cpp
// The boot wizard: maze size, start cell, start heading and goal
//
// Zimmy Levi z5587840
//
// Blocking, and called from setup() before the IMU calibrates. The control loop
// does not exist yet and there is nothing to service but I2C, so a blocking
// wizard is far simpler than a loop() state machine -- and it keeps loop() the
// wiring diagram it is.
//
// The motors are never driven from here, so the wheels turn freely under hand.

#pragma once

#include <math.h>

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#include "constants.h"
#include "i2cRepairer.h"
#include "lidar.h"
#include "mazeMapper.h"  // Cell and RunConfig are declared here, not in types.h
#include "motor.h"
#include "oledDisplay.h"
#include "types.h"

// One wheel as a detented dial.
//
// Incremental, not absolute. Mapping raw position straight onto the value would
// wind up against a clamp: spin ten detents past the maximum and you would need
// ten back before the number moved. take() returns a delta and advances its own
// mark, so a clamped dial responds to the first detent in the other direction.
class UIDial {
    public:

    void begin(long count) { mark = count; origin = count; }

    // Whole detents since the last call, signed, advancing the mark by exactly
    // what it returns so no fraction is lost across calls.
    int take(long count) {
        const long d = (count - mark) / UI_ENCODER_DETENT_COUNTS;
        mark += d * UI_ENCODER_DETENT_COUNTS;
        return static_cast<int>(d);
    }

    // Wheel angle since begin(), for the chrome spoke. Off the raw count rather
    // than the detented value, so the spoke tracks the wheel 1:1 and moves
    // smoothly instead of stepping -- which is what makes it read as live.
    float angle(long count) const {
        return TWO_PI * static_cast<float>(count - origin) / static_cast<float>(ENC_CPR);
    }

    private:

    long mark   = 0;
    long origin = 0;
};

// One side lidar as a momentary button, against an adapting baseline.
class UIButton {
    public:

    void begin(uint16_t reading, unsigned long nowMs) {
        baseline   = reading;
        armed      = false;  // a clean release arms it; see update()
        below      = 0;
        driftSince = nowMs;
    }

    // True exactly once per press, on the edge.
    bool update(uint16_t reading, unsigned long nowMs) {
        const bool near = (reading + UI_BUTTON_PRESS_DELTA_MM) < baseline;

        if (!near) {
            below = 0;
            // Back at the baseline is a release, and a release is what arms the
            // button. Starting unarmed is what stops a hand already in front of
            // a sensor at boot from firing the first screen.
            if ((reading + UI_BUTTON_RELEASE_DELTA_MM) >= baseline) {
                armed      = true;
                driftSince = nowMs;
            }
            return false;
        }

        if (below < UI_BUTTON_DEBOUNCE_SAMPLES) ++below;

        if (armed && below >= UI_BUTTON_DEBOUNCE_SAMPLES) {
            armed      = false;  // needs a release before the next press
            driftSince = nowMs;
            return true;
        }

        // Near, but not consumed as a press -- either unarmed, or still
        // debouncing. If it has been like this past the adopt window it is not
        // a hand, it is the world: take it as the new baseline.
        if (nowMs - driftSince >= UI_BUTTON_BASELINE_ADOPT_MS) {
            baseline   = reading;
            below      = 0;
            armed      = true;
            driftSince = nowMs;
        }
        return false;
    }

    private:

    uint16_t baseline        = 0;
    unsigned long driftSince = 0;
    uint8_t below            = 0;
    bool armed               = false;
};
```

- [ ] **Step 3: Build**

```bash
cd /Users/zimmylevi/Desktop/Uni/MTRN3100/Micromouse && ./compile.sh
```

Expected: clean. `startupUI.h` is not included by anything yet, so this only proves it parses once Task 8 includes it — add `#include "startupUI.h"` to `micromouse.ino`'s include block now so the build actually compiles it.

- [ ] **Step 4: Commit**

```bash
git add firmware/micromouse/constants.h firmware/micromouse/startupUI.h firmware/micromouse/micromouse.ino
git commit -m "Startup UI: constants and input layer"
```

---

### Task 8: The chrome layer

**Files:**
- Modify: `firmware/micromouse/startupUI.h` (append)

**Interfaces:**
- Consumes: `OLEDDisplay::gfx()`, the `UI_*` chrome constants from Task 7.
- Produces: `struct UIChrome { bool leftButton, rightButton, leftDial, rightDial; };` and `inline void drawChrome(OLEDDisplay&, const UIChrome&, float leftAngle, float rightAngle, bool leftBlink, bool rightBlink)`.

Note the signature takes **angles**, not raw counts as the spec sketched. The chrome has no business knowing about `ENC_CPR`; `UIDial::angle()` already converts.

- [ ] **Step 1: Append the chrome layer to `startupUI.h`**

```cpp
// Linter hidden as this is a header-only library
// NOLINTBEGIN(misc-definitions-in-headers)

// Which inputs are live on the current screen.
//
// Drawn every frame, so the panel always answers "what can I do here?" without
// the operator having to remember the sequence. Screen 1 sets leftButton false
// because back is a no-op there: the chrome and the behaviour agree, which is
// the whole point of drawing it.
struct UIChrome {
    bool leftButton;
    bool rightButton;
    bool leftDial;
    bool rightDial;
};

// A wheel, as a circle with one spoke.
inline void drawDial(Adafruit_SSD1306& g, int16_t cx, float angle) {
    g.drawCircle(cx, UI_DIAL_Y, UI_DIAL_RADIUS, SSD1306_WHITE);

    // World angle to screen: the spoke points up at zero and turns the way the
    // wheel does. Screen y grows downward, hence the negated cosine.
    const int16_t ex = cx + static_cast<int16_t>(lroundf(UI_DIAL_RADIUS * sinf(angle)));
    const int16_t ey =
        UI_DIAL_Y - static_cast<int16_t>(lroundf(UI_DIAL_RADIUS * cosf(angle)));
    g.drawLine(cx, UI_DIAL_Y, ex, ey, SSD1306_WHITE);
}

// A lidar button, as a semicircle on its edge.
//
// fillCircle centred on the edge pixel: GFX clips the off-panel half, so this
// is a semicircle bulging inward at no cost over a normal circle. A press
// flashes it to an outline for UI_BLINK_MS.
inline void drawButton(Adafruit_SSD1306& g, int16_t x, bool blink) {
    if (blink) {
        g.drawCircle(x, OLED_HEIGHT / 2, UI_BUTTON_RADIUS, SSD1306_WHITE);
    } else {
        g.fillCircle(x, OLED_HEIGHT / 2, UI_BUTTON_RADIUS, SSD1306_WHITE);
    }
}

// Chrome occupies x < 8, x > 119 and y > 46, leaving a content region of 18
// characters by 5 lines at OLED_CHAR_WIDTH / OLED_TEXT_HEIGHT.
inline void drawChrome(
    OLEDDisplay& display, const UIChrome& chrome, float leftAngle, float rightAngle,
    bool leftBlink, bool rightBlink
) {
    Adafruit_SSD1306& g = display.gfx();

    if (chrome.leftButton) drawButton(g, 0, leftBlink);
    if (chrome.rightButton) drawButton(g, OLED_WIDTH - 1, rightBlink);
    if (chrome.leftDial) drawDial(g, UI_DIAL_LEFT_X, leftAngle);
    if (chrome.rightDial) drawDial(g, UI_DIAL_RIGHT_X, rightAngle);
}

// NOLINTEND(misc-definitions-in-headers)
```

- [ ] **Step 2: Build**

```bash
cd /Users/zimmylevi/Desktop/Uni/MTRN3100/Micromouse && ./compile.sh
```

Expected: clean.

- [ ] **Step 3: Commit**

```bash
git add firmware/micromouse/startupUI.h
git commit -m "Startup UI: input affordance chrome"
```

---

### Task 9: The wizard

**Files:**
- Modify: `firmware/micromouse/startupUI.h` (append)

**Interfaces:**
- Consumes: `UIDial`, `UIButton`, `UIChrome`, `drawChrome` from Tasks 7-8; `RunConfig` from Task 6; `LIDAR`, `BaseMotor`, `OLEDDisplay`, `I2CRepairer`.
- Produces: `RunConfig runStartupUI(OLEDDisplay&, LIDAR&, BaseMotor& left, BaseMotor& right, I2CRepairer&)`.

Per-screen chrome map, which the implementation must match:

| Screen | left button | right button | left dial | right dial |
|--------|-------------|--------------|-----------|------------|
| 1 size | — | yes | — | yes |
| 2 start cell | yes | yes | yes | yes |
| 3 heading | yes | yes | — | yes |
| 4 goal cell | yes | yes | yes | yes |
| 5 countdown | per constant | per constant | — | — |

- [ ] **Step 1: Append the wizard to `startupUI.h`, inside the NOLINT block**

```cpp
// Direction is not an integer dial: the enum is {North = 0, West = 1,
// South = 2, East = -1}, so incrementing it does not walk the compass. This is
// the clockwise order the heading screen indexes.
constexpr Direction UI_HEADING_ORDER[4] = {North, East, South, West};

inline const char* headingName(Direction d) {
    switch (d) {
        case North: return "N";
        case East:  return "E";
        case South: return "S";
        default:    return d == West ? "W" : "?";
    }
}

// MazeMapper::sameCell is a private static member, so it is not reachable from
// here and this is not worth widening the mapper's interface for.
inline bool sameCellUI(const Cell& a, const Cell& b) { return a.x == b.x && a.y == b.y; }

inline int clampInt(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// The wizard's screens, in order.
enum class UIStep : uint8_t { Size, Start, Heading, Goal, Countdown, Done };

inline RunConfig runStartupUI(
    OLEDDisplay& display, LIDAR& lidar, BaseMotor& left, BaseMotor& right,
    I2CRepairer& i2c
) {
    RunConfig cfg{MAZE_SIZE_MAX, Cell{0, 0}, North, Cell{0, 0}};
    uint8_t headingIndex = 0;

    UIDial leftDial;
    UIDial rightDial;
    UIButton backButton;
    UIButton nextButton;

    lidar.update();
    const unsigned long t0 = millis();
    backButton.begin(lidar.getReading(LIDAR::Left), t0);
    nextButton.begin(lidar.getReading(LIDAR::Right), t0);
    leftDial.begin(left.count());
    rightDial.begin(right.count());

    UIStep step                 = UIStep::Size;
    UIStep shownStep             = UIStep::Size;
    unsigned long stepEnteredMs = t0;
    unsigned long leftBlinkMs   = 0;
    unsigned long rightBlinkMs  = 0;

    while (step != UIStep::Done) {
        i2c.update();
        lidar.update();

        const unsigned long now = millis();
        const long lc           = left.count();
        const long rc           = right.count();

        // Chrome for this screen, which is also the gate on the inputs: a
        // button that is not drawn is not read, so the two cannot disagree.
        UIChrome chrome{false, false, false, false};
        switch (step) {
            case UIStep::Size:
                chrome = UIChrome{false, true, false, true};
                break;
            case UIStep::Start:
            case UIStep::Goal:
                chrome = UIChrome{true, true, true, true};
                break;
            case UIStep::Heading:
                chrome = UIChrome{true, true, false, true};
                break;
            case UIStep::Countdown:
                chrome = UIChrome{
                    UI_COUNTDOWN_BACK_ENABLED, UI_COUNTDOWN_SKIP_ENABLED, false, false
                };
                break;
            default: break;
        }

        // Read the dials every frame regardless, so the marks track the wheels
        // and a screen that does not use a dial cannot bank detents for the
        // next one that does.
        const int dl = leftDial.take(lc);
        const int dr = rightDial.take(rc);

        const bool back =
            backButton.update(lidar.getReading(LIDAR::Left), now) && chrome.leftButton;
        const bool next =
            nextButton.update(lidar.getReading(LIDAR::Right), now) && chrome.rightButton;

        if (back) leftBlinkMs = now;
        if (next) rightBlinkMs = now;

        const int last = static_cast<int>(cfg.size) - 1;

        switch (step) {
            case UIStep::Size:
                if (dr != 0) {
                    cfg.size = static_cast<uint8_t>(
                        clampInt(static_cast<int>(cfg.size) + dr, MAZE_SIZE_MIN, MAZE_SIZE_MAX)
                    );
                    // A smaller maze can strand a cell chosen earlier.
                    cfg.start.x = static_cast<int8_t>(clampInt(cfg.start.x, 0, cfg.size - 1));
                    cfg.start.y = static_cast<int8_t>(clampInt(cfg.start.y, 0, cfg.size - 1));
                    cfg.goal.x  = static_cast<int8_t>(clampInt(cfg.goal.x, 0, cfg.size - 1));
                    cfg.goal.y  = static_cast<int8_t>(clampInt(cfg.goal.y, 0, cfg.size - 1));
                }
                if (next) step = UIStep::Start;
                break;

            case UIStep::Start:
                if (dl != 0) cfg.start.x = static_cast<int8_t>(clampInt(cfg.start.x + dl, 0, last));
                if (dr != 0) cfg.start.y = static_cast<int8_t>(clampInt(cfg.start.y + dr, 0, last));
                if (next) step = UIStep::Heading;
                if (back) step = UIStep::Size;
                break;

            case UIStep::Heading:
                if (dr != 0) {
                    headingIndex = static_cast<uint8_t>(((headingIndex + dr) % 4 + 4) % 4);
                    cfg.heading  = UI_HEADING_ORDER[headingIndex];
                }
                if (next) step = UIStep::Goal;
                if (back) step = UIStep::Start;
                break;

            case UIStep::Goal:
                if (dl != 0) cfg.goal.x = static_cast<int8_t>(clampInt(cfg.goal.x + dl, 0, last));
                if (dr != 0) cfg.goal.y = static_cast<int8_t>(clampInt(cfg.goal.y + dr, 0, last));
                if (back) step = UIStep::Heading;
                if (next && !sameCellUI(cfg.start, cfg.goal)) step = UIStep::Countdown;
                break;

            case UIStep::Countdown:
                if (back) step = UIStep::Goal;
                if (next || now - stepEnteredMs >= UI_COUNTDOWN_MS) step = UIStep::Done;
                break;

            default: break;
        }

        if (step == UIStep::Done) break;

        // Entering a screen restarts its clock, which is what the countdown
        // measures against. A plain local, not a function-local static: a
        // static would survive into a second call and start the countdown
        // already expired.
        if (step != shownStep) {
            shownStep     = step;
            stepEnteredMs = now;
        }

        if (!display.due()) continue;

        Adafruit_SSD1306& g = display.gfx();
        g.clearDisplay();
        g.setTextSize(OLED_TEXT_SIZE);
        g.setTextColor(SSD1306_WHITE);

        switch (step) {
            case UIStep::Size:
                g.setCursor(12, 4);
                g.print(F("MAZE SIZE"));
                g.setCursor(12, 20);
                g.print(cfg.size);
                g.print(F(" x "));
                g.print(cfg.size);
                break;
            case UIStep::Start:
                g.setCursor(12, 4);
                g.print(F("START CELL"));
                g.setCursor(12, 20);
                g.print(F("X:"));
                g.print(cfg.start.x);
                g.print(F("  Y:"));
                g.print(cfg.start.y);
                break;
            case UIStep::Heading:
                g.setCursor(12, 4);
                g.print(F("START HEADING"));
                g.setCursor(12, 20);
                g.print(headingName(cfg.heading));
                break;
            case UIStep::Goal:
                g.setCursor(12, 4);
                g.print(F("GOAL CELL"));
                g.setCursor(12, 20);
                g.print(F("X:"));
                g.print(cfg.goal.x);
                g.print(F("  Y:"));
                g.print(cfg.goal.y);
                if (sameCellUI(cfg.start, cfg.goal)) {
                    g.setCursor(12, 32);
                    g.print(F("= START"));
                }
                break;
            case UIStep::Countdown:
                g.setCursor(12, 4);
                g.print(F("STARTING IN"));
                g.setCursor(12, 20);
                g.print((UI_COUNTDOWN_MS - (now - stepEnteredMs) + 999) / 1000);
                break;
            default: break;
        }

        drawChrome(
            display, chrome, leftDial.angle(lc), rightDial.angle(rc),
            now - leftBlinkMs < UI_BLINK_MS, now - rightBlinkMs < UI_BLINK_MS
        );
        g.display();
    }

    return cfg;
}
```

- [ ] **Step 2: Build**

```bash
cd /Users/zimmylevi/Desktop/Uni/MTRN3100/Micromouse && ./compile.sh
```

Expected: clean.

- [ ] **Step 3: Commit**

```bash
git add firmware/micromouse/startupUI.h
git commit -m "Startup UI: the wizard"
```

---

### Task 10: Wire the wizard into bring-up

**Files:**
- Modify: `firmware/micromouse/oledSplash.h` (version overlay)
- Modify: `firmware/micromouse/observers.h` (add `WheelObserver::reset()`)
- Modify: `firmware/micromouse/micromouse.ino:110-180` (`setup()`)
- Modify: `firmware/micromouse/unseenMaze.h` (remove the temporary literal from Task 6)

**Interfaces:**
- Consumes: `runStartupUI` from Task 9, `MazeRunner::configure` from Task 6.
- Produces: `void WheelObserver::reset()`.

- [ ] **Step 1: Draw the version bottom-left on the splash**

In `oledSplash.h`, inside `drawSplash`, after the `drawBitmap` call and before `g.display()`:

```cpp
    // Bottom-left, over a filled rect: drawBitmap paints set bits only, so the
    // logo may well have art underneath this and white-on-white would vanish.
    const int16_t vh = OLED_TEXT_HEIGHT + 2;
    const int16_t vw = static_cast<int16_t>(sizeof(FIRMWARE_VERSION)) * OLED_CHAR_WIDTH;
    g.fillRect(0, OLED_HEIGHT - vh, vw, vh, SSD1306_BLACK);
    g.setTextSize(OLED_TEXT_SIZE);
    g.setTextColor(SSD1306_WHITE);
    g.setCursor(2, OLED_HEIGHT - vh + 1);
    g.print(FIRMWARE_VERSION);
```

- [ ] **Step 2: Add `WheelObserver::reset()`**

`WheelObserver` caches `left_prev_rad` / `right_prev_rad` at construction, which happens at file scope — long before the operator spins the wheels through the wizard. Without this, the first `update()` after the wizard sees the whole hand-spin as one tick's motion and reports an enormous velocity.

In `observers.h`, in `WheelObserver`'s public section:

```cpp
    // Re-seeds the deltas from where the wheels are now.
    //
    // The wizard is driven by turning the wheels by hand, so between this
    // object's construction and the first control tick they may have moved a
    // long way. Odometry integrates deltas, so the absolute counts do not
    // matter and there is nothing to zero -- only the marks need moving.
    void reset() {
        left_prev_rad  = left.angularDisplacement();
        right_prev_rad = right.angularDisplacement();
        velocity       = {0, 0};
    }
```

- [ ] **Step 3: Restructure `setup()`**

The current splash duration is accidental: it lasts ~3.2 s only because `imu_obsv.init()`'s calibration window runs underneath it. Replace the body of `setup()` from the I2C step through `taskBegin()`/`runBegin()` with:

```cpp
    beginStep("Initialising I2C");
    i2cRepairer.begin();
    endStep(true, "");

    beginStep("Initialising OLED");
    const bool oledOk = display.init();
    if (oledOk) drawSplash(display);
    endStep(oledOk, "OLED INIT FAILED");
    const unsigned long splashShownMs = millis();

    // Before the splash hold, because the dials need them.
    beginStep("Initialising Motors");
    leftMotor.init();
    rightMotor.init();
    endStep(true, "");

    // Also before the hold: the side lidars are the wizard's buttons, so they
    // have to be live before it draws its first screen. ~90 ms of the 2 s.
    beginStep("Initialising Lidar");
    const bool lidarOk = lidar.init();
    endStep(lidarOk, "VL6180X INIT FAILED");

    while (millis() - splashShownMs < UI_SPLASH_MS) {}

    // Blocking. The control loop does not exist yet, so there is nothing to
    // starve, and the operator takes as long as they take.
    const RunConfig cfg = runStartupUI(display, lidar, leftMotor, rightMotor, i2cRepairer);

    // After the wizard, not before it: this is a 3 s measurement of the gyro's
    // zero-rate output and it wants the robot settled, which it is not while
    // someone is spinning its wheels.
    beginStep("Initialising IMU Observer (P)");
    if (!imu.init(IMU::GyroScale::DPS_1000, IMU::AccelScale::G_4, IMU::LowPassFrequency::HZ_44)) {
        endStep(false, "MPU6050 INIT FAILED");
    } else {
        imu_obsv.init();
        endStep(imu_obsv.ready(), "IMU OBSERVER INIT FAILED");
    }

    if (lidarOk) {
        lidar_obsv.setPrior(decltype(lidar_obsv)::PoseFunc::create<fusedPose>());
    }

    // The wheels were just turned by hand. Odometry integrates deltas, so the
    // marks have to move before the first tick or the whole hand-spin arrives
    // as one tick of motion.
    wheel_obsv.reset();

    // The world frame is maze-aligned -- origin at the start cell, North at
    // theta 0 -- so a start heading that is not North has to be seeded here.
    // This used to be Pose{0, 0, 0}, which was correct only because the heading
    // was compiled in as North.
    sf.set(Pose{0, 0, directionToTheta(cfg.heading)});

    beginStep("Configuring run");
    endStep(runner.configure(cfg), "RUNNER REJECTED THE CONFIGURATION");

    runBegin();
```

Keep the `Serial.begin` / `delay(1000)` / `Serial.println("Beginning setup:")` opening and the `previous_time = micros(); Serial.println("Setup complete!");` close as they are. Delete the old `Serial.print("Loading goal...");` line and the `// NOTE THAT X-AXIS IS FORWARDS` comment, which now sits above nothing — move that note into `runBegin()`'s comment if it is worth keeping.

- [ ] **Step 4: Remove the temporary configuration from `unseenMaze.h`**

Delete the two lines added in Task 6 Step 7:

```cpp
    // TEMPORARY -- replaced by runStartupUI() in the bring-up reorder.
    runner.configure(RunConfig{9, Cell{1, 1}, North, Cell{5, 5}});
```

`runBegin()` now assumes `runner.configure()` has already been called by `setup()`.

- [ ] **Step 5: Build both ways**

```bash
cd /Users/zimmylevi/Desktop/Uni/MTRN3100/Micromouse && ./compile.sh && ./compile.sh --debug
```

Expected: both clean.

- [ ] **Step 6: Flash and bench-test**

```bash
./compile.sh --flash
```

Work through the checklist, on the bench, robot not in a maze:

1. Splash shows for ~2 s with `v2.0` bottom-left.
2. Size screen: right wheel clockwise raises the number, counter-clockwise lowers it, and it stops at 16 and 2. **If clockwise lowers it, negate `dr` where it is read** — `rightMotor` is constructed with `reverse = true` and the sign of `count()` has not been verified against the dial.
3. Right-edge semicircle is drawn on every screen; left-edge one is absent on the size screen only.
4. Bottom-right dial spoke turns with the right wheel; bottom-left appears only on the start and goal screens.
5. A hand waved at the right sensor advances a screen and flashes the semicircle to an outline.
6. A hand held in front of the right sensor from power-on does **not** advance the first screen.
7. On the goal screen, setting goal equal to start shows `= START` and the right button will not advance.
8. Countdown counts 5 to 1 with no chrome, then the run starts.
9. Pick the robot up mid-wizard and put it in a corridor: after ~1.2 s neither button fires.

- [ ] **Step 7: Commit**

```bash
git add -A firmware/micromouse
git commit -m "Wire the startup wizard into bring-up"
```

---

### Task 11: Deck verification

No code unless something fails. This is the task that decides whether the runtime grid actually works.

**Files:** none expected.

- [ ] **Step 1: Self-check across all four sizes**

```bash
cd /Users/zimmylevi/Desktop/Uni/MTRN3100/Micromouse && ./compile.sh --debug --flash
```

Expected on Serial at 115200: `self-check PASSED`, with no `FAIL:` lines across n = 2, 5, 9 and 16.

- [ ] **Step 2: Full run on the competition deck at n = 9**

Select 9, start `(1, 1)` facing North, goal `(5, 5)` — the configuration that was compiled in before this change, so the run should look exactly like it did.

Confirm: the map pane fits the 9x9 grid, `EXPL` reaches 100%, the phase goes `EXPL` -> `HOME` -> `PLAN` -> `EXEC` -> `DONE`, and it does not report `FAULT`.

Note that with the corner crop gone the robot will now probe the deck's chamfered corners rather than assuming them, so exploration takes a few more moves than it used to. That is expected.

- [ ] **Step 3: Small run at n = 5**

Set up a 5x5 maze. Confirm the map pane fits the 5x5 grid and fills the pane — this is the check that `MazeWallMap`'s extent follows the runtime grid rather than the capacity — and that `EXPL` reaches 100%.

- [ ] **Step 4: Record the results**

If everything passes, note the RAM and flash figures from the build output for the README update in Task 12. If anything fails, stop and diagnose before continuing — a failure here means a `N` that should be `n`, and the self-check output narrows which one.

---

### Task 12: Documentation

**Files:**
- Modify: `firmware/README.md:128,178-245`
- Modify: `README.md:182`
- Modify: `scripts/README.md`
- Modify: `firmware-sim/README.md:49`, `firmware-sim/scenarios.py:6-8,114,255`

- [ ] **Step 1: `firmware/README.md`**

Delete the `#define TASK 43` section and the `| | task42.h | task43.h |` comparison table. Replace with a description of `unseenMaze.h` as the only run, and a section on the boot wizard covering the five screens, the two input kinds and the chrome. Update line 128 (`MotionPlanner is what task42.h drives`) to say `MotionPlanner` is retained but unbuilt, kept for a later upgrade of the race phase from static turns to smooth curves. Update the RAM figures from Task 11 Step 4. Line 214's "The whole of 4.3's configuration is four lines at the top of task43.h" becomes a pointer to the wizard. Line 245's "used by task42.h only" becomes "not used by the sketch; retained for the offline pipeline".

- [ ] **Step 2: `README.md:182`**

"Both `task42.h` and `task43.h` build `SensorFusion sf(obs_v, obs_p, 0.1)`" -> `unseenMaze.h` alone builds it.

- [ ] **Step 3: `scripts/README.md`**

Document `--debug` alongside `--db` and `--flash`, matching the `usage()` text from Task 2 Step 3.

- [ ] **Step 4: `firmware-sim`**

In `README.md:49` and `scenarios.py:6-8,114,255`, the statements about `task43.h` not being ported remain true. Add a note that `task42.h` no longer exists in the sketch and the `planned` scenario now mirrors a configuration the firmware does not build, so it is a retained regression scenario rather than a mirror of current firmware.

- [ ] **Step 5: Commit**

```bash
git add -A README.md firmware/README.md scripts/README.md firmware-sim
git commit -m "Docs: startup wizard, --debug, and the end of task 4.2"
```
