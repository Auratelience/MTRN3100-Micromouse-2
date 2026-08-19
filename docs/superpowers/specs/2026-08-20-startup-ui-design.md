# Startup UI and runtime maze configuration — firmware v2.0

Zimmy Levi z5587840 — 2026-08-20

## Summary

Task 4.2 is dropped. Task 4.3 becomes "Unseen Maze", and is the only thing the
sketch builds. Everything 4.3 previously took as a compile-time constant — maze
size, start cell, start heading, goal cell — is chosen at boot through an
on-panel wizard driven by the wheel encoders and the two side lidars.

Firmware version is `v2.0`, shown on the splash.

## 1. Scope

**Deleted**

- `firmware/micromouse/task42.h`
- the `TASK` define, its `#error` guard and its `#if` in `micromouse.ino`
- `MAZE_CORNER_CROP` in `constants.h`
- `MazeRunner::croppedCell`, `sealCroppedCells`, `croppedCells`, `reachableCells`
- in `task43.h`: the commented-out `bool lambda = []() {...}()` block, and the
  `#include <sys/wait.h>` — a POSIX header that has no business in Arduino
  firmware and is presumably an editor autocomplete accident

**Kept, unreferenced — zero RAM and zero flash, because nothing instantiates or
includes them**

- `MotionPlanner` in `planners.h`, retained for a later upgrade of the race
  phase from static turns to smooth curves
- `maze_map.h`, `maze_path.h`, `scripts/build_maze.sh`, `path-planning/`

Note for that later upgrade: the ~10 kB is `task42.h`'s *instance*, not the
class. When `MotionPlanner` is eventually built alongside `MazeMapper`, a route
discovered by the mapper is bounded by the maze rather than by a CV path, so
`PATH_SEGMENTS_MAX_LEN` can come down a long way from its current value.

**Renamed**

- `task43.h` -> `unseenMaze.h`; "Task 4.3" -> "Unseen Maze" throughout
- `taskBegin` / `taskUpdate` / `taskRender` -> `runBegin` / `runUpdate` /
  `runRender` (3 definitions, 3 call sites)
- stale 4.1/4.2 references in `observers.h` (lines ~324-341) and
  `mazeWallMap.h` (lines ~20, ~32), and the task table in `firmware/README.md`

**New**

- `firmware/micromouse/startupUI.h`
- `--debug` in `scripts/build.sh`

## 2. Runtime maze size

`MAZE_SIZE` stops being the grid dimension and becomes a capacity.

```cpp
constexpr uint8_t MAZE_SIZE_MIN = 2;
constexpr uint8_t MAZE_SIZE_MAX = 16;   // template capacity; RAM is sized here
```

RAM at the capacity, on the Nano R4's 32 kB: `MazeMapper<16>` holds ~1.5 kB, and
its deepest BFS — the frontier pruning, which wants two `uint16_t[16][16]`
distance fields and a `Cell[256]` queue at once — borrows ~1.5 kB of stack. The
searches run one at a time, so the borrow does not stack. 4.3 links at ~33%
today at N = 9, and `task42.h`'s deletion frees ~10 kB, so the capacity increase
is comfortably affordable.

The split is **template parameter `N` = array capacity, runtime `n` = actual
grid**.

### MazeMapper<N>

- keeps `[N][N]` storage and `MAX_CELLS` for array sizing
- gains `uint8_t n` and `configure(uint8_t n, Cell start, Direction heading, Cell goal)`
- the constructor loses its `(startCell, startHeading, goalCell)` arguments —
  configuration arrives at runtime now, and having both a constructor and a
  `configure()` set the same three fields would be two ways to do one thing
- `inside()` tests against `n`
- `begin()` seeds the perimeter to `n`
- every BFS bound (`distancesFrom`, the frontier pruning, `buildShortestPath`)
  iterates to `n`, not `N`
- new `cellCount()` returning `n * n`

### MazeWallMap<N>

`NS_COUNT`, `EW_COUNT`, `POST_COUNT`, `COUNT` and `size()` are `static
constexpr` today, and the index -> (a, b) decomposition strides on `N`. All
become **runtime, strided on the mapper's `n`**.

Nothing indexes a stored array with these — every slot is derived — so a
runtime stride is free.

This matters for speed, not just correctness. `COUNT` is 280 slots at n = 9 and
833 at n = 16, and `candidates()` sweeps all of them on every lidar solve.
Leaving the stride pinned at the capacity would make a 5x5 maze pay the 833-slot
sweep, once per solve, for the whole run. That is a control-loop cost with no
upside.

`present()` needs no size test of its own: it resolves through
`mapper.hasWall()`, which is false outside `n` once `inside()` respects `n`. So
`mapBounds()` fits the display to the selected maze automatically.

### MazeRunner<N>

- constructor becomes `MazeRunner(LIDAR&, PSPlanner&)`
- gains `configure(const RunConfig&)`, forwarding to the mapper
- crop machinery deleted (see Scope)
- **bug fix:** `exploreProgress()` divides by `MazeMapper<N>::MAX_CELLS`. Once
  that is a capacity rather than the grid, a 5x5 maze would read 10% at
  completion. It becomes `mapper.cellCount()`.
- gains `homing()`, `faulted()` and `homeProgress()` pass-throughs, so the
  display stops reaching through `runner.mapper.*`

### Corner crop

Removed entirely, at every size. The maze is unseen; the robot has no basis for
assuming chamfered corners. Exploration discovers walled-off corners itself, at
the cost of a few probing moves. `exploreProgress()` already documents that it
under-reads when part of the maze is unreachable.

## 3. The startup UI

### Interface

`startupUI.h` exposes one free function, so it holds no globals and can be a
normal top-of-file include:

```cpp
struct RunConfig {
    uint8_t   size;
    Cell      start;
    Direction heading;
    Cell      goal;
};

RunConfig runStartupUI(OLEDDisplay&, LIDAR&, BaseMotor& left, BaseMotor& right,
                       I2CRepairer&);
```

It **blocks inside `setup()`**. The control loop does not exist yet and there is
nothing to service but I2C, so a blocking wizard is much simpler than a `loop()`
state machine, and it keeps `loop()` the wiring diagram it currently is. It
calls `i2cRepairer.update()` every iteration, since it may hold the bus for
minutes.

The motors are never driven during the wizard, so the wheels turn freely.

### Screens

| # | Screen | Input |
|---|--------|-------|
| 0 | Splash + `FIRMWARE_VERSION`, 2 s | none |
| 1 | Maze size, `MAZE_SIZE_MIN`..`MAZE_SIZE_MAX` | right wheel |
| 2 | Start cell `X:_ Y:_` | left wheel = x, right wheel = y |
| 3 | Start heading `N / E / S / W` | right wheel |
| 4 | Goal cell `X:_ Y:_` | left wheel = x, right wheel = y |
| 5 | 5 s countdown | right lidar skips, left lidar returns to 4 |

Text only, drawn through `display.gfx()`. No `OLEDScreen` involvement.

### Encoder dials

Read raw signed `motor.count()`, never `angularDisplacement()` — integer, no
float accumulation.

```cpp
constexpr int UI_ENCODER_DETENT_COUNTS = ENC_CPR / 12;   // 58; ~12 clicks/rev
```

Values clamp at their limits rather than wrapping, matching "up to max, down to
min".

The right motor is constructed with `reverse = true`, so the sign of its count
must be checked on the bench against "clockwise increases". This is a one-line
constant, not a structural risk.

**Heading is not an integer dial.** `Direction : int { North = 0, West = 1,
South = 2, East = -1 }` — East is −1, so incrementing the enum does not walk the
compass. Screen 3 indexes an explicit clockwise table:

```cpp
constexpr Direction UI_HEADING_ORDER[4] = {North, East, South, West};
```

### Lidar buttons — baseline-relative

Left lidar goes back, right lidar continues.

An absolute "a hand is nearer than a wall" threshold does not work here. The
side sensors are mounted at `LIDAR_MOUNT_LEFT_Y = 35.0f` / `RIGHT_Y = -35.0f`,
and a wall's inner face is 84 mm from a cell centre, so **a side sensor reads
about 49 mm to an adjacent wall**. There is not enough room between that and a
hand for a fixed threshold to be reliable.

Instead, each side sensor takes a **baseline** at wizard entry — whatever it
happens to be looking at, corridor or open floor:

```cpp
constexpr uint16_t UI_BUTTON_PRESS_DELTA_MM   = 25;  // below baseline
constexpr uint16_t UI_BUTTON_RELEASE_DELTA_MM = 15;  // back toward baseline
constexpr uint8_t  UI_BUTTON_DEBOUNCE_SAMPLES = 3;
```

- press: reading is more than `PRESS_DELTA` below baseline for
  `DEBOUNCE_SAMPLES` consecutive samples
- release: reading returns to within `RELEASE_DELTA` of baseline
- the button is **armed only after one clean release**, so a hand already in
  front of a sensor at boot cannot fire the first screen
- a press is consumed on its edge; the next press requires an intervening
  release

This behaves identically in a corridor and on open floor, which is the property
a fixed threshold cannot have.

### Validation

Dials clamp to `[0, n-1]`, so an out-of-range cell is unreachable by
construction. Screen 4 refuses to continue while `goal == start` and says so on
the panel.

### Back behaviour

Screen 1 is the first step; back is a no-op there. Back on the countdown
returns to screen 4.

## 4. Bring-up order

The current splash duration is accidental: it lasts ~3.2 s only because
`imu_obsv.init()`'s calibration window runs underneath it. That comes apart
here.

1. Serial, I2C
2. OLED init, `drawSplash` (now with the version overlay)
3. Motors init — needed for the dials
4. **Lidar init** — needed as UI input, ~90 ms
5. hold until 2000 ms have elapsed since the splash was drawn
6. `runStartupUI(...)`, blocking
7. 5 s countdown
8. IMU init and `imu_obsv.init()` — the ~3 s zero-rate calibration
9. **zero both encoders and re-seed `WheelObserver`** — the wheels were just
   spun by hand
10. `sf.set(Pose{0, 0, directionToTheta(cfg.heading)})`
11. `runner.configure(cfg)`, `runner.begin()`, `screen.init()`

Step 10 fixes a latent bug. The world frame is maze-aligned — origin at the
start cell, North = theta 0 (`types.h:1267`, `directionToTheta`) — and today's
`sf.set(Pose{0,0,0})` is correct only because the heading is hardcoded North. A
selectable heading without this makes the robot drive a map rotated from
reality.

Steps 7 and 8 stay in this order. Overlapping the calibration with the
countdown would save 3 s, but the operator's hand has just left the robot and
the gyro's zero-rate window wants it settled.

## 5. Version string

```cpp
constexpr char FIRMWARE_VERSION[] = "v2.0";
```

Drawn **bottom-left** on the splash. The bitmap is a full 128x64 with
`static_assert`s pinning it to the panel, so the version needs a filled black
rect behind it. May want an art tweak once it is on the panel.

## 6. `--debug` builds

`scripts/build.sh` gains `--debug`, which appends

```
--build-property compiler.cpp.extra_flags=-DMICROMOUSE_DEBUG=1
```

to the `arduino-cli compile` invocation. It slots into the existing `while`
option loop beside `--db` and `--flash`, and is documented in `usage()`.

The build directory is wiped every run, so there is no stale-define hazard.

Caveat: `--db` generates `compile_commands.json` without the define unless
`--debug` is also passed, so debug-guarded code reads as inactive in the editor.

Guarded by `MICROMOUSE_DEBUG`:

- the **boot self-check** (below)
- both existing `DIAGNOSTIC` blocks in `loop()` — the IMU read-failure report
  and the FIFO samples-per-cycle report

### Boot self-check

The risk in this change is concentrated in the runtime-`n` bounds work, not in
the UI. A missed `N` that should be `n` gives a mapper that explores phantom
cells, which is not obvious from the panel.

So under `MICROMOUSE_DEBUG`, before the wizard, run the mapper's bounds and
perimeter seeding across `n = 2, 5, 9, 16` and print pass/fail to Serial:
`inside()` agrees with `n`, the perimeter is sealed exactly at the `n`
boundary and nowhere else, `cellCount() == n*n`, and `MazeWallMap::size()`
matches the slot count for `n`.

## 7. Simplification folded in

Confined to code this change already touches. No unrelated refactoring.

1. **`loop()` shrinks from ~60 lines to ~25.** The two `DIAGNOSTIC` blocks are
   ~35 of those lines and one already says "Delete freely". Behind
   `MICROMOUSE_DEBUG` they stay one flag away without dominating the function.

2. **`setup()`'s `\b\b\b [OKAY]` pattern**, repeated six times with variations,
   collapses into a `beginStep(...)` / `endStep(ok, failureText)` pair. `setup()`
   is being restructured anyway.

3. **`MazeRunner` / `MazeMapper` single configuration path** — constructors stop
   taking start/heading/goal; `configure()` is the only way to set them. This
   also removes the global `startCell` / `startHeading` / `goalCell` from the
   task header, which are now runtime state.

4. **Law of Demeter on the display hooks.** `screenMode()` and `screenMetric()`
   reach through `runner.mapper.homing()`, `.faulted()`, `.homeProgress()`.
   `MazeRunner` exposes these directly.

5. **Dead code removed** from the renamed task header — the commented-out lambda
   and `<sys/wait.h>`.

Explicitly *not* combined: `MazeWallMap::nsWall` and `ewWall` are near-mirrors,
but they differ in axis and in their boundary special-case, and merging them
would obscure more than it saves.

The `taskBegin` / `taskUpdate` / `taskRender` hooks are kept (renamed). They
were an interface for two implementations and there is now one, but they cost
nothing and are what keeps `loop()` readable.

The "both headers declare the same names" contract that shaped `task43.h` is
dead and its comments go. The include-part-way-down-the-sketch arrangement
stays necessary regardless: `lidar_obsv`'s type depends on `MazeWallMap`, which
depends on the mapper, so the ordering constraint is real.

## 8. Testing

Honest position: no existing harness reaches this code. `firmware-sim` never
ported `task43.h`, and `mazeMapper.h` pulls in `Arduino.h` and ETL, so a host
build is its own project and is out of scope here.

Verification is therefore:

1. `./compile.sh` clean, and `./compile.sh --debug` clean
2. the boot self-check green at n = 2, 5, 9, 16
3. bench: each wizard screen, both dial directions, both lidar buttons, the
   armed-on-release behaviour with a hand held in front at boot, and the
   goal == start refusal
4. a full run on the deck at n = 9, and one small run at n = 5, confirming the
   panel fits the selected maze and `exploreProgress` reaches 100%

## 9. Documentation

- `firmware/README.md`: the `TASK` define section and the 4.2/4.3 comparison
  table go; the wizard, `--debug` and the RAM figures are updated
- `scripts/README.md`: `--debug`
- `README.md:182`: the "both task42.h and task43.h build SensorFusion" line
- `firmware-sim/README.md` and `scenarios.py`: their references to `task43.h`
  not being ported stay true, but the `task42.h` naming needs a note that the
  sketch no longer builds it
