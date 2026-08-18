# firmware

The robot. One Arduino sketch, `micromouse/micromouse.ino`, and the header-only
libraries it is assembled from.

```sh
../compile.sh                        # build into firmware/build, ~15s
../compile.sh --flash                # ...and upload it to the connected board
../compile.sh --db                   # regenerate compile_commands.json for clangd
../compile.sh --help                 # full usage
```

Build with `compile.sh` rather than calling `arduino-cli compile` directly: it
empties `firmware/build` first, which is what keeps a build at ~15s instead of
the 4+ minutes arduino-cli takes when it re-enters a populated build directory.
`../compile.sh` is a forwarder for `../scripts/build.sh`.

Third-party libraries: **Embedded Template Library** (`etl/`), **Adafruit
SSD1306** and **Adafruit GFX**, **VL6180X** (Pololu). Everything else in
`micromouse/` is local. C++17; `std::array` and `std::variant` are used, but
nothing that allocates — `etl::vector`, `etl::string` and `etl::delegate` stand
in for the heap-using equivalents, and every container has a compile-time
capacity in `constants.h`.

Everything sits in one flat sketch directory because the Arduino build requires
it: headers beside the `.ino`, no include paths, no subfolders that are not
libraries.

## The control loop

`loop()` is five calls, at whatever rate the board manages (`dt` is measured,
not assumed; `MIN_LOOP_DT_S` only rejects a zero-length tick):

```
i2cRepairer.update()      probe the bus, rebuild it if it has wedged
sf.update(dt)             step every observer, fuse -> Pose + Velocity
taskUpdate(pose, dt)      the selected task's planner -> desired Velocity
mc.update(desired, ...)   IK -> per-wheel PID -> PWM
taskRender()              the selected task's display, throttled internally
```

Two of those five are hooks the task header supplies, which is what keeps
`loop()` free of any `#if` — see [Task blocks](#task-blocks).

Two rates are decoupled from that: the OLED refreshes every `OLED_REFRESH_MS`
(≈24 Hz), and the VL6180Xs free-run at `LIDAR_CONTINUOUS_PERIOD_MS` (10 ms), so
`LidarObserver` skips a solve rather than re-solving a reading it has already
seen.

## Modules

| header | owns |
| --- | --- |
| `constants.h` | every tunable number in the project, with the reason for its value |
| `pins.h` | the Nano R4 pin map. `Pin` is a `uint8_t` alias |
| `types.h` | `Pose`, `Vec2D`, `Velocity`, `Segment`, `RingBuffer`, the `Trig<>` LUTs, and the obstacle/`Map` layer the lidar localiser casts rays into |
| `kinematics.h` | differential-drive FK and IK, with the IK scaling both wheels down together when either exceeds `MAXIMUM_WHEEL_ANGULAR_VELOCITY` |
| `motor.h` | `Motor<ID>` — DRV8835 output plus encoder counting: a rising-edge interrupt on channel A, reading B for direction. Templated on ID so each instance gets its own static ISR |
| `control.h` | `PID` (saturating, NaN-guarded) and `MotionController` (velocity → IK → two PIDs → PWM) |
| `planners.h` | the five planners, below |
| `observers.h` | `WheelObserver`, `ImuObserver`, `FrontLidarObserver`, `LidarObserver`, `ModelObserver` |
| `sensorFusion.h` | `SensorFusion` — a trust-weighted blend of velocity sources and pose sources over the dead-reckoning model |
| `lidar.h` | VL6180X driver: bring-up and re-addressing, status decoding, non-blocking reads |
| `imu.h` | raw I2C MPU6050 driver with a rolling average |
| `i2cRepairer.h` | I2C bring-up and runtime recovery |
| `mazeMapper.h` | frontier exploration of an unknown N×N maze, and the shortest route through what the exploration actually saw. Cells only — no sensor, no motor, no pose |
| `mazeRunner.h` | `MazeMapper` closed loop against the robot: `Init → Explore → Plan → Race → Done`, non-blocking, one `update()` per tick |
| `mazeWallMap.h` | a `Map`-shaped view of the mapper's wall bits, so `LidarObserver` can localise against discovered walls. Derives every obstacle on demand and stores nothing |
| `maze_map.h`, `maze_path.h` | **generated** — see below |

The display is two headers, and one screen serves every task:

| header | owns |
| --- | --- |
| `oledDisplay.h` | the only owner of the SSD1306 — one framebuffer, one `begin()`, and the `OLED_REFRESH_MS` throttle. Also `OLEDView`, the mm → px projection |
| `oledScreen.h` | `OLEDScreen` — the map as *geometry* in a 64 px pane on the left (every panel a line at its own angle, every post and cylinder a filled disc, the route a polyline, the robot a dot and a heading tick), and five rows of values on the right. Templated on the map type, so it draws either `Map<S>` or `MazeWallMap` |

The values pane is the mode, then `X`, `Y` and `T` from the pose, then one
percentage whose meaning travels with its label — `E` for cells explored, `P`
for distance along a route:

```
+---------- 64 px ----------+--- 62 px ---+
| walls as lines, obstacles | EXPL        |  mode
| as filled circles, the    | X    340    |  mm
| route as a polyline, the  | Y    128    |  mm
| robot as a dot and a tick | T    -87    |  deg
|                           | E    42%    |  metric
+---------------------------+-------------+
```

Everything on it arrives through an `etl::delegate`, so the screen knows nothing
about `MotionPlanner`, `MazeRunner` or `MazeMapper`. The map extent it fits to
comes from `mapBounds()` in `types.h`, which is a property of the map rather than
of the panel; only the projection that consumes it lives in `oledDisplay.h`.

### Planners

All five expose the same shape: `update(pose, dt) -> Velocity`. Which one is
live is decided by the selected task header — see [Task blocks](#task-blocks).
`MotionPlanner` is what `task42.h` drives; `PSPlanner` is what `task43.h`'s
`MazeRunner` drives, and it drives each grid pose through a `PosePlanner` of its
own. `HeadingPlanner` and `DistancePlanner` are reached only by `firmware-sim`'s
retained 3.x scenarios.

| planner | what it drives to |
| --- | --- |
| `MotionPlanner` | a list of `Segment`s (straights and minor arcs). Feed-forward `curvature × v` plus proportional heading and lateral-error terms; advances a segment at `SEGMENT_ADVANCE_THRESHOLD` of its length. This is the one the generated path targets |
| `PosePlanner` | a single `Pose`: seek the position, then align to the heading. `setHeadingOnly()` for rotations in place |
| `PSPlanner` | a string of grid instructions — `"ffrfllfrlf"` — over 180 mm cells, driving each grid pose through a `PosePlanner` |
| `HeadingPlanner` | one heading, rotating in place |
| `DistancePlanner` | one forward distance, holding heading zero |

### Observers and fusion

`SensorFusion` owns a `ModelObserver` (dead reckoning: integrate the fused
velocity forward) and blends two kinds of source into it.

*Velocity sources* are averaged by trust, per component:

* `WheelObserver` — encoder deltas through the FK. Good `v`, noisy `omega`.
* `ImuObserver` — gyro Z, bias measured over 500 samples at rest in `init()`.
  The accelerometer is read but not integrated; it was too noisy to be useful.

*Pose sources* produce a correction that is folded into dead reckoning at the
gain handed to `SensorFusion` — `0.1` in both task headers, against a
`FusionWeights::PoseCorrectionGain` default of `0.2` — per tick, so a fix nudges
rather than teleports:

* `FrontLidarObserver` — front range as an x measurement. No longer wired by
  either task; `firmware-sim`'s `task32` scenario is the only thing that drives
  it, and it ignores its mount offset (see below).
* `LidarObserver<S>` — the real one. Casts the three beams into `MAZE_MAP`,
  gates each return (incidence angle, residual, implied heading), and runs a
  damped Levenberg–Marquardt solve for `(x, y, theta)` against a prior. The
  prior comes from a delegate you wire to the fused pose:

  ```cpp
  lidar_obsv.setPrior(decltype(lidar_obsv)::PoseFunc::create<fusedPose>());
  ```

  Unwired it falls back to its own last estimate, which is only good for
  bench-testing a single solve. The whole `LIDAR_OBSERVER_*` block in
  `constants.h` documents why each gate is where it is; the prior sigmas in
  particular are load-bearing, because three beams routinely leave one direction
  of pose space unobservable and something has to decide how the correction is
  split between translation and rotation.

## Task blocks

The two current tasks live in `task42.h` and `task43.h`, and `micromouse.ino`
picks one with a single `#define` near the top:

```cpp
#define TASK 43   // 42 -> task42.h, 43 -> task43.h
```

Anything other than 42 or 43 is an `#error`. The sketch itself holds only the
shared hardware — motors, IMU, lidar, `obs_v`, `dt`, the display and the
`MotionController` — plus a `setup()`/`loop()` skeleton with no `#if` in it.
The selected header is included part way down, after the objects it builds on
exist, and supplies the parts that differ:

| | task42.h | task43.h |
| --- | --- | --- |
| the maze is | known — fitted by CV from a photo | unknown; finding it is the exercise |
| pose source | `LidarObserver` over `MAZE_MAP` | `LidarObserver` over `MazeWallMap` |
| motion | `MotionPlanner(10, 0.06, 200)` | `MazeRunner` over `PSPlanner(8, 8)` |
| `taskBegin()` | `#include "maze_path.h"` | `runner.begin()` |
| `taskRender()` | `OLEDScreen` over `MAZE_MAP`, mode `CV` | `OLEDScreen` over `MazeWallMap`, mode `EXPL`/`HOME`/`EXEC` |

Both build `SensorFusion sf(obs_v, obs_p, 0.1)` and wire `setPrior()` in
`setup()`. Those two go together: the observer needs the prior to be worth
anything.

Each builds exactly one `OLEDScreen` and draws it every tick, for every phase
of the run. There is no renderer to select, which is what `OLEDDisplay::due()`
being *consuming* asks for: it hands the refresh window to its first caller, so
two renderers sharing a tick would silently starve whichever asked second.

4.3's mode is the one place the display reads a phase that `MazeRunner` does not
have. The leg back to the start cell is rule 3 of `MazeMapper::planMove` and sits
inside `Explore`, so `MazeMapper` exposes `homing()` and `homeProgress()` and
`screenMode()` folds them in — otherwise the screen would report `EXPL` for the
whole return trip.

The whole of 4.3's configuration is four lines at the top of `task43.h`:

```cpp
constexpr uint8_t MAZE_SIZE = 5;        // cells per side
mazeMapper::Cell startCell = {0, 0};
Direction startHeading     = North;
mazeMapper::Cell goalCell  = {2, 4};
```

`MAZE_SIZE` sizes every templated class below it, and cost grows as N². Change
those four and nothing else has to move.

Both headers declare the same names — `lidar_obsv`, `obs_p`, `sf`,
`fusedPose()` and the three `task*()` hooks — which is what lets `setup()` and
`loop()` be written once. They are alternatives rather than layers because
`MotionPlanner`'s segment array alone is about 10 kB: 4.2 links at 48% of RAM,
4.3 at 33%.

Earlier assessment tasks are no longer in the sketch. `firmware-sim`
reproduces each by name (`run.py task31` …); for reference, they were:

| block | fusion | planner | setup |
| --- | --- | --- | --- |
| TASK 3.1 | wheels + gyro | `MotionPlanner(10, 0.06)` | three hand-written segments — a 1 m straight, an arc, a return |
| TASK 3.2 | + `FrontLidarObserver` as an x source | `DistancePlanner(3, 0.06)` | `setTarget(200.0f)` — hold 200 mm off the wall |
| TASK 3.3 | wheels + gyro | `HeadingPlanner(5)` | `setTarget(PI/2)` |
| TASK 3.4 | wheels + gyro | `PSPlanner(10, 5)` | `addInstructions("ffrfllfrlf")` |

## Generated headers

`maze_map.h` and `maze_path.h` are written by `scripts/build_maze.sh`, and are
used by `task42.h` only — 4.3 discovers its maze instead.
**Do not edit them by hand.** `maze_map.h` records the photo it was fitted from,
the lattice fit RMS and the obstacle counts; `maze_path.h` records the start and
goal cells and the turn radius. Both record — critically — the start pose they
were exported against:

```
// Robot frame: x forward, y left, mm. The origin is the start pose
// [270.0, 270.0] mm heading 90 deg in map coordinates (auto), which is where a
// freshly reset odometry frame reads (0, 0, 0).
```

Both files must come from the same photo *and the same `--from`/`--theta0`*, or
the robot localises against a map offset from the path it is driving. That is
the entire reason `build_maze.sh` exists rather than two separate invocations.

`maze_map.h` is `constexpr`, so the map lives in flash and costs no RAM.
`maze_path.h` is not a header in any real sense — it is a run of
`planner.appendSegment(...)` statements, `#include`d *inside* `setup()`.

`.bak` beside each is the previous installed copy, kept by the wrapper.

## Things that will bite

* **The I2C bus wedges.** The renesas_uno core's `Wire` never recovers on its
  own: a timeout does not abort the in-flight FSP transfer, so one slave holding
  SDA low kills every device until the bus is manually clocked out and the
  driver reopened. `I2CRepairer` probes every 250 ms and rebuilds after three
  consecutive failures. Do not shorten `I2C_WIRE_TIMEOUT_US` below one SSD1306
  framebuffer chunk (≈5.7 ms at 400 kHz) or transfers get abandoned mid-flight.
* **The three VL6180Xs boot at the same address.** `LIDAR::init()` pulls all
  three low over their GPO pins, then brings them up one at a time and
  re-addresses each. GPO is driven **low or left as an input, never driven high**
  — the sensor pulls itself to 2.8 V and 5 V destroys it (`pins.h`).
* **Encoders are calibrated per wheel.** `ENC_RAD_PER_REV_LEFT`/`RIGHT` were
  hand-measured; readings are scaled so one turn reads exactly 2π. Re-measure
  after any drivetrain change.
* **Turn radius comes in bands.** Up to 26 mm, or 73–182 mm, and nothing
  between — the geometry is worked through in `path-planning/README.md`, and the
  bands are narrower than the axle-centred arithmetic suggests because the body
  rides ahead of the axle and swings wider through every turn. The obvious
  "slightly tighter than a cell" 70 mm cannot clear a pivot post.
* **The axle offset is not the same number here as in the planner.**
  `AXLE_DIST_FROM_CENTRE` is `20` mm; `AXLE_OFFSET_MM` in
  `path-planning/rrt_star.py` is `25.0`, and the bands above are derived from the
  25. Conservative in the direction that matters, but the two are not mirroring
  each other — see the root README's known divergences.
* **`Serial` in the loop costs milliseconds.** `loop()` is kept free of it;
  `setup()` prints freely because nothing is timing-critical yet. Add a print to
  the loop only while you are actually debugging, and take it out again.
