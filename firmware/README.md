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
runUpdate(pose, dt)       the run's planner -> desired Velocity
mc.update(desired, ...)   IK -> per-wheel PID -> PWM
runRender()               the run's display, throttled internally
```

Two of those five are hooks `unseenMaze.h` supplies, which is what keeps
`loop()` free of any `#if` — see [Unseen Maze](#unseen-maze).

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
| `maze_map.h`, `maze_path.h`, `splash_screen.h` | **generated** — see below |

The display is three headers, and one screen serves the run:

| header | owns |
| --- | --- |
| `oledDisplay.h` | the only owner of the SSD1306 — one framebuffer, one `begin()`, and the `OLED_REFRESH_MS` throttle. Also `OLEDView`, the mm → px projection |
| `oledScreen.h` | `OLEDScreen` — the map as *geometry* in a 64 px pane on the left (every panel a line at its own angle, every post and cylinder a filled disc, the route a polyline, the robot a dot and a heading tick), and five rows of values on the right. Templated on the map type, so it draws either `Map<S>` or `MazeWallMap` |
| `oledSplash.h` | `drawSplash()` — the logo from the generated `splash_screen.h`, blitted once during bring-up |

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

Before any of that, `setup()` calls `drawSplash()` in place of a `clear()`, so
the panel shows the logo for the rest of bring-up and the first `OLEDScreen`
frame in `loop()` overwrites it. There is no splash state to leave and nothing in
`loop()` to gate: the run display is reached by the run starting. A `delay()`
after the call holds it longer.

**The OLED block belongs directly under the I2C bring-up, and moving it is what
breaks the splash.** `display.init()` needs nothing but `Wire`, and the logo is
on screen for exactly as long as the bring-up *below* it takes. Almost all of
that is `imu_obsv.init()`, which spends `IMU_STARTUP_SETTLE_MS` settling and then
a `IMU_CALIBRATION_MS` window averaging the gyro's zero-rate output — 3 s in
which the robot is required to stand still anyway. With the lidar's ~90 ms of
settle delays after it, the splash holds for about 3.2 s and needs no `delay()`
of its own.

It was written at the end of `setup()` first, after both of those, where all that
was left to outlast were two `Serial` prints and `runBegin()`: the logo was gone
inside ~10 ms and the only visible effect was the `clearDisplay()` inside
`display.init()` — a black blink.

Deliberately, `drawSplash()` does not ask `OLEDDisplay::due()`. That throttle is
*consuming* and opens once per `OLED_REFRESH_MS`, so a lone frame going through
it would be swallowed whenever bring-up had already drawn inside the window — and
the milliseconds of I2C it exists to protect are a cost `loop()` has, not
`setup()`.

Everything on it arrives through an `etl::delegate`, so the screen knows nothing
about `MotionPlanner`, `MazeRunner` or `MazeMapper`. The map extent it fits to
comes from `mapBounds()` in `types.h`, which is a property of the map rather than
of the panel; only the projection that consumes it lives in `oledDisplay.h`.

### Planners

All five expose the same shape: `update(pose, dt) -> Velocity`, but only one
drives the robot today. `PSPlanner` is what `unseenMaze.h`'s `MazeRunner`
drives — see [Unseen Maze](#unseen-maze) — and it drives each grid pose
through a `PosePlanner` of its own. `MotionPlanner` is retained but unbuilt:
nothing instantiates or includes it, so it costs zero RAM and zero flash, and
it is kept for a later upgrade of the race phase from static turns to smooth
curves. `HeadingPlanner` and `DistancePlanner` are retained the same way, now
that the TASK 3.x exercises that used to reach them are gone from the sketch
(see below).

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
gain handed to `SensorFusion` — `0.1` in `unseenMaze.h`, against a
`FusionWeights::PoseCorrectionGain` default of `0.2` — per tick, so a fix nudges
rather than teleports:

* `FrontLidarObserver` — front range as an x measurement. Nothing wires it
  today; the TASK 3.2 exercise that used to drive it is gone from the sketch,
  and it ignores its mount offset regardless (see below).
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

## Unseen Maze

`micromouse.ino` builds exactly one run — Unseen Maze, in `unseenMaze.h` —
and there is no `#define` left to pick between two, nor an `#error` guarding
it. The sketch itself holds only the shared hardware — motors, IMU, lidar,
`obs_v`, `dt`, the display and the `MotionController` — plus a
`setup()`/`loop()` skeleton with no `#if` in it. `unseenMaze.h` is included
part way down, after the objects it builds on exist, and supplies the rest:
`LidarObserver` over a discovered `MazeWallMap` rather than a photographed
`MAZE_MAP`, `MazeRunner` driving a `PSPlanner`, the
`SensorFusion sf(obs_v, obs_p, 0.1)` that wires `setPrior()` in `setup()`, and
the `runBegin()` / `runUpdate()` / `runRender()` hooks `loop()` calls.

It builds exactly one `OLEDScreen` and draws it every tick, for every phase of
the run. There is no renderer to select, which is what `OLEDDisplay::due()`
being *consuming* would otherwise make a hazard: it hands the refresh window to
its first caller, so two renderers sharing a tick would silently starve
whichever asked second.

The run's mode is the one place the display reads a phase that `MazeRunner`
does not have of its own. The leg back to the start cell is rule 3 of
`MazeMapper::planMove` and sits inside `Explore`, so `MazeMapper` exposes
`homing()` and `homeProgress()` (and `faulted()` alongside them), `MazeRunner`
passes all three straight through, and `screenMode()` folds them in —
otherwise the screen would report `EXPL` for the whole return trip.

Configuration — maze size, start cell, start heading and goal — used to be
four lines at the top of `task43.h`. None of it is compiled in any more: every
one of those four is chosen at boot through the startup wizard below, and lands
in a `RunConfig` that `runner.configure()` takes at the end of `setup()`.

Today's build: `./compile.sh` links at 111812 bytes of flash (42%) and 14228
bytes of RAM (43%) on the Nano R4's 32 kB. `./compile.sh --debug` additionally
builds the boot self-check and the two `loop()` diagnostics behind
`MICROMOUSE_DEBUG` — see [`scripts/README.md`](../scripts/README.md) — and
comes to 114284 bytes of flash (43%) and 14244 bytes of RAM (43%). Both figures
have room to spare because deleting `task42.h` freed the ~10 kB
`MotionPlanner` instance that used to share the binary with it, which is what
makes the runtime maze-size capacity below comfortably affordable.

### The startup wizard

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

It **blocks inside `setup()`**. The control loop does not exist yet and there
is nothing to service but I2C, so a blocking wizard is simpler than a `loop()`
state machine of its own, and it keeps `loop()` the wiring diagram it already
is; it calls `i2cRepairer.update()` every iteration, since it may hold the bus
for minutes. The motors are never driven during the wizard, so the wheels turn
freely and the operator's hands are the only actuator in the room.

Five screens, each with its own input:

| # | screen | input |
| --- | --- | --- |
| 0 | splash + `FIRMWARE_VERSION`, 2 s | none |
| 1 | maze size, `MAZE_SIZE_MIN`..`MAZE_SIZE_MAX` | right wheel |
| 2 | start cell `X:_ Y:_` | left wheel = x, right wheel = y |
| 3 | start heading `N`/`E`/`S`/`W` | right wheel |
| 4 | goal cell `X:_ Y:_` | left wheel = x, right wheel = y |
| 5 | 5 s countdown | none, by default |

There are two input kinds, and neither is a button in the usual sense.

**The wheel encoders are detented dials.** Each dial reads the raw signed
`motor.count()` — never `angularDisplacement()`, so there is no float
accumulation to drift — and turns it into one click every

```cpp
constexpr int UI_ENCODER_DETENT_COUNTS = ENC_CPR / 12;   // 58; ~12 clicks/rev
```

counts, clamping at its limits (`[0, n-1]`, or the heading table's four
entries) rather than wrapping. Heading is not read as an integer dial, because
`Direction : int { North = 0, West = 1, South = 2, East = -1 }` does not walk
the compass in enum order; screen 3 indexes an explicit clockwise table
instead.

**The two side lidars are momentary buttons**, left for back and right for
continue — but a fixed "hand versus wall" distance threshold does not work
here. The side sensors mount at `LIDAR_MOUNT_LEFT_Y`/`RIGHT_Y = ±35 mm`, and a
wall's inner face sits 84 mm from a cell centre, so a side beam reads only
about 49 mm to an adjacent wall — not enough room between that and a hand for
a fixed number to be reliable. Each sensor instead takes a **baseline** at
wizard entry, whatever it happens to be looking at, and calls a reading a
press once it sits more than `UI_BUTTON_PRESS_DELTA_MM` (25 mm) below that
baseline for `UI_BUTTON_DEBOUNCE_SAMPLES` (3) consecutive samples; the button
re-arms only once the reading returns within `UI_BUTTON_RELEASE_DELTA_MM`
(15 mm) of the baseline, so a hand already in front of a sensor at boot cannot
fire the first screen. Because placing the robot into a corridor is itself a
~150 mm drop from open bench — six times the press delta — the baseline
adapts: any reading that has sat away from it for longer than
`UI_BUTTON_BASELINE_ADOPT_MS` (1200 ms) without being consumed as a press is
adopted as the new baseline. A hand tap is short and fires; a wall that
appears and stays is absorbed and does not.

Every screen draws a **chrome** layer answering "what can I do here?": a
filled semicircle on the panel's edge for a live lidar button, and a
circle-with-spoke in the corresponding bottom corner for a live wheel dial,
its spoke angle tracking the raw encoder count 1:1 so it reads as live rather
than stepped. A consumed press blinks its semicircle for `UI_BLINK_MS`. Per
screen: size binds the right button and right dial only; the start and goal
cell screens bind all four; heading binds both buttons but only the right
dial; the countdown binds nothing unless the compile-time skip/back constants
are turned on, which they are not by default, because a phantom input while
the robot is being carried into the maze is worse than a wizard that cannot be
interrupted from the countdown alone.

Screen 4 refuses to continue while the goal equals the start cell, and says so
on the panel — `= START` — rather than accepting a run with nowhere to
explore to.

If the OLED or the lidar fails to initialise, the wizard has no screen to draw
and no button to read, so `setup()` skips it entirely and falls back to the
pre-wizard default, `RunConfig{9, Cell{1, 1}, North, Cell{5, 5}}`, reporting the
fallback over `Serial`. The wizard's only way out is its own lidar buttons and
its only output is the panel, so with either sensor dead there is no gesture
left to advance a screen and nothing to show if there were.

Earlier assessment tasks are no longer in the sketch, and the simulator that
used to reproduce them by name is gone too; for reference, they were:

| block | fusion | planner | setup |
| --- | --- | --- | --- |
| TASK 3.1 | wheels + gyro | `MotionPlanner(10, 0.06)` | three hand-written segments — a 1 m straight, an arc, a return |
| TASK 3.2 | + `FrontLidarObserver` as an x source | `DistancePlanner(3, 0.06)` | `setTarget(200.0f)` — hold 200 mm off the wall |
| TASK 3.3 | wheels + gyro | `HeadingPlanner(5)` | `setTarget(PI/2)` |
| TASK 3.4 | wheels + gyro | `PSPlanner(10, 5)` | `addInstructions("ffrfllfrlf")` |

## Generated headers

`maze_map.h` and `maze_path.h` are written by `scripts/build_maze.sh`. Neither
is used by the sketch — the maze is unseen, and `unseenMaze.h` discovers it
instead. `build_maze.sh` still lives in `scripts/`, but the offline
photo-to-headers project it drove — its `uv` environment, `maze_demo.py` and
`export_map.py` — has been deleted, so it now has nothing left to invoke. What
is committed here is whatever that pipeline last produced.
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

`splash_screen.h` is separate from those two and unrelated to the maze:
`scripts/export_splash.py` writes it from `hardware/Splashscreen.png` as a
`constexpr uint8_t[1024]` in `drawBitmap`'s own layout — row-major, 16 bytes per
row, MSB leftmost, a set bit lit — so it is in flash and costs no RAM either.
`scripts/build.sh --flash` re-exports it before every compile it is going to
upload, because a stale logo is invisible until the panel lights up. Its
`snake_case` name is the same signal the maze headers carry: **generated, do not
edit by hand.** The hand-written half is `oledSplash.h`, which is what a change
to how the splash is *drawn* should touch.

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
  between — narrower than the axle-centred arithmetic suggests, because the
  body rides ahead of the axle and swings wider through every turn. The obvious
  "slightly tighter than a cell" 70 mm cannot clear a pivot post.
* **`Serial` in the loop costs milliseconds.** `loop()` is kept free of it;
  `setup()` prints freely because nothing is timing-critical yet. Add a print to
  the loop only while you are actually debugging, and take it out again.
