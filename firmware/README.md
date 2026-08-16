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
`../compile.sh` is a forwarder for `../scripts/build.sh`, which takes the sketch
to build as its first argument and defaults to this one.

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

`loop()` is four calls, at whatever rate the board manages (`dt` is measured,
not assumed; `MIN_LOOP_DT_S` only rejects a zero-length tick):

```
i2cRepairer.update()      probe the bus, rebuild it if it has wedged
sf.update(dt)             step every observer, fuse -> Pose + Velocity
planner.update(pose, dt)  where am I on the path -> desired Velocity
mc.update(desired, ...)   IK -> per-wheel PID -> PWM
```

Two rates are decoupled from that: the OLED refreshes every `OLED_REFRESH_MS`
(≈58 Hz), and the VL6180Xs free-run at `LIDAR_CONTINUOUS_PERIOD_MS` (10 ms), so
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
| `oled.h` | SSD1306 output. Takes `{label, callable}` pairs and lays them out in one or two columns |
| `i2cRepairer.h` | I2C bring-up and runtime recovery |
| `maze_map.h`, `maze_path.h` | **generated** — see below |

### Planners

All five expose the same shape: `update(pose, dt) -> Velocity`. Which one is
live is decided by which TASK block is uncommented in the `.ino`.

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

*Pose sources* produce a correction that is folded into dead reckoning at
`PoseCorrectionGain` (0.2) per tick, so a fix nudges rather than teleports:

* `FrontLidarObserver` — front range as an x measurement. Used by TASK 3.2.
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

`micromouse.ino` carries one commented block per assessment task. Uncomment
exactly one — each defines its own `obs_v`/`obs_p`, planner and setup body.

| block | fusion | planner | setup |
| --- | --- | --- | --- |
| TASK 3.1 | wheels + gyro | `MotionPlanner(10, 0.06)` | three hand-written segments — a 1 m straight, an arc, a return |
| TASK 3.2 | + `FrontLidarObserver` as an x source | `DistancePlanner(3, 0.06)` | `setTarget(200.0f)` — hold 200 mm off the wall |
| TASK 3.3 | wheels + gyro | `HeadingPlanner(5)` | `setTarget(PI/2)` |
| TASK 3.4 | wheels + gyro | `PSPlanner(10, 5)` | `addInstructions("ffrfllfrlf")` |
| TASK 4.1 / 4.2 | + `LidarObserver` over `MAZE_MAP` | `MotionPlanner(10, 0.06, 200)` | `#include "maze_path.h"` |

4.1/4.2 is what is uncommented today, with the lidar pose source itself
commented out of `SensorFusion` (`SensorFusion sf(obs_v)` rather than
`sf(obs_v, obs_p)`) — dead reckoning drives the generated path, and the
localiser is built but not fed in. Re-enable both that and `setPrior()` together;
the observer needs the prior to be worth anything.

`firmware-sim` reproduces each of these blocks by name (`run.py task31` …).

## Generated headers

`maze_map.h` and `maze_path.h` are written by `scripts/build_maze.sh`.
**Do not edit them by hand.** Each carries a header comment recording the photo
it came from, the lattice fit RMS, the obstacle counts and — critically — the
start pose it was exported against:

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
  rides 25 mm ahead of the axle and swings wider through every turn. The obvious
  "slightly tighter than a cell" 70 mm cannot clear a pivot post.
* **`Serial` in the loop costs milliseconds.** The prints left in `loop()` are
  commented out for that reason.
