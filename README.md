# MTRN3100 Micromouse

A micromouse firmware for exploring an unseen maze. Given a start cell, a start
heading and a goal — all chosen at boot through an on-panel startup wizard —
the robot maps the maze with its three range sensors, plans a route over what
it found, and races it. Earlier revisions could also be told the maze from an
overhead photo, planned offline and driven while localising against a map
exported from that photo; that whole pipeline has since been deleted.

Four directories, each with its own README:

| directory | what it is | language |
| --- | --- | --- |
| [`firmware/`](firmware/) | the robot. `micromouse.ino` plus the header-only libraries it is built from | C++17, Arduino |
| [`scripts/`](scripts/) | every entry point: `build.sh` compiles the sketch, `build_maze.sh` ran the photo-to-headers pipeline (inert, see below), `export_splash.py` turns the splash art into a bitmap | zsh, Python |
| [`hardware/`](hardware/) | CAD for the printed chassis and the maze wall panels, and the OLED splash art | Bambu Studio, Grasshopper, Aseprite |
| [`notes/`](notes/) | course handouts and working notes. Nothing here is read by any code | PDF, JPEG |

Both scripts take `--help`, run from any directory, and derive their paths from
the repo root rather than the caller's cwd.

## The robot

An Arduino Nano R4 (`arduino:renesas_uno:nanor4`) driving a two-wheel
differential base, with everything but the motors and encoders on one I2C bus.

| part | detail | where it is described |
| --- | --- | --- |
| 2 × DC motor + quadrature encoder | 700 CPR, 31.4 mm effective wheel diameter, 92.5 mm axle | `firmware/micromouse/motor.h`, `constants.h` |
| DRV8835 H-bridge | PWM on `xEN`, direction on `xPH` | `pins.h` |
| MPU6050 IMU | gyro Z only; accelerometer measured and left unused | `imu.h`, `observers.h` |
| 3 × VL6180X ToF | front / left / right, addresses `0x30`–`0x32`, re-addressed at boot over their GPO pins | `lidar.h` |
| SSD1306 OLED 128×64 | a splash through bring-up, then one screen for the run: the map on the left, the run's numbers on the right | `oledDisplay.h`, `oledScreen.h`, `oledSplash.h` |

Pin assignment lives in one place, `firmware/micromouse/pins.h`. Every tunable
number lives in one place, `firmware/micromouse/constants.h`, and each one
carries the reasoning for its value.

The maze is the full-size standard: 180 mm cells, 12 mm panels and posts.

## Getting started

### Flash the robot

Open `firmware/micromouse/micromouse.ino` in the Arduino IDE, or:

```sh
./compile.sh                        # build into firmware/build, ~15s (--help for options)
./compile.sh --flash                # ...and upload it to the connected board, ~11s more
```

`--flash` finds the port by asking `arduino-cli` which one has a board matching
the FQBN on it, so nothing has to be hardcoded; pass `--port /dev/cu.usbmodemXXXX`
to override it. The port is resolved *before* the build, so an unplugged board
fails immediately instead of after a full compile.

`./compile.sh` forwards to `./scripts/build.sh`.

Libraries: Embedded Template Library, Adafruit SSD1306 (and Adafruit GFX),
VL6180X. The sketch builds one run, Unseen Maze: explore an unseen grid, plan a
route through what was found, then race it. Everything that run used to take as
a compile-time constant — maze size, start cell, start heading, goal — is
chosen at boot instead, through an on-panel startup wizard; see
[`firmware/README.md`](firmware/README.md) for what it does.

### Plan a maze

```sh
./scripts/build_maze.sh 5.png --from 1,1 --to 3,3   # overlay on screen, headers installed
./scripts/build_maze.sh 5.png --no-install          # ...or leave the firmware alone
```

This was the only supported way to produce the pair, because the map and the
path have to be exported against the *same* start pose or the robot localises
against a map offset from the path it is driving; the wrapper exists to enforce
that. It still lives in `scripts/`, but the offline photo-to-headers project it
drove — its `uv` environment, `maze_demo.py`, `export_map.py` — has been
deleted, so it now has nothing left to invoke. `maze_map.h` and `maze_path.h`
in `firmware/micromouse/` are whatever that pipeline last produced; nothing in
the repo can regenerate them anymore.

## Conventions that cross directory boundaries

**Two frames, two handednesses.** The map was in image convention — +x east, +y
south, origin at the lattice corner — and left-handed. The robot's frame is
right-handed: **x forward, y left**, theta CCW, origin wherever odometry was last
reset. `segments.py::to_firmware`, in the now-deleted offline planner, mirrored
between the two and flipped every `Left`/`Right` as it went, before writing the
robot-frame result into `maze_map.h`/`maze_path.h`. Everything left in the repo
— `firmware/` included — is robot frame only.

The same convention carries into cells: `Direction` in `types.h` has North
stepping +x and West stepping +y, so a route `MazeMapper` produces feeds
`PSPlanner` with no axis or sign fixup in between.

**Units are mm, radians and seconds,** everywhere, including inside the
generated headers.

**`maze_map.h`, `maze_path.h` and `splash_screen.h` are generated.** Do not
hand-edit them; the `snake_case` name is the signal. The first two used to be
regenerated by re-running `build_maze.sh` (see [Plan a maze](#plan-a-maze));
each kept the previous copy as `.bak` beside it: `maze_map.h`
records the photo, the lattice fit RMS and the start pose it was exported
against; `maze_path.h` records the start and goal cells, the turn radius and the
same frame note. `splash_screen.h` is the OLED logo, and `build.sh --flash`
re-exports it from `hardware/Splashscreen.png` on its own.

## Known divergences

Real, verified, and worth knowing before trusting a run:

* **The simulator ran a pose correction gain of `0.2` where the sketch passes
  `0.1`.** `unseenMaze.h` — the only header the sketch builds, now that
  `task42.h` is gone — builds `SensorFusion sf(obs_v, obs_p, 0.1)`, against the
  simulator's `FusionWeights::PoseCorrectionGain` default of `0.2`. A lidar fix
  folded in twice as fast there as it does on the robot.
* **`FrontLidarObserver` ignores its 57 mm mount offset**
  (`LIDAR_MOUNT_FRONT_X`). It writes `pose.x = -getReading(Front)`, so the pose
  it reports is 57 mm short. Fine as a relative wall-distance measurement,
  wrong as a pose.
* **`scripts/build.sh` still advertises a `lidar` target.** It points at
  `firmware-ds/lidar`, a bring-up sketch that has been deleted from the repo, so
  `./scripts/build.sh lidar` fails. `micromouse` is the only target that builds.

---

MTRN3100 micromouse project — Zimmy Levi (z5587840).
