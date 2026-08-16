# MTRN3100 Micromouse

A micromouse that is told the maze rather than made to explore it: an overhead
photo of the deck goes in, a collision-checked path in the firmware's own
`Segment` alphabet comes out, and the robot drives it while localising against a
map exported from the same photo.

```
                    path-planning/                     firmware/
   maze photo  ->   photo -> lattice -> map        ->  maze_map.h   ->  Arduino
   (.png/.jpg)      map   -> Dubins RRT* -> path        maze_path.h      Nano R4

                                    firmware-sim/
                          the same two headers, driven against a
                          line-by-line Python port of the sketch
```

Six directories, each with its own README:

| directory | what it is | language |
| --- | --- | --- |
| [`firmware/`](firmware/) | the robot. `micromouse.ino` plus the header-only libraries it is built from | C++17, Arduino |
| [`firmware-ds/`](firmware-ds/) | a cut-down sketch that does nothing but print the three lidar ranges — hardware bring-up | C++17, Arduino |
| [`firmware-sim/`](firmware-sim/) | `micromouse.ino`'s control loop with a simulated robot underneath it, and 256 tests over it | Python, stdlib only |
| [`path-planning/`](path-planning/) | photo in, `maze_map.h` and `maze_path.h` out | Python + OpenCV/NumPy/SciPy, via `uv` |
| [`scripts/`](scripts/) | every shell entry point: `build.sh` compiles a sketch, `build_maze.sh` runs the photo-to-headers pipeline | zsh |
| [`notes/`](notes/) | CAD for the printed frame and the PCB | Bambu Studio, Rhino |

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
| SSD1306 OLED 128×64 | live pose and loop `dt` | `oled.h` |

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

`./compile.sh` forwards to `./scripts/build.sh`, which takes the sketch to build
as its first argument — `./scripts/build.sh lidar --flash` builds and flashes the
bring-up sketch in [`firmware-ds/`](firmware-ds/) the same way.

Libraries: Embedded Template Library, Adafruit SSD1306 (and Adafruit GFX),
VL6180X. See [`firmware/README.md`](firmware/README.md) for which task block to
uncomment and what each one demonstrates.

### Plan a maze

```sh
./scripts/build_maze.sh 2.jpg --from 1,1 --to 5,3   # overlay on screen, headers installed
./scripts/build_maze.sh 2.jpg --no-install          # ...or leave the firmware alone
```

This is the only supported way to produce the pair, because the map and the path
have to be exported against the *same* start pose or the robot localises against
a map offset from the path it is driving. The wrapper exists to enforce that.
Details in [`path-planning/README.md`](path-planning/README.md).

The first run lets `uv` build the environment from `path-planning/pyproject.toml`
(Python ≥ 3.14, OpenCV, NumPy, SciPy).

### Drive it without a robot

```sh
cd firmware-sim
python3 -P run.py --viz --open                # watch it drive the installed headers
python3 -P run.py task34                      # a retained TASK 3.x regression scenario
```

[`firmware-sim/README.md`](firmware-sim/README.md) covers the arguments and what
the sim is and is not a port of.

### Run the tests

The sim's 256 tests, from the repo root:

```sh
python3 -P -c "
import importlib, pathlib, sys, unittest
sys.path.insert(0, '.')
suite = unittest.TestSuite()
for p in sorted(pathlib.Path('firmware-sim/tests').glob('test_*.py')):
    suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(
        importlib.import_module('firmware-sim.tests.' + p.stem)))
sys.exit(not unittest.TextTestRunner().run(suite).wasSuccessful())
"
```

and the planner's own checks:

```sh
cd path-planning
uv run python selftest.py            # Dubins, segments, world, planner
uv run python selftest.py --image    # ...and the CV pass, which needs mazes/1.png
```

The `--image` pass reads `mazes/1.png`, which is not tracked — see
[`path-planning/mazes/README.md`](path-planning/mazes/README.md).

**Why `-P`, and why not `unittest discover`.** `firmware-sim/types.py` mirrors
the firmware's `types.h` and so shadows the standard library's `types` module
whenever its directory lands on `sys.path` — which is exactly what running a
script from that directory does. `-P` (or `PYTHONSAFEPATH=1`) keeps it off.
`python3 -m unittest discover -s . -t ..` additionally needs the loader to
accept a namespace package: the directory is called `firmware-sim`, a hyphen is
not a legal identifier, and there is no `__init__.py` to fall back on. CPython
3.11 through 3.14 refuse it with *Start directory is not importable*, hence the
explicit loader above.

## Conventions that cross directory boundaries

**Two frames, two handednesses.** The map is in image convention — +x east, +y
south, origin at the lattice corner — and is left-handed. The robot's frame is
right-handed: **x forward, y left**, theta CCW, origin wherever odometry was last
reset. `path-planning/segments.py::to_firmware` mirrors between them and flips
every `Left`/`Right` as it goes. Anything under `firmware/` is in the robot
frame; anything under `path-planning/` is in the map frame until it is emitted.

**Units are mm, radians and seconds,** everywhere, including inside the
generated headers.

**`maze_map.h` and `maze_path.h` are generated.** Do not hand-edit them; re-run
`build_maze.sh`. Each keeps the previous copy as `.bak` beside it. Their header
comments record the photo, the fit RMS and the start pose they were exported
against.

**Numbers that appear in two languages are mirrored, not shared.**
`firmware/micromouse/constants.h`, `firmware-sim/constants.py` and the geometry
constants in `path-planning/maze_map.py` carry the same values by hand. When the
C++ and the Python disagree, the C++ is right — that is the premise the sim is
built on.

## Known divergences

Real, verified, and worth knowing before trusting a run:

* **`fusePose`'s weights differ between the two firmware copies.**
  `firmware/micromouse/` and `firmware-sim/fusion.py` both start
  `xWeightTotal`/`yWeightTotal`/`dthetaWeightTotal` at `0.0f`;
  `firmware-ds/lidar/` still starts them at `1.0f`. At `1.0f` the model's vote is
  a vote for the origin rather than for dead reckoning, so any pose source drags
  the estimate to (0, 0) within milliseconds at loop rate. `firmware-ds` has no
  pose source wired, so it does not trip over this today.
* **The sim runs a pose correction gain of `0.2` where the sketch passes `0`.**
  `micromouse.ino` builds `SensorFusion sf(obs_v, obs_p, 0)`, which disables the
  lidar correction outright; `firmware-sim` treats that as a debug value and
  keeps the `FusionWeights::PoseCorrectionGain` default so its localisation
  scenarios still test something. See `firmware-sim/README.md`.
* **`FrontLidarObserver` ignores its 57 mm mount offset** (`LIDAR_MOUNT_FRONT_X`),
  in all three copies. Fine as a relative wall-distance measurement, wrong as a
  pose. See `firmware-sim/README.md`.
* **The maze photos the scripts default to are not in the repo.** `mazes/` holds
  `2.jpg` and `3.jpg`; `maze_demo.py` defaults to `4.png`, and `export_map.py`,
  `bench.py` and `selftest.py` to `1.png`. Pass an image explicitly. See
  [`path-planning/mazes/README.md`](path-planning/mazes/README.md).
* **`firmware-ds/lidar/` is a copy, not a symlink or an include path** — the
  Arduino toolchain requires headers to sit beside the sketch. Every header
  there except `sensorFusion.h` is currently byte-identical to `firmware/`;
  changes made in one do not reach the other.

---

MTRN3100 micromouse project — Zimmy Levi (z5587840).
