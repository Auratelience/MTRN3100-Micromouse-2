# firmware-sim

`micromouse.ino`'s control loop, with a simulated robot underneath it.

```
python3 run.py                            # drive the installed maze_path.h
python3 run.py --fusion-fix --viz --open  # ...and watch it
python3 run.py task34                     # any of the TASK blocks in the .ino
python3 run.py --plan 5 --from 1,1 --to 4,6   # plan a fresh path first, then drive it
python3 -m unittest discover -s . -t ..   # 256 tests, from the repo root
```

Stdlib only. `--plan` is the one exception and it runs path-planning in
path-planning's own uv environment, not this one.

Two portability notes, both consequences of the directory layout rather than of
the sim. `types.py` mirrors `types.h` and so shadows the standard library's
`types` module on any interpreter that has not already imported it by the time
this directory reaches `sys.path` — if `run.py` dies importing `enum`, run it as
`python3 -P run.py` (or set `PYTHONSAFEPATH=1`). And `unittest discover` has to
accept a namespace package to name a directory with a hyphen in it: CPython 3.11
through 3.14 do not, and answer *Start directory is not importable*. The root
[`README.md`](../README.md#run-the-tests) carries an explicit loader that runs
the same 256 tests without either problem.

## What is a port and what is not

Every module named after a header is a line-by-line port of it, method names and
all, so the two can be diffed by eye. **When they disagree, the header is
right** — that is the whole point, and it is why the `fusePose` defect below is
mirrored rather than fixed.

| module | mirrors | notes |
| --- | --- | --- |
| `constants.py` | `constants.h` | verbatim; derived values stay derived |
| `types.py` | `types.h` | `Segment`, and the `Obstacle`/`Map` layer `LidarObserver` casts into. No `trig<>` LUTs, no `RingBuffer` |
| `kinematics.py` | `kinematics.h` | |
| `control.py` | `control.h` | `PID`, `MotionController` |
| `planners.py` | `planners.h` | all five planners |
| `observers.py` | `observers.h` | including `LidarObserver`'s full Levenberg–Marquardt solve |
| `fusion.py` | `sensorFusion.h` | see below |
| `lidar.py` | `lidar.h` | only the VL6180X's *observable* behaviour: 10 ms period, 2 mm quantisation, the min/max clamps |
| `runner.py` | `setup()` + `loop()` | |
| `scenarios.py` | the TASK comment blocks | one entry per block |

Sim-only, with no firmware counterpart: `plant.py` (ground truth — the only
place true state exists), `world.py`, `mapper.py`, `png.py`, `maze_header.py`,
`cli.py`, `viz/`.

`sim.py` and `sim_ps.py` predate the package and are kept as standalone
artefacts; both say so at the top.

## The generated headers

`scripts/build_maze.sh` writes `maze_map.h` and `maze_path.h` and installs
them into `firmware/micromouse/`. `maze_header.py` parses both back, so the
default run is *exactly* what the sketch compiles — same obstacles, same
segments, same frame — with no OpenCV and no re-planning.

Both are exported against one `--from`/`--theta0`, which is why the wrapper
passes the same pair to `maze_demo.py` and `export_map.py`. `--plan` here does
the same thing for the same reason.

Ground truth is built from the map (`MapWorld`), and ranges come from
`Map.cast`, not from the occupancy raster — a 5 mm DDA would put 2.5 mm of
quantisation into every reading, which is the same order as the error the
observer exists to remove. The raster is only for the viewer and for `Mapper`.

`--world-map` points the plant at a *different* header from the one the robot
localises against. That is how you ask what a stale export costs.

## Two things the sim found

**`fusePose` collapses the pose estimate onto the origin.** In
`sensorFusion.h`, `xWeightTotal`/`yWeightTotal` start at `1.0f` "to represent the
Model_Observer's trust", but `xTotal`/`yTotal` start at `0.0f` — so the model's
vote is a vote for the origin rather than for where dead reckoning says the robot
is. A pose source agreeing *exactly* with dead reckoning still drags the estimate
toward (0, 0) by `g/(1+t)` of the remaining distance every tick: 17% per control
loop at the `.ino`'s `t = 0.2`, `g = 0.2`. At 1 kHz the position is pinned within
milliseconds. `theta` is unaffected — its numerator accumulates deltas, and the
model's own delta really is zero.

The sim mirrors this. `--fusion-fix` seeds the two numerators with the
dead-reckoned term, which is what the header's own comment describes:

```
python3 run.py                # 5300 mm final error, robot leaves the maze
python3 run.py --fusion-fix   # 1.0 mm final error, path driven clean
```

**`FrontLidarObserver` ignores its mount offset.** It writes `pose.x =
-getReading(Front)`, but the front sensor sits 57 mm ahead of the robot centre
(`LIDAR_MOUNT_FRONT_X`), so the pose it reports is 57 mm short. Visible as the
steady offset in `run.py task32 --fusion-fix`. Harmless for a relative
wall-distance test, wrong as a pose.

The mount offset is still unfixed everywhere. The `fusePose` defect has since
been addressed in `firmware/micromouse/sensorFusion.h`, which now starts the
three weight totals at `0.0f` and so drops the model's vote entirely rather than
seeding it with dead reckoning — a stronger correction than `--fusion-fix`
applies, in the same direction. `firmware-ds/lidar/sensorFusion.h` and
`fusion.py` both still carry the original, so the sim's *default* run mirrors a
header the robot no longer compiles; `--fusion-fix` is the closer comparison.

## Arguments

`run.py --help` is authoritative. The groups:

- **path and map** — `--path`, `--map`, `--world-map`, `--no-localisation`,
  `--cruise`, `--fusion-fix`
- **generate a path first** — `--plan IMAGE`, `--from`, `--to`, `--theta0`,
  `--robot-radius`, `--turn-radius`, `--mode`, `--iters`, `--plan-seed`,
  `--install`, `--verbose`
- **run** — `--loop-hz`, `--max-seconds`, `--start-error X,Y,DEG`,
  `--mm-per-pixel`
- **plant** — `--motor-tau`, `--pwm-deadband`, `--gyro-bias`, `--gyro-noise`,
  `--lidar-noise`, `--enc-error-left/right`, `--seed`. None of these are
  measured; they are knobs, and `plant.py` says so per field.
- **output** — `--viz`, `--port`, `--speed`, `--open`, `--svg`, `--json`,
  `--quiet`

Exit status is 0 when the planner reached the end, 1 when it did not, so a sweep
over seeds or turn radii needs no output parsing:

```sh
for s in 1 2 3 4 5; do
  python3 run.py --plan 5 --plan-seed $s --fusion-fix --quiet && echo "seed $s ok"
done
```

`--start-error` puts the robot somewhere `sf.set(Pose{0,0,0})` does not know
about — the error a lidar fix has to find:

```
python3 run.py --no-localisation --start-error 25,15,4   # 29 mm out, stays out
python3 run.py --fusion-fix      --start-error 25,15,4   # 1.1 mm at the goal
```

## Why `run.py` and not `python3 -m firmware-sim`

The directory name has a hyphen, which is not a legal Python identifier, so the
package cannot be named in an `import` statement. `run.py` imports it by string
through `importlib`, which has no such restriction. The tests reach it the same
way, via `unittest discover`'s own importer — hence `-t ..` from this directory.
