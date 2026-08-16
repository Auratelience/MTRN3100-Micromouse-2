# firmware-sim

`micromouse.ino`'s control loop, with a simulated robot underneath it.

```
python3 run.py                            # drive the installed maze_path.h
python3 run.py --viz --open               # ...and watch it
python3 run.py task34                     # a retained TASK 3.x regression scenario
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
right** — that is the whole point. Where a port deliberately departs from the
sketch it is a *configuration* choice, never a behavioural one, and each is
listed under [Where the sim departs from the
sketch](#where-the-sim-departs-from-the-sketch).

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
| `scenarios.py` | the TASK comment blocks | `planned` is the sketch's only live block; `task31`–`task34` are retained regression scenarios |

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

**`fusePose` collapsed the pose estimate onto the origin.** `xWeightTotal` and
`yWeightTotal` started at `1.0f` "to represent the Model_Observer's trust", but
`xTotal`/`yTotal` started at `0.0f` — so the model's vote was a vote for the
origin rather than for where dead reckoning said the robot was. A pose source
agreeing *exactly* with dead reckoning still dragged the estimate toward (0, 0)
by `g/(1+t)` of the remaining distance every tick: 17% per control loop at the
`.ino`'s `t = 0.2`, `g = 0.2`. At 1 kHz the position was pinned within
milliseconds. `theta` escaped it — its numerator accumulates deltas, and the
model's own delta really is zero.

**Fixed**, in `firmware/micromouse/sensorFusion.h` and now mirrored here: all
six totals start at `0.0f`, so the weighted mean is over the pose sources alone
and the result is applied as a correction to dead reckoning rather than as a
replacement for it. `python3 run.py` with no arguments is the header's own
behaviour again, and drives the installed path clean. Two consequences worth
knowing:

- Only the *ratio* of the trusts survives the mean now, not their scale. A lone
  source at `(0.2, 0.2, 0.1)` and the same source at `XYPTrust` land in the same
  place, which is why the `.ino` switching between them changed nothing.
- An axis no source has an opinion on falls through the `<= 0.0f` guard and
  keeps dead reckoning. Under `XYPTrust` (1, 1, 0) that is `theta`, every tick —
  heading on the robot is pure dead reckoning by construction, not by accident.

`firmware-ds/lidar/sensorFusion.h` still carries the original.

**`FrontLidarObserver` ignores its mount offset.** It writes `pose.x =
-getReading(Front)`, but the front sensor sits 57 mm ahead of the robot centre
(`LIDAR_MOUNT_FRONT_X`), so the pose it reports is 57 mm short. Visible as the
steady offset in `run.py task32`. Harmless for a relative wall-distance test,
wrong as a pose. Still unfixed everywhere.

## Where the sim departs from the sketch

Two places, both deliberate, both worth re-checking when the `.ino` changes:

**Pose correction gain.** `micromouse.ino` currently builds `SensorFusion
sf(obs_v, obs_p, 0)`. A gain of `0` makes `fusePose` return dead reckoning
unchanged, so mirroring it would compute the lidar fix and multiply it by zero —
every localisation scenario here would quietly become a dead-reckoning one.
`scenarios.py` keeps the `FusionWeights::PoseCorrectionGain` default of `0.2` and
treats the `0` as a debug value left in the sketch. `--no-localisation` is how
you ask for dead reckoning on purpose. If the `0` is meant to stay, change
`_wheel_imu_and_lidar_pose_fusion` to match.

**`task31`–`task34`.** The sketch is down to a single live `TASK 4.1 | 4.2`
block; the commented-out 3.1–3.4 blocks these scenarios were written against
have been deleted from it. They are kept as regression scenarios, because
`DistancePlanner`, `HeadingPlanner`, `PosePlanner` and `PSPlanner` are all still
in `planners.h` and this is the only thing that drives them end to end. A failure
in one is a report about `planners.h`, not about the `.ino`.

## Arguments

`run.py --help` is authoritative. The groups:

- **path and map** — `--path`, `--map`, `--world-map`, `--no-localisation`,
  `--cruise`
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
  python3 run.py --plan 5 --plan-seed $s --quiet && echo "seed $s ok"
done
```

`--start-error` puts the robot somewhere `sf.set(Pose{0,0,0})` does not know
about — the error a lidar fix has to find:

```
python3 run.py --no-localisation --start-error 25,15,4   # 41 mm out, stays out
python3 run.py                   --start-error 25,15,4   # 2.2 mm at the goal
```

## Why `run.py` and not `python3 -m firmware-sim`

The directory name has a hyphen, which is not a legal Python identifier, so the
package cannot be named in an `import` statement. `run.py` imports it by string
through `importlib`, which has no such restriction. The tests reach it the same
way, via `unittest discover`'s own importer — hence `-t ..` from this directory.
