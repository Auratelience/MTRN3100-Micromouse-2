"""Command line for the firmware sim.

    python3 firmware-sim/run.py --help

Two things it does. First, it runs any of the TASK scenarios from
micromouse.ino in a blank arena, which is what the sim was for before. Second --
the `planned` scenario, and the default -- it runs the *generated* path against
the *generated* map, i.e. exactly what the sketch compiles today:

    run.py                                  # firmware/micromouse/maze_{path,map}.h
    run.py --plan mazes/5.png --from 1,1 --to 4,6

The second form shells out to path-planning to produce a fresh pair before
simulating them. It is the same call build_maze.sh makes, with the same
--from/--theta0 handed to both maze_demo.py and export_map.py, because a path
and a map exported against different start poses do not share a frame -- the
robot would then localise against a map offset from the path it is driving.
Nothing is installed into firmware/ unless --install is given: planning here is
for asking "would this path drive", not for flashing.

Ground truth is built from the map by default, so the robot's beams see the
maze the map describes. --world-map points the plant at a *different* header,
which is how you ask what a stale map does to the fix.
"""

import argparse
import json
import math
import pathlib
import shutil
import subprocess
import sys
import tempfile

from . import maze_header
from .constants import MAXIMUM_FORWARD_VELOCITY
from .plant import PlantConfig
from .runner import Runner
from .scenarios import SCENARIOS, planned, task32_bench
from .types import Pose
from .world import MapWorld, World

REPO = pathlib.Path(__file__).resolve().parent.parent
PATH_PLANNING = REPO / "path-planning"

# Blank-arena size for the TASK scenarios. Large enough that nothing in them
# reaches a wall, which is the point: they are planner tests, not maze runs.
BLANK_ARENA_MM = 8000.0
BLANK_ARENA_MM_PER_PX = 10.0


# --------------------------------------------------------------- planning
def run_path_planning(args, out_dir):
    """maze_demo.py + export_map.py, into `out_dir`. Returns (path_h, map_h).

    Under `uv run --directory path-planning`, matching build_maze.sh: that is
    what pins the OpenCV/numpy versions the pipeline was written against. The
    sim itself needs none of them -- it reads the headers those two write.
    """
    if shutil.which("uv") is None:
        raise SystemExit(
            "--plan needs uv on PATH (the path-planning pipeline runs under "
            "'uv run'). Drop --plan to simulate the installed headers instead."
        )

    image = pathlib.Path(args.plan)
    if not image.exists():
        candidates = [
            PATH_PLANNING / "mazes" / args.plan,
            *(PATH_PLANNING / "mazes" / f"{args.plan}{e}" for e in (".png", ".jpg")),
        ]
        image = next((c for c in candidates if c.exists()), None)
        if image is None:
            available = sorted(p.name for p in (PATH_PLANNING / "mazes").glob("*"))
            raise SystemExit(
                f"no such maze {args.plan!r}. In path-planning/mazes/: "
                f"{', '.join(available)}"
            )

    py = ["uv", "run", "--directory", str(PATH_PLANNING), "python"]
    path_h = out_dir / "maze_path.h"
    map_h = out_dir / "maze_map.h"

    print(
        f"planning   {image.name}, cell {args.src} -> {args.dst}, "
        f"r={args.robot_radius:.0f} mm, turn={args.turn_radius:.0f} mm"
    )
    demo = subprocess.run(
        py
        + [
            "maze_demo.py",
            str(image.resolve()),
            "--from", args.src,
            "--to", args.dst,
            "--theta0", args.theta0,
            "--r", str(args.robot_radius),
            "--turn-radius", str(args.turn_radius),
            "--mode", args.mode,
            "--iters", str(args.iters),
            "--seed", str(args.plan_seed),
            "--out", str(out_dir / "overlay.png"),
            "--emit", str(path_h),
        ],
        capture_output=not args.verbose,
        text=True,
    )
    if demo.returncode != 0:
        sys.stderr.write(demo.stdout or "")
        sys.stderr.write(demo.stderr or "")
        raise SystemExit("maze_demo.py failed")
    if not args.verbose and "no path" in (demo.stdout or ""):
        raise SystemExit(
            "no path found -- raise --iters, change --plan-seed, or shrink "
            "--robot-radius. Re-run with --verbose to see the planner's output."
        )
    if not path_h.exists():
        raise SystemExit("maze_demo.py reported success but emitted no path")

    # The SAME --from/--theta0. This is the coupling build_maze.sh exists for.
    export = subprocess.run(
        py
        + [
            "export_map.py",
            str(image.resolve()),
            "--from", args.src,
            "--theta0", args.theta0,
            "--r", str(args.robot_radius),
            "-o", str(map_h),
        ],
        capture_output=not args.verbose,
        text=True,
    )
    if export.returncode != 0:
        sys.stderr.write(export.stdout or "")
        sys.stderr.write(export.stderr or "")
        raise SystemExit("export_map.py failed")

    return path_h, map_h


def install(path_h, map_h):
    """Copy a freshly planned pair into firmware/micromouse/, keeping the
    previous ones as .bak -- build_maze.sh's install step, because neither
    header is tracked by git and a bad run would take the only copy."""
    for src, dest in ((map_h, maze_header.DEFAULT_MAP), (path_h, maze_header.DEFAULT_PATH)):
        if dest.exists():
            shutil.copy2(dest, dest.with_suffix(dest.suffix + ".bak"))
        shutil.copy(src, dest)
        print(f"installed  {dest} (previous kept as {dest.name}.bak)")


# -------------------------------------------------------------- scenarios
def build_scenario(args):
    """(scenario, world, true_start_pose). Everything header-dependent lands
    here so the run loop below never has to care which mode it is in."""
    if args.scenario != "planned":
        scenario = SCENARIOS[args.scenario]()
        if args.scenario == "task32":
            # The one task that needs something to look at; see task32_bench().
            world, true_start = task32_bench()
        else:
            world = World.blank(BLANK_ARENA_MM, BLANK_ARENA_MM, BLANK_ARENA_MM_PER_PX)
            true_start = scenario.start_pose
        return scenario, world, _offset(true_start, args.start_error)

    tmp = None
    if args.plan:
        tmp = tempfile.TemporaryDirectory(prefix="firmware-sim-plan-")
        path_h, map_h = run_path_planning(args, pathlib.Path(tmp.name))
        if args.install:
            install(path_h, map_h)
    else:
        path_h = pathlib.Path(args.path) if args.path else maze_header.DEFAULT_PATH
        map_h = pathlib.Path(args.map) if args.map else maze_header.DEFAULT_MAP

    segments = maze_header.load_path(path_h)
    robot_map = maze_header.load_map(map_h)

    note = maze_header.path_note(path_h)
    prov = maze_header.map_provenance(map_h)
    print(f"path       {note or f'{len(segments)} segments'}")
    print(f"map        {len(robot_map)} obstacles from {prov.get('image', map_h)}")
    if "origin_mm" in prov:
        print(
            f"origin     map {prov['origin_mm']} mm heading "
            f"{prov['origin_deg']:.0f} deg == robot (0, 0, 0)"
        )

    # The world the beams actually hit. Same map unless told otherwise, which
    # is the honest default: the export is the best description of the maze we
    # have, so believing it twice is not a lie, it just means this run cannot
    # tell you anything about map error.
    world_map = (
        maze_header.load_map(pathlib.Path(args.world_map))
        if args.world_map
        else robot_map
    )
    if args.world_map:
        print(f"world      {len(world_map)} obstacles from {args.world_map} (differs)")
    world = MapWorld(world_map, mm_per_pixel=args.mm_per_pixel)

    scenario = planned(
        segments,
        map=robot_map,
        localise=not args.no_localisation,
        cruise=args.cruise,
    )
    scenario._tmp = tmp  # keep the planning dir alive for the run's lifetime
    return scenario, world, _offset(scenario.start_pose, args.start_error)


def _offset(pose, error):
    if error is None:
        return Pose(pose.x, pose.y, pose.theta)
    dx, dy, dtheta = error
    return Pose(pose.x + dx, pose.y + dy, pose.theta + math.radians(dtheta))


# -------------------------------------------------------------- reporting
def summarise(runner, finished, args):
    """What the run did, in the terms the .ino would print over serial."""
    true, est = runner.true_pose, runner.est_pose
    frame = runner.last_frame

    print()
    print(f"scenario   {runner.scenario.name} -- {runner.scenario.description}")
    print(f"ran        {runner.t:.2f} s of simulated time, {runner.steps} loops "
          f"at {runner.loop_hz:.0f} Hz")
    print("outcome    " + ("planner idle (reached the end)" if finished
                           else f"still running at --max-seconds {args.max_seconds}"))
    print(f"true       x={true.x:8.1f} y={true.y:8.1f} th={math.degrees(true.theta):7.1f} deg")
    print(f"estimate   x={est.x:8.1f} y={est.y:8.1f} th={math.degrees(est.theta):7.1f} deg")
    print(f"error      {runner.position_error():.1f} mm position, "
          f"{math.degrees(abs(_wrap(true.theta - est.theta))):.1f} deg heading")

    print(f"peak error {runner.peak_error:.1f} mm at its worst during the run")

    if frame is not None:
        print(f"segment    {frame.planner_idx} of {_path_len(runner)}")
        print(f"lidar      front {frame.readings['front']:3d} left "
              f"{frame.readings['left']:3d} right {frame.readings['right']:3d} mm "
              f"at the end")

    if runner.scenario.map:
        # A solve with no beams is a tick the estimate spent on dead reckoning,
        # so this is the first number to look at when a run drifts.
        share = 100.0 * runner.beam_loops / max(1, runner.steps)
        per = runner.beams_total / max(1, runner.beam_loops)
        print(f"fixes      {runner.beam_loops} loops corrected ({share:.0f}% of "
              f"{runner.steps}), {per:.1f} beams each on average")
        collided = _collisions(runner)
        print("clearance  " + ("clean" if not collided
                               else f"{collided} trail points inside an obstacle"))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _path_len(runner):
    """How many steps the planner has. MotionPlanner counts segments, PSPlanner
    counts grid poses, and the rest have no path at all."""
    p = runner.planner
    for attr in ("path", "instructions"):
        seq = getattr(p, attr, None)
        if seq is not None:
            return len(seq)
    return 0


def _collisions(runner):
    """Trail points that ended up inside an obstacle. The plant has no contact
    model -- it drives through walls -- so this is how a run says it crashed."""
    return sum(1 for x, y in runner.true_trail if runner.world.occupied_at(x, y))


def write_json(runner, out, finished):
    true, est = runner.true_pose, runner.est_pose
    data = {
        "scenario": runner.scenario.name,
        "description": runner.scenario.description,
        "finished": finished,
        "seconds": runner.t,
        "steps": runner.steps,
        "loop_hz": runner.loop_hz,
        "true_pose": [true.x, true.y, true.theta],
        "est_pose": [est.x, est.y, est.theta],
        "position_error_mm": runner.position_error(),
        "peak_position_error_mm": runner.peak_error,
        "heading_error_rad": _wrap(true.theta - est.theta),
        "corrected_loops": runner.beam_loops,
        "beams_total": runner.beams_total,
        "segments": _path_len(runner),
        "segment_reached": runner.last_frame.planner_idx if runner.last_frame else 0,
        "geometry": runner.geometry(),
        "trail_true": [list(p) for p in runner.true_trail],
        "trail_est": [list(p) for p in runner.est_trail],
    }
    pathlib.Path(out).write_text(json.dumps(data, indent=1))
    print(f"wrote      {out}")


def write_svg(runner, out):
    """Both trails and the planned geometry, as a standalone SVG. No
    dependencies, and it opens in anything -- the point is to be able to look at
    a run from a terminal without starting a server."""
    x0, y0, x1, y1 = runner.world.bounds_mm()
    pad = 20.0
    x0, y0, x1, y1 = x0 - pad, y0 - pad, x1 + pad, y1 + pad
    w, h = x1 - x0, y1 - y0

    def pts(trail):
        # SVG y grows downward; world +y grows up.
        return " ".join(f"{x - x0:.1f},{y1 - y:.1f}" for x, y in trail)

    # Both width and height, to scale. A viewBox with only one of them leaves
    # the other at the SVG default of 100%, and a renderer with a square
    # viewport then letterboxes the drawing instead of fitting it.
    scale = min(1.0, 1600.0 / max(w, h))
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w:.0f} {h:.0f}" '
        f'width="{w * scale:.0f}" height="{h * scale:.0f}">',
        f'<rect width="{w:.0f}" height="{h:.0f}" fill="#fbfbfb"/>',
    ]

    world_map = getattr(runner.world, "map", None)
    if world_map is not None:
        from .types import CircularObstacle

        for o in world_map.obstacles:
            cx, cy = o.centre.x - x0, y1 - o.centre.y
            if isinstance(o.form, CircularObstacle):
                parts.append(
                    f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{o.form.radius:.1f}" '
                    f'fill="#c9ccd1"/>'
                )
            else:
                f = o.form
                # -alpha: the map's heading is in world axes, and the SVG y flip
                # reverses the sense of every angle with it.
                parts.append(
                    f'<rect x="{-f.length / 2:.1f}" y="{-f.thickness / 2:.1f}" '
                    f'width="{f.length:.1f}" height="{f.thickness:.1f}" '
                    f'fill="#8b9199" transform="translate({cx:.1f},{cy:.1f}) '
                    f'rotate({-math.degrees(f.alpha):.2f})"/>'
                )

    for seg in getattr(runner.planner, "path", ()) or ():
        parts.append(
            f'<line x1="{seg.start.x - x0:.1f}" y1="{y1 - seg.start.y:.1f}" '
            f'x2="{seg.end.x - x0:.1f}" y2="{y1 - seg.end.y:.1f}" '
            f'stroke="#f0a030" stroke-width="3" stroke-linecap="round" opacity="0.6"/>'
        )

    parts += [
        f'<polyline points="{pts(runner.est_trail)}" fill="none" stroke="#d04a4a" '
        f'stroke-width="2" stroke-dasharray="6 4"/>',
        f'<polyline points="{pts(runner.true_trail)}" fill="none" stroke="#2d6cdf" '
        f'stroke-width="2"/>',
        '<g font-family="monospace" font-size="18" fill="#333">',
        f'<text x="10" y="24">{runner.scenario.name}: {runner.scenario.description}</text>',
        '<text x="10" y="46" fill="#2d6cdf">true</text>'
        '<text x="70" y="46" fill="#d04a4a">estimate</text>'
        '<text x="180" y="46" fill="#f0a030">planned</text>',
        "</g></svg>",
    ]
    pathlib.Path(out).write_text("\n".join(parts))
    print(f"wrote      {out}")


# -------------------------------------------------------------------- main
def parser():
    p = argparse.ArgumentParser(
        prog="firmware-sim",
        description="Run micromouse.ino's control loop against a simulated robot.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Two things it does.", 1)[0].strip(),
    )
    p.add_argument(
        "scenario",
        nargs="?",
        default="planned",
        choices=[*SCENARIOS, "planned"],
        help="which block of micromouse.ino to reproduce (default: planned, "
             "the one that is uncommented today)",
    )

    g = p.add_argument_group("path and map (the `planned` scenario)")
    g.add_argument("--path", metavar="FILE",
                   help=f"maze_path.h to drive (default {maze_header.DEFAULT_PATH})")
    g.add_argument("--map", metavar="FILE",
                   help=f"maze_map.h to localise against (default {maze_header.DEFAULT_MAP})")
    g.add_argument("--world-map", metavar="FILE",
                   help="build ground truth from a DIFFERENT map than the robot "
                        "localises against, to see what a stale export costs")
    g.add_argument("--no-localisation", action="store_true",
                   help="drop the lidar pose source: dead reckoning only, i.e. "
                        "the PLANNED PATH block without LIDAR LOCALISATION")
    g.add_argument("--cruise", type=float, default=200.0, metavar="MM_S",
                   help="MotionPlanner cruise velocity (default 200, the .ino's; "
                        f"the IK ceiling is {MAXIMUM_FORWARD_VELOCITY:.0f})")
    g = p.add_argument_group("generate a path first (runs path-planning under uv)")
    g.add_argument("--plan", metavar="IMAGE",
                   help="maze photo -- a path in mazes/ or a name like '5'")
    g.add_argument("--from", dest="src", default="1,1", metavar="I,J",
                   help="start cell, shared by the path and the map (default 1,1)")
    g.add_argument("--to", dest="dst", default="7,7", metavar="I,J",
                   help="goal cell (default 7,7)")
    g.add_argument("--theta0", default="auto", metavar="DEG",
                   help="start heading in degrees, or 'auto' (default)")
    g.add_argument("--robot-radius", type=float, default=40.0, metavar="MM")
    g.add_argument("--turn-radius", type=float, default=30.0, metavar="MM",
                   help="arc radius; 30 or 75..178, nothing between (default 30)")
    g.add_argument("--mode", choices=("dubins", "polyline"), default="dubins")
    g.add_argument("--iters", type=int, default=4000, help="RRT* iterations")
    g.add_argument("--plan-seed", type=int, default=1, help="RRT* seed")
    g.add_argument("--install", action="store_true",
                   help="copy the planned pair into firmware/micromouse/ "
                        "(keeps the previous ones as .bak)")
    g.add_argument("--verbose", action="store_true",
                   help="let the planner's own output through")

    g = p.add_argument_group("run")
    g.add_argument("--loop-hz", type=float, default=1000.0,
                   help="control loop rate (default 1000)")
    g.add_argument("--max-seconds", type=float, default=60.0,
                   help="give up after this much simulated time (default 60)")
    g.add_argument("--start-error", metavar="X,Y,DEG",
                   help="put the robot this far from where the estimator thinks "
                        "it starts -- the error a lidar fix has to find")
    g.add_argument("--mm-per-pixel", type=float, default=5.0,
                   help="raster resolution of the ground-truth grid, for the "
                        "viewer and the built map only (default 5)")

    g = p.add_argument_group("plant (nothing here is measured; they are knobs)")
    g.add_argument("--motor-tau", type=float, default=0.060, metavar="S")
    g.add_argument("--pwm-deadband", type=float, default=20.0)
    g.add_argument("--gyro-bias", type=float, default=0.01, metavar="RAD_S")
    g.add_argument("--gyro-noise", type=float, default=0.005, metavar="RAD_S")
    g.add_argument("--lidar-noise", type=float, default=1.5, metavar="MM")
    g.add_argument("--enc-error-left", type=float, default=1.0,
                   help="1.0 means ENC_SCALE_LEFT is exactly right")
    g.add_argument("--enc-error-right", type=float, default=1.0)
    g.add_argument("--seed", type=int, default=0, help="plant noise seed")

    g = p.add_argument_group("output")
    g.add_argument("--viz", action="store_true",
                   help="serve a live view instead of running headless")
    g.add_argument("--port", type=int, default=8420)
    g.add_argument("--speed", type=float, default=1.0,
                   help="viewer playback rate, x real time (default 1)")
    g.add_argument("--open", action="store_true", help="open a browser at --viz")
    g.add_argument("--svg", metavar="FILE", help="write the trails as an SVG")
    g.add_argument("--json", dest="js", metavar="FILE", help="write the run as JSON")
    g.add_argument("--quiet", action="store_true", help="summary only")
    return p


def parse_start_error(text):
    if text is None:
        return None
    try:
        parts = [float(v) for v in text.split(",")]
    except ValueError:
        raise SystemExit(f"--start-error wants x,y,deg in mm and degrees, got {text!r}")
    if len(parts) != 3:
        raise SystemExit(f"--start-error wants three values x,y,deg, got {text!r}")
    return tuple(parts)


def main(argv=None):
    args = parser().parse_args(argv)
    args.start_error = parse_start_error(args.start_error)

    scenario, world, true_start = build_scenario(args)

    runner = Runner(
        scenario,
        world,
        loop_hz=args.loop_hz,
        plant_config=PlantConfig(
            motor_tau=args.motor_tau,
            pwm_deadband=args.pwm_deadband,
            enc_scale_error_left=args.enc_error_left,
            enc_scale_error_right=args.enc_error_right,
            gyro_bias=args.gyro_bias,
            gyro_noise=args.gyro_noise,
            lidar_noise_mm=args.lidar_noise,
            seed=args.seed,
        ),
        true_start_pose=true_start,
    )

    if args.start_error:
        print(
            f"start      robot at ({true_start.x:.1f}, {true_start.y:.1f}, "
            f"{math.degrees(true_start.theta):.1f} deg), estimator told (0, 0, 0)"
        )

    if args.viz:
        return serve(runner, args)

    finished = runner.run_until_done(args.max_seconds)
    if not args.quiet:
        summarise(runner, finished, args)
    if args.svg:
        write_svg(runner, args.svg)
    if args.js:
        write_json(runner, args.js, finished)

    # Non-zero when the plan did not finish, so this is usable in a loop over
    # seeds or turn radii without parsing the text above.
    return 0 if finished else 1


def serve(runner, args):
    from .viz.server import VizServer

    viz = VizServer(runner, port=args.port, speed=args.speed)
    viz.start()
    print(f"serving    {viz.url}  (ctrl-c to stop)")
    if args.open:
        import webbrowser

        webbrowser.open(viz.url)
    try:
        import time

        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print()
    finally:
        viz.stop()
    return 0
