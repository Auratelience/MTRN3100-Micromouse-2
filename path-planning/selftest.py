#!/usr/bin/env -S uv run --script
"""Checks that matter, runnable without a test framework:

    python selftest.py            # geometry + planner, ~20 s
    python selftest.py --image    # also the CV pass over mazes/1.png

The geometry half is the important half.  A wrong Dubins word or an arc the
firmware rebuilds differently is not something you notice on the overlay -- the
path looks fine and the robot drives a different curve -- so every claim those
modules make is checked numerically here against an independent computation.
"""

import sys

import numpy as np

import dubins as db
import rrt_star as rs
import segments as sg

FAIL = []


def check(name, ok, detail=""):
    print(f"  {'pass' if ok else 'FAIL'}  {name}{('  ' + detail) if detail else ''}")
    if not ok:
        FAIL.append(name)


def rand_pose(rng, span=600.0):
    return np.array(
        [rng.uniform(-span, span), rng.uniform(-span, span), rng.uniform(-np.pi, np.pi)]
    )


# --------------------------------------------------------------------- dubins
def test_dubins():
    print("dubins")
    rng = np.random.default_rng(0)
    rho = 45.0

    worst = {w: (0.0, 0) for w in db.WORDS}
    for _ in range(600):
        q0, q1 = rand_pose(rng), rand_pose(rng)
        alpha, beta, d = db._normalise(q0[None], q1[None], rho)
        params, ok = db._all_words(alpha, beta, d)
        for i, w in enumerate(db.WORDS):
            if not ok[i, 0]:
                continue
            qe = db.endpoint(q0, w, params[i, 0], rho)
            e = max(
                float(np.linalg.norm(qe[:2] - q1[:2])),
                abs(float(db.wrap_pi(qe[2] - q1[2]))) * rho,
            )
            worst[w] = (max(worst[w][0], e), worst[w][1] + 1)
    for w, (e, n) in worst.items():
        check(f"{w} reaches its target pose", n > 20 and e < 1e-6, f"{n} cases, {e:.1e}")

    # the returned word really is the shortest feasible one
    bad = 0
    for _ in range(300):
        q0, q1 = rand_pose(rng), rand_pose(rng)
        L, w, p = db.shortest(q0, q1, rho)
        allL, _ = db.all_lengths(q0[None], q1[None], rho)
        if abs(L - np.nanmin(allL)) > 1e-9:
            bad += 1
    check("shortest() picks the minimum over all words", bad == 0)

    # sampling honours the arc-length spacing that curve_valid's proof needs
    worst_gap = 0.0
    for _ in range(200):
        q0, q1 = rand_pose(rng), rand_pose(rng)
        L, w, p = db.shortest(q0, q1, rho)
        P = db.sample(q0, w, p, rho, 5.0)
        if len(P) > 1:
            worst_gap = max(worst_gap, float(np.linalg.norm(np.diff(P, axis=0), axis=1).max()))
    check("sample() spacing stays under ds", worst_gap <= 5.0 + 1e-9, f"{worst_gap:.3f} mm")

    # a truncated word is the prefix it claims to be
    err = 0.0
    for _ in range(200):
        q0, q1 = rand_pose(rng), rand_pose(rng)
        L, w, p = db.shortest(q0, q1, rho)
        for f in (0.0, 0.13, 0.5, 0.87, 1.0):
            cut = db.truncate(w, p, rho, f * L)
            a = db.endpoint(q0, w, cut, rho)
            b = db.interpolate(q0, w, p, rho, f * L)
            err = max(err, float(np.abs(a - b).max()))
            err = max(err, abs(float(np.sum(cut) * rho - f * L)))
    check("truncate() == interpolate() and has the right length", err < 1e-9, f"{err:.1e}")


# ------------------------------------------------------------------- segments
def test_segments():
    print("segments")
    rng = np.random.default_rng(1)
    rho = 30.0

    worst_pt, worst_centre, worst_len, max_sweep = 0.0, 0.0, 0.0, 0.0
    problems = 0
    for _ in range(200):
        q0, q1 = rand_pose(rng), rand_pose(rng)
        L, w, p = db.shortest(q0, q1, rho)
        segs = sg.from_dubins(q0, w, p, rho)
        if not segs:
            continue
        problems += len(sg.check(segs, heading_tol=1e-6, gap_tol=1e-9))
        worst_len = max(worst_len, abs(sg.length(segs) - L))
        for s in segs:
            if s.is_arc:
                worst_centre = max(
                    worst_centre, float(np.linalg.norm(s.firmware_centre() - s.centre))
                )
                max_sweep = max(max_sweep, s.sweep)
        # the segment list traces the same curve the planner checked
        A = sg.polyline(segs, 1.0)
        B = db.sample(q0, w, p, rho, 1.0)
        worst_pt = max(worst_pt, float(rs._pt_seg_dist(A, B[:-1], B[1:]).min(1).max()))

    # 1 mm chords across a 30 mm arc bulge by 1^2/(8r) = 4e-3 mm, so that -- not
    # the segment maths -- is the floor on this comparison.
    check("segments reproduce the Dubins curve", worst_pt < 2e-2, f"{worst_pt:.1e} mm")
    check("total length matches the Dubins length", worst_len < 1e-6, f"{worst_len:.1e}")
    check(
        "the firmware rebuilds every arc centre",
        worst_centre < 1e-6,
        f"{worst_centre:.1e} mm",
    )
    check("no arc exceeds a minor arc", max_sweep <= np.pi / 2 + 1e-9,
          f"max {np.degrees(max_sweep):.1f} deg")
    check("check() finds nothing to complain about", problems == 0)

    # a 270 deg arc is exactly what the firmware cannot represent: prove that
    # check() catches it rather than letting it through
    c = np.array([0.0, 0.0])
    bad = sg.Segment([30.0, 0.0], [0.0, -30.0], 1.0 / 30.0, sg.LEFT, centre=c)
    check("check() rejects a major arc", len(sg.check([bad])) > 0)

    # frame conversion: shape preserved, handedness flipped
    q0, q1 = rand_pose(rng), rand_pose(rng)
    L, w, p = db.shortest(q0, q1, rho)
    segs = sg.clean(sg.from_dubins(q0, w, p, rho))
    fw, start = sg.to_firmware(segs, q0, local=True)
    A, B = sg.polyline(segs, 2.0), sg.polyline(fw, 2.0)
    dA = np.linalg.norm(np.diff(A, axis=0), axis=1)
    dB = np.linalg.norm(np.diff(B, axis=0), axis=1)
    check(
        "to_firmware is an isometry",
        len(dA) == len(dB) and float(np.abs(dA - dB).max()) < 1e-6,
    )
    check(
        "to_firmware starts the path at the robot's pose",
        float(np.linalg.norm(fw[0].start)) < 1e-6
        and abs(db.wrap_pi(fw[0].start_theta)) < 1e-6,
        f"{np.round(fw[0].start, 4)}",
    )
    flipped = all(
        (a.direction != b.direction) for a, b in zip(segs, fw) if a.is_arc
    )
    check("to_firmware swaps every turn direction", flipped)
    check("to_firmware keeps the segment list continuous", not sg.check(fw))

    # merging is exact, and cleaning cannot move the path far
    q0 = np.array([0.0, 0.0, 0.0])
    straights = [
        sg.Segment([0.0, 0.0], [50.0, 0.0]),
        sg.Segment([50.0, 0.0], [90.0, 0.0]),
        sg.Segment([90.0, 0.0], [90.0, 40.0]),
    ]
    m = sg.merge(straights)
    check(
        "merge() joins collinear straights only",
        len(m) == 2 and abs(m[0].length - 90.0) < 1e-9,
        f"{len(m)} segments",
    )


# ---------------------------------------------------------------------- world
def _box_world(**kw):
    """A 3-cell corridor with a post in the middle of the open end."""
    walls = [
        [[0.0, 0.0], [540.0, 0.0]],
        [[0.0, 180.0], [540.0, 180.0]],
    ]
    return rs.MazeWorld(walls, robot_radius_mm=40.0, extra_clearance_mm=5.0, **kw)


def test_world():
    print("world")
    free = _box_world()
    blocked = _box_world(posts_mm=[[270.0, 90.0]])
    p = np.array([270.0, 90.0])
    check("bare corridor centre is free", bool(free.is_free(p)[0]))
    check("a post on the centreline blocks it", not bool(blocked.is_free(p)[0]))
    check(
        "a post blocks the motion through it",
        free.motion_valid([90.0, 90.0], [450.0, 90.0])
        and not blocked.motion_valid([90.0, 90.0], [450.0, 90.0]),
    )

    # posts with no walls at all: the case a wall-only model cannot see
    only_posts = rs.MazeWorld([], posts_mm=[[0.0, 0.0], [180.0, 0.0]], robot_radius_mm=40.0)
    check(
        "a wall-free maze still collides with its posts",
        not bool(only_posts.is_free([0.0, 0.0])[0])
        and bool(only_posts.is_free([90.0, 200.0])[0]),
    )
    check(
        "a wall-free maze has sane bounds",
        np.all(np.isfinite(only_posts.lo)) and np.all(only_posts.hi >= only_posts.lo),
    )

    # curve_valid is conservative: it must never pass a curve that clips
    rng = np.random.default_rng(3)
    leaks = 0
    for _ in range(300):
        q0 = np.array([rng.uniform(40, 500), rng.uniform(40, 140), rng.uniform(-np.pi, np.pi)])
        q1 = np.array([rng.uniform(40, 500), rng.uniform(40, 140), rng.uniform(-np.pi, np.pi)])
        L, w, par = db.shortest(q0, q1, 30.0)
        if not free.curve_valid(db.sample(q0, w, par, 30.0, 4.0), 4.0):
            continue
        fine = db.sample(q0, w, par, 30.0, 0.2)  # 20x denser ground truth
        if free.clearance(fine).min() <= 0.0:
            leaks += 1
    check("curve_valid never passes a clipping curve", leaks == 0, f"{leaks} leaks")


# -------------------------------------------------------------------- planner
def test_planner():
    print("planner")
    world = _box_world(posts_mm=[[270.0, 0.0], [270.0, 180.0]])
    out = rs.plan_dubins(
        world, (60.0, 90.0, 0.0), (480.0, 90.0, None), rho=30.0, max_iter=800, seed=2
    )
    check("finds a path down a corridor", out["path"] is not None)
    if out["path"] is None:
        return
    segs = out["segments"]
    check("the path is collision free", float(world.clearance(sg.polyline(segs, 1.0)).min()) > 0.0,
          f"min clearance {world.clearance(sg.polyline(segs, 1.0)).min():.1f} mm")
    check("the path is firmware-representable", not sg.check(segs))
    check(
        "the path starts at the start pose and ends at the goal",
        float(np.linalg.norm(segs[0].start - [60.0, 90.0])) < 1.0
        and float(np.linalg.norm(segs[-1].end - [480.0, 90.0])) < 1.0,
    )
    check(
        "the start heading is honoured",
        abs(float(db.wrap_pi(segs[0].start_theta - 0.0))) < 0.05,
        f"{np.degrees(segs[0].start_theta):.2f} deg",
    )
    check(
        "every arc uses the planner's radius",
        all(abs(s.radius - 30.0) < 1e-6 for s in segs if s.is_arc),
    )

    fixed = rs.plan_dubins(
        world, (60.0, 90.0, 0.0), (480.0, 90.0, np.pi / 2), rho=30.0, max_iter=1500, seed=2
    )
    if fixed["path"] is not None:
        check(
            "a fixed goal heading is honoured",
            abs(float(db.wrap_pi(fixed["segments"][-1].end_theta - np.pi / 2))) < 0.05,
            f"{np.degrees(fixed['segments'][-1].end_theta):.1f} deg",
        )

    check(
        "best_heading finds the open side",
        abs(db.wrap_pi(rs.best_heading(world, [60.0, 90.0]) - 0.0)) < 1e-9
        or abs(db.wrap_pi(rs.best_heading(world, [60.0, 90.0]) - np.pi)) < 1e-9,
    )


# ------------------------------------------------------------------------- cv
def test_image():
    import cv2

    import maze_grid as mg
    import maze_map as mp

    print("vision (mazes/1.png)")
    img = cv2.imread("mazes/1.png")
    if img is None:
        check("image loads", False)
        return
    res = mg.solve(img)
    M = mp.build_map(img, res)
    P, W = M["posts"], M["frame"]
    nx, ny = W.shape

    check("lattice is 10x10", (nx, ny) == (10, 10), f"{nx}x{ny}")
    # Pruning is allowed to drop a cap the deck already keeps the robot away
    # from; it is not allowed to drop one standing on the deck.
    on_deck = np.zeros_like(P["detected"])
    poly = np.asarray(M["deck_mm"], np.float32).reshape(-1, 1, 2)
    for i, j in np.argwhere(P["detected"]):
        c = (float(mp.PITCH_MM * i), float(mp.PITCH_MM * j))
        on_deck[i, j] = cv2.pointPolygonTest(poly, c, True) > 0.0
    check(
        "every blue-cap post on the deck survives into the map",
        bool((P["present"] | ~on_deck).all()),
        f"{int(on_deck.sum())} on deck, {P['pruned']} pruned",
    )
    check(
        "post presence is neither all nor nothing",
        0 < int(P["present"].sum()) < P["present"].size,
        f"{int(P['present'].sum())} of {P['present'].size}",
    )
    check(
        "the chamfered corners have no post",
        not P["present"][0, 1] and not P["present"][1, 0],
    )
    check(
        "some posts stand with no panel on them",
        len(M["isolated_posts"]) > 0,
        f"{len(M['isolated_posts'])} isolated",
    )
    check(
        "post centres land on lattice nodes",
        bool(np.all(np.abs(M["post_centres_mm"] % mp.PITCH_MM) < 1e-9)),
    )
    check(
        "isolated posts are obstacles in the world",
        not rs.MazeWorld(
            M["wall_segments_mm"],
            posts_mm=M["post_centres_mm"],
            post_radius_mm=M["post_radius_mm"],
            robot_radius_mm=40.0,
        ).is_free(mp.PITCH_MM * np.array(M["isolated_posts"][0], float))[0],
    )
    # A cylinder standing on a lattice node scores as a post.  If the post ever
    # masks it out of the cylinder pass, a 47 mm obstacle silently becomes an
    # 8.5 mm one -- so pin the counts with and without the mask, on every photo.
    import glob

    for path in sorted(glob.glob("mazes/*")):
        im = cv2.imread(path)
        if im is None:
            continue
        r = mg.solve(im)
        w = mp.detect_walls(im, r)
        ps = mp.detect_posts(im, r)
        with_mask = len(mp.detect_cylinders(im, r, w, ps)["cylinders"])
        without = len(mp.detect_cylinders(im, r, w, None)["cylinders"])
        check(
            f"post masking hides no cylinder in {path}",
            with_mask >= without,
            f"{with_mask} with, {without} without",
        )

    check(
        "no cylinder sits on a post",
        all(
            float(np.linalg.norm(c["centre_mm"] - M["post_centres_mm"], axis=1).min())
            > 40.0
            for c in M["cylinders"]
        ),
        f"{len(M['cylinders'])} cylinders",
    )


if __name__ == "__main__":
    test_dubins()
    test_segments()
    test_world()
    test_planner()
    if "--image" in sys.argv:
        test_image()
    print()
    if FAIL:
        print(f"{len(FAIL)} FAILED: " + ", ".join(FAIL))
        sys.exit(1)
    print("all checks passed")
