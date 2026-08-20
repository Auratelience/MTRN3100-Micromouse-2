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

    # The body disc rides ahead of the axle, so around a turn it sweeps a larger
    # circle and covers more arc than the reference point does.  curve_valid's
    # proof is about the *body's* curve, so that is what has to honour ds.
    off = 25.0
    worst_gap, worst_pt = 0.0, 0.0
    for _ in range(200):
        q0, q1 = rand_pose(rng), rand_pose(rng)
        L, w, p = db.shortest(q0, q1, rho)
        Q = db.sample_poses(q0, w, p, rho, 5.0, offset=off)
        B = Q[:, :2] + off * np.stack([np.cos(Q[:, 2]), np.sin(Q[:, 2])], 1)
        if len(B) > 1:
            worst_gap = max(
                worst_gap, float(np.linalg.norm(np.diff(B, axis=0), axis=1).max())
            )
        # offset=0 must be the old sampler exactly, or every caller that still
        # wants points has silently changed
        A = db.sample_poses(q0, w, p, rho, 5.0)
        worst_pt = max(worst_pt, float(np.abs(A[:, :2] - db.sample(q0, w, p, rho, 5.0)).max()))
    check(
        "sample_poses() spacing stays under ds for the offset body",
        worst_gap <= 5.0 + 1e-9,
        f"{worst_gap:.3f} mm",
    )
    check("sample_poses() at offset 0 reproduces sample()", worst_pt < 1e-12, f"{worst_pt:.1e}")

    # the heading carried by each sample really is the curve's tangent there
    worst_th = 0.0
    for _ in range(50):
        q0, q1 = rand_pose(rng), rand_pose(rng)
        L, w, p = db.shortest(q0, q1, rho)
        Q = db.sample_poses(q0, w, p, rho, 0.05)
        d = np.diff(Q[:, :2], axis=0)
        keep = np.linalg.norm(d, axis=1) > 1e-9
        chord = np.arctan2(d[keep, 1], d[keep, 0])
        a, b = Q[:-1, 2][keep], Q[1:, 2][keep]
        mid = a + 0.5 * db.wrap_pi(b - a)  # the chord bisects the swept angle
        worst_th = max(worst_th, float(np.abs(db.wrap_pi(chord - mid)).max()))
    check("sample_poses() headings follow the tangent", worst_th < 2e-3, f"{worst_th:.1e} rad")

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

    # scale: the uniform resize maze_demo applies to the firmware-frame path.
    # Size is linear in k, shape is not: sweeps, turn directions and the
    # firmware's own centre reconstruction all have to survive it.
    for k in (0.5, 2.5):
        sc = sg.scale(fw, k)
        worst_centre = max(
            (float(np.linalg.norm(s.firmware_centre() - s.centre))
             for s in sc if s.is_arc),
            default=0.0,
        )
        worst_sweep = max(
            (abs(a.sweep - b.sweep) for a, b in zip(fw, sc) if a.is_arc), default=0.0
        )
        check(
            f"scale({k}) multiplies the length by {k}",
            len(sc) == len(fw)
            and abs(sg.length(sc) - k * sg.length(fw)) <= 1e-9 * k * sg.length(fw),
            f"{sg.length(sc):.3f} vs {k * sg.length(fw):.3f} mm",
        )
        check(
            f"scale({k}) keeps every turn direction",
            all(a.direction == b.direction for a, b in zip(fw, sc)),
        )
        check(
            f"scale({k}) leaves every sweep alone",
            worst_sweep < 1e-9,
            f"{np.degrees(worst_sweep):.1e} deg",
        )
        check(
            f"scale({k}) keeps the arcs firmware-representable",
            worst_centre < 1e-6 and not sg.check(sc),
            f"centre {worst_centre:.1e} mm",
        )

    back = sg.scale(sg.scale(fw, 4.0), 0.25)
    worst_rt = max(
        max(float(np.linalg.norm(a.start - b.start)),
            float(np.linalg.norm(a.end - b.end)))
        for a, b in zip(fw, back)
    )
    check("scale() round-trips", worst_rt < 1e-9, f"{worst_rt:.1e} mm")

    # a scale big enough to push an arc past MAX_ARC_RADIUS_MM stops being
    # representable, and that is exactly what maze_demo re-checks for
    check(
        "check() catches an arc scaled up into a straight",
        any("straight line" in c for c in sg.check(sg.scale(fw, 100.0)))
        if any(s.is_arc for s in fw) else True,
    )

    rejected = 0
    for bad_k in (0.0, -1.0):
        try:
            sg.scale(fw, bad_k)
        except ValueError:
            rejected += 1
    check("scale() rejects a non-positive factor", rejected == 2)

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

    def leak_count(w_, off):
        leaks = 0
        for _ in range(300):
            q0 = np.array([rng.uniform(40, 500), rng.uniform(40, 140), rng.uniform(-np.pi, np.pi)])
            q1 = np.array([rng.uniform(40, 500), rng.uniform(40, 140), rng.uniform(-np.pi, np.pi)])
            L, word, par = db.shortest(q0, q1, 30.0)
            if not w_.curve_valid(db.sample_poses(q0, word, par, 30.0, 4.0, offset=off), 4.0):
                continue
            fine = db.sample_poses(q0, word, par, 30.0, 0.2, offset=off)  # 20x denser
            if w_.pose_clearance(fine).min() <= 0.0:
                leaks += 1
        return leaks

    n0 = leak_count(_box_world(axle_offset_mm=0.0), 0.0)
    check("curve_valid never passes a clipping curve", n0 == 0, f"{n0} leaks")

    # -- an axle that is not at the centre of the robot ---------------------
    off = rs.AXLE_OFFSET_MM
    n1 = leak_count(_box_world(axle_offset_mm=off), off)
    check("curve_valid never passes a curve the offset body clips", n1 == 0, f"{n1} leaks")

    posted = _box_world(posts_mm=[[270.0, 90.0]], axle_offset_mm=off)
    axle = [205.0, 90.0]  # 65 mm short of a post that reaches 53.5 mm
    check(
        "an axle clear of a post can still put the body inside it",
        bool(posted.is_free(axle)[0]) and not bool(posted.pose_free([*axle, 0.0])[0]),
    )
    check(
        "the same axle facing away from the post is free",
        bool(posted.pose_free([*axle, np.pi])[0]),
    )
    check(
        "body_xy puts the disc offset ahead along the heading",
        float(np.linalg.norm(posted.body_xy([0.0, 0.0, np.pi / 2.0])[0] - [0.0, off]))
        < 1e-9,
    )
    check(
        "a zero offset leaves the body on the axle",
        float(
            np.abs(
                _box_world(axle_offset_mm=0.0).body_xy([12.0, 34.0, 1.0])[0] - [12.0, 34.0]
            ).max()
        )
        < 1e-12,
    )


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
    # the body's curve, not the axle's: with an offset those are different
    # curves, and it is the body that has to fit
    body = world.body_xy(sg.pose_polyline(segs, 1.0))
    check("the path is collision free", float(world.clearance(body).min()) > 0.0,
          f"min clearance {world.clearance(body).min():.1f} mm")
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
    th = rs.best_heading(world, [60.0, 90.0])
    check(
        "best_heading leaves the body somewhere it fits",
        bool(world.pose_free([60.0, 90.0, th])[0]),
    )

    # A start whose axle is clear but whose body is not is a start the robot
    # cannot be put in, and saying so beats searching from it and finding
    # nothing.  205 mm is 65 mm short of the post; the body reaches to 230.
    posted = _box_world(posts_mm=[[270.0, 90.0]], axle_offset_mm=rs.AXLE_OFFSET_MM)
    try:
        rs.plan_dubins(posted, (205.0, 90.0, 0.0), (60.0, 90.0, None), rho=30.0, max_iter=10)
        rejected = False
    except ValueError:
        rejected = True
    check("a start pose the body cannot hold is rejected", rejected)


# ------------------------------------------------------------------ collision
def test_collision():
    """The overlay's red marks: samples of the drawn path that sit inside an
    obstacle.  A planned path has none, so what this really pins is that the
    marks are honest -- sampled along the curve rather than at its waypoints,
    and never claimed anywhere the world says there is room."""
    import maze_demo as md

    print("collision")
    # the corridor is free for y in (51, 129): wall_t/2 + r + pad = 51 mm
    world = _box_world()

    def line(y):
        x = np.linspace(60.0, 480.0, 50)
        return np.stack([x, np.full_like(x, y)], 1)

    clean, _, n = md.collision_points(world, line(90.0))
    check("a centreline path has no collisions", len(clean) == 0)
    check(
        "the corridor is sampled at the reported spacing",
        abs(n - round(420.0 / md.COLLISION_DS_MM) - 1) <= 1,
        f"{n} samples over 420 mm at {md.COLLISION_DS_MM:.0f} mm",
    )

    hits, slack, _ = md.collision_points(world, line(20.0))
    check(
        "a path inside the wall clearance collides", len(hits) > 0, f"{len(hits)} samples"
    )
    check(
        "every reported sample really is in collision",
        len(hits) > 0
        and bool((slack <= 0.0).all())
        and bool(np.allclose(slack, world.clearance(hits))),
    )

    # a path that only dips out of the corridor: the marks must land on the dip
    P = line(90.0)
    P[20:30, 1] = 20.0
    hits, _, _ = md.collision_points(world, P)
    check(
        "marks land only where the path leaves the free corridor",
        len(hits) > 0 and bool((hits[:, 1] < 51.0).all()),
    )

    # the whole point of resampling: endpoints free, middle not.  Checking the
    # waypoints alone would call this path clean.
    blocked = _box_world(posts_mm=[[270.0, 90.0]])
    ends = np.array([[60.0, 90.0], [480.0, 90.0]])
    check(
        "a post between two free waypoints is still caught",
        bool(blocked.is_free(ends).all()) and len(md.collision_points(blocked, ends)[0]) > 0,
    )

    # Given headings, the marks belong under the body, which is where the robot
    # actually is.  This run stops 16 mm short of the post on the axle and 9 mm
    # inside it on the body, so the two readings disagree.
    offw = _box_world(posts_mm=[[270.0, 90.0]], axle_offset_mm=rs.AXLE_OFFSET_MM)
    axle = np.column_stack([np.linspace(60.0, 200.0, 60), np.full(60, 90.0)])
    check("the axle's own track reads clean", len(md.collision_points(offw, axle)[0]) == 0)
    hits, _, _ = md.collision_points(offw, np.column_stack([axle, np.zeros(60)]))
    check(
        "with headings the body's collision is reported",
        len(hits) > 0 and bool((hits[:, 0] > 216.0).all()),
        f"{len(hits)} samples",
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
    test_collision()
    if "--image" in sys.argv:
        test_image()
    print()
    if FAIL:
        print(f"{len(FAIL)} FAILED: " + ", ".join(FAIL))
        sys.exit(1)
    print("all checks passed")
