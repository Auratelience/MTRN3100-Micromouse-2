#!/usr/bin/env -S uv run --script
"""RRT* over the fitted maze, in world millimetres.

``MazeWorld`` turns the vision output into a collision model: panels are
capsules of half-width ``wall_t/2 + robot_r``, cylinders are discs grown by the
robot radius, and the deck polygon is a keep-in region.  Checks are exact
(segment-segment and segment-circle distances) rather than sampled, so there is
no resolution parameter to get wrong at the doorways.

Free-standing posts are obstacles too, and they are the ones a wall-only model
misses: a post with no panel on any of its four bonds sits in the middle of open
floor and never appears in ``wall_segments_mm``.  ``MazeWorld`` takes them as a
separate list of discs so a maze that has posts everywhere, posts nowhere, or
posts in some places and not others all collide correctly.

``plan`` is textbook RRT* -- steer, choose-parent over the near set, rewire --
with goal biasing, a shrinking near radius capped in both size and count, and a
final shortcut pass.  ``plan_dubins`` is the same search in SE(2) with Dubins
steering, so every edge is already a straight-and-arc sequence the firmware can
drive; see ``dubins.py`` and ``segments.py``.

Turn radius vs. corner posts
----------------------------
The Dubins planner's counterpart of the same trap.  A 90 deg turn tangent to two
cell centrelines has its centre at (90+R, 90+R) from the corner cell, so the
pivot post sits ``sqrt(2)|90-R|`` from that centre and the arc misses it by
``|R - sqrt(2)|90-R||``.  For a 40 mm robot that has to clear 53.5 mm, so the
feasible radii are two bands, R <= 30 mm and 75 <= R <= 178 mm, not a range:
R = 70 mm, the obvious "a bit tighter than a cell" choice, sits in the gap and
cannot turn a corner at all.

Both bands are drivable; only the small one is searchable.  A 90 mm turn is the
textbook micromouse arc -- centred exactly on the post, tangent to both
centrelines -- but it is only clear if the robot enters it within a few mm of
the centreline and square to it, which is a near-measure-zero target for a
sampler: at R = 90 the tree spread across two thirds of this maze and never once
connected.  A 30 mm arc turns comfortably from anywhere across the corridor, and
the search connects in a couple of thousand samples.  Hence the 30 mm default.
It costs speed rather than safety: holding 4 m/s^2 lateral through a 30 mm arc
caps the corner at 346 mm/s, against the robot's 392 mm/s top speed.

Step size vs. free width
------------------------
The one non-obvious parameter.  Inflating a 180 mm cell by a 40 mm robot plus
half a 12 mm panel leaves a 78 mm free square.  A fixed 90 mm step could not
fit inside it: from any node in a pocket, steering a full step toward any
outside sample left the pocket and hit a panel, so the tree saturated at ~20
nodes and never escaped, on roughly a third of seeds.  It reads like a sampling
failure and is purely geometric.  Two guards: ``step_mm`` defaults below the
free width, and ``extend_frac`` retries the extension at shorter lengths so a
blocked full step becomes a short one rather than a discarded sample.
"""

import numpy as np

import dubins as db
import segments as sg

# firmware/micromouse/constants.h, in mm and mm/s^2
MAX_V_MM_S = 15.7 * 25.0  # WHEEL_RADIUS * MAXIMUM_WHEEL_ANGULAR_VELOCITY
MAX_A_LAT_MM_S2 = 4000.0  # MAXIMUM_LATERAL_ACCELERATION


# ------------------------------------------------------------------ geometry
def _pt_seg_dist(P, A, B):
    """(n,2) points against (m,2)+(m,2) segments -> (n,m)."""
    AB = B - A
    L2 = np.einsum("ij,ij->i", AB, AB)
    L2 = np.where(L2 < 1e-12, 1e-12, L2)
    t = np.einsum("nmj,mj->nm", P[:, None, :] - A[None], AB) / L2[None]
    C = A[None] + np.clip(t, 0.0, 1.0)[..., None] * AB[None]
    return np.linalg.norm(P[:, None, :] - C, axis=2)


def _seg_seg_dist(p, q, A, B):
    """One segment p->q against (m,2)+(m,2) segments -> (m,).

    Endpoint-clamped closest approach, with an explicit orientation test for
    the crossing case -- exact, and cheaper than solving for it.
    """
    d1 = q - p
    d2 = B - A
    r = p[None] - A
    a = float(d1 @ d1)
    e = np.einsum("ij,ij->i", d2, d2)
    f = np.einsum("ij,ij->i", d2, r)
    c = np.einsum("j,ij->i", d1, r)
    b = np.einsum("j,ij->i", d1, d2)
    den = a * e - b * b
    s = np.where(
        np.abs(den) > 1e-12, (b * f - c * e) / np.where(den == 0, 1.0, den), 0.0
    )
    s = np.clip(s, 0.0, 1.0)
    t = np.clip((b * s + f) / np.where(e < 1e-12, 1.0, e), 0.0, 1.0)
    s = np.clip((b * t - c) / (a if a > 1e-12 else 1.0), 0.0, 1.0)
    d = np.linalg.norm(
        (p[None] + s[:, None] * d1[None]) - (A + t[:, None] * d2), axis=1
    )

    def cr(o, u, v):
        return (u[..., 0] - o[..., 0]) * (v[..., 1] - o[..., 1]) - (
            u[..., 1] - o[..., 1]
        ) * (v[..., 0] - o[..., 0])

    cross = ((cr(p, q, A) * cr(p, q, B)) < 0) & (
        (cr(A, B, p[None]) * cr(A, B, q[None])) < 0
    )
    return np.where(cross, 0.0, d)


def _in_poly(P, A, B):
    """Even-odd test, (n,2) against edges A->B of a closed polygon -> (n,)."""
    x = P[:, 0][:, None]
    y = P[:, 1][:, None]
    y0, y1 = A[None, :, 1], B[None, :, 1]
    dy = np.where(y1 == y0, 1e-12, y1 - y0)
    xint = A[None, :, 0] + (y - y0) * (B[None, :, 0] - A[None, :, 0]) / dy
    return (((y0 > y) != (y1 > y)) & (x < xint)).sum(1) % 2 == 1


# --------------------------------------------------------------------- world
class MazeWorld:
    """Collision model in world mm.

    Parameters
    ----------
    robot_radius_mm : the disc the robot is approximated by; everything else is
        inflated by it, so planning happens for a point.
    extra_clearance_mm : padding on top, to absorb the map's own error.  The
        lattice fit is good to ~4 mm rms and the cylinder radius to ~10 mm, so
        this is the knob to turn if a path grazes too close for comfort.
    """

    def __init__(
        self,
        wall_segments_mm,
        cylinders=(),
        deck_mm=None,
        robot_radius_mm=40.0,
        wall_t_mm=12.0,
        extra_clearance_mm=5.0,
        use_conservative_radius=True,
        posts_mm=(),
        post_radius_mm=8.5,
    ):
        S = np.asarray(wall_segments_mm, float).reshape(-1, 2, 2)
        self.A, self.B = S[:, 0], S[:, 1]
        self._mid = 0.5 * (self.A + self.B)  # broad phase
        self._half = 0.5 * np.linalg.norm(self.B - self.A, axis=1)

        self.robot_r = float(robot_radius_mm)
        self.pad = float(extra_clearance_mm)
        self.wall_clear = 0.5 * wall_t_mm + self.robot_r + self.pad
        self.circ_clear = self.robot_r + self.pad

        # "Conservative" means the larger of the two radii, not the bound on its
        # own: a poor silhouette fit can return a base radius above the minimum
        # enclosing circle, and taking the bound blindly then plans against an
        # obstacle smaller than the one that was measured.
        def _radius(c):
            if not use_conservative_radius:
                return c["radius_mm"]
            return max(c["radius_mm"], c.get("radius_mm_bound", c["radius_mm"]))

        cyl = np.array(
            [(c["centre_mm"][0], c["centre_mm"][1], _radius(c)) for c in cylinders],
            float,
        ).reshape(-1, 3)

        # Posts are discs like cylinders, just small and known-radius.  Keeping
        # the counts lets clearance reports say which kind of thing is close.
        P = np.asarray(posts_mm, float).reshape(-1, 2)
        self.post_r = float(post_radius_mm)
        posts = np.column_stack([P, np.full(len(P), self.post_r)]) if len(P) else P
        self.n_cyl, self.n_post = len(cyl), len(P)
        self.circ = np.vstack([cyl, posts.reshape(-1, 3)])

        if deck_mm is None:
            self.poly = self.poly_next = None
        else:
            self.poly = np.asarray(deck_mm, float)
            self.poly_next = np.roll(self.poly, -1, axis=0)

        # A maze can legitimately have no panels at all -- posts on bare floor --
        # so the extent comes from whatever geometry does exist.
        corners = []
        if len(S):
            corners.append(np.vstack([self.A, self.B]))
        if len(self.circ):
            corners.append(self.circ[:, :2] - self.circ[:, 2:3])
            corners.append(self.circ[:, :2] + self.circ[:, 2:3])
        if self.poly is not None:
            corners.append(self.poly)
        if not corners:
            raise ValueError("empty world: no walls, no obstacles, no deck")
        C = np.vstack(corners)
        self.lo, self.hi = C.min(0), C.max(0)

    @property
    def free_width_mm(self):
        """Free width of a lattice corridor -- the ceiling on a useful step."""
        from maze_map import PITCH_MM

        return PITCH_MM - 2.0 * self.wall_clear

    # -- point queries -----------------------------------------------------
    def clearance(self, P):
        """Signed slack in mm: >0 is free, and says how much room is left."""
        P = np.atleast_2d(np.asarray(P, float))
        d = np.full(len(P), np.inf)
        if len(self.A):
            d = _pt_seg_dist(P, self.A, self.B).min(1) - self.wall_clear
        if len(self.circ):
            d = np.minimum(
                d,
                (
                    np.linalg.norm(P[:, None, :] - self.circ[None, :, :2], axis=2)
                    - self.circ[None, :, 2]
                    - self.circ_clear
                ).min(1),
            )
        if self.poly is not None:
            db = _pt_seg_dist(P, self.poly, self.poly_next).min(1) - self.circ_clear
            db = np.where(_in_poly(P, self.poly, self.poly_next), db, -np.abs(db) - 1.0)
            d = np.minimum(d, db)
        return d

    def is_free(self, P):
        return self.clearance(P) > 0.0

    # -- edge queries ------------------------------------------------------
    def motion_valid(self, p, q):
        p = np.asarray(p, float)
        q = np.asarray(q, float)
        m = 0.5 * (p + q)
        h = 0.5 * float(np.linalg.norm(q - p))
        if len(self.A):
            near = (
                np.linalg.norm(self._mid - m, axis=1) <= self._half + h + self.wall_clear
            )
            if (
                near.any()
                and _seg_seg_dist(p, q, self.A[near], self.B[near]).min()
                <= self.wall_clear
            ):
                return False
        if len(self.circ):
            d = _pt_seg_dist(self.circ[:, :2], p[None], q[None])[:, 0]
            if np.any(d <= self.circ[:, 2] + self.circ_clear):
                return False
        if self.poly is not None:
            if _seg_seg_dist(p, q, self.poly, self.poly_next).min() <= self.circ_clear:
                return False
            if not _in_poly(np.array([p, q]), self.poly, self.poly_next).all():
                return False
        return True

    # -- curve queries -----------------------------------------------------
    def curve_valid(self, P, ds):
        """Is a curve free, given points spaced at most ``ds`` of *arc length*?

        Every point of the curve is within ds/2 of arc length from some sample,
        and Euclidean distance never exceeds arc length, so requiring
        ``clearance > ds/2`` at the samples proves the whole curve free.  That
        makes this conservative rather than approximate.

        The price is a ds/2 slice of usable clearance, and it is not academic:
        this deck's chamfered corners leave some doorways under 2 mm of slack
        for a 40 mm robot, so ds is small enough not to wall them off.  Halving
        ds doubles the samples, and the test is vectorised over all of them, so
        that trade is cheap.
        """
        P = np.atleast_2d(np.asarray(P, float))
        if not len(P):
            return False
        # Most candidate edges are blocked, and a blocked one usually fails at
        # many samples at once, so a quarter-density pass rejects the common case
        # for a quarter of the work.  Only rejection is decided here -- a hit on
        # the subsample is a hit on the curve -- so the full test still has the
        # last word on anything that survives.
        if len(P) > 8 and self.clearance(P[::4]).min() <= 0.5 * ds:
            return False
        return bool(self.clearance(P).min() > 0.5 * ds)

    def sample(self, rng):
        return rng.uniform(self.lo, self.hi)

    def sample_pose(self, rng):
        p = self.sample(rng)
        return np.array([p[0], p[1], rng.uniform(-np.pi, np.pi)])


# ---------------------------------------------------------------------- rrt*
def plan(
    world,
    start,
    goal,
    max_iter=4000,
    step_mm=None,
    goal_connect_mm=None,
    goal_bias=0.10,
    gamma=1800.0,
    k_near_max=16,
    extend_frac=(1.0, 0.5, 0.25),
    seed=0,
    stop_on_first=False,
):
    """Returns a dict with ``path`` (k,2) in mm, or ``path=None`` on failure.

    ``step_mm`` defaults to 0.85 of the free corridor width, which for a 40 mm
    robot in a 180 mm maze is ~66 mm.  See the module docstring: a step wider
    than the free space traps the tree in whichever pocket it starts in.

    A short step needs a proportionally larger rewire radius or the tree never
    straightens out; ``gamma=1800`` puts that radius around 150 mm at n=1000,
    which cut the cost spread from +-224 mm to +-68 mm at 4000 iterations for
    about 8% more time.
    """
    rng = np.random.default_rng(seed)
    if step_mm is None:
        step_mm = max(20.0, 0.85 * world.free_width_mm)
    goal_connect_mm = 2.0 * step_mm if goal_connect_mm is None else goal_connect_mm

    start = np.asarray(start, float)
    goal = np.asarray(goal, float)
    for name, p in (("start", start), ("goal", goal)):
        if not world.is_free(p)[0]:
            raise ValueError(
                f"{name} {np.round(p, 1)} is in collision "
                f"(clearance {world.clearance(p)[0]:.1f} mm)"
            )

    V = np.empty((max_iter + 2, 2))
    V[0] = start
    parent = np.full(max_iter + 2, -1, int)
    cost = np.zeros(max_iter + 2)
    n = 1
    best_goal, best_cost = -1, np.inf
    it = -1

    for it in range(max_iter):
        x = goal if rng.random() < goal_bias else world.sample(rng)
        d = np.linalg.norm(V[:n] - x, axis=1)
        near_i = int(np.argmin(d))
        if d[near_i] < 1e-9:
            continue
        u = (x - V[near_i]) / d[near_i]

        x_new = None
        for frac in extend_frac:  # partial extend: a blocked
            cand = V[near_i] + u * min(step_mm * frac, d[near_i])  # full step
            if world.is_free(cand)[0] and world.motion_valid(V[near_i], cand):
                x_new = cand  # becomes a short one, not a
                break  # discarded sample
        if x_new is None:
            continue

        rad = min(step_mm * 3.0, gamma * np.sqrt(np.log(n + 1) / (n + 1)))
        dn = np.linalg.norm(V[:n] - x_new, axis=1)
        near = np.flatnonzero(dn <= rad)
        if len(near) > k_near_max:
            near = near[np.argsort(dn[near])[:k_near_max]]

        pi, pc = near_i, cost[near_i] + np.linalg.norm(x_new - V[near_i])
        for j in near[np.argsort(cost[near] + dn[near])]:
            c = cost[j] + dn[j]
            if c >= pc:
                break
            if world.motion_valid(V[j], x_new):
                pi, pc = int(j), c
                break

        V[n], parent[n], cost[n] = x_new, pi, pc
        new = n
        n += 1

        for j in near:  # rewire
            if j == pi:
                continue
            c = pc + dn[j]
            if c < cost[j] - 1e-9 and world.motion_valid(x_new, V[j]):
                delta = c - cost[j]
                parent[j] = new
                cost[j] = c
                kids = np.flatnonzero(parent[:n] == j)  # one level; the tree
                cost[kids] += delta  # self-heals after

        dg = float(np.linalg.norm(x_new - goal))
        if (
            dg <= goal_connect_mm
            and pc + dg < best_cost
            and world.motion_valid(x_new, goal)
        ):
            best_cost, best_goal = pc + dg, new
            if stop_on_first:
                break

    if best_goal < 0:
        return dict(
            path=None,
            cost=np.inf,
            raw_cost=np.inf,
            nodes=V[:n],
            parent=parent[:n],
            iters=it + 1,
            step_mm=step_mm,
        )

    path = [goal]
    k = best_goal
    while k != -1:
        path.append(V[k])
        k = parent[k]
    path = shortcut(world, np.array(path[::-1]), rng)
    return dict(
        path=path,
        cost=path_length(path),
        raw_cost=float(best_cost),
        nodes=V[:n],
        parent=parent[:n],
        iters=it + 1,
        step_mm=step_mm,
    )


def path_length(P):
    return float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())


def shortcut(world, path, rng, rounds=300):
    """Random pairwise shortcutting.  RRT* paths are optimal in the graph, not
    in the free space; this takes out most of the remaining zig-zag."""
    P = list(path)
    for _ in range(rounds):
        if len(P) < 3:
            break
        i = int(rng.integers(0, len(P) - 2))
        j = int(rng.integers(i + 2, len(P)))
        if world.motion_valid(P[i], P[j]):
            del P[i + 1 : j]
    return np.array(P)


# ------------------------------------------------------------- dubins rrt*
def _snap(v, pitch, offset):
    """Nearest corridor centreline coordinate."""
    return offset + pitch * np.round((v - offset) / pitch)


def _edge_valid(q0, word, params, rho, ds, world):
    return world.curve_valid(db.sample(q0, word, params, rho, ds), ds)


def best_heading(world, xy, reach_mm=400.0, ds_mm=5.0):
    """The cardinal heading with the longest clear run ahead of ``xy``.

    A forward-only car cannot start facing a panel 40 mm away, and in a maze
    the start cell often has exactly one open side, so guessing wrong looks
    identical to "no path exists".  Ties break toward +x.
    """
    xy = np.asarray(xy, float).reshape(2)
    best, best_run = 0.0, -1.0
    for th in (0.0, np.pi / 2.0, np.pi, -np.pi / 2.0):
        u = np.array([np.cos(th), np.sin(th)])
        P = xy + np.outer(np.arange(0.0, reach_mm + ds_mm, ds_mm), u)
        c = world.clearance(P)
        blocked = np.flatnonzero(c <= 0.0)
        run = reach_mm if not len(blocked) else ds_mm * (blocked[0] - 1)
        if run > best_run:
            best, best_run = th, run
    return best


def speed_limit_mm_s(rho_mm, a_lat_mm_s2=MAX_A_LAT_MM_S2, v_max_mm_s=MAX_V_MM_S):
    """Fastest a path of radius ``rho`` can be driven, from v = sqrt(a r).

    The planner does not pick a speed, but the radius it picks caps one, and the
    cap belongs next to the path: MotionPlanner drives a whole path at a single
    cruiseVelocity, so the tightest arc sets it.  At 4 m/s^2 a 30 mm arc allows
    346 mm/s against the robot's 392 mm/s ceiling.
    """
    return float(min(v_max_mm_s, np.sqrt(a_lat_mm_s2 * max(rho_mm, 1e-9))))


def plan_dubins(
    world,
    start,
    goal,
    rho=30.0,
    max_iter=4000,
    step_mm=None,
    goal_bias=0.10,
    gamma=2600.0,
    k_near_max=12,
    k_steer=3,
    extend_frac=(1.0, 0.5, 0.25),
    ds_mm=2.5,
    goal_headings=16,
    lattice_frac=0.7,
    lattice_pitch=180.0,
    lattice_offset=90.0,
    axis_sigma=0.10,
    seed=0,
    stop_on_first=False,
    shortcut_rounds=200,
):
    """RRT* in SE(2) with Dubins steering: a path of straights and arcs.

    ``start`` and ``goal`` are ``(x, y, theta)`` in world mm/radians.  A goal
    ``theta`` of ``None`` leaves the final heading free -- the connect step then
    picks the cheapest of ``goal_headings`` evenly spaced headings, which is
    what you want when the goal is "be in that cell", not "be in that cell
    facing north".

    ``rho`` is the turn radius every arc uses, so the output is single-curvature
    by construction: the robot only ever drives straight or at 1/rho.  Read the
    module docstring before changing it: the feasible radii in a 180 mm maze are
    a pair of bands rather than a range, and only the lower one -- where the
    default sits -- is searchable.

    Returns the same keys as ``plan`` plus ``poses`` (k,3) and ``segments``, the
    firmware-ready ``Segment`` list.  ``path`` is a dense polyline through the
    curve, so the existing overlay and clearance code keeps working.
    """
    rng = np.random.default_rng(seed)
    if step_mm is None:
        step_mm = max(30.0, 0.85 * world.free_width_mm)

    start = np.asarray(start, float).reshape(3)
    g = list(goal)
    goal_theta = float(g[2]) if len(g) > 2 and g[2] is not None else None
    goal_xy = np.asarray(g[:2], float)
    headings = (
        np.array([goal_theta])
        if goal_theta is not None
        else np.linspace(-np.pi, np.pi, goal_headings, endpoint=False)
    )

    goal_poses = np.column_stack(
        [np.full(len(headings), goal_xy[0]), np.full(len(headings), goal_xy[1]), headings]
    )

    for name, p in (("start", start[:2]), ("goal", goal_xy)):
        if not world.is_free(p)[0]:
            raise ValueError(
                f"{name} {np.round(p, 1)} is in collision "
                f"(clearance {world.clearance(p)[0]:.1f} mm)"
            )

    V = np.empty((max_iter + 2, 3))
    V[0] = start
    parent = np.full(max_iter + 2, -1, int)
    cost = np.zeros(max_iter + 2)
    e_word = np.zeros(max_iter + 2, int)  # the edge that reaches each node,
    e_par = np.zeros((max_iter + 2, 3))  # kept because a steer's edge is a
    n = 1  # *prefix*, not the shortest path
    best_goal, best_cost, best_edge = -1, np.inf, None
    it = -1

    axis = np.array([0.0, np.pi / 2.0, np.pi, -np.pi / 2.0])  # x+, y+, x-, y-

    for it in range(max_iter):
        if rng.random() < goal_bias:
            q_rand = np.array([goal_xy[0], goal_xy[1], rng.choice(headings)])
        else:
            q_rand = world.sample_pose(rng)
            # Maze-aware sampling.  The free space is not a blob, it is a thin
            # lattice: a 180 mm cell inflated by a 45 mm robot leaves a 78 mm
            # corridor, so a uniform (x, y, theta) sample is in a panel about
            # four times out of five and points across the corridor even when it
            # is not.  Snapping one coordinate onto the corridor centreline and
            # the heading along that corridor puts the sample where the robot
            # can actually be.  The uniform remainder is what keeps the sampler
            # honest -- open floor, the deck margin and the space around a
            # cylinder are all off-lattice, and still get covered.
            if rng.random() < lattice_frac:
                if rng.random() < 0.5:
                    q_rand[0] = _snap(q_rand[0], lattice_pitch, lattice_offset)
                    q_rand[2] = rng.choice(axis[1::2])  # along a +/-y corridor
                else:
                    q_rand[1] = _snap(q_rand[1], lattice_pitch, lattice_offset)
                    q_rand[2] = rng.choice(axis[0::2])  # along a +/-x corridor
                q_rand[2] += rng.normal(0.0, axis_sigma)

        L, k, par = db.lengths(V[:n], q_rand[None], rho)

        # Steer from the k nearest nodes, not just the nearest.  A node whose
        # heading points into a panel cannot move at all, yet its Dubins basin
        # can cover half the maze -- with a single-nearest rule it swallows most
        # samples and the tree stalls at a few dozen nodes.  Goal biasing makes
        # that worse, because it keeps handing the same stuck node the same
        # sample.  Trying the runners-up is the same idea as extend_frac: do not
        # throw a sample away because the first thing you tried was blocked.
        q_new = None
        for i_near in np.argsort(L)[:k_steer]:
            i_near = int(i_near)
            if not np.isfinite(L[i_near]) or L[i_near] < 1e-9:
                continue
            word = db.WORDS[k[i_near]]
            for frac in extend_frac:
                s = min(step_mm * frac, L[i_near])
                p_cut = db.truncate(word, par[i_near], rho, s)
                cand = db.endpoint(V[i_near], word, p_cut, rho)
                if not world.is_free(cand[:2])[0]:
                    continue
                if not _edge_valid(V[i_near], word, p_cut, rho, ds_mm, world):
                    continue
                q_new, w_new, p_new = cand, word, p_cut
                c_new = cost[i_near] + s
                i_par = i_near
                break
            if q_new is not None:
                break
        if q_new is None:
            continue

        # choose parent: Dubins cost-to-come over a Euclidean near set
        rad = min(step_mm * 4.0, gamma * np.sqrt(np.log(n + 1) / (n + 1)))
        dn = np.linalg.norm(V[:n, :2] - q_new[:2], axis=1)
        near = np.flatnonzero(dn <= rad)
        if len(near) > k_near_max:
            near = near[np.argsort(dn[near])[:k_near_max]]
        if len(near):
            Ln, kn, pn = db.lengths(V[near], q_new[None], rho)
            tot = cost[near] + Ln
            for m in np.argsort(tot):
                if tot[m] >= c_new:
                    break
                j = int(near[m])
                if _edge_valid(V[j], db.WORDS[kn[m]], pn[m], rho, ds_mm, world):
                    i_par, c_new = j, float(tot[m])
                    w_new, p_new = db.WORDS[kn[m]], pn[m]
                    break

        V[n], parent[n], cost[n] = q_new, i_par, c_new
        e_word[n], e_par[n] = db.WORDS.index(w_new), p_new
        new = n
        n += 1

        # rewire: Dubins is asymmetric, so this is a second set of queries
        if len(near):
            Lr, kr, pr = db.lengths(q_new[None], V[near], rho)
            for m, j in enumerate(near):
                j = int(j)
                if j == i_par or j == new:
                    continue
                c = c_new + Lr[m]
                if c < cost[j] - 1e-9 and _edge_valid(
                    q_new, db.WORDS[kr[m]], pr[m], rho, ds_mm, world
                ):
                    delta = c - cost[j]
                    parent[j] = new
                    cost[j] = c
                    e_word[j], e_par[j] = int(kr[m]), pr[m]
                    kids = np.flatnonzero(parent[:n] == j)
                    cost[kids] += delta

        # connect to the goal, over every allowed final heading, cheapest first
        Lg, kg, pg = db.lengths(q_new[None], goal_poses, rho)
        for m in np.argsort(Lg):
            if c_new + Lg[m] >= best_cost:
                break
            if _edge_valid(q_new, db.WORDS[kg[m]], pg[m], rho, ds_mm, world):
                best_cost, best_goal = c_new + float(Lg[m]), new
                best_edge = (db.WORDS[kg[m]], pg[m].copy())
                break
        if stop_on_first and best_goal >= 0:
            break

    out = dict(
        nodes=V[:n],
        parent=parent[:n],
        iters=it + 1,
        step_mm=step_mm,
        rho=rho,
        ds_mm=ds_mm,
    )
    if best_goal < 0:
        return dict(out, path=None, poses=None, segments=None, cost=np.inf,
                    raw_cost=np.inf)

    chain = []
    k = best_goal
    while k != -1:
        chain.append(k)
        k = parent[k]
    chain.reverse()

    # Every edge here is one the search validated, taken verbatim: rebuilding
    # them from the endpoint poses would silently swap a steered prefix for a
    # shortest path that was never collision checked.
    edges = [
        (V[parent[i]], db.WORDS[e_word[i]], e_par[i].copy()) for i in chain[1:]
    ]
    edges.append((V[best_goal], best_edge[0], best_edge[1]))
    edges = shortcut_edges(world, edges, rho, rng, ds_mm, shortcut_rounds)

    segs = []
    for q, word, par in edges:
        segs += sg.from_dubins(q, word, par, rho)
    segs = sg.clean(segs)
    poses = np.array([e[0] for e in edges] + [db.endpoint(*edges[-1], rho)])
    return dict(
        out,
        poses=poses,
        edges=edges,
        segments=segs,
        path=sg.polyline(segs, 5.0),
        cost=sg.length(segs),
        raw_cost=float(best_cost),
    )


def _edge_len(edge, rho):
    return float(np.sum(edge[2]) * rho)


def shortcut_edges(world, edges, rho, rng, ds_mm=2.5, rounds=200):
    """Random pairwise shortcutting over a chain of Dubins edges.

    Same idea as ``shortcut`` for polylines, except the replacement is itself a
    Dubins path, so a successful splice cannot introduce a corner the robot
    can't drive.  The intermediate poses carry headings inherited from the tree,
    which are often badly aligned with the corridor; dropping them is where most
    of the length comes off.
    """
    E = list(edges)
    for _ in range(rounds):
        if len(E) < 2:
            break
        i = int(rng.integers(0, len(E) - 1))
        j = int(rng.integers(i + 1, len(E)))  # replace edges i..j inclusive
        q_a = E[i][0]
        q_b = db.endpoint(*E[j], rho)
        direct, word, par = db.shortest(q_a, q_b, rho)
        if word is None:
            continue
        if direct >= sum(_edge_len(E[k], rho) for k in range(i, j + 1)) - 1e-6:
            continue
        if _edge_valid(q_a, word, par, rho, ds_mm, world):
            E[i : j + 1] = [(q_a, word, par)]
    return E


def resample(path, ds_mm=20.0):
    """Even arc-length resampling, for feeding a trajectory generator."""
    s = np.concatenate(
        [[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    )
    t = np.arange(0.0, s[-1] + 1e-9, ds_mm)
    return np.stack([np.interp(t, s, path[:, 0]), np.interp(t, s, path[:, 1])], 1)
