#!/usr/bin/env -S uv run --script
"""Maze occupancy from a fitted lattice.

Consumes the dict returned by ``maze_grid.solve`` and adds:

    WorldFrame        mm <-> px, on the floor plane or the post mid-height plane
    detect_walls      is there a panel on each of the 2*N*(N-1) lattice bonds?
    detect_posts      is there a post on each lattice node?
    detect_deck       the octagonal deck surface, as a polygon in mm
    detect_cylinders  free-standing round obstacles, as (centre_mm, radius_mm)
    build_map         all of the above in one dict, in world mm

Posts are obstacles, not just landmarks
---------------------------------------
The lattice fit needs posts to exist; the *map* must not assume they do.  A node
can carry a post with no panel on any of its four bonds -- an isolated stump in
open floor that a wall-only collision model drives straight through -- and a
node can equally carry nothing at all, which is why 7 of the 100 nodes in
mazes/1.png carry none.  ``detect_posts`` scores every node independently and
returns per-node presence, so both cases come out of the same measurement;
``prune_posts`` then drops the claims that the deck or a cylinder already
accounts for.

Geometry note
-------------
Everything above the floor images where its radial projection onto the floor
would: a point at height h maps to ``G + (X - G) * Hc/(Hc - h)`` with G the
ground nadir.  So in ground coordinates every horizontal slice of the scene is
a homothety about G.  That single fact is what the panel sweep and the cylinder
silhouette model both use, and it needs no lens or attitude knowledge beyond
the homography maze_grid already fitted.

The vertical scale comes from ``streak["slope"]``, which maze_grid flags as the
softest number in the pipeline.  Nothing here is sensitive to it at the few
percent level -- it only sets how wide a band gets swept, and the bands are
deliberately generous.
"""

import cv2
import numpy as np

PITCH_MM = 180.0
WALL_T_MM = 12.0            # panel thickness; used for masking and inflation
POST_T_MM = 12.0            # post cross-section, square
POST_R_MM = 0.5 * POST_T_MM * np.sqrt(2.0)   # circumscribed, so inflation is safe


# ------------------------------------------------------------------- frames
class WorldFrame:
    """mm <-> px on the two useful planes.

    World origin is lattice node (0, 0) and the axes run along the lattice, so
    node (i, j) sits at (180i, 180j) mm.  ``plane="floor"`` is the deck;
    ``plane="mid"`` is post mid-height, where the detected centroids live and
    where the panels are widest in the image.
    """

    def __init__(self, res):
        self.H = {"floor": np.asarray(res["H_floor"], float),
                  "mid": np.asarray(res["H_mid"], float)}
        self.Hinv = {k: np.linalg.inv(v) for k, v in self.H.items()}
        self.nadir_px = np.asarray(res["nadir"], float)
        self.slope = float(res["streak"]["slope"])
        self.shape = tuple(int(v) for v in np.asarray(res["idx"]).max(0) + 1)

    def _apply(self, M, p):
        p = np.asarray(p, float).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(p, M).reshape(-1, 2)

    def mm_to_px(self, p_mm, plane="floor"):
        return self._apply(self.H[plane], p_mm)

    def px_to_mm(self, p_px, plane="floor"):
        return self._apply(self.Hinv[plane], p_px)

    def node_mm(self, i, j):
        return np.array([PITCH_MM * i, PITCH_MM * j], float)

    def cell_centre_mm(self, i, j):
        return np.array([PITCH_MM * i + 90.0, PITCH_MM * j + 90.0], float)

    @property
    def nadir_mm(self):
        """Ground point directly under the camera, in world mm."""
        return self.px_to_mm(self.nadir_px, "floor")[0]

    @property
    def mm_per_px(self):
        """Local floor scale at the deck centre."""
        c = self.bounds_mm().mean(0)
        a = self.mm_to_px(c, "floor")[0]
        b = self.mm_to_px(c + np.array([10.0, 0.0]), "floor")[0]
        return 10.0 / max(float(np.linalg.norm(b - a)), 1e-9)

    def lift_px(self, p_px, h_frac):
        """Image position of a floor point raised to ``h_frac`` post heights."""
        p = np.asarray(p_px, float)
        return self.nadir_px + (p - self.nadir_px) * (1.0 + self.slope * h_frac)

    def bounds_mm(self):
        nx, ny = self.shape
        return np.array([[0.0, 0.0],
                         [PITCH_MM * (nx - 1), PITCH_MM * (ny - 1)]])


# -------------------------------------------------------------------- walls
def _ribbon(gray, a, b, half_px, nt=21, ns=15, trim=0.18):
    """Sample a ribbon centred on segment a->b, ``half_px`` to either side."""
    d = b - a
    L = float(np.linalg.norm(d))
    if L < 1e-6:
        return None
    u = d / L
    n = np.array([-u[1], u[0]])
    t = np.linspace(trim, 1.0 - trim, nt)[:, None, None]
    s = np.linspace(-half_px, half_px, ns)[None, :, None]
    pts = (a + t * d + s * n).astype(np.float32)
    return cv2.remap(gray, pts[..., 0], pts[..., 1], cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_REPLICATE)


def detect_walls(img_bgr, res, thresh=45.0, ambiguous_band=20.0, half_frac=0.14):
    """One score per lattice bond; a panel darkens the bond line.

    score = local floor brightness - darkest point of the cross profile,
    median-reduced along the bond.

    Both halves matter.  An absolute grey threshold fails because the panels
    are not one colour in the image: side faces read ~40 and grazing-lit tops
    read ~120 against a ~175 deck, and the deck itself vignettes toward the
    corners.  Taking the reference locally (80th percentile of a 1.5-pitch box)
    makes both cases the same measurement.  Reducing each cross section by its
    minimum handles the lean -- a panel is centred on the bond at the floor but
    displaces outward from the nadir with height, so the ribbon is sampled
    wider than the panel is thick and the darkest sample lands on the panel
    wherever it happens to have fallen.

    Returns
    -------
    dict with ``vertical`` (nx-1, ny) and ``horizontal`` (nx, ny-1) boolean
    arrays -- bonds along +x and +y respectively -- the raw scores, and the
    bonds sitting inside ``ambiguous_band`` of the threshold.  Check that list
    before trusting a map.
    """
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    W = WorldFrame(res)
    nx, ny = W.shape
    ij = np.mgrid[0:nx, 0:ny].reshape(2, -1).T.astype(float)
    P = W.mm_to_px(PITCH_MM * ij, "mid").reshape(nx, ny, 2)

    pitch_px = float(res["pitch_px"])
    half = half_frac * pitch_px
    win = int(round(0.75 * pitch_px))

    sc = {"vertical": np.full((nx - 1, ny), np.nan),
          "horizontal": np.full((nx, ny - 1), np.nan)}
    for key, (di, dj) in (("vertical", (1, 0)), ("horizontal", (0, 1))):
        for i in range(nx - di):
            for j in range(ny - dj):
                a, b = P[i, j], P[i + di, j + dj]
                v = _ribbon(gray, a, b, half)
                dark = float(np.median(v.min(1)))
                m = 0.5 * (a + b)
                x0 = int(np.clip(m[0] - win, 0, gray.shape[1] - 1))
                y0 = int(np.clip(m[1] - win, 0, gray.shape[0] - 1))
                ref = float(np.percentile(gray[y0:y0 + 2 * win,
                                               x0:x0 + 2 * win], 80))
                sc[key][i, j] = ref - dark

    amb = []
    for key in sc:
        for i, j in zip(*np.where(np.abs(sc[key] - thresh) < ambiguous_band)):
            amb.append((key, int(i), int(j), float(sc[key][i, j])))
    amb.sort(key=lambda r: -r[3])

    return dict(vertical=sc["vertical"] > thresh,
                horizontal=sc["horizontal"] > thresh,
                score=sc, thresh=thresh, ambiguous=amb, node_px=P)


def wall_segments_mm(walls):
    """Bonds carrying a panel, as (m, 2, 2) mm endpoint pairs."""
    segs = []
    for key, (di, dj) in (("vertical", (1, 0)), ("horizontal", (0, 1))):
        for i, j in zip(*np.where(walls[key])):
            segs.append([[PITCH_MM * i, PITCH_MM * j],
                         [PITCH_MM * (i + di), PITCH_MM * (j + dj)]])
    return np.array(segs, float).reshape(-1, 2, 2)


def cell_walls(walls):
    """Per-cell N/E/S/W flags; cell (i, j) spans nodes i..i+1, j..j+1.

    +x is East and +y is South (image convention), so N is the j-side bond.
    """
    nx = walls["vertical"].shape[0] + 1
    ny = walls["horizontal"].shape[1] + 1
    out = np.zeros((nx - 1, ny - 1, 4), bool)
    for i in range(nx - 1):
        for j in range(ny - 1):
            out[i, j] = (walls["vertical"][i, j],          # N   y = j
                         walls["horizontal"][i + 1, j],    # E   x = i+1
                         walls["vertical"][i, j + 1],      # S   y = j+1
                         walls["horizontal"][i, j])        # W   x = i
    return out


def cell_graph(cells):
    """Cell adjacency, for a BFS sanity check before spending time on RRT."""
    nx, ny = cells.shape[:2]
    step = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return {(i, j): [(i + di, j + dj) for k, (di, dj) in enumerate(step)
                     if not cells[i, j, k]
                     and 0 <= i + di < nx and 0 <= j + dj < ny]
            for i in range(nx) for j in range(ny)}


def reachable(cells, start=(0, 0)):
    """Cells reachable from ``start`` through the detected gaps."""
    from collections import deque
    g = cell_graph(cells)
    seen = {start}
    q = deque([start])
    while q:
        for m in g[q.popleft()]:
            if m not in seen:
                seen.add(m)
                q.append(m)
    return seen


# -------------------------------------------------------------------- posts
def _streak_dark(gray, base, top, half, min_len=4.0):
    """Darkest point of a post's silhouette, swept base -> top.

    Near the nadir a post images as a dot rather than a streak, and the ribbon
    degenerates; there the darkest pixel in a disc of the post's own radius is
    the same measurement taken the only way that is left.
    """
    if float(np.linalg.norm(top - base)) >= min_len:
        v = _ribbon(gray, base, top, half, trim=0.05)
        if v is not None:
            return float(np.median(v.min(1)))
    r = int(max(1, round(half)))
    x, y = int(round(base[0])), int(round(base[1]))
    x0, x1 = np.clip([x - r, x + r + 1], 0, gray.shape[1])
    y0, y1 = np.clip([y - r, y + r + 1], 0, gray.shape[0])
    patch = gray[y0:y1, x0:x1]
    return float(patch.min()) if patch.size else float(gray.max())


def detect_posts(img_bgr, res, thresh=None, ambiguous_band=15.0, blue_min_px=3,
                 blue_radius_frac=0.16):
    """One presence flag per lattice node.

    Two independent pieces of evidence, because neither alone covers every node.
    The blue cap is what maze_grid segmented to fit the lattice in the first
    place, so a node it already indexed is a post with no further argument --
    but that HSV gate is deliberately tight (widening it merges neighbouring
    posts at ~50 px pitch) and drops caps that are dim, clipped by the frame, or
    half hidden behind a panel.  The fallback is the wall detector's darkness
    measurement pointed along the post's own streak, base to top, since a post
    leans away from the nadir exactly like a panel does.

    An empty socket reads as a faint ring on a bright deck and scores near zero;
    a post reads as a dark column, so the two populations separate cleanly.  The
    threshold is self-calibrating -- half the median score of the nodes maze_grid
    already accepted -- to stay in keeping with the rest of the pipeline.

    Nodes with a panel on an adjoining bond score high whether or not a post is
    there, since the panel is in the ribbon.  That costs nothing: the panel's own
    inflation already covers the node.  The nodes that matter are the isolated
    ones, where the only thing in the ribbon is the post or the bare deck.

    Returns ``present`` (nx, ny) bool, the scores, and the nodes sitting within
    ``ambiguous_band`` of the threshold -- check that list before trusting a map.
    """
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    W = WorldFrame(res)
    nx, ny = W.shape

    detected = np.zeros((nx, ny), bool)
    for i, j in np.asarray(res["idx"], int):
        if 0 <= i < nx and 0 <= j < ny:
            detected[i, j] = True

    pitch_px = float(res["pitch_px"])
    half = max(2.0, 0.5 * POST_T_MM / W.mm_per_px)
    win = int(round(0.75 * pitch_px))
    br = int(max(2, round(blue_radius_frac * pitch_px)))
    mask = res["mask"]

    score = np.zeros((nx, ny))
    blue = np.zeros((nx, ny), int)
    for i in range(nx):
        for j in range(ny):
            node = W.node_mm(i, j)
            base = W.mm_to_px(node, "floor")[0]
            top = W.lift_px(base, 1.0)
            dark = _streak_dark(gray, base, top, half)
            m = 0.5 * (base + top)
            x0 = int(np.clip(m[0] - win, 0, gray.shape[1] - 1))
            y0 = int(np.clip(m[1] - win, 0, gray.shape[0] - 1))
            ref = float(np.percentile(gray[y0:y0 + 2 * win, x0:x0 + 2 * win], 80))
            score[i, j] = ref - dark

            mid = W.mm_to_px(node, "mid")[0]
            bx, by = int(round(mid[0])), int(round(mid[1]))
            bx0, bx1 = np.clip([bx - br, bx + br + 1], 0, mask.shape[1])
            by0, by1 = np.clip([by - br, by + br + 1], 0, mask.shape[0])
            blue[i, j] = int(np.count_nonzero(mask[by0:by1, bx0:bx1]))

    if thresh is None:
        seen = score[detected]
        thresh = float(0.5 * np.median(seen)) if seen.size else 45.0

    present = detected | (score > thresh) | (blue >= blue_min_px)

    amb = [(int(i), int(j), float(score[i, j]))
           for i, j in zip(*np.where((np.abs(score - thresh) < ambiguous_band)
                                     & ~detected & (blue < blue_min_px)))]
    amb.sort(key=lambda r: -r[2])

    return dict(present=present, score=score, blue=blue, detected=detected,
                thresh=float(thresh), ambiguous=amb)


def post_centres_mm(posts):
    """Standing posts, as (m, 2) mm centres."""
    ij = np.argwhere(posts["present"])
    return PITCH_MM * ij.astype(float)


def prune_posts(posts, deck_mm=None, cylinders=(), margin_mm=30.0):
    """Drop post claims that some other obstacle already accounts for.

    Two sources of them, both of which show up as an obstacle sitting where no
    post is:

    Off the deck.  The lattice fit spans whatever blue blobs it found, and a
    reflection off the frame can add a whole phantom row outside the maze.  Its
    "posts" score high because the frame is dark, and the deck polygon is
    already a keep-in region, so they are pure noise.

    Under a cylinder.  A free-standing obstacle blackens the node it stands on,
    so the silhouette test claims a post there.  The fitted cylinder covers that
    ground with a much larger disc, so the claim adds nothing.

    Neither removal weakens the model.  A post ``d`` mm outside the deck can
    only be approached to ``d + robot_clear`` by a robot the deck already keeps
    ``robot_clear`` inside, so once ``d`` exceeds the post's own radius the deck
    constraint is the tighter of the two; ``margin_mm`` is several times that.
    A post inside a cylinder's fitted radius is inside a strictly larger disc.

    What it does assume is that the deck polygon is trusted, which the collision
    model already assumes -- it is a hard keep-in region there too.
    """
    keep = posts["present"].copy()
    for i, j in np.argwhere(posts["present"]):
        c = PITCH_MM * np.array([i, j], float)
        if deck_mm is not None:
            poly = np.asarray(deck_mm, np.float32).reshape(-1, 1, 2)
            if cv2.pointPolygonTest(poly, (float(c[0]), float(c[1])), True) < -margin_mm:
                keep[i, j] = False
                continue
        for cyl in cylinders:
            if np.linalg.norm(c - cyl["centre_mm"]) <= cyl["radius_mm_bound"]:
                keep[i, j] = False
                break
    return dict(posts, present=keep, pruned=int(posts["present"].sum() - keep.sum()))


def isolated_posts(posts, walls):
    """Posts with no panel on any of their four bonds.

    These are the ones a wall-only collision model misses entirely, so the count
    is worth printing: if it is non-zero, the wall segments are not the whole
    obstacle set.
    """
    V, H = walls["vertical"], walls["horizontal"]
    nx, ny = posts["present"].shape
    out = []
    for i, j in np.argwhere(posts["present"]):
        touched = (
            (i > 0 and V[i - 1, j])
            or (i < nx - 1 and V[i, j])
            or (j > 0 and H[i, j - 1])
            or (j < ny - 1 and H[i, j])
        )
        if not touched:
            out.append((int(i), int(j)))
    return out


# --------------------------------------------------------------------- deck
def detect_deck(img_bgr, res, bright_pct=45, simplify_mm=20.0):
    """The lit deck surface, as a convex polygon in world mm.

    The deck is the one large bright region inside the lattice; its outline
    recovers the chamfered corners, which a square lattice knows nothing about.
    ``approxPolyDP`` collapses the hull to the real octagon, which also keeps
    the planner's point-in-polygon test cheap -- the raw 26-gon dominated the
    planning profile.
    """
    W = WorldFrame(res)
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    (x0, y0), (x1, y1) = W.bounds_mm()
    corners = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)
    box = np.zeros(gray.shape, np.uint8)
    cv2.fillConvexPoly(box, np.int32(cv2.convexHull(
        W.mm_to_px(corners, "floor").astype(np.float32))), 255)

    thr = np.percentile(gray[box > 0], bright_pct)
    bw = ((gray > thr) & (box > 0)).astype(np.uint8)
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(bw, 8)
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    ys, xs = np.nonzero(lab == k)
    hull = cv2.convexHull(np.stack([xs, ys], 1).astype(np.float32))
    hull = cv2.approxPolyDP(hull, simplify_mm / W.mm_per_px, True)
    return W.px_to_mm(hull.reshape(-1, 2), "floor")


# ---------------------------------------------------------------- cylinders
def _occluder_mask(res, walls, W, posts=None, grow_px=3.0):
    """Everything a dark blob is allowed to be: panels and posts.

    Each panel is swept from its floor footprint to its top edge by the nadir
    homothety, so the mask covers the leaning face rather than just the bond
    line.  Near the nadir that quad collapses to nothing, which is why the
    footprint is also stroked at the panel's own projected thickness -- without
    it a near-nadir panel leaks a sliver that reads as a small obstacle.

    Posts get the same sweep, but only where a blue cap was actually segmented.
    Masking a silhouette-only claim would be circular and it is dangerous: a
    cylinder standing on a lattice node scores as a post, and stamping a stripe
    down its middle splits the blob into two pieces that then fail the solidity
    test -- a 47 mm obstacle quietly demoted to an 8.5 mm one.  Erring the other
    way is cheap, because a capless post reported as a cylinder is still an
    obstacle in roughly the right place, and ``prune_posts`` drops the duplicate
    claim afterwards.
    """
    m = np.zeros(res["mask"].shape, np.uint8)
    thick = int(max(3, round(WALL_T_MM / W.mm_per_px + 2)))
    for s in wall_segments_mm(walls):
        base = W.mm_to_px(s, "floor")
        top = W.lift_px(base, 1.15)
        cv2.fillConvexPoly(m, np.int32(np.round(
            np.array([base[0], base[1], top[1], top[0]]))), 255)
        for p, q in ((base[0], base[1]), (top[0], top[1])):
            cv2.line(m, np.int32(np.round(p)), np.int32(np.round(q)), 255, thick)
    for P in res["blobs"]:                       # the posts, already segmented
        m[np.int32(P[:, 1]), np.int32(P[:, 0])] = 255
    if posts is not None:
        pthick = int(max(3, round(POST_T_MM / W.mm_per_px + 2)))
        for c in PITCH_MM * np.argwhere(posts["detected"]).astype(float):
            base = W.mm_to_px(c, "floor")[0]
            top = W.lift_px(base, 1.15)
            cv2.line(m, np.int32(np.round(base)), np.int32(np.round(top)),
                     255, pthick)
    k = int(max(1, round(grow_px)))
    return cv2.dilate(m, np.ones((2 * k + 1, 2 * k + 1), np.uint8))


def _hull_sdf(pts, C, rho, G, k):
    """Signed distance of ``pts`` to a vertical cylinder's silhouette.

    In ground coordinates that silhouette is the convex hull of the base circle
    (C, rho) and its homothety about the ground nadir G by k -- a 2D capsule
    with linearly varying radius.  Distance is the min over both discs and the
    connecting cone.
    """
    C = np.asarray(C, float)
    C2 = G + (C - G) * k
    d1 = np.linalg.norm(pts - C, axis=1) - rho
    d2 = np.linalg.norm(pts - C2, axis=1) - rho * k
    ax = C2 - C
    L = float(np.linalg.norm(ax))
    if L < 1e-9:
        return np.minimum(d1, d2)
    t = np.clip(((pts - C) @ ax) / (L * L), 0.0, 1.0)
    foot = C + t[:, None] * ax
    rad = rho * (1.0 + t * (k - 1.0))
    return np.minimum(np.minimum(d1, d2),
                      np.linalg.norm(pts - foot, axis=1) - rad)


def _fit_cylinder(contour_mm, G, k_max=1.35, n_k=36):
    """Least squares (centre, base radius, height factor) from a silhouette.

    k = Hc/(Hc-h) is the badly conditioned axis -- a short obstacle far from
    the nadir looks much like a tall one near it -- so it is swept on a grid
    and (C, rho) solved inside.  Also returns the minimum enclosing radius,
    a hard upper bound on the footprint and the safer number to plan against
    when the height is unresolved.
    """
    from scipy.optimize import minimize

    c0, r0 = cv2.minEnclosingCircle(contour_mm.astype(np.float32))
    c0 = np.asarray(c0, float)
    best = None
    for k in np.linspace(1.0, k_max, n_k):
        def loss(p, k=k):
            return float(np.sqrt(np.mean(
                _hull_sdf(contour_mm, p[:2], abs(p[2]), G, k) ** 2)))
        p0 = np.array([c0[0], c0[1], r0 / (0.5 * (1.0 + k))])
        r = minimize(loss, p0, method="Nelder-Mead",
                     options=dict(xatol=1e-3, fatol=1e-5, maxiter=4000))
        if best is None or r.fun < best[0]:
            best = (float(r.fun), float(k), r.x.copy())
    rms, k, x = best
    return x[:2], abs(x[2]), k, rms, float(r0)


def detect_cylinders(img_bgr, res, walls, posts=None, min_radius_mm=20.0,
                     max_radius_mm=170.0, dark_drop=70.0, min_solidity=0.85,
                     post_h_mm=50.0):
    """Free-standing round obstacles, anywhere on the deck.

    A blob qualifies if it is dark, on the deck, convex, big enough to matter,
    and not already accounted for by a detected panel or post.  Its outline is
    then fitted with the cylinder silhouette model, which returns the *base*
    circle -- the footprint a planner needs -- rather than the fatter leaning
    silhouette a plain ellipse fit would give.
    """
    W = WorldFrame(res)
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    deck = detect_deck(img_bgr, res)

    on_deck = np.zeros(gray.shape, np.uint8)
    cv2.fillConvexPoly(on_deck, np.int32(np.round(
        W.mm_to_px(deck, "floor"))), 255)
    on_deck = cv2.erode(on_deck, np.ones((5, 5), np.uint8))

    floor_level = np.percentile(gray[on_deck > 0], 70)
    dark = ((gray < floor_level - dark_drop) & (on_deck > 0)).astype(np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    known = _occluder_mask(res, walls, W, posts)
    free = cv2.bitwise_and(dark, cv2.bitwise_not(known))
    free = cv2.morphologyEx(free, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    n, lab, stats, cent = cv2.connectedComponentsWithStats(free, 8)
    G = W.nadir_mm
    min_area_px = np.pi * (min_radius_mm / W.mm_per_px) ** 2
    Hc = post_h_mm * (1.0 + W.slope) / W.slope if W.slope > 1e-6 else np.inf

    out = []
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < min_area_px:
            continue
        blob = (lab == i).astype(np.uint8)
        cnts, _ = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        c_px = max(cnts, key=cv2.contourArea).reshape(-1, 2).astype(float)
        if len(c_px) < 12:
            continue
        hull_a = cv2.contourArea(cv2.convexHull(c_px.astype(np.float32)))
        if hull_a <= 0 or area / hull_a < min_solidity:
            continue                    # panel slivers and shadows are not convex
        c_mm = W.px_to_mm(c_px, "floor")
        C, rho, k, rms, r_enc = _fit_cylinder(c_mm, G)
        if not (min_radius_mm <= rho <= max_radius_mm):
            continue
        out.append(dict(centre_mm=C, radius_mm=float(rho),
                        radius_mm_bound=float(r_enc), k=float(k),
                        height_mm=float(Hc * (1.0 - 1.0 / k))
                        if np.isfinite(Hc) else np.nan,
                        fit_rms_mm=rms, area_px=area,
                        centroid_px=cent[i], contour_px=c_px))
    out.sort(key=lambda d: -d["area_px"])
    return dict(cylinders=out, dark=dark, free=free, known=known, deck_mm=deck)


# ----------------------------------------------------------------------- io
def build_map(img_bgr, res, wall_kw=None, cyl_kw=None, post_kw=None):
    """Everything a planner needs, in world mm."""
    walls = detect_walls(img_bgr, res, **(wall_kw or {}))
    posts = detect_posts(img_bgr, res, **(post_kw or {}))
    # Posts first: the cylinder pass needs them masked off, or a post reads as a
    # small obstacle.  Then prune the other way round, now that the deck and the
    # cylinders are known.
    obs = detect_cylinders(img_bgr, res, walls, posts, **(cyl_kw or {}))
    raw = posts
    posts = prune_posts(posts, obs["deck_mm"], obs["cylinders"])
    cells = cell_walls(walls)
    # Two post sets, because the two consumers want opposite errors.  A planner
    # wants ``post_centres_mm``: the pruned set drops claims another obstacle
    # already covers, and a phantom obstacle there would only cost it a route.
    # A lidar reference map wants to weigh that against the other side -- an
    # obstacle the beams cannot actually see is a reading the observer will not
    # be able to explain, and so is one it should have seen and did not.
    return dict(frame=WorldFrame(res), walls=walls, posts=posts, cells=cells,
                graph=cell_graph(cells),
                wall_segments_mm=wall_segments_mm(walls),
                post_centres_mm=post_centres_mm(posts),
                post_centres_all_mm=post_centres_mm(raw),
                isolated_posts=isolated_posts(posts, walls),
                post_radius_mm=POST_R_MM,
                cylinders=obs["cylinders"], deck_mm=obs["deck_mm"], debug=obs)
