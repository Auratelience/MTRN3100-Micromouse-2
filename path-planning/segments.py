#!/usr/bin/env -S uv run --script
"""The firmware's path alphabet, in Python.

``Segment`` mirrors ``firmware/micromouse/types.h``: a start point, an end
point, a curvature (1/mm) and a turn direction.  Straights carry curvature 0.
Arcs are circles, and the firmware does *not* store the centre -- it rebuilds it
from the chord in ``centrePreCalcRadiusAndMidpoint``.  Two consequences drive
this module:

Minor arcs only
    That reconstruction places the centre at the chord midpoint plus
    ``sqrt(r^2 - (L/2)^2)`` along the chord normal, which is the centre of the
    *minor* arc.  Hand it a 270 deg arc and it silently rebuilds the wrong
    circle -- the robot then drives a different curve than the planner checked
    for collisions.  ``from_dubins`` therefore splits every turn into pieces of
    at most ``max_sweep`` (default 90 deg, which also keeps the chord well away
    from the ``r^2 - (L/2)^2 -> 0`` conditioning cliff at 180 deg).

Curvature threshold
    ``STRAIGHT_TOLERANCE`` is 1e-3, so anything with radius above 1000 mm reads
    as a straight line on the robot.  ``check`` flags that rather than letting a
    nearly-straight arc quietly lose its centre.

Frames
    Segments live in whatever frame they were built in.  The maze map is image
    convention (+x east, +y south), which is left-handed -- a CCW turn there is
    a physical clockwise turn.  ``to_firmware`` mirrors into the robot's
    right-handed frame (x forward, y left), flipping every Left <-> Right, and
    re-origins onto the start pose so the emitted path can be pasted against a
    fresh odometry frame.
"""

from dataclasses import dataclass, field

import numpy as np

import dubins as db

STRAIGHT_TOLERANCE = 1e-3  # firmware constants.h
MAX_ARC_RADIUS_MM = 1.0 / STRAIGHT_TOLERANCE

LEFT = "Left"
RIGHT = "Right"


@dataclass
class Segment:
    """One straight or one circular arc, in mm.

    ``direction`` is the sense of the turn in this segment's own frame, with
    ``Left`` = CCW, matching ``Segment::Direction`` on the robot.  It is
    meaningless for a straight and is kept as ``Left`` there, as the firmware's
    default constructor does.
    """

    start: np.ndarray
    end: np.ndarray
    curvature: float = 0.0
    direction: str = LEFT
    centre: np.ndarray | None = field(default=None, repr=False)

    def __post_init__(self):
        self.start = np.asarray(self.start, float).reshape(2)
        self.end = np.asarray(self.end, float).reshape(2)
        self.curvature = float(self.curvature)
        if self.centre is not None:
            self.centre = np.asarray(self.centre, float).reshape(2)

    # -- geometry ----------------------------------------------------------
    @property
    def is_arc(self):
        return self.curvature > STRAIGHT_TOLERANCE

    @property
    def radius(self):
        return np.inf if self.curvature <= 0.0 else 1.0 / self.curvature

    @property
    def sign(self):
        """+1 for a CCW (Left) arc, -1 for CW."""
        return 1.0 if self.direction == LEFT else -1.0

    def firmware_centre(self):
        """The centre the robot will reconstruct -- a straight port of
        ``centrePreCalcRadiusAndMidpoint``.  ``check`` compares it against the
        centre we intended; a mismatch means the arc is not representable."""
        m = 0.5 * (self.start + self.end)
        if not self.is_arc:
            return m
        chord = self.end - self.start
        L = float(np.linalg.norm(chord))
        disc = max(self.radius**2 - (0.5 * L) ** 2, 0.0)
        h = np.sqrt(disc)
        scale = self.sign * h / L if L > 1e-12 else 0.0
        return m + scale * np.array([-chord[1], chord[0]])

    @property
    def sweep(self):
        """Swept angle in radians, >= 0.  Zero for a straight."""
        if not self.is_arc:
            return 0.0
        c = self.centre if self.centre is not None else self.firmware_centre()
        a0 = np.arctan2(*(self.start - c)[::-1])
        a1 = np.arctan2(*(self.end - c)[::-1])
        return float(db.mod2pi(self.sign * (a1 - a0)))

    @property
    def length(self):
        if not self.is_arc:
            return float(np.linalg.norm(self.end - self.start))
        return self.sweep * self.radius

    @property
    def start_theta(self):
        return self._theta(self.start)

    @property
    def end_theta(self):
        return self._theta(self.end)

    def _theta(self, p):
        if not self.is_arc:
            d = self.end - self.start
            return float(np.arctan2(d[1], d[0]))
        c = self.centre if self.centre is not None else self.firmware_centre()
        r = p - c
        return float(np.arctan2(r[1], r[0]) + self.sign * np.pi / 2.0)

    def sample(self, ds=5.0):
        """Points along the segment at spacing <= ``ds``, endpoints included."""
        return self.sample_poses(ds)[:, :2]

    def sample_poses(self, ds=5.0):
        """``(x, y, theta)`` along the segment at spacing <= ``ds``.

        The heading is what tells a collision check where an off-centre body
        sits, so a bare point is not enough to place the robot.
        """
        n = max(1, int(np.ceil(self.length / ds)))
        if not self.is_arc:
            t = np.arange(n + 1)[:, None] / n
            P = self.start + t * (self.end - self.start)
            return np.column_stack([P, np.full(n + 1, self.start_theta)])
        c = self.centre if self.centre is not None else self.firmware_centre()
        a0 = np.arctan2(*(self.start - c)[::-1])
        a = a0 + self.sign * self.sweep * np.arange(n + 1) / n
        P = c + self.radius * np.stack([np.cos(a), np.sin(a)], 1)
        # the same quarter-turn lead _theta applies, evaluated all at once
        return np.column_stack([P, a + self.sign * np.pi / 2.0])


# --------------------------------------------------------------- construction
def from_dubins(q0, word, params, rho, max_sweep=np.pi / 2.0, min_len=1e-6):
    """Expand a Dubins word into firmware segments, splitting long turns."""
    segs = []
    for mode, qa, qb, value, centre in db.primitives(q0, word, params, rho):
        if value <= min_len:
            continue
        if mode == "S":
            segs.append(Segment(qa[:2], qb[:2]))
            continue
        n = max(1, int(np.ceil(value / max_sweep)))
        s = 1.0 if mode == "L" else -1.0
        a0 = np.arctan2(qa[1] - centre[1], qa[0] - centre[0])
        a = a0 + s * value * np.arange(n + 1) / n
        pts = centre + rho * np.stack([np.cos(a), np.sin(a)], 1)
        for k in range(n):
            segs.append(
                Segment(
                    pts[k],
                    pts[k + 1],
                    1.0 / rho,
                    LEFT if s > 0 else RIGHT,
                    centre=centre,
                )
            )
    return segs


def from_poses(poses, rho, max_sweep=np.pi / 2.0):
    """Chain Dubins paths through a pose sequence -> one segment list."""
    out = []
    for a, b in zip(poses[:-1], poses[1:]):
        L, word, params = db.shortest(a, b, rho)
        if word is None:
            raise ValueError(f"no Dubins path {a} -> {b}")
        out += from_dubins(a, word, params, rho, max_sweep)
    return out


# ------------------------------------------------------------------ transform
def _map(segs, f, mirror, radius_scale=1.0):
    """Apply a point map to every segment.  ``radius_scale`` is for the one
    transform that is not an isometry: curvature is 1/mm, so a resize divides
    it by the same factor the points were multiplied by."""
    out = []
    for s in segs:
        d = s.direction
        if mirror and s.is_arc:
            d = RIGHT if s.direction == LEFT else LEFT
        c = None if s.centre is None else f(s.centre[None])[0]
        k = s.curvature / radius_scale
        out.append(Segment(f(s.start[None])[0], f(s.end[None])[0], k, d, c))
    return out


def mirror_y(segs):
    """Flip the y axis, swapping every turn direction with it."""
    return _map(segs, lambda P: P * np.array([1.0, -1.0]), True)


def rigid(segs, theta=0.0, translate=(0.0, 0.0)):
    """Rotate by ``theta`` then translate.  Turn directions are preserved."""
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    t = np.asarray(translate, float)
    return _map(segs, lambda P: P @ R.T + t, False)


def scale(segs, k):
    """Uniform resize about the origin by a positive factor.

    Radii grow with the geometry, so curvature goes the other way -- leave it
    alone and every arc's chord stops matching its stated radius, which is
    exactly the "firmware rebuilds the centre N mm off" that ``check`` reports.
    A positive factor preserves orientation, so turn directions stand.

    Nothing else in the pipeline scales with this: the map, the clearance check
    and the overlay all describe the maze as photographed.  Scale the path and
    it is no longer the path that was checked against them.
    """
    k = float(k)
    if not k > 0.0:
        raise ValueError(f"scale factor must be positive, got {k}")
    return _map(segs, lambda P: P * k, False, radius_scale=k)


def to_firmware(segs, start_pose=None, local=True):
    """Map-frame segments -> robot-frame segments.

    The maze map is left-handed (+y south), the robot is right-handed (x
    forward, y left), so the mirror is not optional -- without it every turn
    comes out reversed.  With ``local`` the path is then re-origined so it
    starts at pose (0, 0, 0), which is what a freshly reset odometry frame
    reads.  Returns ``(segments, start_pose_in_output_frame)``.
    """
    out = mirror_y(segs)
    if start_pose is None:
        q = np.array([out[0].start[0], out[0].start[1], out[0].start_theta])
    else:
        q = np.asarray(start_pose, float).copy()
        q[1] = -q[1]
        q[2] = -q[2]
    if not local:
        return out, q
    out = rigid(out, -q[2], (0.0, 0.0))
    c, s = np.cos(-q[2]), np.sin(-q[2])
    origin = np.array([c * q[0] - s * q[1], s * q[0] + c * q[1]])
    return rigid(out, 0.0, -origin), np.array([0.0, 0.0, 0.0])


# -------------------------------------------------------------------- tidying
def merge(segs, angle_tol=1e-6, zero_len=1e-6):
    """Join what the robot would drive as one move.  Geometry preserving.

    RRT* hands over a chain of Dubins words, so the joins between edges are
    full of zero-length primitives and collinear straight pairs.  Each surviving
    segment costs the firmware a ``progress()`` evaluation per control tick and
    a slot in a 256-entry array, so they are worth removing here rather than on
    the robot.

    Only exact merges happen: collinear straights, and same-direction arcs that
    share a centre and still fit under 180 deg combined.  Nothing moves.
    """
    out = []
    for s in segs:
        if s.length <= zero_len:
            continue
        if not out:
            out.append(s)
            continue
        p = out[-1]
        if not p.is_arc and not s.is_arc:
            u, v = p.end - p.start, s.end - s.start
            nu, nv = np.linalg.norm(u), np.linalg.norm(v)
            u, v = u / nu, v / nv
            straight = abs(float(u[0] * v[1] - u[1] * v[0])) < angle_tol
            if straight and np.linalg.norm(s.start - p.end) <= zero_len:
                out[-1] = Segment(p.start, s.end)
                continue
        elif (
            p.is_arc
            and s.is_arc
            and p.direction == s.direction
            and abs(p.curvature - s.curvature) < 1e-9
            and p.centre is not None
            and s.centre is not None
            and np.linalg.norm(p.centre - s.centre) < 1e-6
            and p.sweep + s.sweep <= np.pi - 1e-6
        ):
            out[-1] = Segment(p.start, s.end, p.curvature, p.direction, p.centre)
            continue
        out.append(s)
    return out


def drop_slivers(segs, min_len=0.5, max_kink=0.02):
    """Remove segments too short to drive, snapping the gap closed.

    This one *moves the path*: the following segment's start is pulled back onto
    the previous segment's end, and an arc's centre is rebuilt from its new
    chord so it stays a true circle of the same radius.  Re-run the clearance
    check afterwards -- the demo does.

    Worth doing because the firmware advances a segment at 0.995 progress: a
    sub-millimetre segment can be entered and satisfied inside one control tick,
    and a chain of them wastes the 256-slot path array.  Dubins joins produce
    them constantly, since a word whose middle primitive is nearly unused still
    contributes all three.

    Both bounds matter.  Dropping an arc leaves a heading step equal to its
    sweep, so a 2 mm arc at r = 30 mm -- trivially short -- would kink the path
    by 3.8 deg; ``max_kink`` caps that at a fifth of the firmware's own
    STD_ANG_TOL.  ``min_len`` caps how far the snap can move the path, and with
    it how much the next segment can rotate.
    """
    segs = list(segs)
    out = []
    for k, s in enumerate(segs):
        droppable = s.length < min_len and (not s.is_arc or s.sweep < max_kink)
        # Dropping the first or last segment shifts the path's endpoint by up to
        # min_len, which is well inside the firmware's own STD_DIST_TOL of 2 mm,
        # so they are fair game too -- a 0.01 mm leading arc is noise, not a
        # commitment to a start pose.
        if droppable and len(segs) > 1:
            continue
        if out and np.linalg.norm(s.start - out[-1].end) > 1e-9:
            s = _resnap(s, out[-1].end)
        out.append(s)
    return out


def _resnap(s, new_start):
    """Move a segment's start to ``new_start``, keeping its end, radius and
    turn direction; an arc's centre is rebuilt from the new chord."""
    if not s.is_arc:
        return Segment(new_start, s.end)
    moved = Segment(new_start, s.end, s.curvature, s.direction)
    return Segment(new_start, s.end, s.curvature, s.direction, moved.firmware_centre())


def clean(segs, min_len=0.5, max_kink=0.02):
    """``merge`` then ``drop_slivers`` then ``merge`` again."""
    return merge(drop_slivers(merge(segs), min_len, max_kink))


def polyline(segs, ds=5.0):
    """Whole path as points, for drawing."""
    return pose_polyline(segs, ds)[:, :2]


def pose_polyline(segs, ds=5.0):
    """Whole path as ``(x, y, theta)``, for clearance checks -- an off-centre
    body needs the heading to be placed at all."""
    if not segs:
        return np.zeros((0, 3))
    Q = [segs[0].sample_poses(ds)]
    for s in segs[1:]:
        Q.append(s.sample_poses(ds)[1:])
    return np.concatenate(Q, 0)


def length(segs):
    return float(sum(s.length for s in segs))


# ------------------------------------------------------------------ validation
def check(segs, gap_tol=1e-3, heading_tol=0.05, centre_tol=0.5):
    """Everything that would make the robot drive a different curve.

    Returns a list of human-readable problems; empty means the firmware will
    reconstruct this geometry, and drive it without a discontinuity it would
    notice.  ``heading_tol`` defaults to the firmware's own STD_ANG_TOL: a join
    inside that is smaller than the angle the controller settles for anyway.
    """
    bad = []
    for k, s in enumerate(segs):
        if s.length <= 1e-9:
            bad.append(f"[{k}] zero length")
        # A curvature under the tolerance is not a gentle arc on the robot, it
        # is a straight: the firmware drives the chord and loses the bulge.
        # Tested outside ``is_arc`` because that property is this same
        # threshold -- inside it, radius > MAX_ARC_RADIUS_MM cannot ever hold.
        if 0.0 < s.curvature <= STRAIGHT_TOLERANCE:
            bad.append(
                f"[{k}] radius {s.radius:.0f} mm -> curvature {s.curvature:.2e} "
                f"reads as a straight line on the robot"
            )
        if s.is_arc:
            if s.sweep > np.pi + 1e-6:
                bad.append(f"[{k}] sweep {np.degrees(s.sweep):.0f} deg exceeds 180")
            if s.centre is not None:
                e = float(np.linalg.norm(s.firmware_centre() - s.centre))
                if e > centre_tol:
                    bad.append(f"[{k}] firmware rebuilds the centre {e:.2f} mm off")
            chord = float(np.linalg.norm(s.end - s.start))
            if chord > 2.0 * s.radius + 1e-6:
                bad.append(f"[{k}] chord {chord:.1f} mm exceeds the diameter")
        if k:
            p = segs[k - 1]
            gap = float(np.linalg.norm(s.start - p.end))
            if gap > gap_tol:
                bad.append(f"[{k}] {gap:.3f} mm gap from the previous segment")
            dth = abs(db.wrap_pi(s.start_theta - p.end_theta))
            if dth > heading_tol:
                bad.append(f"[{k}] {np.degrees(dth):.2f} deg heading step at the join")
    return bad


# ------------------------------------------------------------------- emitters
def to_cpp(segs, var="planner", note=""):
    """``planner.appendSegment(...)`` lines, ready to paste into the sketch."""
    L = length(segs)
    lines = [
        f"// {len(segs)} segments, {L:.0f} mm{(' -- ' + note) if note else ''}",
        "// Robot frame: x forward, y left, mm; path starts at the robot's pose.",
    ]
    for s in segs:
        a = f"{{{s.start[0]:.2f}f, {s.start[1]:.2f}f}}"
        b = f"{{{s.end[0]:.2f}f, {s.end[1]:.2f}f}}"
        if s.is_arc:
            lines.append(
                f"{var}.appendSegment(Segment({a}, {b}, 1.0f / {s.radius:.2f}f, "
                f"Segment::Direction::{s.direction}));"
            )
        else:
            lines.append(f"{var}.appendSegment(Segment({a}, {b}));")
    return "\n".join(lines)


def to_dicts(segs):
    """JSON-ready records, for a serial uploader or the sim."""
    return [
        dict(
            start=[float(s.start[0]), float(s.start[1])],
            end=[float(s.end[0]), float(s.end[1])],
            curvature=float(s.curvature),
            direction=s.direction if s.is_arc else None,
            length=float(s.length),
        )
        for s in segs
    ]
