"""Mirrors micromouse/types.h.

Method and field names follow the header rather than PEP 8 so the two can be
compared by eye.

Deliberately not ported:

* ``trig<NV>`` -- the LUT-based sin/cos/atan2/acos tables are a microcontroller
  speed optimisation. The sim uses ``math`` directly; the LUTs' interpolation
  error is not behaviour worth reproducing.
* ``RingBuffer`` -- nothing in the simulated path uses it.

``Map<S>`` is ported without its compile-time size: the header makes it a
template so the obstacle array lives in flash, which is a storage concern with
no meaning here. ``Map.candidates()`` takes its cap as an argument instead of
writing into a caller-supplied ``std::array``.
"""

import math
from dataclasses import dataclass
from enum import Enum

from .constants import STD_TOL, STRAIGHT_TOLERANCE, TWO_PI


@dataclass
class WheelVelocities:
    left: float
    right: float


@dataclass
class Pose:
    x: float = 0.0  # mm
    y: float = 0.0  # mm
    theta: float = 0.0  # rad


@dataclass
class Velocity:
    v: float = 0.0
    omega: float = 0.0


class Vec2D:
    __slots__ = ("x", "y")

    def __init__(self, x=0.0, y=0.0):
        self.x = float(x)
        self.y = float(y)

    def __add__(self, other):
        return Vec2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec2D(self.x - other.x, self.y - other.y)

    def __neg__(self):
        return Vec2D(-self.x, -self.y)

    def __mul__(self, scalar):
        return Vec2D(scalar * self.x, scalar * self.y)

    __rmul__ = __mul__

    def __eq__(self, other):
        return isinstance(other, Vec2D) and self.x == other.x and self.y == other.y

    def __repr__(self):
        return f"Vec2D({self.x}, {self.y})"


def dist(v):
    return math.hypot(v.x, v.y)


def distSq(v):
    return v.x * v.x + v.y * v.y


def arg(v):
    return math.atan2(v.y, v.x)


def dot(a, b):
    return a.x * b.x + a.y * b.y


def cross(a, b):
    return a.x * b.y - a.y * b.x


def perp(v):
    """Rotated a quarter turn CCW, which is also d/dtheta of a vector rigidly
    rotating by theta. The lidar observer's heading Jacobian is built from it."""
    return Vec2D(-v.y, v.x)


def angleBetween(a, b):
    return math.atan2(cross(a, b), dot(a, b))


def wrapAngle(a):
    """Wrap to [-PI, PI]. Inf or NaN returns 0 rather than poisoning downstream
    control maths."""
    if not math.isfinite(a):
        return 0.0
    a = math.fmod(a, TWO_PI)
    if a > math.pi:
        a -= TWO_PI
    elif a < -math.pi:
        a += TWO_PI
    return a


def withinTolerance(a, b, tol=0.0):
    return distSq(a - b) <= tol * tol


class Segment:
    """Line and arc path primitive. Port of Segment in types.h."""

    class Direction(Enum):
        Left = "Left"
        Right = "Right"

    def __init__(self, start=None, end=None, curvature=0.0, direction=None):
        self.start = start if start is not None else Vec2D(0, 0)
        self.end = end if end is not None else Vec2D(0, 0)
        self.curvature = curvature
        self.direction = direction if direction is not None else Segment.Direction.Left
        # C++ initialises `Vec2D c = centre()` as a default member initialiser,
        # i.e. once at construction. Same here.
        self.c = self._centre()

    # --- geometry -------------------------------------------------------

    def lateralDistance(self, pos):
        if self.curvature <= STRAIGHT_TOLERANCE:
            return self._lateralDistanceForStraightLine(pos)
        r = 1.0 / self.curvature
        return dist(pos - self.c) - r

    def lateralPoint(self, pos):
        if self.curvature <= STRAIGHT_TOLERANCE:
            return self._lateralPointForStraightLine(pos)
        r = 1.0 / self.curvature
        pc = pos - self.c
        return self.c + (r / dist(pc)) * pc

    def progress(self, pos):
        """Progress along the segment in [0, 1+], where >=1 means the nearest
        point has passed the segment's end."""
        if self.curvature <= STRAIGHT_TOLERANCE:
            return self._lineProgress(pos)
        return self._arcProgress(pos)

    def remainingDistance(self, pos):
        """Distance from the projected position to the segment end, measured
        along the path (arc length for arcs). Zero once the end is passed."""
        p = self.progress(pos)
        if p >= 1.0:
            return 0.0

        if self.curvature <= STRAIGHT_TOLERANCE:
            return (1.0 - p) * dist(self.end - self.start)

        c = self.c
        startAngle = arg(self.start - c)
        endAngle = arg(self.end - c)
        directionScalar = 1.0 if self.direction == Segment.Direction.Left else -1.0

        totalSweep = endAngle - startAngle
        while directionScalar * totalSweep < 0:
            totalSweep += directionScalar * TWO_PI

        r = 1.0 / self.curvature
        return (1.0 - p) * abs(totalSweep) * r

    # --- internals ------------------------------------------------------

    def _centre(self):
        m = 0.5 * (self.start + self.end)
        if self.curvature <= STRAIGHT_TOLERANCE:
            return m
        return self._centrePreCalcRadiusAndMidpoint(1.0 / self.curvature, m)

    def _centrePreCalcRadiusAndMidpoint(self, r, m):
        chordVector = self.end - self.start
        chordLength = dist(chordVector)
        if chordLength == 0.0:
            return m
        halfChordLength = 0.5 * chordLength
        directionScalar = 1.0 if self.direction == Segment.Direction.Left else -1.0
        # Guard an infeasible radius (r < half the chord): the discriminant
        # would go negative and sqrt would return NaN, poisoning the centre.
        # Clamp to 0 so the arc degenerates to its midpoint instead.
        discriminant = (r * r) - (halfChordLength * halfChordLength)
        if discriminant < 0.0:
            discriminant = 0.0
        h = math.sqrt(discriminant)
        scale = (directionScalar * h) / chordLength
        chordVector_L90 = Vec2D(-chordVector.y, chordVector.x)
        return m + (scale * chordVector_L90)

    def _arcProgress(self, pos):
        """Fraction of the arc travelled, measured as angular progress from
        start toward end in the turn direction (Left = CCW, Right = CW).

        A pe/se ratio via angleBetween() would wrap to (-PI, PI]; a point just
        *behind* the arc start (e.g. when the previous straight hands over a
        few mm early) would then read as ">1" ("past the end"), so the
        planner's advance loop would skip the whole arc. Here a behind-start
        point reads slightly negative instead.
        """
        startAngle = arg(self.start - self.c)
        sweep = self._arcTravel(startAngle, arg(self.end - self.c))
        if sweep <= 0.0:
            return 1.0
        travelled = self._arcTravel(startAngle, arg(pos - self.c))
        # Points on the unused side of the circle wrap toward TWOPI; treat
        # those as "not started" (negative) rather than "past the end".
        if travelled > 0.5 * (sweep + TWO_PI):
            travelled -= TWO_PI
        return travelled / sweep

    def _arcTravel(self, frm, to):
        """Positive angular distance from `frm` to `to` along the turn
        direction (Left = CCW / increasing, Right = CW / decreasing),
        in [0, TWOPI)."""
        delta = (to - frm) if self.direction == Segment.Direction.Left else (frm - to)
        while delta < 0.0:
            delta += TWO_PI
        while delta >= TWO_PI:
            delta -= TWO_PI
        return delta

    def _lineProgress(self, pos):
        line = self.end - self.start
        posFromStart = pos - self.start
        return dot(posFromStart, line) / distSq(line)

    def _clampedT(self, pos):
        line = self.end - self.start
        posFromStart = pos - self.start
        t = dot(posFromStart, line) / distSq(line)
        return 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)

    def _lateralDistanceForStraightLine(self, pos):
        line = self.end - self.start
        t = self._clampedT(pos)
        return dist(pos - (self.start + (t * line)))

    def _lateralPointForStraightLine(self, pos):
        line = self.end - self.start
        t = self._clampedT(pos)
        return self.start + t * line

    def __repr__(self):
        kind = "line" if self.curvature <= STRAIGHT_TOLERANCE else "arc"
        return (f"Segment({kind} ({self.start.x:.1f},{self.start.y:.1f})"
                f"->({self.end.x:.1f},{self.end.y:.1f}))")


# OBSTACLES AND MAPS
#
# The world LidarObserver measures against, mirroring the same block in
# types.h. A beam leaves a sensor at s travelling along the unit vector b, and
# the observer wants two things from whatever it lands on: the range the sensor
# ought to report, and the surface normal there, which is what turns the
# difference between expected and measured into a pose correction.
#
# The firmware's d()/phi()/d_0() relations from notes/Lidar Maths are ported
# too, even though only cast() is on the observer's hot path: selftest-style
# checks compare a cast against the closed forms, and impliedHeadingError()
# needs phi(). See types.h for the derivations.


def incidenceAngle(outward, beam):
    """Angle between a beam and the inward normal of the surface it hit.

    Through atan2 rather than acos of the dot product, matching the header: an
    almost square-on hit puts that dot product just under 1, where acos is
    ill-conditioned. Square-on is the common case for a maze.
    """
    inward = -outward
    return math.atan2(abs(cross(inward, beam)), dot(inward, beam))


@dataclass
class RayHit:
    """What a beam meets."""

    # Range from the sensor to the surface, mm.
    distance: float = 0.0
    # Unit surface normal at the hit, pointing back towards the sensor.
    normal: Vec2D = None
    # The note's alpha with the beam taken as the reference direction: the
    # angle between the beam and the inward normal. 0 is square on, PI/2 is
    # grazing.
    incidence: float = 0.0
    # Index of the obstacle within its Map, or -1 for a miss.
    index: int = -1
    valid: bool = False

    def __post_init__(self):
        if self.normal is None:
            self.normal = Vec2D(0.0, 0.0)


class CircularObstacle:
    """A post, or a free-standing cylinder."""

    __slots__ = ("radius",)

    def __init__(self, radius):
        self.radius = float(radius)

    def d(self, d_0, phi):
        """Range to the near surface. -1 when the beam misses, or the obstacle
        is behind the sensor."""
        s = math.sin(phi)
        if d_0 <= self.radius or abs(s) * d_0 > self.radius or math.cos(phi) <= 0.0:
            return -1.0
        return d_0 * math.cos(phi) - math.sqrt(
            self.radius * self.radius - d_0 * d_0 * s * s
        )

    def dFar(self, d_0, phi):
        s = math.sin(phi)
        if abs(s) * d_0 > self.radius:
            return -1.0
        return d_0 * math.cos(phi) + math.sqrt(
            self.radius * self.radius - d_0 * d_0 * s * s
        )

    def phi(self, d_0, d):
        """Beam angle implied by a measurement, from the law of cosines.
        Unsigned. -1 when d cannot have come from this obstacle at this d_0."""
        if d <= 0.0 or abs(d_0 - self.radius) > d or d > d_0:
            return -1.0
        c = (d_0 * d_0 + d * d - self.radius * self.radius) / (2.0 * d_0 * d)
        return math.acos(1.0 if c > 1.0 else (-1.0 if c < -1.0 else c))

    def d_0(self, d, phi):
        """Distance to the centre implied by a measurement."""
        s = math.sin(phi)
        if d <= 0.0 or abs(s) * d > self.radius:
            return -1.0
        return d * math.cos(phi) + math.sqrt(
            self.radius * self.radius - d * d * s * s
        )

    def boundingRadius(self):
        return self.radius

    def cast(self, to_centre, beam):
        """to_centre is the centre relative to the sensor; beam is a unit
        vector."""
        hit = RayHit()
        centre_range = dist(to_centre)
        if centre_range <= self.radius:
            return hit  # sensor inside the obstacle

        rng = self.d(centre_range, angleBetween(beam, to_centre))
        if rng < 0.0:
            return hit

        hit.distance = rng
        hit.normal = (1.0 / self.radius) * ((rng * beam) - to_centre)
        hit.incidence = incidenceAngle(hit.normal, beam)
        hit.valid = True
        return hit

    def __repr__(self):
        return f"CircularObstacle({self.radius:.2f})"


class WallObstacle:
    """One panel: a rectangle of `length` by `thickness`, centred on its
    lattice bond. `alpha` is the panel's heading in map coordinates."""

    __slots__ = ("length", "thickness", "alpha")

    def __init__(self, length, thickness, alpha):
        self.length = float(length)
        self.thickness = float(thickness)
        self.alpha = float(alpha)

    def d(self, d_0, phi, alpha_rel):
        """Expected range. -1 when the beam runs along the face or away from
        it. `alpha_rel` is the note's alpha, not the member."""
        c = math.cos(phi + alpha_rel)
        if c <= STD_TOL:
            return -1.0
        return d_0 * math.cos(alpha_rel) / c

    def phi(self, d_0, d, alpha_rel):
        """Beam angle implied by a measurement. -3 PI when d is short of the
        face."""
        perpendicular = d_0 * math.cos(alpha_rel)
        if d <= 0.0 or perpendicular > d:
            return -3.0 * math.pi
        return math.acos(min(1.0, perpendicular / d)) - alpha_rel

    def d_0(self, d, phi, alpha_rel):
        c = math.cos(alpha_rel)
        if abs(c) <= STD_TOL:
            return -1.0
        return d * math.cos(phi + alpha_rel) / c

    def boundingRadius(self):
        return 0.5 * math.hypot(self.length, self.thickness)

    def cast(self, to_centre, beam):
        """Beam against the face the sensor is on. The two ends are left out:
        every bond terminates on a post, whose own circle covers that corner
        more accurately than a square end cap would."""
        hit = RayHit()

        along = Vec2D(math.cos(self.alpha), math.sin(self.alpha))
        normal = perp(along)

        # Signed distance from the sensor to the panel's centre plane. Its sign
        # picks which of the two faces is the one facing the sensor.
        centre_offset = dot(normal, to_centre)
        outward = -normal if centre_offset >= 0.0 else normal

        perpendicular = abs(centre_offset) - 0.5 * self.thickness
        if perpendicular <= 0.0:
            return hit  # sensor inside the panel

        cos_alpha = -dot(outward, beam)
        if cos_alpha <= STD_TOL:
            return hit  # parallel to the face, or behind it

        rng = perpendicular / cos_alpha
        if abs(dot(along, (rng * beam) - to_centre)) > 0.5 * self.length:
            return hit

        hit.distance = rng
        hit.normal = outward
        hit.incidence = incidenceAngle(outward, beam)
        hit.valid = True
        return hit

    def __repr__(self):
        return (f"WallObstacle({self.length:.1f}, {self.thickness:.1f}, "
                f"{self.alpha:.4f})")


class Obstacle:
    """A form (CircularObstacle or WallObstacle) placed at a map centre."""

    __slots__ = ("form", "centre")

    def __init__(self, form, centre):
        self.form = form
        self.centre = centre

    def boundingRadius(self):
        return self.form.boundingRadius()

    def cast(self, origin, beam):
        return self.form.cast(self.centre - origin, beam)

    def impliedHeadingError(self, prior, origin, beam, measured):
        """How far the beam would have to swing for this obstacle to return
        `measured`, given that it returns `prior.distance` as aimed.

        A diagnostic, not a gate -- square on to a wall the range hardly
        responds to heading at all, so a few mm of position error comes back as
        an implausible half a radian. -1 when the reading is impossible.
        """
        if not prior.valid or measured <= 0.0:
            return -1.0

        if isinstance(self.form, CircularObstacle):
            to_centre = self.centre - origin
            centre_range = dist(to_centre)
            implied = self.form.phi(centre_range, measured)
            if implied < 0.0:
                return -1.0
            # phi() is unsigned, so compare magnitudes: both branches are the
            # same swing, mirrored about the centre line.
            return abs(implied - abs(angleBetween(beam, to_centre)))

        implied = self.form.phi(prior.distance, measured, prior.incidence)
        if implied < -math.pi:
            return -1.0
        return abs(implied)

    def __repr__(self):
        return f"Obstacle({self.form!r}, {self.centre!r})"


class Map:
    """A fixed set of obstacles in map coordinates.

    Built offline: path-planning/export_map.py runs the vision pipeline over a
    photo of the maze and emits a constexpr Map, which the firmware keeps in
    flash. maze_header.load_map() parses that same header back into this.
    """

    __slots__ = ("obstacles",)

    def __init__(self, obstacles=()):
        self.obstacles = list(obstacles)

    def size(self):
        return len(self.obstacles)

    def __len__(self):
        return len(self.obstacles)

    def __getitem__(self, i):
        return self.obstacles[i]

    def cast(self, origin, beam, max_range, indices=None):
        """Nearest surface along `beam` from `origin`, out to `max_range`.

        `indices` restricts the search to a subset -- see candidates(). None
        searches everything, which is correct but walks the whole map.
        """
        best = RayHit()
        best.distance = max_range
        order = range(len(self.obstacles)) if indices is None else indices

        for i in order:
            hit = self.obstacles[i].cast(origin, beam)
            if not hit.valid or hit.distance >= best.distance:
                continue
            hit.index = i
            best = hit
        return best

    def candidates(self, centre, radius, limit):
        """Indices of every obstacle whose bounding circle reaches within
        `radius` of `centre`, capped at `limit`.

        The C++ signature fills a caller-supplied std::array and returns the
        count; here the cap is passed in and the list returned. A full list may
        have been truncated, which the caller must treat as "fall back to a
        full-map cast" -- exactly as the header's overflow flag does.
        """
        out = []
        for i, o in enumerate(self.obstacles):
            if len(out) >= limit:
                break
            reach = radius + o.boundingRadius()
            if distSq(o.centre - centre) > reach * reach:
                continue
            out.append(i)
        return out

    def bounds(self):
        """(min_x, min_y, max_x, max_y) over every obstacle's bounding circle.
        Sim-only: the firmware never needs the extent of its own map, but the
        world builder and the viewer do."""
        if not self.obstacles:
            return (0.0, 0.0, 0.0, 0.0)
        xs0, ys0, xs1, ys1 = [], [], [], []
        for o in self.obstacles:
            r = o.boundingRadius()
            xs0.append(o.centre.x - r)
            ys0.append(o.centre.y - r)
            xs1.append(o.centre.x + r)
            ys1.append(o.centre.y + r)
        return (min(xs0), min(ys0), max(xs1), max(ys1))

    def __repr__(self):
        return f"Map({len(self.obstacles)} obstacles)"
