"""Reads the two generated headers back into the sim.

``path-planning/build_maze.sh`` writes ``maze_map.h`` (a ``constexpr Map<N>``)
and ``maze_path.h`` (bare ``planner.appendSegment(...)`` statements) and installs
both into ``firmware/micromouse/``. Parsing those files, rather than re-running
the vision pipeline, is what lets the sim run *exactly* what is on the robot:
same obstacles, same segments, same frame, and no dependency on OpenCV or the
path-planning virtualenv.

Both are machine-generated with a fixed shape, so these are regular expressions
over a known emitter (``export_map.py:header`` and ``segments.to_cpp``) rather
than a C++ parser. A hand-edited header is not supported -- the emitters both
say so at the top of what they write.

The headers only mean anything together: they share the ``--from``/``--theta0``
the wrapper passed to both, so the origin of each is the robot's start pose and
a freshly reset odometry frame reads (0, 0, 0). Loading a map from one run and a
path from another puts the robot in a different frame from its own map.
"""

import pathlib
import re

from .types import (
    CircularObstacle,
    Map,
    Obstacle,
    Segment,
    Vec2D,
    WallObstacle,
)

# Default install location, i.e. what build_maze.sh writes and what the sketch
# includes.
FIRMWARE_DIR = pathlib.Path(__file__).resolve().parent.parent / "firmware" / "micromouse"
DEFAULT_MAP = FIRMWARE_DIR / "maze_map.h"
DEFAULT_PATH = FIRMWARE_DIR / "maze_path.h"

_F = r"(-?[\d.eE+]+)f?"

_WALL_RE = re.compile(
    rf"Obstacle\{{\s*WallObstacle\{{\s*{_F}\s*,\s*{_F}\s*,\s*{_F}\s*\}}\s*,"
    rf"\s*Vec2D\{{\s*{_F}\s*,\s*{_F}\s*\}}\s*\}}"
)
_CIRCLE_RE = re.compile(
    rf"Obstacle\{{\s*CircularObstacle\{{\s*{_F}\s*\}}\s*,"
    rf"\s*Vec2D\{{\s*{_F}\s*,\s*{_F}\s*\}}\s*\}}"
)
_COUNT_RE = re.compile(r"MAZE_OBSTACLE_COUNT\s*=\s*(\d+)")

# Segment({a, b}) or Segment({a, b}, 1.0f / r, Segment::Direction::Left)
_SEGMENT_RE = re.compile(
    rf"appendSegment\(\s*Segment\(\s*"
    rf"\{{\s*{_F}\s*,\s*{_F}\s*\}}\s*,\s*"
    rf"\{{\s*{_F}\s*,\s*{_F}\s*\}}\s*"
    rf"(?:,\s*1\.0f?\s*/\s*{_F}\s*,\s*Segment::Direction::(Left|Right)\s*)?"
    rf"\)\s*\)"
)

# The provenance comment export_map.py writes, so a loaded map can say which
# photo and which start pose it came from.
_PROVENANCE_RE = re.compile(
    r"^//\s*(\S+):\s*(\d+)x(\d+) lattice", re.MULTILINE
)
_ORIGIN_RE = re.compile(
    r"^//\s*\[([-\d.]+),\s*([-\d.]+)\] mm heading ([-\d.]+) deg", re.MULTILINE
)


class MazeHeaderError(ValueError):
    """A header that does not look like something the emitters wrote."""


def _read(path):
    path = pathlib.Path(path)
    if not path.exists():
        raise MazeHeaderError(f"no such header: {path}")
    return path.read_text()


def load_map(path=DEFAULT_MAP, strict=True):
    """Parse a generated ``maze_map.h`` into a ``types.Map``.

    ``strict`` checks the obstacle count against the header's own
    ``MAZE_OBSTACLE_COUNT``, which catches a truncated or half-written file --
    the failure build_maze.sh guards against with its temp-file dance.
    """
    text = _read(path)
    obstacles = []

    # One pass in file order, so an obstacle's index here is the index the
    # firmware's Map has -- which is what RayHit.index reports.
    for m in re.finditer(r"Obstacle\{[^}]*\}[^}]*\}\}", text):
        chunk = m.group(0)
        w = _WALL_RE.search(chunk)
        if w:
            length, thickness, alpha, cx, cy = (float(v) for v in w.groups())
            obstacles.append(
                Obstacle(WallObstacle(length, thickness, alpha), Vec2D(cx, cy))
            )
            continue
        c = _CIRCLE_RE.search(chunk)
        if c:
            radius, cx, cy = (float(v) for v in c.groups())
            obstacles.append(Obstacle(CircularObstacle(radius), Vec2D(cx, cy)))

    if not obstacles:
        raise MazeHeaderError(
            f"{path}: no obstacles found. build_maze.sh refuses to install a "
            f"header like this because it compiles and quietly breaks every fix."
        )

    if strict:
        declared = _COUNT_RE.search(text)
        if declared and int(declared.group(1)) != len(obstacles):
            raise MazeHeaderError(
                f"{path}: MAZE_OBSTACLE_COUNT says {declared.group(1)} but "
                f"{len(obstacles)} obstacles parsed -- truncated or hand-edited"
            )

    return Map(obstacles)


def map_provenance(path=DEFAULT_MAP):
    """What ``export_map.py`` recorded about a map: source image, lattice size
    and the map-frame start pose its origin sits on. ``{}`` if absent."""
    try:
        text = _read(path)
    except MazeHeaderError:
        return {}

    out = {}
    m = _PROVENANCE_RE.search(text)
    if m:
        out["image"] = m.group(1)
        out["lattice"] = (int(m.group(2)), int(m.group(3)))
    o = _ORIGIN_RE.search(text)
    if o:
        out["origin_mm"] = (float(o.group(1)), float(o.group(2)))
        out["origin_deg"] = float(o.group(3))
    return out


def load_path(path=DEFAULT_PATH):
    """Parse a generated ``maze_path.h`` into a list of ``types.Segment``.

    The file is a body include: bare ``planner.appendSegment(...)`` statements
    in the robot frame, the first starting at (0, 0). It carries its own start,
    which is why the sketch needs no setStart() for it.
    """
    text = _read(path)
    segments = []

    for m in _SEGMENT_RE.finditer(text):
        sx, sy, ex, ey, radius, direction = m.groups()
        start = Vec2D(float(sx), float(sy))
        end = Vec2D(float(ex), float(ey))
        if radius is None:
            segments.append(Segment(start, end))
            continue
        r = float(radius)
        if r <= 0.0:
            raise MazeHeaderError(f"{path}: arc with radius {r}")
        segments.append(
            Segment(
                start,
                end,
                1.0 / r,
                Segment.Direction.Left
                if direction == "Left"
                else Segment.Direction.Right,
            )
        )

    if not segments:
        raise MazeHeaderError(
            f"{path}: no appendSegment() calls found. The sketch prints "
            f"'[maze_path.h APPENDED NO SEGMENTS]' for the same reason."
        )
    return segments


def path_note(path=DEFAULT_PATH):
    """The emitter's header comment -- '16 segments, 1889 mm -- 1,1 -> 4,6,
    r=30 mm'. Empty string if absent."""
    try:
        text = _read(path)
    except MazeHeaderError:
        return ""
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("//") and "segments" in line:
            return line.lstrip("/ ").strip()
    return ""
