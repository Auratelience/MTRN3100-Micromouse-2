# path-planning

Overhead photo of the maze in, a path the firmware can drive out.

```
python maze_demo.py                        # photo -> map -> path -> appendSegment() calls
python maze_demo.py --from 4,4 --to 7,7 --emit path.h
python selftest.py --image                 # geometry, planner and CV checks
python bench.py                            # success rate and cost over a set of trips
```

These run under `uv` from this directory. To produce the firmware's header pair,
do not call `maze_demo.py` and `export_map.py` yourself — use
[`scripts/build_maze.sh`](../scripts/README.md), which gives both the same start
pose. Its per-run outputs (`map_<stem>.h`, `path_<stem>.h`, the overlay) land
here.

## Pipeline

| module | what it owns |
| --- | --- |
| `maze_grid.py` | finds the blue post caps and fits the 180 mm lattice, the nadir, the pitch and the camera height |
| `maze_map.py` | walls per bond, **posts per node**, the deck polygon, free-standing cylinders |
| `rrt_star.py` | collision model in world mm, straight-line RRT\*, and **Dubins RRT\*** |
| `dubins.py` | the six Dubins words, vectorised |
| `segments.py` | the firmware's `Segment` in Python, plus the C++ emitter |
| `maze_demo.py` | end to end, with an overlay and the pasteable output |

## Output: straights and arcs

`plan_dubins` searches in `(x, y, theta)` and steers with Dubins curves, so every
tree edge is already a sequence of straights and minimum-radius arcs — the exact
alphabet of `Segment` in `firmware/micromouse/types.h`. Nothing is smoothed
afterwards, so the curve that was collision checked is the curve that gets
driven.

```
segments   17 (7 straight, 10 arc at r=30 mm), 1960 mm; hold cruiseVelocity <= 346 mm/s
           firmware-representable, joins continuous

planner.appendSegment(Segment({0.01f, 0.00f}, {288.41f, 0.13f}));
planner.appendSegment(Segment({288.41f, 0.13f}, {291.24f, 0.00f}, 1.0f / 30.00f, Segment::Direction::Right));
```

Two details of the firmware contract that the emitter enforces, both of which
fail silently if ignored:

* **Minor arcs only.** `centrePreCalcRadiusAndMidpoint` rebuilds an arc's centre
  from its chord, which recovers the *minor* arc. A turn over 180 deg would come
  back as a different circle, so turns are split at 90 deg.
* **Handedness.** The map is image convention (+x east, +y south) and is
  left-handed; the robot's frame is right-handed (x forward, y left). `to_firmware`
  mirrors and flips every `Left`/`Right` with it, then re-origins the path onto
  the robot's start pose so it can be pasted against a fresh odometry frame.

`selftest.py` checks both against an independent computation, including a
deliberately unrepresentable 270 deg arc that `segments.check` must reject.

The demo defaults to `mazes/4.png`; the tests and benchmark pin `mazes/1.png` so
their numbers stay comparable. All four photos in `mazes/` fit and map.

Only `2.jpg` and `3.jpg` are tracked, though, so both of those defaults are
missing from a fresh clone — pass an image explicitly, or restore the others.
See [`mazes/README.md`](mazes/README.md) for what a usable photo needs.

## Posts

A post is an obstacle, not just a landmark, and the two failure modes are
opposite:

* **Posts with no walls.** A node can carry a post with no panel on any of its
  four bonds — an isolated stump on open floor. It never appears in
  `wall_segments_mm`, so a wall-only collision model drives straight through it.
  `mazes/1.png` has 10 of them.
* **No posts.** A node can equally carry nothing. `detect_posts` scores every
  node independently rather than assuming the lattice is populated, so a maze
  with gaps — or with no posts at all — maps correctly. `MazeWorld` likewise
  accepts an empty wall list, for a maze that is posts on bare floor.

Detection uses two independent pieces of evidence. The blue cap is what
`maze_grid` already segmented to fit the lattice; the fallback, for caps the
tight HSV gate drops, is the wall detector's darkness measurement swept along
the post's own leaning silhouette. The threshold is self-calibrating at half the
median score of the nodes `maze_grid` accepted. On `mazes/1.png` that finds 93
of 100 nodes — 80 by cap, 13 by silhouette — and the seven it rejects are the
chamfered corners, which have no post.

`prune_posts` then drops claims another obstacle already covers: nodes off the
deck (a reflection off the frame can add a whole phantom lattice row — `4.png`
fits 10x11 nodes for that reason) and nodes under a fitted cylinder, whose dark
body is what scored. Neither weakens the model: a post further outside the deck
than its own radius is unreachable behind the keep-in constraint, and a post
inside a cylinder is inside a strictly larger disc. That leaves 84 obstacles on
`1.png`. The ASCII dump marks a bare node `.`:

```
+---.   +   +   +   +   +---+   .---+
|       |                   |   |   |
.   +   +   +---+   +---+   +---+---.
```

Posts also join the occluder mask, so a post whose cap was missed is not
reported as a 15 mm cylinder.

## Two numbers worth knowing before you tune

**Turn radius comes in bands, not a range.** A 90 deg turn tangent to two cell
centrelines misses the pivot post by `|hypot(R, 25) - sqrt(2)|90-R||` — the
`hypot` because the body orbits the turn centre 25 mm off the axle, which is
what the arc has to fit. For a 40 mm robot that has to clear 53.5 mm, leaving
`R <= 26 mm` or `73 <= R <= 182 mm`. The obvious "a bit tighter than a cell"
choice of 70 mm sits in the gap and cannot turn a corner at all.

Of the two bands only the small one is searchable. A 90 mm arc is the textbook
micromouse turn — centred exactly on the post — but it is clear only if entered
within a few mm of the centreline and square to it, which a sampler essentially
never hits: at R = 90 the tree spread over two thirds of the maze and never once
connected. The 30 mm default turns comfortably from anywhere across the corridor
and costs speed, not safety: `sqrt(a r)` caps a 30 mm arc at 346 mm/s against the
robot's 392 mm/s ceiling.

The default now sits just outside the lower band, which reached 30.6 mm before
the axle offset was accounted for. That bounds the textbook corner, not the
planner — a centreline-tangent turn is one arc out of a continuum and the search
turns from wherever it likes, which is the whole reason 30 mm beat 90 mm. It does
cost the tightest corners some tries; `--turn-radius 25` buys the margin back at
316 mm/s.

**The corner cells are not reachable.** The deck's chamfer leaves cell (0,1) an
escape slot about 1 mm wide for a 40 mm robot. The straight-line planner threads
it — that is what its 1.2 mm minimum clearance was — but no bounded-curvature
path fits, at any radius, even with the padding set to zero. That is geometry,
not search: if you need to start there, shrink `--r` or `extra_clearance_mm`
and check the reported clearance yourself.

## The axle is not in the middle

A differential drive turns about its axle, so the axle is the point a Dubins
curve tracks and the point odometry reports — but on this chassis it sits 25 mm
behind the middle, and it is the middle that hits things. `AXLE_OFFSET_MM` in
`rrt_star.py` is that number; set it to 0 for a centred robot.

The two are genuinely different curves. The body rides 25 mm ahead of every
planned pose, and around a turn it orbits the same centre at `hypot(R, 25)` —
39 mm on a 30 mm arc, nearly a third wider — so an arc that clears for the axle
can still put a corner into a post. The alternative, a single disc of
`40 + 25 = 65 mm` about the axle, is sound but costs 25 mm of clearance
everywhere on a deck whose tightest doorways already have under 2 mm.

So `MazeWorld` splits its queries by what they are handed:

| query | takes |
| --- | --- |
| `clearance`, `is_free`, `motion_valid` | the **body centre** |
| `pose_clearance`, `pose_free`, `curve_valid` | an **axle pose**, and works the body out |

`plan_dubins` searches over axle poses and uses the pose queries throughout;
`plan` has no heading to speak of and plans the body centre directly. The
overlay's red marks and the `clearance` line are read off the body too, so what
the picture shows is where the robot actually is.

## Collision checking

Straight edges are checked exactly (segment-segment and segment-circle
distances). Curved edges are checked by sampling at `ds` of arc length and
requiring clearance above `ds/2`, which is conservative rather than approximate:
every point of the curve is within `ds/2` of arc length from a sample, and
Euclidean distance never exceeds arc length. `ds` defaults to 2.5 mm, small
enough not to wall off the tight doorways this deck has. `selftest.py` checks
20x-denser ground truth against it and expects zero leaks.

That proof is about the body's curve, so it is the body that has to be sampled
at `ds` — and it covers `hypot(R, 25) / R` more arc than the axle for the same
swept angle. `dubins.sample_poses` takes the offset and tightens the angular
step by exactly that factor; handing `curve_valid` axle samples at `ds` while
the body swings wider would leave the proof about nothing in particular.
