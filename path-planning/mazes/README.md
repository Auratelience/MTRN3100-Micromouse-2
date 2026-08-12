# mazes

Input photos. One overhead shot of the deck per file; everything else in
`path-planning/` is derived from one of these.

Tracked here: **`2.jpg`** and **`3.jpg`**. Note that several scripts default to
photos that are *not* in the repo — `maze_demo.py` to `mazes/4.png`, and
`export_map.py`, `bench.py` and `selftest.py --image` to `mazes/1.png`. Pass an
image explicitly:

```sh
./build_maze.sh 2.jpg --from 1,1 --to 5,3
uv run python maze_demo.py mazes/3.jpg
```

`selftest.py --image` additionally sweeps *every* file in this directory for its
post-masking check, so anything dropped in here has to be a fittable maze photo,
not a screenshot or a crop. Files OpenCV cannot decode are skipped; files it can
decode but cannot fit will fail the sweep.

## What a usable photo has to have

`maze_grid.py` calibrates itself from the photo, but it calibrates from
*something*, and these are the parts that are not negotiable:

* **Blue post caps, in focus and lit.** The lattice fit is a clustering of the
  blue mask; the HSV gate is `(80, 55, 75)`–`(110, 255, 255)` and a minimum blob
  area of 6 px. Fewer than 8 blobs and the fit refuses to start.
* **An overhead view, but not a perfectly vertical one.** The posts must project
  as visible streaks converging on a nadir — that convergence is what yields the
  camera height and the pitch. A dead-flat orthographic render has nothing to
  fit.
* **The whole deck in frame,** including the chamfered corners. The deck polygon
  is what keeps the planner inside the maze, and phantom lattice rows from a
  reflection off the frame are pruned by it.
* **Enough resolution to resolve a post cap.** ~50 px of lattice pitch is what
  the current photos give and what the "no morphological closing" decision in
  `detect_blobs` is calibrated against.

Free-standing cylinders and posts with no adjoining wall are both handled — see
[`../README.md`](../README.md) — so a maze does not have to be tidy, only
photographed properly.

## Naming

`<n>.png` or `<n>.jpg`. `build_maze.sh` accepts any of `2`, `2.jpg` or
`mazes/2.jpg`, and names its outputs after the stem (`map_2.h`, `path_2.h`,
`map_2_overlay.png`), so keep stems distinct.

Large images are fine to commit here; they are inputs, and a fit is only
reproducible against the exact photo it was made from. The generated headers in
`firmware/micromouse/` name their source photo in a header comment.
