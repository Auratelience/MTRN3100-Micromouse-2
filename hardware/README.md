# hardware

CAD for the things that get printed: the robot's chassis plate, and the maze wall
panels the deck is built from. Nothing here is built, generated or read by any
code in this repository — it is kept alongside the source so the chassis that the
constants in `firmware/micromouse/constants.h` describe does not live on
somebody's laptop.

| file | format | tracked | opens with |
| --- | --- | --- | --- |
| `Micromouse Lower Frame v2.3mf` | 3MF, Bambu Studio project (~73 KB) | yes | Bambu Studio, Orca Slicer; the mesh alone in any slicer or CAD tool |
| `Wall.3mf` | 3MF, Bambu Studio project (~118 KB) | yes | as above |
| `Wall.gh` | Grasshopper definition (~15 KB) | yes | Grasshopper, inside Rhinoceros |
| `Wall.stl`, `Wall-nolegs.stl` | STL meshes | **no** — `.gitignore` excludes `*.stl` | anything |

Both `.3mf` files are slicer *projects*, not bare meshes: they carry the plate
layout and the print profile as well as the geometry. Re-slice against your own
printer before sending either anywhere.

A `.3mf` is a zip, so the geometry can be pulled out without a slicer:

```sh
unzip -o "Micromouse Lower Frame v2.3mf" -d frame/   # 3D/Objects/*.model is the mesh
```

## Lower frame

The printed chassis plate the motors, board and sensor mounts bolt to. Version 2.

One object, 2160 vertices, bounding box ≈ 82.2 × 84.2 × 24.5 mm. Saved profile:
**Bambu Lab X1E, 0.4 mm nozzle, Bambu PLA Basic, 0.2 mm layers**.

## Wall

The maze panels. One wall part, tiled as 16 instances across three plates, so a
run of the project prints a batch rather than a single piece. The part's mesh
spans 183 × 183 × 85 mm — a cell-pitch footprint, matching the 180 mm cells and
12 mm panels in `constants.h`. Saved profile: **Bambu Lab X1E, 0.4 mm nozzle,
generic PLA, 0.28 mm layers** — a coarser layer height than the frame, since
these are bulk structural prints.

`Wall.gh` is the Grasshopper definition the geometry came from, so the panels are
parametric: change the pitch or the height there and re-bake rather than editing
a mesh. It needs Rhino, and it is binary, so git cannot merge it.

`Wall.stl` is the baked mesh and `Wall-nolegs.stl` the same panel without its
feet. Neither is tracked — re-export them from `Wall.gh` or `Wall.3mf`.

Neither the `.3mf` nor the `.gh` has a plain-text representation, so git cannot
merge any of this. If two people need to change one, take turns.
