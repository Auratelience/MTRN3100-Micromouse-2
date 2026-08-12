# notes

Design files for the physical robot. Nothing here is built, generated or read by
any code in this repository — it is kept alongside the source so the chassis and
the board that the constants in `firmware/micromouse/constants.h` describe do not
live on somebody's laptop.

| file | format | opens with |
| --- | --- | --- |
| `Micromouse Lower Frame v2.3mf` | 3MF, Bambu Studio project (~73 KB) | Bambu Studio, Orca Slicer; the mesh alone in any slicer or CAD tool |
| `Micromouse PCB Figure.3dm` | Rhino 8 (~87 MB) | Rhinoceros 8, or Rhino's free viewer |

## Lower frame

The printed chassis plate the motors, board and sensor mounts bolt to. Version 2.

It is a slicer *project*, not a bare mesh: the file carries the plate layout and
the print profile as well as the geometry. As saved, that profile is a **Bambu
Lab X1E, 0.4 mm nozzle, PLA, 0.2 mm layers**. Re-slice against your own printer
before sending it anywhere.

The mesh is a single object, ~2160 vertices, bounding box ≈ 82 × 84 × 24.5 mm.

A `.3mf` is a zip, so the geometry can be pulled out without a slicer:

```sh
unzip -o "Micromouse Lower Frame v2.3mf" -d frame/   # 3D/Objects/*.model is the mesh
```

## PCB figure

A Rhino model of the board and its placement in the robot — the drawing the
sensor mount offsets in `constants.h` (`LIDAR_MOUNT_FRONT_X` and friends) were
measured against. It is large because Rhino saves render meshes with the file;
expect a slow first open and do not re-save it casually, since a round trip
through another tool will rewrite the whole thing in the diff.

Neither file has a plain-text representation, so git cannot merge them. If two
people need to change one, take turns.
