# notes

Reference material: the course handouts the project is built against, and
working notes taken while building it. Nothing here is built, generated or read
by any code in this repository.

| file | what it is |
| --- | --- |
| `MTRN3100_Micromouse.pdf` | the assignment specification — the tasks the firmware's `TASK` define selects between |
| `Micromouse_Kit_Information.pdf` | the supplied kit: parts, pinouts and the board's own documentation |
| `Lidar Maths.jpeg` | hand-worked derivation of the lidar localisation geometry, behind `LidarObserver` in `firmware/micromouse/observers.h` |

**None of it is tracked.** `.gitignore` excludes `notes/*.pdf` and
`notes/*.jpeg`, so this directory arrives holding only this README. The handouts
come from the course, not from here.

CAD for the physical robot is not in this directory — it is in
[`hardware/`](../hardware/).
