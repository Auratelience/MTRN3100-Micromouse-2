# scripts

Every shell entry point in the repo. Both scripts are zsh, both take `--help`,
and both derive their paths from the repo root — `${0:A:h:h}` — so they can be
run from any directory and do not care about the caller's cwd.

| script | what it does |
| --- | --- |
| `build.sh` | compiles an Arduino sketch into `build/` beside it |
| `build_maze.sh` | maze photo in, overlay on screen and `maze_map.h`/`maze_path.h` installed into the firmware |

```sh
./scripts/build.sh                    # micromouse, the default target, ~15s
./scripts/build.sh --flash            # ...and upload it, ~11s more
./scripts/build.sh --db               # compile_commands.json only, for clangd
./scripts/build_maze.sh 5 --from 1,1 --to 3,3
```

`./compile.sh` at the repo root is a forwarder to `build.sh`, kept because that
is the command in the READMEs and in `firmware/.clangd`'s own comment.

## build.sh

`micromouse` (`firmware/micromouse`) is the default target and the only one that
builds; it builds into `build/` beside the sketch, which is where that tree's
`.clangd` pins its compilation database.

> `--help` also lists a `lidar` target, pointing at `firmware-ds/lidar`. That
> bring-up sketch has been deleted from the repo, so the target is dead and
> `./scripts/build.sh lidar` fails on the missing directory.

A target, if given, must be the **first** argument, so that a bare word appearing
later is read as a passthrough flag's value rather than as a mistyped target —
`--warnings all` works, and `./scripts/build.sh bogus` is an error rather than
something `arduino-cli` gets to be confused by.

Everything that is not one of the script's own options is passed straight
through to `arduino-cli compile` — never to the upload step.

### Flashing

`--flash` uploads after a successful build. The port is not hardcoded: the
script asks `arduino-cli board list` which serial ports have a board matching
the FQBN on them, and requires **exactly one** match. Zero matches and two
matches are both errors that name what was seen, because silently picking one of
two boards is how you flash the wrong device. `--port /dev/cu.usbmodemXXXX`
skips detection entirely.

Two ordering choices worth knowing: the port is resolved *before* the build, so
an unplugged board costs a second rather than a full compile, and `--db --flash`
is rejected up front, since `--db` skips code generation and leaves no binary to
upload. Detection is the only thing in this script that needs `python3`, and it
runs only on the `--flash` path.

The Nano R4 uploads over DFU, so the port re-enumerates during the flash; that
is normal and `arduino-cli` reports the new port when it finishes.

The build directory is **emptied at the start of every run**, which is what
keeps a build at ~15s instead of the 4+ minutes `arduino-cli` takes when it
re-enters a populated one. The reasoning, and why nothing is lost by doing it,
is in the comment at the top of the script. `build/.cache` is preserved so
clangd does not reindex every time.

## build_maze.sh

The wrapper exists to enforce one coupling: `maze_demo.py` and `export_map.py`
must be given the *same* `--from`/`--theta0`/`--r`, because the exported map and
the exported path are both re-origined onto that start pose. Give them different
ones and the robot localises against a map offset from the path it is driving.

Although the script lives here, everything it reads and writes belongs to
[`path-planning/`](../path-planning/): `mazes/`, the `uv` project it runs under,
and the per-run `map_<stem>.h`/`path_<stem>.h`/overlay it leaves behind. That is
deliberate — `.gitignore` covers those outputs as `path-planning/*.h`, a rule
that does not reach into `scripts/`. The installed headers go to
`firmware/micromouse/`, each keeping the previous copy as `.bak`.
