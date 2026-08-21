# scripts

Every entry point in the repo. All three take `--help` and all three derive
their paths from the repo root, so they can be run from any directory and do not
care about the caller's cwd. The two `.sh` are zsh; `export_splash.py` is a
`uv run --script` file that carries its own dependency block.

| script | what it does |
| --- | --- |
| `build.sh` | compiles an Arduino sketch into `build/` beside it |
| `build_maze.sh` | maze photo in, overlay on screen and `maze_map.h`/`maze_path.h` installed into the firmware |
| `export_splash.py` | splash art in, `splash_screen.h` installed into the firmware |

```sh
./scripts/build.sh                    # micromouse, the default target, ~15s
./scripts/build.sh --flash            # ...and upload it, ~11s more
./scripts/build.sh --db               # compile_commands.json only, for clangd
./scripts/build.sh --debug            # -DMICROMOUSE_DEBUG=1: boot self-check + loop diagnostics
./scripts/build_maze.sh 5 --from 1,1 --to 3,3
./scripts/export_splash.py            # re-export the OLED splash bitmap
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

### Debug builds

`--debug` appends `--build-property compiler.cpp.extra_flags=-DMICROMOUSE_DEBUG=1`
to the `arduino-cli compile` invocation, which turns on the boot self-check for
the runtime maze grid and the two `DIAGNOSTIC` reports already written into
`loop()` — the IMU read-failure count and the FIFO samples-per-cycle report.
Both are off by default because a periodic `Serial` write from inside the
control loop costs it milliseconds, which is exactly the kind of stall those
reports exist to catch; `--debug` is how you accept that cost on purpose, for a
bench session.

The build directory is wiped at the start of every run regardless of this flag
(see below), so there is no stale-define hazard in switching between a plain
build and a `--debug` one.

One caveat worth knowing if you also use `--db` with clangd: `--db` alone
generates `compile_commands.json` *without* `MICROMOUSE_DEBUG` defined, so
debug-only code reads as inactive in the editor unless `--debug` is passed
alongside it — `./scripts/build.sh --db --debug`.

### Flashing

`--flash` uploads after a successful build. The port is not hardcoded: the
script asks `arduino-cli board list` which serial ports have a board matching
the FQBN on them, and requires **exactly one** match. Zero matches and two
matches are both errors that name what was seen, because silently picking one of
two boards is how you flash the wrong device. `--port /dev/cu.usbmodemXXXX`
skips detection entirely.

`--flash` also re-runs [`export_splash.py`](#export_splashpy) before the
compile, so the logo the board boots with is always the current art. A plain
build skips it and compiles the committed header as it stands, which keeps the
fast inner loop free of `uv`.

Three ordering choices worth knowing: the port is resolved *before* the build, so
an unplugged board costs a second rather than a full compile; the splash is
re-exported after that check but before the compile, so a missing board still
fails first and the regenerated header is the one that gets built; and
`--db --flash` is rejected up front, since `--db` skips code generation and
leaves no binary to upload. Detection is the only thing in this script that
needs `python3`, and it runs only on the `--flash` path.

The Nano R4 uploads over DFU, so the port re-enumerates during the flash; that
is normal and `arduino-cli` reports the new port when it finishes.

The build directory is **emptied at the start of every run**, which is what
keeps a build at ~15s instead of the 4+ minutes `arduino-cli` takes when it
re-enters a populated one. The reasoning, and why nothing is lost by doing it,
is in the comment at the top of the script. `build/.cache` is preserved so
clangd does not reindex every time.

## build_maze.sh

The wrapper existed to enforce one coupling: `maze_demo.py` and `export_map.py`
had to be given the *same* `--from`/`--theta0`/`--r`, because the exported map
and the exported path were both re-origined onto that start pose. Give them
different ones and the robot localises against a map offset from the path it
is driving.

Although the script still lives here, everything it read and wrote belonged to
the offline planning project beside it — `mazes/`, the `uv` environment it ran
under, and the per-run `map_<stem>.h`/`path_<stem>.h`/overlay it left behind —
and that project has been deleted along with the rest of the offline pipeline.
`build_maze.sh` has nothing left to invoke; the header pair it last installed,
`maze_map.h`/`maze_path.h` in `firmware/micromouse/`, is what remains, each
still keeping the previous copy as `.bak`.

## export_splash.py

`hardware/Splashscreen.png` — a two-tone 128x64 export of
`hardware/Splashscreen.aseprite` — becomes `firmware/micromouse/splash_screen.h`,
a `constexpr uint8_t[1024]` in the layout `Adafruit_GFX::drawBitmap` reads.
`oledSplash.h` blits it once from `setup()`, so it holds the panel through
bring-up until the first `OLEDScreen` frame overwrites it.

Unlike `build_maze.sh`, this needs nothing from another project: it imports no
local modules and declares `pillow` in a PEP 723 block, so `uv run --script`
resolves it on its own. `build.sh --flash` runs it that way.

Two things are hard errors rather than something the script fixes up — art that
is not exactly 128x64, and art holding any value between black and white. Both
mean re-exporting from Aseprite, because rescaling or thresholding here would put
pixels on the robot that nobody drew, and the panel has no grey to render with.
Nothing is written unless the bytes actually change, so an unchanged re-export
does not restamp the header's mtime and send `arduino-cli` rebuilding.
