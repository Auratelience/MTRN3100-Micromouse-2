#!/usr/bin/env zsh
#
# build_maze.sh -- one maze photo in, an overlay on screen and the firmware's
# map header installed.
#
#     ./scripts/build_maze.sh 4                  # mazes/4.png, cell 1,1 -> 7,7
#     ./scripts/build_maze.sh 2.jpg --from 1,1 --to 5,3
#     ./scripts/build_maze.sh 1 --no-install     # overlay and headers there, firmware untouched
#
# Runs maze_demo.py for the plan and the overlay, then export_map.py for
# maze_map.h.  Both get the *same* --from/--theta0/--r, because the exported
# map and the exported path are both re-origined onto that start pose: give
# them different ones and the robot localises against a map offset from the
# path it is driving.  That coupling is the whole reason this wrapper exists.
#
# Every run leaves its own two headers in path-planning/ -- map_<stem>.h and
# path_<stem>.h -- and those are what get installed as maze_map.h /
# maze_path.h.  The installed header is written via a temp file and the previous
# one is kept as maze_map.h.bak -- it is not tracked by git, so a bad run would
# otherwise take the only copy with it.
#
# Both scripts run as 'uv run --directory path-planning python ...', so they get
# that directory's pyproject.toml and .venv (uv syncs it first if it is missing
# or stale) no matter where you invoked the wrapper from.

set -euo pipefail

# This script lives in scripts/ but every input and output belongs to
# path-planning/: mazes/, the uv project, and the generated map_<stem>.h /
# path_<stem>.h, which .gitignore covers as 'path-planning/*.h'.  Anchoring
# those to the script's own directory would put generated headers in scripts/,
# where that ignore rule does not reach.
ROOT=${0:A:h:h}
ME=${0:t}        # captured out here: inside a function $0 is the function name
PLANNING=$ROOT/path-planning
MAZES=$PLANNING/mazes
FIRMWARE=$ROOT/firmware/micromouse   # HEADER/PATH_HEADER derived after parsing

# planner defaults; --from 1,1 rather than export_map.py's 0,1 because the
# deck chamfer leaves the corner cell unreachable at bounded curvature (README)
FROM=1,1
TO=7,7
THETA0=auto
RADIUS=40
TURN=30
ITERS=4000
SEED=1
MODE=dubins
OUT=
INSTALL=1
OPEN=1

die() { print -u2 -- "$ME: $*"; exit 1 }
note() { print -- "\e[1m==>\e[0m $*" }

usage() {
    print -- "usage: $ME <image> [options]

  <image>            a name in mazes/ (4, 4.png, mazes/4.png) or any path

  --from i,j         start cell, shared by the plan and the map (default $FROM)
  --to i,j           goal cell for the plan (default $TO)
  --theta0 deg|auto  start heading, shared (default $THETA0)
  --r mm             robot radius (default $RADIUS)
  --turn-radius mm   arc radius; 30 or 75..178, nothing between (default $TURN)
  --iters n          RRT* iterations (default $ITERS)
  --seed n           RRT* seed (default $SEED)
  --mode m           dubins | polyline (default $MODE)
  --out file         overlay path (default map_<name>_overlay.png)
  --firmware dir     where maze_map.h and maze_path.h go (default $FIRMWARE)
  --no-install       plan, draw and export only, leave the firmware alone
  --no-open          do not open the overlay
  -h, --help"
    exit ${1:-0}
}

# ------------------------------------------------------------------ arguments
(( $# )) || usage 1
IMAGE_ARG=
while (( $# )); do
    case $1 in
        --from)        FROM=${2:?--from needs i,j}; shift 2 ;;
        --to)          TO=${2:?--to needs i,j}; shift 2 ;;
        --theta0)      THETA0=${2:?--theta0 needs deg|auto}; shift 2 ;;
        --r)           RADIUS=${2:?--r needs mm}; shift 2 ;;
        --turn-radius) TURN=${2:?--turn-radius needs mm}; shift 2 ;;
        --iters)       ITERS=${2:?--iters needs n}; shift 2 ;;
        --seed)        SEED=${2:?--seed needs n}; shift 2 ;;
        --mode)        MODE=${2:?--mode needs dubins|polyline}; shift 2 ;;
        --out)         OUT=${2:?--out needs a file}; shift 2 ;;
        --firmware)    FIRMWARE=${2:?--firmware needs a directory}; shift 2 ;;
        --no-install)  INSTALL=0; shift ;;
        --no-open)     OPEN=0; shift ;;
        -h|--help)     usage ;;
        -*)            die "unknown option $1 (--help)" ;;
        *)             [[ -n $IMAGE_ARG ]] && die "one image at a time, got $IMAGE_ARG and $1"
                       IMAGE_ARG=$1; shift ;;
    esac
done

HEADER=$FIRMWARE/maze_map.h
PATH_HEADER=$FIRMWARE/maze_path.h

[[ -n $IMAGE_ARG ]] || die "no image given (--help)"
[[ $FROM == <->,<-> ]] || die "--from wants i,j cell indices, got '$FROM'"
[[ $TO   == <->,<-> ]] || die "--to wants i,j cell indices, got '$TO'"
[[ $MODE == (dubins|polyline) ]] || die "--mode wants dubins or polyline, got '$MODE'"

# ---------------------------------------------------------------- the image
# accept a path, a name in mazes/, or a bare stem to extension-match
resolve() {
    local a=$1 c
    [[ -f $a ]] && { print -r -- ${a:A}; return }
    for c in $MAZES/$a $MAZES/$a.png $MAZES/$a.jpg $MAZES/$a.jpeg; do
        [[ -f $c ]] && { print -r -- ${c:A}; return }
    done
    return 1
}

if ! IMAGE=$(resolve $IMAGE_ARG); then
    print -u2 -- "$ME: no such maze '$IMAGE_ARG'. In ${MAZES:t}/:"
    print -u2 -- "  "${(j:, :)${(@f)"$(cd $MAZES && print -l -- *.(png|jpg|jpeg)(N))"}}
    exit 1
fi
STEM=${${IMAGE:t}:r}
: ${OUT:=$PLANNING/map_${STEM}_overlay.png}
EMIT=$PLANNING/path_${STEM}.h
MAP=$PLANNING/map_${STEM}.h
# the scripts run with $PLANNING as their cwd, so a relative --out would land
# there while the checks below looked for it next to the caller.  Pin it now.
OUT=${OUT:A}

# --------------------------------------------------------------- interpreter
# --directory instead of a bare cd: uv picks up path-planning's
# pyproject.toml/uv.lock/.venv and runs the script from there, while the wrapper
# keeps the caller's cwd so a relative --firmware still means what it says.
command -v uv >/dev/null || die "uv not on PATH; the pipeline runs under 'uv run'"
PY=(uv run --directory $PLANNING python)
[[ -x $PLANNING/.venv/bin/python ]] ||
    note "no .venv in ${PLANNING:t}/ yet -- uv will create and sync one, this first run is slow"

# ------------------------------------------------------------------- plan
note "planning $IMAGE_ARG -> ${IMAGE:t}, cell $FROM -> $TO, r=${RADIUS}mm turn=${TURN}mm"
LOG=$(mktemp -t build_maze) || die "mktemp failed"
trap 'rm -f $LOG' EXIT

# tee so the ASCII maze and the appendSegment() block stay on screen while we
# also get to inspect them
if ! $PY maze_demo.py "$IMAGE" \
        --from "$FROM" --to "$TO" --theta0 "$THETA0" \
        --r "$RADIUS" --turn-radius "$TURN" --mode "$MODE" \
        --iters "$ITERS" --seed "$SEED" \
        --out "$OUT" --emit "$EMIT" 2>&1 | tee $LOG; then
    die "maze_demo.py failed, nothing installed"
fi

PLANNED=1
if grep -q '^no path' $LOG; then
    PLANNED=0
    print -u2 -- "\e[33m!!\e[0m no path found -- raise --iters, change --seed, or shrink --r."
    print -u2 -- "   the map below does not depend on the plan, so the export still stands."
fi
if grep -q '^  PROBLEM' $LOG; then
    print -u2 -- "\e[33m!!\e[0m segments.check flagged the path above; do not paste it yet."
fi

# ------------------------------------------------------------------ overlay
[[ -f $OUT ]] || die "maze_demo.py reported success but $OUT is missing"
note "overlay ${OUT:t} ($(du -h $OUT | cut -f1))"
if (( OPEN )); then
    if command -v open >/dev/null; then
        open "$OUT"
    else
        print -- "   (no 'open' on this platform; view $OUT yourself)"
    fi
fi

# --------------------------------------------------------------------- map
# Exported before the --install gate and kept in path-planning/ as map_<stem>.h, next to the
# run's path_<stem>.h: the pair is what the firmware gets, and having both on
# disk is how you diff this run against the last one or read the header without
# digging it out of the firmware tree.  Still written via a temp file first --
# a failed or empty export must not eat the good copy from the previous run.
TMP=$(mktemp -t maze_map_h) || die "mktemp failed"
trap 'rm -f $LOG $TMP' EXIT

note "exporting map for the same start pose"
# sed so its "wrote ..." line names the kept header, not the scratch file
$PY export_map.py "$IMAGE" \
    --from "$FROM" --theta0 "$THETA0" --r "$RADIUS" -o "$TMP" |
    sed "s|$TMP|$MAP|" ||
    die "export_map.py failed, ${MAP:t} left as it was"

# a header with no obstacles would compile and quietly break every lidar fix
COUNT=$(sed -n 's/.*MAZE_OBSTACLE_COUNT = \([0-9]*\).*/\1/p' $TMP)
[[ -n $COUNT && $COUNT -gt 0 ]] ||
    die "generated header has no obstacles, ${MAP:t} left as it was"

cat $TMP > $MAP
note "map header ${MAP:t} ($COUNT obstacles)"

# ------------------------------------------------------------------ install
if (( ! INSTALL )); then
    note "--no-install: ${HEADER:t} and ${PATH_HEADER:t} untouched"
    print -- "   the generated headers are ${MAP:t} and ${EMIT:t} in ${PLANNING:t}/"
    (( PLANNED )) || exit 1
    exit 0
fi

[[ -d $FIRMWARE ]] || die "no firmware directory $FIRMWARE"

# install <src> <dest> <description> -- backs up, because neither header is
# tracked by git and a bad run would take the only copy with it
install_header() {
    local src=$1 dest=$2 what=$3
    if [[ -f $dest ]] && cmp -s $src $dest; then
        note "${dest:t} already up to date ($what)"
        return 0
    fi
    if [[ -f $dest ]]; then
        cp -p $dest $dest.bak
        print -- "   previous ${dest:t} kept as ${dest:t}.bak"
    fi
    cat $src > $dest   # not mv: keeps the destination's own permissions
    note "installed ${dest:t} -- $what"
}

install_header $MAP $HEADER "$COUNT obstacles"

# the path, which setup() body-includes.  Only from this run: a stale
# path_<stem>.h from an earlier one would drive the robot somewhere else.
if (( PLANNED )) && [[ -f $EMIT ]]; then
    NSEG=$(grep -c 'appendSegment' $EMIT)
    install_header $EMIT $PATH_HEADER "$NSEG segments, $FROM -> $TO"
else
    print -u2 -- "\e[33m!!\e[0m no path this run, so ${PATH_HEADER:t} was left as it was."
fi

# both headers are only meaningful from the pose they were exported against
print -- "   start the robot on cell $FROM at the heading above, odometry reset to (0, 0, 0)"
if grep -qE '^\s*//\s*#include "maze_map.h"' $FIRMWARE/micromouse.ino 2>/dev/null; then
    print -- "   maze_map.h is installed but still commented out in micromouse.ino"
    print -- "   (LIDAR LOCALISATION block) -- the path runs on dead reckoning until you enable it"
fi
(( PLANNED )) || exit 1
