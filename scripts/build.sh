#!/bin/zsh
# Compile an Arduino sketch in this repo into its build/ directory.
#
# This clears the build directory first, on purpose. That sounds backwards, but
# it is what makes the build fast — measured on the micromouse sketch:
#
#   build into an empty firmware/build     ~15s
#   build into an existing firmware/build  4+ minutes (often longer)
#
# arduino-cli stalls in "Detecting libraries used..." when it re-enters a build
# directory that already has artifacts in it, and that stall dwarfs the cost of
# just rebuilding. Nothing is lost by starting empty: the expensive part, the
# renesas_uno core, is cached *globally* by arduino-cli in
# ~/Library/Caches/arduino/cores/arduino_renesas_uno_nanor4_*/core.a and is
# reused across wipes. That cache is what keeps a from-scratch build at 15s.
#
# clangd's index (build/.cache) is preserved so Zed does not reindex every time.
#
# ./compile.sh at the repo root forwards here, so both spellings work.
set -e

FQBN=arduino:renesas_uno:nanor4
REPO_DIR=${0:A:h:h}

# Build targets, as <name> -> <sketch directory relative to the repo root>.
# Arduino requires a sketch's .ino to be named after its directory, so the
# sketch file is always $SKETCH_DIR/${SKETCH_DIR:t}.ino, and the build dir is
# always build/ beside the sketch — firmware/build and firmware-ds/build, which
# is where firmware/.clangd and firmware-ds/.clangd each pin their database.
typeset -A TARGETS=(
	micromouse firmware/micromouse
	lidar      firmware-ds/lidar
)
TARGET_ORDER=(micromouse lidar) # display order; the first one is the default
DEFAULT_TARGET=$TARGET_ORDER[1]

ME=${0:t}
die() {
	print -u2 -- "$ME: $*"
	exit 1
}

# Ask arduino-cli which serial ports have a board matching our FQBN on them and
# print "<address>\t<board name>" for the one match. Anything other than exactly
# one match is a hard error: silently picking one of two boards, or falling back
# to whichever port looks Arduino-ish, is how you flash the wrong device.
#
# Only ever called on the --flash path, so a plain build keeps this script's
# dependencies down to arduino-cli itself.
detect_board() {
	command -v python3 >/dev/null ||
		die "python3 is needed to auto-detect the board; pass --port instead"
	arduino-cli board list --json | python3 -c '
import json, sys

fqbn = sys.argv[1]
ports = json.load(sys.stdin).get("detected_ports") or []
matches = [
    (p["port"]["address"], b.get("name") or fqbn)
    for p in ports
    for b in (p.get("matching_boards") or [])
    if b.get("fqbn") == fqbn
]
if len(matches) == 1:
    print("%s\t%s" % matches[0])
    sys.exit(0)
if not matches:
    seen = [p["port"]["address"] for p in ports]
    print("no board matching %s is connected." % fqbn, file=sys.stderr)
    if seen:
        print("serial ports seen: %s" % ", ".join(seen), file=sys.stderr)
    print("check the cable, double-tap RESET to enter the bootloader, "
          "or pass --port.", file=sys.stderr)
    sys.exit(1)
print("%d boards matching %s are connected: %s" % (
    len(matches), fqbn, ", ".join(a for a, _ in matches)), file=sys.stderr)
print("pass --port to choose one.", file=sys.stderr)
sys.exit(1)
' "$FQBN"
}

usage() {
	cat <<EOF
Usage: ./scripts/build.sh [target] [--db] [--flash] [--port dev] [--help]
                          [arduino-cli args...]

Builds an Arduino sketch for $FQBN into build/ beside it,
in about 15 seconds, and optionally flashes it to a connected board (--flash,
about 11 seconds more).

Targets (the default is $DEFAULT_TARGET; ./compile.sh at the repo root is a
forwarder for this script, so ./compile.sh builds $DEFAULT_TARGET):
$(for t in $TARGET_ORDER; do printf '  %-12s %s -> %s\n' $t $TARGETS[$t] ${TARGETS[$t]:h}/build; done)

A target, if given, must be the FIRST argument. Anything after it that is not
one of this script's own options is passed straight through to
\`arduino-cli compile\`, so a bare word later on is a flag's value rather than a
target:
  ./scripts/build.sh --warnings all      surface every compiler warning
  ./scripts/build.sh lidar --verbose     show each compiler invocation
  ./scripts/build.sh --flash             build micromouse and upload it
  ./scripts/build.sh lidar --flash       build the bring-up sketch and upload it

Passthrough reaches \`arduino-cli compile\` only, never the upload step.

Options:
  --db        Regenerate the target's compile_commands.json only, so clangd can
              resolve Arduino and library headers. Skips code generation, so no
              .bin/.elf is produced. Re-run after adding an #include from a new
              library. (A plain build writes the same file, so this is only
              worth using when you want the database without the binary.)
  --flash     Upload to the connected board after a successful build. The port
              is found by asking arduino-cli which serial ports have a board
              matching $FQBN on them, and is resolved
              *before* the build, so an unplugged board fails immediately
              rather than after a full compile. Incompatible with --db, which
              produces no binary to upload.
  --port dev  Skip that detection and upload to this port, e.g.
              --port /dev/cu.usbmodem2101. Only meaningful with --flash.
  --help      Show this message.

Outputs (in the target's build directory):
  <target>.ino.bin         binary to flash
  <target>.ino.elf         symbols, for size analysis
  compile_commands.json    clangd database, pinned by the sketch tree's .clangd

Note: the build directory is emptied at the start of every run. That is
deliberate and is what keeps builds at ~15s rather than 4+ minutes — see the
comment at the top of this script. clangd's index (build/.cache) is preserved.
EOF
}

# The target is positional and must come first, so that a bare word appearing
# later can be safely treated as a passthrough flag's value (`--warnings all`)
# rather than as a mistyped target.
target=$DEFAULT_TARGET
if [[ $# -gt 0 && $1 != -* ]]; then
	[[ -n $TARGETS[$1] ]] || die "unknown target '$1' (targets: ${(j:, :)TARGET_ORDER})"
	target=$1
	shift
fi

# A while loop rather than `for arg in "$@"` because --port has to consume the
# argument after it.
db_only=0
flash=0
port=
passthru=()
while (($#)); do
	case $1 in
	--db)
		db_only=1
		shift
		;;
	--flash)
		flash=1
		shift
		;;
	--port)
		port=${2:?--port needs a device, e.g. /dev/cu.usbmodem2101}
		shift 2
		;;
	--help | -h)
		usage
		exit 0
		;;
	*)
		passthru+=("$1")
		shift
		;;
	esac
done

if ((db_only && flash)); then
	die "--db and --flash are incompatible: --db skips code generation, so there is no binary to upload"
fi
if [[ -n $port ]] && ((!flash)); then
	die "--port only means something with --flash"
fi

if ! command -v arduino-cli >/dev/null; then
	die "arduino-cli not found. Install it with: brew install arduino-cli"
fi

SKETCH_DIR=$REPO_DIR/$TARGETS[$target]
BUILD_DIR=${SKETCH_DIR:h}/build

# Paths are derived from this script's own location, so check the sketch is
# really there before the rm below runs against anything.
if [[ ! -f $SKETCH_DIR/${SKETCH_DIR:t}.ino || ${BUILD_DIR:t} != build ]]; then
	die "expected to find $SKETCH_DIR/${SKETCH_DIR:t}.ino - keep build.sh in the repo's scripts/ directory"
fi

# Resolve the upload port before building, not after: an unplugged board should
# cost a second to discover, not a full compile.
if ((flash)); then
	if [[ -n $port ]]; then
		[[ -e $port ]] || die "no such port $port"
		PORT=$port
		BOARD="board"
	else
		detected=$(detect_board) || exit 1
		PORT=${detected%%$'\t'*}
		BOARD=${detected#*$'\t'}
	fi
	echo "will flash $BOARD on $PORT"
fi

# Empty the build dir but keep clangd's index cache.
if [[ -d $BUILD_DIR ]]; then
	setopt local_options extended_glob glob_dots
	rm -rf $BUILD_DIR/^.cache(N)
fi

mode=()
((db_only)) && mode=(--only-compilation-database)

arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_DIR" "${mode[@]}" "${passthru[@]}" "$SKETCH_DIR"

if ((db_only)); then
	echo "compile_commands.json -> $BUILD_DIR/compile_commands.json (no binary built)"
else
	echo "binary -> $BUILD_DIR/${SKETCH_DIR:t}.ino.bin"
	echo "compile_commands.json -> $BUILD_DIR/compile_commands.json"
fi

# --input-dir because we built into a custom --build-path; without it
# arduino-cli looks for the binary in its own default location and misses it.
if ((flash)); then
	echo
	arduino-cli upload --fqbn "$FQBN" --input-dir "$BUILD_DIR" -p "$PORT" "$SKETCH_DIR"
	echo "flashed ${SKETCH_DIR:t}.ino -> $PORT"
fi
