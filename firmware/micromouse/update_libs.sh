#!/bin/zsh
# Regenerate compile_commands.json so clangd can resolve Arduino/library headers.
# Re-run this after adding a new #include from a library or changing the FQBN.
set -e

SKETCH_DIR=${0:A:h}
BUILD_DIR=${SKETCH_DIR:h}/build

arduino-cli compile --fqbn arduino:renesas_uno:nanor4 --only-compilation-database \
	--build-path "$BUILD_DIR" "$SKETCH_DIR"

echo "compile_commands.json -> $BUILD_DIR/compile_commands.json"
