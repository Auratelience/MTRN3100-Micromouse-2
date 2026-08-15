#!/bin/zsh
# Forwarder kept at the repo root because ./compile.sh is the command in the
# READMEs and in firmware/.clangd's own comment. The script itself, and every
# other build target, lives in scripts/.
exec ${0:A:h}/scripts/build.sh "$@"
