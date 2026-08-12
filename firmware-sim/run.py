#!/usr/bin/env python3
"""Entry point for the sim: ``python3 firmware-sim/run.py --help``.

A launcher rather than a ``__main__.py`` because the directory is called
``firmware-sim``. The hyphen is not a legal identifier, so ``python3 -m
firmware-sim`` cannot work and ``python3 firmware-sim/`` would run the package's
modules as top-level scripts, where their relative imports fail. Importing the
package by string sidesteps both -- ``importlib`` takes a name the ``import``
statement cannot spell.

Stdlib only, deliberately. The sim has no dependencies; only ``--plan`` needs
anything else, and that runs in path-planning's own uv environment.
"""

import importlib
import pathlib
import sys

PACKAGE = pathlib.Path(__file__).resolve().parent


def main():
    sys.path.insert(0, str(PACKAGE.parent))
    cli = importlib.import_module(f"{PACKAGE.name}.cli")
    return cli.main(sys.argv[1:])


if __name__ == "__main__":
    sys.exit(main())
