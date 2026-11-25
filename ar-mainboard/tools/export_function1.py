#!/usr/bin/env python3
"""Export the production F1 controller as one portable .c file, without compiling."""
import argparse
from pathlib import Path
import re

core = Path(__file__).resolve().parents[1] / "Core"
source = core / "Src" / "function1.c"


def expand(path, seen):
    """Inline project headers once; leave standard C headers to the compiler."""
    if path in seen:
        return ""
    seen.add(path)
    text = path.read_text(encoding="utf-8")
    return re.sub(r'^#include "([^"\n]+)"[ \t]*$',
                  lambda m: expand(core / "Inc" / m[1], seen), text, flags=re.MULTILINE)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="destination .c file, outside the board's Core directory")
    out = parser.parse_args().output.resolve()
    if out.suffix != ".c" or core in out.parents:
        parser.error("choose an export .c file outside Core; never overwrite the production sources")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("/* Standalone F1 controller. Compile with: cc -std=c11 -c function1.c\n"
                   " * Generated from Core/Src/function1.c by tools/export_function1.py.\n"
                   " * The caller supplies sensor events and applies motor interlocks. */\n"
                   + expand(source, set()), encoding="utf-8")
