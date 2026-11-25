#!/usr/bin/env python3
"""Every tft_prints format must use only what the TFT's own printf (Core/Inc/lcd/strhelper.h) supports:
%s %d %u %x %X %c %f with optional '-', '0', width and .precision. No '+', ' ', '#' flags, no 'l'/'h' sizes.
An unsupported spec prints garbage on the robot (e.g. "%+7ld" showed "71d") while the host tests' vsnprintf hides it."""
import pathlib, re, sys

root = pathlib.Path(__file__).resolve().parents[1]
spec = re.compile(r'%(%|[^a-zA-Z%]*[a-zA-Z])')
ok = re.compile(r'%-?0*\d*(\.\d+)?[sduxXcf]$')
bad = []
for src in sorted((root / "Core/Src").glob("*.c")):
    text = src.read_text(errors="replace")
    for m in re.finditer(r'tft_prints\s*\([^"]*"((?:[^"\\]|\\.)*)"', text):
        for s in spec.finditer(m.group(1)):
            if s.group(0) != "%%" and not ok.match(s.group(0)):
                bad.append(f"{src.name}:{text.count(chr(10), 0, m.start()) + 1}: {s.group(0)!r} in {m.group(1)!r}")
assert not bad, "unsupported TFT printf specs:\n" + "\n".join(bad)
print("TFT format strings: only specs the board's printf supports PASS")
