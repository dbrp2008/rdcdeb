#!/bin/sh
# Behavioral mocks only; no STM32 target or hardware is exercised.
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT HUP INT TERM
cc=${CC:-gcc}
flags='-std=c11 -Wall -Wextra -Werror'
inc="-I$root/tests/mocks -I$root/Core/Inc"
# the interlock tests exercise the IR-based Key A run, so they start on Key A
$cc $flags -DAR_APP_DEFAULT_KEY=AR_KEY_A $inc "$root/Core/Src/ar_merged_app.c" "$root/Core/Src/ar_nav.c" \
  "$root/Core/Src/ir_calibration.c" "$root/Core/Src/encoder_calibration.c" "$root/Core/Src/us_capture_core.c" \
  "$root/tests/test_active_ui.c" -o "$tmp/active"
for case in 1 2 3 4 5 6 7 8 9 10 11; do "$tmp/active" "$case"; done
for test in ar_nav_test test_nav_guard; do
  $cc $flags $inc "$root/Core/Src/ar_nav.c" "$root/tests/$test.c" -o "$tmp/$test"
  "$tmp/$test"
done
# hardcoded route: with IR assist (-DAR_ROUTE_USE_IR=1) and encoder-only (default)
$cc $flags -DAR_ROUTE_USE_IR=1 -DAR_ROUTE_HOME=0 -DAR_RIGHT_STRAIGHT=0 -DAR_TOP_TURN_US=0 $inc "$root/Core/Src/ar_nav.c" "$root/tests/ar_route_test.c" -o "$tmp/ar_route_ir"
"$tmp/ar_route_ir"
# zig-zag version of the right key (AR_RIGHT_STRAIGHT 0) and the default straight version
$cc $flags -DAR_RIGHT_STRAIGHT=0 -DAR_TOP_TURN_US=0 -DAR_J_US=0 $inc "$root/Core/Src/ar_nav.c" "$root/tests/ar_route_enc_test.c" -o "$tmp/ar_route_enc" -lm
"$tmp/ar_route_enc"
$cc $flags $inc "$root/Core/Src/ar_nav.c" "$root/tests/ar_route_straight_test.c" -o "$tmp/ar_route_straight" -lm
"$tmp/ar_route_straight"
# drift trim (one wheel effectively bigger) and the STR test mode, with and without a trim
for trim in 0 20 -15; do
  $cc $flags -DAR_DRIFT_TRIM_PERMILLE=$trim $inc "$root/Core/Src/ar_nav.c" "$root/tests/ar_trim_test.c" -o "$tmp/ar_trim" -lm
  "$tmp/ar_trim"
done
$cc $flags $inc "$root/Core/Src/us_capture_core.c" "$root/tests/test_us_capture.c" -o "$tmp/usc"
"$tmp/usc"
# F20: 20 cm forward goes straight with mismatched motors
$cc $flags $inc "$root/Core/Src/ar_nav.c" "$root/tests/ar_f20_test.c" -o "$tmp/ar_f20" -lm
"$tmp/ar_f20"
