#!/bin/sh
# Behavioral mocks only; no STM32 target or hardware is exercised.
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
tmp=${KEEP_TMP:-$(mktemp -d)}
[ -n "${KEEP_TMP:-}" ] || trap 'rm -rf "$tmp"' EXIT HUP INT TERM
cc=${CC:-gcc}
flags='-std=c11 -Wall -Wextra -Werror'
inc="-I$root/tests/mocks -I$root/Core/Inc"
# Compile the exact same combined drive objects once, with no function-specific
# defines. Exercise the real release-based route selector, then pin individual routes only
# in host replays. Probes reject inactive route callbacks, including F3.
# These replays pin the ORIGINAL F1 behaviour: build them with F1_LEGACY=1 (the fixes are tested by test_wm_fixes.c).
# They also pin the analog IR calibration page (IR_DIGITAL=0); the digital line IRs are tested by test_ir_digital.c.
for src in calibration_app ir_setup side_ir drive_plan function1 function2 function3; do
  $cc $flags -DF1_LEGACY=1 -DIR_DIGITAL=0 -I"$root/tests/mocks_app" $inc -c "$root/Core/Src/$src.c" -o "$tmp/$src.o"
done
wrap="-Wl,--wrap=DrivePlan_InitialFunction -Wl,--wrap=DrivePlan_NextFunction"
for api in F2_Init F2_Start F2_Update F2_PivotCommanded F2_Stop F2_BranchCount F2_Turning F2_Done F2_Status \
           F3_Reset F3_Scan F3_CommandAt F3_RightTurns; do
  wrap="$wrap -Wl,--wrap=$api"
done
board_test() {
  $cc $flags -I"$root/tests/mocks_app" $inc "$tmp/calibration_app.o" "$tmp/ir_setup.o" "$tmp/drive_plan.o" \
    "$tmp/function1.o" "$tmp/function2.o" "$tmp/function3.o" "$root/tests/drive_probe.c" "$root/tests/$1.c" \
    "$tmp/side_ir.o" "$root/tests/side_ir_io.c" $wrap -o "$tmp/$1"
}
board_test test_calibration_ui
"$tmp/test_calibration_ui"
"$tmp/test_calibration_ui" side-ir-setup
"$tmp/test_calibration_ui" pin-f1
"$tmp/test_calibration_ui" invalid-plan
"$tmp/test_calibration_ui" simulate-f2
# The digital line-IR pages: the same board objects, built with IR_DIGITAL=1.
for src in calibration_app ir_setup; do
  $cc $flags -DF1_LEGACY=1 -DIR_DIGITAL=1 -I"$root/tests/mocks_app" $inc -c "$root/Core/Src/$src.c" -o "$tmp/dig-$src.o"
done
$cc $flags -DIR_DIGITAL=1 -DDIGITAL_SCREENS_TEST -I"$root/tests/mocks_app" $inc "$tmp/dig-calibration_app.o" "$tmp/dig-ir_setup.o" "$tmp/drive_plan.o" \
  "$tmp/function1.o" "$tmp/function2.o" "$tmp/function3.o" "$root/tests/drive_probe.c" "$root/tests/test_calibration_ui.c" \
  "$tmp/side_ir.o" "$root/tests/side_ir_io.c" $wrap -o "$tmp/ui-digital"
"$tmp/ui-digital" digital-screens
board_test test_f3_board
"$tmp/test_f3_board"
# Compile the portable single-file export without even a project include path.
# Its standard-C client uses the public API; no board mocks, F2 or F3 are linked.
python3 "$root/tools/export_function1.py" "$tmp/function1.c"
$cc $flags -c "$tmp/function1.c" -o "$tmp/f1-new.o"   # the default (fixed) build must compile standalone too
$cc $flags -DF1_LEGACY=1 -c "$tmp/function1.c" -o "$tmp/f1-alone.o"
$cc $flags -DF1_LEGACY=1 -I"$root/Core/Inc" "$root/tests/test_function1.c" "$tmp/f1-alone.o" -o "$tmp/f1"
"$tmp/f1"
# Pure policy tests have no board or motor API.
$cc $flags $inc "$root/Core/Src/function2.c" "$root/tests/test_function2.c" -o "$tmp/f2"
"$tmp/f2"
$cc $flags $inc "$root/Core/Src/function3.c" "$root/tests/test_function3.c" -o "$tmp/f3"
"$tmp/f3"
# Exercise setup in isolation as well: no motor or mission API is available here.
$cc $flags -DIR_DIGITAL=0 $inc "$root/Core/Src/ir_setup.c" "$root/tests/test_ir_setup.c" "$tmp/side_ir.o" "$root/tests/side_ir_io.c" -o "$tmp/ui-ir"
"$tmp/ui-ir"
$cc $flags -DPART_SETUP $inc "$root/Core/Src/ir_setup.c" "$root/tests/test_ir_digital.c" "$tmp/side_ir.o" "$root/tests/side_ir_io.c" -o "$tmp/ui-ir-digital"
"$tmp/ui-ir-digital"
$cc $flags -DPART_SENSOR -include "$root/tests/mocks_adc/host_hal.h" -I"$root/tests" -I"$root/Core/Inc" "$root/Core/Src/ir_sensor.c" "$root/tests/test_ir_digital.c" -o "$tmp/ir-digital"
"$tmp/ir-digital"
for sensor in us1 us2 us3; do
  $cc $flags $inc "$root/Core/Src/us2_sensor.c" "$root/tests/test_$sensor.c" -o "$tmp/$sensor"
  "$tmp/$sensor"
done
$cc $flags $inc "$root/Core/Src/rotation_estimate.c" "$root/tests/test_rotation_estimate.c" -o "$tmp/rotation"
"$tmp/rotation"
$cc $flags $inc "$root/Core/Src/j3_rotation_run.c" "$root/tests/test_j3_rotation_run.c" -o "$tmp/j3run"
"$tmp/j3run"
python3 "$root/tests/test_us3_config.py"
$cc $flags -I"$root/tests/mocks_gpio" "$root/Core/Src/gpio.c" "$root/tests/test_us3_gpio.c" -o "$tmp/gpio"
"$tmp/gpio"
$cc $flags -I"$root/Core/Inc" "$root/Core/Src/ir_calibration.c" "$root/tests/test_ir_calibration.c" -o "$tmp/cal"
"$tmp/cal"
$cc $flags -DIR_DIGITAL=0 -include "$root/tests/mocks_adc/host_hal.h" -I"$root/tests/mocks_adc" -I"$root/Core/Inc" "$root/Core/Src/ir_sensor.c" "$root/tests/test_ir_poll.c" -o "$tmp/adc"
"$tmp/adc"
motorinc="-I$root/tests/mocks_motor -I$root/Core/Inc"
# Same source-local rename as the board; link the real generated IRQ vectors,
# board dispatcher and reference motor code in one executable.
$cc $flags $motorinc -DHAL_GPIO_EXTI_Callback=ELCP_EncoderEXTI_Callback -c "$root/Core/Src/elcp_motor/elcp_motor.c" -o "$tmp/encoder.o"
$cc $flags -I"$root/tests/mocks_motor" $inc -c "$root/Core/Src/side_ir.c" -o "$tmp/motor-side-ir.o"
$cc $flags $motorinc "$root/Core/Src/elcp_zero_pwm.c" "$root/Core/Src/stm32f4xx_it.c" "$tmp/encoder.o" "$tmp/motor-side-ir.o" \
  "$root/Core/Src/j3_rotation_run.c" "$root/Core/Src/rotation_estimate.c" "$root/tests/test_elcp_zero_pwm.c" -o "$tmp/motor"
for case in 1 2 3 4 5; do "$tmp/motor" "$case"; done
# Passive image: real EXTI vectors, active-low BTN1, no motor or PWM-start API.
# Pin only the replay clock. Both idle rates AND both bands must come from FB.
benchflags='-DENCODER_IDLE_SAMPLE_MS=3000u -DENCODER_NET_WINDOW_MS=200u'
$cc $flags $benchflags -I"$root/tests/mocks_motor" $inc "$root/Core/Src/stm32f4xx_it.c" \
  "$root/Core/Src/encoder_bench.c" "$root/Core/Src/encoder_monitor.c" "$root/tests/test_encoder_bench.c" -o "$tmp/bench"
"$tmp/bench"
$cc $flags $benchflags $motorinc "$root/Core/Src/encoder_monitor.c" "$root/tests/test_encoder_background.c" -o "$tmp/bg"
for case in calibration irregular bands settling startup-bursts bursts jitter reversal fractions wide; do "$tmp/bg" "$case"; done
python3 "$root/tests/test_calibration_config.py"
# The fixes (default build, F1_LEGACY 0): steering, 101 latch release, retrace hand-off, retrace home on a simulated AR.
$cc $flags $inc "$root/tests/test_wm_fixes.c" "$root/Core/Src/function1.c" "$root/Core/Src/function2.c" "$root/Core/Src/retrace.c" -o "$tmp/wmfix" -lm
$cc $flags $inc "$root/tests/test_f1_home.c" "$root/Core/Src/function1.c" -o "$tmp/f1home" -lm
"$tmp/wmfix"
"$tmp/f1home" > "$tmp/f1home.txt" || { cat "$tmp/f1home.txt"; exit 1; }
tail -1 "$tmp/f1home.txt"
$cc $flags $inc "$root/tests/test_f2_home.c" "$root/Core/Src/function1.c" "$root/Core/Src/function2.c" -o "$tmp/f2home" -lm
"$tmp/f2home" > "$tmp/f2home.txt" || { cat "$tmp/f2home.txt"; exit 1; }
tail -1 "$tmp/f2home.txt"
$cc $flags $inc "$root/tests/test_f3_home.c" "$root/Core/Src/function1.c" -o "$tmp/f3home" -lm
"$tmp/f3home" > "$tmp/f3home.txt" || { cat "$tmp/f3home.txt"; exit 1; }
tail -1 "$tmp/f3home.txt"
$cc $flags -O2 $inc "$root/tests/test_f4_home.c" "$root/Core/Src/function1.c" -o "$tmp/f4home" -lm
"$tmp/f4home" > "$tmp/f4home.txt" || { cat "$tmp/f4home.txt"; exit 1; }
tail -2 "$tmp/f4home.txt"
python3 "$root/tests/test_tft_formats.py"
$cc $flags -I"$root/tests/mocks_app" $inc "$root/Core/Src/side_ir.c" "$root/tests/test_enc_signed.c" -o "$tmp/enc_signed"
"$tmp/enc_signed"
$cc $flags $inc "$root/Core/Src/wheel_spin.c" "$root/tests/test_wheel_spin.c" -o "$tmp/wheel_spin"
"$tmp/wheel_spin"
$cc $flags $inc "$root/Core/Src/enc_watch.c" "$root/tests/test_enc_watch.c" -o "$tmp/enc_watch"
"$tmp/enc_watch"
