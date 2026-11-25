# AR mainboard — calibrate, select F1 or F2, then run

**The normal Debug and Release images command the drive motors.** Disconnect motor-driver power
while flashing, wiring and checking the display. Use an independent E-stop
that physically disconnects motor-driver power, with an operator ready to use
it. Zero PWM is not a power cutoff and was previously unreliable in another
setup, although the current setup was reported to stop at zero. The reference
motor initialization discards HAL PWM-start status; software cannot prove
motor stop or motion direction.

This is an **unvalidated drive test** (22.5% straight, up to 23.85% on the
outside wheel during a line correction). Calibrate on the actual floor and
tape after every hardware reset. With drive
power disconnected, verify the displayed black/white classifications on the
actual tape and floor, and check US1 against a ruler. Bench and build checks
do not establish stopping distance, wheel direction or turn angle. At the reduced
duty, verify that the inside wheel turns under load and the timed reverse clears the target.

A separate [passive encoder preset](#passive-encoder-bench) reads both wheels without driving either. It is not a button mode in F1/F2.

## Branch `wm-ar-fix`: line-following and return-home fixes

Default build (`F1_LEGACY 0`, see `Core/Inc/function1.h`). Set `F1_LEGACY 1` to get the original behaviour back exactly;
the existing host replays are built that way and still pass, `tests/test_wm_fixes.c` covers the fixes.

- **Steering:** `110` / `011` (line between the centre and a side sensor) now steer **gently**; `100` / `001` steer hard as
  before. Previously 110/011 drove straight, so nothing corrected until the line left the centre sensor (zig-zag).
- **Lost line (`000`):** if the line had just slipped out to one side it turns back that way, slower, for
  `F1_LOST_TURN_MS`; if it was centred (a gap in the tape) it goes straight on, as before.
- **`101` hold-left latch:** released by **any** scan with the centre black (`010/110/011/111`) or after `F1_SPLIT_MAX_MS`.
  Before, only an exact `010` released it, so with the line on the right (`001`, `011`) it kept turning left.
- **Return home by retracing (`retrace.c`):** on the way out every wheel command is recorded with the encoder pulses it
  moved (a breadcrumb about every 1 cm). After the 1 s target hold the AR replays the breadcrumbs **backwards with both
  wheels reversed**: it reverses exactly the path it drove (curves, corrections and the F2 pivot), no turn-around, no line
  and no US needed. Ends with `HOME (RETRACED)`; `HOME STALLED` if the wheels stop moving for 1 s. US1 errors / staleness
  are ignored while retracing (it reverses away from the target).
- **No "armed three-black finish"** on the next run (a committed hold no longer arms it): with the retrace home it would
  have stopped the next run at its first crossing.
- **Encoder edge filter** (`side_ir.c`, in front of the unchanged reference ELCP driver): FB edges closer than 250 us to the
  previous one on that wheel are dropped (ringing / double pulses; on our AR this took the noisy wheel from ~2600 to ~115
  counts per turn). Needed for the retrace to measure both wheels.
- **Odometry counts every pulse (sim only, not field-tested):** the reference driver's `pulse_count` only counts a pulse if
  the pin is still high when its handler reads it. Distance, heading, pivot angle, straight driving and the retrace use
  `SideIR_EncoderSigned` (`side_ir.c`) instead: every filtered pulse, + or - by the wheel's commanded direction, none at
  0 %, like the driver but without that loss. Pulses per wheel turn (`AR_PULSES_PER_TURN_R` / `_L` in `calibration_app.c`)
  were measured **under power** with the setup page's wheel check: **115.5 right, 115 left**. Slow hand turns read
  ~650-770 per turn (the slot edges flicker as they pass slowly), so don't calibrate from hand turns.
  The 250 us edge filter caps a wheel at ~4000 pulses/s, far above any drive speed at 115 per turn.
- **Into the zone, shifted left (sim only, not field-tested):** the AR ended square but 1-2.5 cm over the start zone's
  right edge, and rule 5.5.3.c.i scores a key only if the whole AR's ground projection is inside the AR-SZ. After the
  outline it now heads `F1_ZONE_SHIFT_DEG` (8) left of the line's direction until it has moved `F1_ZONE_SHIFT_MM` (23)
  sideways, then straight on to the wall (`function1.h`). With < 4 cm spare on the left, ~2.5 cm centred it on the floor (18); now 0.5 cm more for margin: `F1_ZONE_SHIFT_MM` 23
  ends ~30 mm further left in the sim, still square. The zone is only ~1-3 cm wider than the AR, so expect ~1 cm a side. Still over on
  the right: raise `F1_ZONE_SHIFT_MM`; over on the left now: lower it. 0 = off. F2 comes home the same way.
- **`PIVOT 123 DEG` (drive screen, row 8):** the F1 home pivot's angle as the encoders measure it, live while pivoting and
  kept after a stop (0 until a run pivots). After `LINE NOT FOUND` / `TURN NO LINE`, compare it with how far the robot
  really turned. Row 4's R/L are totals since power-on, not the pivot.
- **Encoder pins on timer input capture (sim only, not field-tested):** J1 FB (PB15) is TIM12_CH2 and J3 FB (PC8) is
  TIM8_CH3, set as input capture in `MainBoard.ioc` with input filter 15 and clock division /4. The timer only takes an
  edge once the pin has held its new level for 8 filter samples (about 12 us on TIM12, 6 us on TIM8), so short spikes from
  the motor wires are thrown away before they reach the count. `enc_capture.c` owns the two interrupt handlers
  (`TIM8_BRK_TIM12_IRQHandler`, `TIM8_CC_IRQHandler`) and passes each capture into the same `HAL_GPIO_EXTI_Callback` path
  as before (250 us filter, raw count, driver). **In CubeMX leave NVIC unticked for TIM8 capture compare and
  TIM8 break / TIM12.** If you tick them, CubeMX generates a second copy of each handler and the link fails.
  After changing the `.ioc`, run Generate Code. This filter only removes short spikes: long noise pulses, a missing common
  ground or a floating line still need the wiring fix (twist each FB wire with ground and keep it away from the motor wires,
  4.7 k pull-up to 3.3 V, 1 k + 10 nF RC at the pin).

- **US1 without waiting (`us1_capture.c`, from our AR / TR):** the echo is timed by TIM2 CH1 input capture + interrupt, so
  the loop no longer stops for up to ~15 ms per ping and the steering keeps reacting. Set up entirely in code (PA0 -> AF1
  TIM2_CH1, CH1 input capture, its own `TIM2_IRQHandler`): **no CubeMX change**. If TIM2 is not the expected 1 MHz / 32-bit
  free-running counter it falls back to the original blocking `US1_Measure`.
- **IR average of 2 scans (was 4):** the steering sees the line ~10-20 ms sooner.
- **TFT every 300 ms while driving (was 100 ms):** each update blocks the loop while it sends (as on our TR).
- **F1 goes home by turning 180 (`F1_RETURN_TURN 1`), not by reversing:** reversing, the IRs trail behind the wheels, so
  line following backwards runs away (the "trailer" problem). After the target hold it backs off 10 cm (both wheels held to
  equal pulses), pivots CCW measuring the angle on the encoders - tape before 140 deg is ignored (the line it left), the
  centre IR on tape between 140 and 220 deg ends the turn AT the line (warmaster's extra 0.45 s of turning after finding
  the line is gone), no tape by 220 deg stops it ("TURN NO LINE") - then follows the line home forward. Near home (within
  50 cm of the distance it drove out) an OUTER IR black (the 20 mm line cannot reach them while following; 2 inner IRs
  black is normal following) = maybe the start-zone outline: it stops steering and holds the LINE's direction (its heading
  averaged over the last 25 cm of following, from the encoders - not the angle of a correction in progress); the outline
  counts once the 3 analog IRs and an outer IR have each been black within 6 cm - they need not trip together (an angle,
  unequal knobs: warmaster's return failed at the zone). Then straight into the zone, still on the line's heading, until
  US1 < 55 mm (the zone's wall) - near home that wall alone also finishes it; at the latest 30 cm past the expected
  distance ("HOME BY DIST"). A missing echo while it turns / heads home does not stop it. Backup if US1 misses the
  wall: near home (or in the zone), the wheels moving under 5 mm in 0.5 s = against the wall, stop ("HOME AT WALL"); it
  only works if the wheels stall there rather than spin. US1 is the CENTRE front US (face 55 mm behind the front edge), so
  US1 < 55 mm = the front at the wall: the 265 mm AR ends inside the 300 mm zone.
- **F2 also goes home by turning 180** (`F2_RETURN_TURN` 1; 0 = the retrace): the same back-off and anticlockwise pivot as
  F1 (140-220 deg window), then it follows the vertical line down. The bottom line (the T) counts only within 30 cm before /
  25 cm after the key -> T distance driven out (`HOME NO JUNCTION` if never seen): 111, or 2 analog + an outer IR. It then
  drives **8 cm straight past the line** (wheels matched), and pivots LEFT in place until the centre IR is white then
  black (35-130 deg, else `HOME T NO TAPE`). Turning as soon as the IRs reach the line cannot work: their circle only
  touches the line there. After advancing, the circle crosses the line for any IR-to-axle distance (pickup ~60-80 deg),
  and line following straightens it. The line's direction home = the vertical line's averaged heading + 90 deg. The leg home
  = the start -> T distance out, corrected by how early / late the T came (the same IR-to-axle offset, opposite sign).
  Then the F1 finish (outline, into the zone, wall). Sim (`tests/test_f2_home.c`, the ARA map geometry): home within 4
  deg and 4 cm of the line for IRs 10-16 cm ahead of the axle, a weaker reverse motor, uneven IR knobs, 3 deg start error.
- **F2's outbound right turn at the T goes 8 cm past it first** (`F2_T_OUT_ADVANCE_MM` 80, 0 = the old turn at once):
  `F2 PAST THE T` (straight, wheels matched on the encoders), then the 5:3 pivot (`F2 TURN RIGHT` / `F2 FIND LINE`) with
  its 75 ms gap and 54..117 deg window. Turning where the side IR sees the line left the IRs' circle tangent to the
  vertical line: the centre never saw white then black. 80 mm past, it crosses the line ~73 deg into the pivot at the
  measured ~112 mm IR-to-axle. Sim (`tests/test_f2_home.c` now runs function2.c itself): 51 / 51 home with it, 0 / 51
  with the old turn; works for IRs 9-16 cm ahead of the axle (18 cm: `F2 TURN NO TAPE`). Not field-tested.
- **Odometry since boot:** F1_Init keeps the board's totals and the board feeds them before each start (before, every run
  after the first had the previous run's distance in its expected distance home).
  Sim (`tests/test_f1_home.c`): IRs 10-16 cm ahead of the axle, a motor 15% weaker in reverse, IR knobs +-4 mm apart, a
  start 3 deg off, an outer IR that never reads black, a 4 cm bump in the line just before the zone: always home, heading
  within 2 deg, within 2 cm of the line.
- **Retrace replays each wheel's pulses** (not just the motor power): a motor weaker in reverse no longer bends it.
 the first time all three front IRs see black within the first ~35 cm of the
  run (the start zone's black outline) the distance is noted. Reversing home, the moment the IRs come back **off** that
  outline the AR is that same distance from the start, so the rest of the replay is corrected to match: any slip /
  miscount over the whole trip is cancelled there. If the trail runs out before the outline is reached it keeps
  reversing (up to ~10 cm) to find it; a black crossing more than ~10 cm away from where it is expected is ignored. In the
  sim, 5-6 % wheel slip on the way home ends 70-80 mm off without it and 3-13 mm with it. It fixes the **distance**, not
  heading: slip in turns can still leave the AR ~1 cm to the side. Constants: `RETRACE_EDGE_MAX_PULSES`,
  `RETRACE_EDGE_WINDOW_PULSES` (`retrace.h`).
- **F2 branch turn only accepts tape inside an angle window:** the pivot angle is measured from the encoders
  (65 mm wheels, ~115 pulses per wheel turn under power, 233 mm track). Tape seen before `F2_TURN_MIN_DEG` (0.6 x expected) is ignored (the
  old line, a crossing), and if no tape is found by `F2_TURN_MAX_DEG` (1.3 x expected) it stops with **`F2 TURN NO TAPE`**
  instead of spinning. `F2_TURN_EXPECT_DEG` = 90 (the branch is a right angle). `AR_TRACK_MM` = 233 is the value tuned on
  this AR (246 mm measured centre to centre; 233 makes the encoder turns come out right).
- **Encoder safety stops (sim only, not field-tested).** Every move that ends on an encoder distance or angle also has a
  time limit, and a watch stops any run whose driven wheel stops giving pulses, so a dead, unplugged or miscounting encoder
  ends the run with a message instead of driving or spinning away:

  | Status | Meaning | Limit (`#define`) |
  |---|---|---|
  | `R NO PULSES` / `L NO PULSES` / `NO PULSES L+R` | a wheel commanded at >= 15 % gave no FB pulse for 1 s: dead / unplugged encoder, or the wheel is stalled | `ENC_WATCH_MS`, `ENC_WATCH_MIN_PCT` (`enc_watch.h`) |
  | `F2 ADV TIMEOUT` | F2 out: 80 mm past the T not reached | `F2_ADVANCE_MAX_MS` 3 s (`function2.h`) |
  | `F2 TURN TIMEOUT` | F2 out: the right pivot found no branch | `F2_TURN_MAX_MS` 6 s |
  | `HOME T TIMEOUT` | F2 home: past the bottom T, or the left pivot, overran | `F2_T_ADVANCE_MAX_MS` 3 s, `F2_T_TURN_MAX_MS` 6 s (`function1.h`) |
  | `HOME BY TIME` | the return took longer than 2 x the outbound time + 5 s | `F1_RETURN_EXTRA_MS` |

  Wheels below 15 % are not watched (the inside wheel of a held turn 9.9 %, the F2 pivot's reverse wheel 9.45 %, the 180
  pivot 11.25 %, the launch 13.5 %): a slow wheel may legitimately barely turn. The 180 pivot keeps its 10 s
  `LINE NOT FOUND`. The 1 s watch is longer than `F1_STALL_MS` (500 ms), so `HOME AT WALL` still ends a run at the zone
  wall; anywhere else, pushing into something stops with `NO PULSES L+R`. The 3 s / 6 s limits assume at least ~27 mm/s
  and ~20 deg/s (the sim does ~225 mm/s and ~60 deg/s): raise them if the real AR is slower. Sims: `test_f1_home.c` and
  `test_f2_home.c` freeze the encoders mid-run and check each stop; `test_enc_watch.c` tests the watch.

Checked: host suite (`F1_LEGACY=1`) and `test_wm_fixes.c` pass; the whole drive image compiles and links for the STM32
against ST's HAL (39 KB flash, 33 KB RAM). Not yet run on the robot.

## Operator sequence

**Digital line IRs (default, `Core/Inc/ir_mode.h` `IR_DIGITAL 1`).** L/C/R are digital modules like L2/R2: PA2 / PC2 / PC3
as GPIO inputs, no pull, **1 on black, 0 on the floor**; ADC1 is removed from `MainBoard.ioc` (no `adc.c`). There is no
white/black capture. The boot page (`AR SETUP`) shows:

| Row | Shows |
|---|---|
| `1=TAPE  0=FLOOR` | the key |
| `FRONT L0 C1 R0` | live L/C/R - set each module's knob so it reads 1 on the tape, 0 on the floor, at running height |
| `SIDE  L0     R0` | live L2/R2 |
| `R WHEEL 123` / `L WHEEL 45` | J1 / J3 **raw** encoder pulses since reset (`SideIR_EncoderRaw`, counted in the EXTI dispatch at any speed - the reference driver's `pulse_count` ignores pulses while a wheel is commanded to 0 %, so it can't show a hand turn). Turn each wheel by hand: its number goes **up** whichever way (by hand it reads ~650-770 per turn: slow turns over-count at the slot edges, which is fine for this check; the true figure is ~115 under power, see the powered check below). A wheel parked exactly on a slot edge can creep up with motor power on (motor hum flicks it across the edge); nudge it. Those pulses are ignored while the wheel is commanded to 0 %. Stays 0 = no pulses reach the pin; `--` in the `F1_LEGACY` build |
| `HOLD B1=L B2=R` → `L SPIN 431` / `R LAST 1155` | **Powered wheel check, robot LIFTED** (`wheel_spin.c`): hold **B1 alone** 1 s and the **left** wheel turns forward at 12 %; **B2 alone** = the **right** wheel. Release (or press both) stops it. The number is the FB pulses of this hold (`SPIN` while turning, `LAST` after). Put a tape mark on the wheel, hold until it has turned exactly 10 times, release: pulses / 10 = pulses per wheel turn **under power** (the figure odometry needs; slow hand turns can over-count at the slot edges). `MOTORS HELD AT 0` in the `F1_LEGACY` build |
| `READY` | then `B1+B2 = DRIVE`: **hold B1+B2 and release** -> `F1 LINE BR 0`, stopped |

The drive page (digital) shows `FRONT L C R` (black on 2 scans in a row, the value the steering uses), `SIDE L R`,
`US1 ... mm`, the wheel counts `R 1234  L -56`, the status, then `B1 = RUN` (stopped) / `B2 = STOP` (running) and
`B1+B2 = TO F2` / `TO F1`. The analog calibration below applies only with `IR_DIGITAL 0` and ADC1 IN2/IN12/IN13
restored in the ioc (its drive page keeps the old `IR RAW` / `LINE` rows and `B1 RUN  1+2 NEXT` hints).

Every hardware reset opens **`IR CAL 100 SAMP`**; there is no saved or default drive calibration. Leave motor-driver power disconnected.

1. Place all three analog sensors over white at running height. Press and release **B1 alone**. Hold still until the `W` row fills in.
2. Move all three over black. Press and release **B2 alone**. Wait for `CAL READY`; all three `T` values and `BLACK` polarities must be valid.
3. **Hold B1+B2 together, then release both.** The firmware applies the displayed left/centre/right thresholds and polarities to drive, in RAM. An overlap suppresses both single-button actions, even when one button leads or is released much later. A rejected chord leaves the captures alone. Acceptance does **not** start a motor.

**Motor outputs during calibration:** Both PWM compare registers are set to zero
before either channel starts and before LCD initialization. Every calibration
pass rewrites J1 (TIM12 CH1) and J3 (TIM8 CH2) to zero and both DIR pins low,
without restarting the timers. This includes waiting, sampling, errors,
rejected chords and the successful apply pass. Applying T leaves F1 stopped;
nonzero duty requires a separate accepted start. With motor-driver power
disconnected, a scope on PB14 (J1) and PC7 (J3) can check for any active PWM
through setup and apply. Software zero is not a power cutoff.

### Drive-mode buttons

The calibration **apply** gesture is consumed: the robot starts this screen stopped, with **F1 selected**. Check the top row for the selected function and branch count. The [selector](Core/Src/drive_plan.c) contains **F1, F2, F3 and F4** (legacy build: F1 and F2 only), not the passive encoder bench.

| Input | While stopped | While running |
|---|---|---|
| **B1 alone** | Tap and release to start the displayed function **fresh**, subject to motor, timer, IR and US1 interlocks. | Ignored; a B1 press spanning a finish or fault cannot queue a restart. |
| **B2 alone** | Stay stopped. Keep the selection, last count and applied calibration. | Request zero on both wheels at the first raw B2 check, without waiting for debounce or release. Keep the current function selected. |
| **B1+B2 together** | Hold both, then release both to cycle **F1 → F2 → F3 → F4 → F1** once (legacy build `F1_LEGACY 1`: F1 → F2 → F1). Never starts the motors. | B2 stops the current run first. On release of a qualified chord, select the next function and remain stopped. |

A single or overlapping pair must be stable for **30 ms**, followed by **30 ms with both buttons up**, before a run or selection is accepted. Either button can lead or be released last. A brief observation of the other button cancels a single, but an overlap too short to qualify cannot select either. B2's raw stop is independent of these timings; it is a cooperative software stop, not the physical E-stop.

The hints are `B1 = RUN` (stopped) and `B2 = STOP` (running), plus `B1+B2 = TO F2` / `TO F3` / `TO F4` / `TO F1` (analog build: `B1 RUN  1+2 NEXT` / `B2 STOP 1+2 NEXT`). A finish, abort, rejected start or sensor fault never advances the selector. Rerun the same function with a fresh B1 tap; every accepted run resets the route state and branch count. Cycling resets the newly selected function's displayed count, not the calibration or side-IRQ diagnostics.

## Drive motor PWM: 10 kHz on both wheels

| Wheel | Output | Timer input | Prescaler (PSC) | Period (ARR) | PWM carrier |
|---|---|---:|---:|---:|---:|
| J3 left | PC7 · TIM8 CH2 | 168 MHz (APB2 timer) | 15 | 1049 | 10 kHz |
| J1 right | PB14 · TIM12 CH1 | 84 MHz (APB1 timer) | 7 | 1049 | 10 kHz |

Each counter runs at **10.5 MHz** after division by `PSC + 1`; each PWM cycle is
`ARR + 1` = 1,050 ticks, or **100 µs**. The [clock check](tests/test_calibration_config.py)
compares [MainBoard.ioc](MainBoard.ioc) with the generated clock and [timer setup](Core/Src/tim.c).
The F1/F2 speed profile changes PWM duty only; it does not change the carrier
frequency or the millisecond timebase.
US1 still uses the separate 1 MHz TIM2 counter. The passive EncoderBench measures
feedback edges against elapsed milliseconds, not PWM cycles; its per-wheel calibration is automatic.

**Scope check:** With the board logic powered and motor-driver power **disconnected**,
probe PC7 and PB14 during a nonzero drive command. Expect a 100 µs period on each
and about 22.5 µs high during F1/F2 straight following. During calibration,
stop, or in EncoderBench, both pins should stay low; there is no PWM waveform to measure then. Equal carrier
frequency does not guarantee equal wheel RPM.

## Where to edit F1

- [`function1.c`](Core/Src/function1.c) owns the launch, crossing count, US1
  approach/hold/reverse/pivot/alignment, forward return and armed finish. It takes
  millisecond timestamps, analog tape masks and digital side observations, and
  returns left/right wheel requests. Seed `F1_Start` with the current L2-or-R2
  level; feed `F1_SideSample` for crossings and `F1_Scan` for analog tape. Feed
  each valid US1 result (at completion time) to `F1_TargetPing`. A true return
  means **approach latched**, not stop. Call `F1_CommandAt` every control tick
  after checking interlocks, supplying the latest fresh distance. Apply zero
  on both wheels for `F1_KEEP_STOPPED`, including the first hold command.
  It compiles without HAL, F2 or F3; see [`function1.h`](Core/Inc/function1.h).
- [`ir_setup.c`](Core/Src/ir_setup.c) captures white/black and returns exactly the `T` and `BLACK` rows shown on the TFT.
  [`calibration_app.c`](Core/Src/calibration_app.c) uses that profile for raw and filtered drive classification and handles sensor freshness, drive buttons and motor polarity.
- [`line_drive.h`](Core/Inc/line_drive.h) defines `DRIVE_US1_STOP_MM` for the
  F1/F2 start inhibit and approach trigger, and `DRIVE_US1_FINISH_MM` for the
  armed return finish. `US_APPROACH_MS` is in `function1.c`. `F12_DUTY` applies
  **90% of base duty** to every F1/F2 movement, including the separate F2 branch
  pivot. `F1_CommandAt` already returns scaled duties; do not scale them again
  at the motor write. F3 and the passive bench are unchanged.
- F2 reuses the base run timings, keeps its original per-scan steering, and
  substitutes its configured branch turn only on the ordinary outbound leg.
  F3 has its own controller but is not selectable.

For a **single portable `.c` file** with the project headers included, run this
from `ar-mainboard`. The export compiles as C11 using only standard
headers. Use it **instead of**, not alongside, the project controller. It is a
controller module, not a `main()` or a standalone flash image.

```sh
python tools/export_function1.py standalone/function1.c
```

## Build the drive firmware

Open `ar-mainboard` in VS Code. **Terminal → Run Task** offers these project actions:

| Task | What it does |
|---|---|
| **Flash Existing AR MainBoard (Release)** | Programs `build/Release/MainBoard.elf` as it is, verifies it, then resets. Requires a previous build; it does **not** include later source edits. |
| **Build + Flash AR MainBoard (Release)** | Configures and compiles the current sources, then flashes, verifies and resets only if the build succeeds. |
| **Build + Flash AR Encoder Bench** | Configures and builds the **EncoderBench** preset, then programs only `build/EncoderBench/EncoderBench.elf`, verifies and resets. |

Disconnect motor-driver power before any flash action. The build steps are hidden
from the picker; nothing is assigned to `Ctrl+Shift+B`. To run the **same
Release build without flashing**, open a terminal in `ar-mainboard`:

```sh
cmake -P tools/build_release.cmake
```

This runs CMake configure (`-G Ninja`, Release) and then compile, without a
preset. The last step of **Build + Flash** runs the programmer only if **both**
succeeded; it does not fall back to flashing an old ELF. **Flash Existing**
still deliberately skips the build and does not require a compiler.

Both Debug and Release build the calibration-first drive firmware. The task
keeps the existing cache, including its check for retired stationary-only
modes. If that check fires, review the intended firmware before starting a
fresh drive build. For a different output directory, use
`cmake -S . -B <dir> -G Ninja -DCMAKE_BUILD_TYPE=Release` and `cmake --build <dir>`
with the Arm tools available; the Release flash actions still target `build/Release`.

### Windows: `arm-none-eabi-gcc` not found for C and ASM

Both messages refer to **one executable**. The board uses GCC for C and startup
assembly; you do **not** need `g++` or a CXX setting. The build task needs
CMake (3.22+), Ninja and a complete bare-metal Arm GNU toolchain, including
`arm-none-eabi-objcopy` and the C runtime. It prints the full GCC path it
found and makes that `bin` available to configure **and** compile. It does
not install tools or edit Windows `PATH`.

The helper checks an explicit `AR_GCC_BIN` first. Otherwise it tries existing
CMake cache hints, the configured `CUBE_BUNDLE_PATH`, `PATH`, the per-user
`%LOCALAPPDATA%/stm32cube/bundles` layout, and CubeCLT/CubeIDE GNU toolchains
under `%SystemDrive%/ST`. For ST bundles it prefers `14.3.1+st.2` if complete,
then tries other installed versions. GCC and objcopy must be in the same `bin`.

**Custom installation:** from the board folder, run this once in PowerShell,
substituting the directory that actually contains `arm-none-eabi-gcc.exe`
(not the `.exe` itself):

```powershell
cmake "-DAR_GCC_BIN=D:/path to your toolchain/bin" -P tools/build_release.cmake
```

This only builds. It saves the selected directory as a CMake search hint in
`build/Release`, so the next VS Code task can reuse it without a system PATH
edit. If no such executable exists, install **GNU Tools for STM32** or an
**Arm GNU Toolchain for `arm-none-eabi`** first; installing CMake or a VS Code
extension alone is not a compiler installation. If you deliberately switched
toolchains and CMake still reports an old **absolute** compiler path, move
`build/Release` aside and retry. A missing CubeMX toolchain **file** or missing
`Drivers` is a different error: supply the generated AR project files (see
Verification). Do not use **Flash Existing** as a workaround for a failed build.

The [field card](../../docs/AR-F1-field-card.md) includes a motor-power-off branch-counter check. See the [calibration guide](tools/ir_calibration/README.md) for the threshold calculation and help with weak readings.

## Passive encoder bench

Use **Build + Flash AR Encoder Bench** with motor-driver power disconnected. The
screen should say **`ENCODER NO DRIVE`**, not `IR CAL 100 SAMP`. Both PWM pins
(J3/PC7, J1/PB14) are driven low before the TFT starts. The bench does not link
the motor driver or start PWM. Neither button can drive a motor. Software zero
is not electrical isolation; leave motor power disconnected for hand testing.

### Let the encoders settle, then turn the wheels

1. Power the encoder logic and **keep both wheels still** at `AUTO: KEEP STILL`.
   **No button press is needed.** J3 (left/PC8) and J1 (right/PB15) each learn
   their own idle rate **and** deadzone from the feedback pulses. There is no
   preset rate, preset wheel tolerance, or profile saved across reset. A steady
   turn can look as stable as rest on a single feedback wire.
2. With a clean signal, expect `READY BTN1:RECAL` after about **3 seconds**.
   For example, `J3 BG600 +/-15` means a learned idle of 600 pulses/s and an
   inclusive deadzone of 585–615 pulses/s. A cleaner wheel can learn a narrower
   band. While a wheel shows `WAIT STABLE`, its `RAW/s` is live, `NET/s` is `--`
   and it earns no counts. `WAIT J1: STILL` means J3 is already measuring and J1
   is still looking; the reverse applies for `WAIT J3: STILL`.
3. Turn a ready wheel by hand. `RAW/s` is the **unfiltered** rising-edge rate.
   `NET/s` is the signed change from that wheel’s idle over **200 ms**. Inside
   its learned band, the change is zero; outside, the **full** accepted change
   counts. `COUNT` integrates using actual elapsed time: a sustained +100 PPS
   adds about +20 edges per 200 ms. Counts can reverse through zero.

**How a wheel qualifies:** the monitor checks the latest five blocks (normally
600 ms each). It looks for a nonzero plateau: a majority must agree with the
median block rate, including an early block and the last two. Both the spread
of individual 200 ms readings and the change in block averages are checked;
large oscillations cannot qualify just by averaging to the same number. The
window spread limit is about 1/16 of the carrier; the block drift limit is
about 1/64, with allowances for edge resolution. Block centers must also agree
with their observed scatter. These are checks for stability, **not** the resulting
deadzone. If the signal has not settled, the monitor keeps checking new blocks.
A silent window or a poll gap of at least two rate windows during learning
requires fresh observations on that wheel.

The idle rate is total edges / actual time across agreeing blocks, rounded to
whole PPS. Each block contributes a candidate deadzone: its largest deviation
from idle in the **same windows used for `NET/s`**, or one edge per window plus
the difference between that block’s mean and idle, whichever is larger. The
second-largest candidate sets the symmetric band, so repeated jitter counts but
a lone extreme does not. The minimum is one edge per window (5 PPS at 200 ms).
If BG and DZ cannot both fit on the TFT row, they alternate. Calibration windows
never enter `COUNT`.

### If the stationary pulse rate briefly doubles

A high `RAW/s` with `NET/s 0 HOLD` means the bench saw extra edges but has **not**
added them to `COUNT`. If the rate settles back to idle, goes silent or reverses,
the held positive excess is discarded. A sustained high rate, **including twice
idle or more**, is accepted and the deferred edges are credited once. `NET/s`
then shows only the latest window’s rate, not the accumulated catch-up.

For example, with a learned baseline of **300 PPS**, steady idle gives 60 edges in
200 ms. One window of 120 edges displays `RAW/s 600` but adds **zero** if idle
resumes. Two fully observed windows at 600 PPS confirm a sustained rise: `NET/s`
is **+300** and `COUNT` has added **+120** in total. A drop to 200 PPS gives
`NET/s -100` in the next window and subtracts **20** edges. Each wheel is independent.

- The foreground takes short probes (nominally 50 ms) without filtering the IRQ
  counters. A probe at or above **1.5×** learned idle flags a possible high step.
  Ordinary steady increases can pass in the first 200 ms window. Mixed or high
  increases wait for six above-band probes or three consecutive above-band rate
  windows; this also covers a brief double split across a window boundary. A
  return to clearly moderate motion does not release a preceding unconfirmed high spike.
- The filter targets isolated doubles lasting up to **one 200 ms window** at
  rest. A genuine hand movement this short can be rejected too. Longer or
  recurring interference can resemble real motion, and a mostly corrupted
  feedback stream cannot establish a reliable idle. Recalibrate with the wheels still.
- To diagnose the source, leave motor-driver power disconnected and encoder
  logic powered. With both wheels still, compare the TFT’s `RAW/s` with rising
  edges on **PC8** and **PB15** using a scope or logic analyzer with logic ground.
  Both inputs count rising edges only. Check whether extra crossings are present
  at the pin (for example, closely spaced edges or ringing); the display alone
  cannot identify their cause. The 10 kHz drive PWM is off in this image.

**What the sign means:** this measures a **change in feedback frequency**, not
quadrature or independently sensed shaft direction. Check one wheel at a time
in two known directions. If both directions produce the same rate change, rising-edge
counts cannot distinguish clockwise from anticlockwise; that requires a
second phase/direction signal or a documented direction-encoded feedback output.
Missed pulses can imitate a negative rate change. Do not enable motor drive to test the sign.

- **Recalibration:** learned BG and deadzone are **locked**. They do not follow
  a steady turn, drift, signal loss, or reconnection. To replace them, stop
  both wheels and tap **BTN1** after at least one wheel is ready. Both profiles
  re-arm; already credited counts (including fractions) stay intact. The
  unfinished motion window and any unconfirmed rise are discarded. A held button
  or a press started while both wheels were waiting does not queue a re-arm.
  Reset clears the RAM profiles and net counts and starts automatic learning again.
- **Missing or unstable signal:** a wheel that has never qualified keeps waiting
  and can qualify later without BTN1. Check encoder logic power, common ground
  and feedback if `WAIT STABLE` persists. After lock, an entire window with no
  edges holds the count rather than reporting negative motion.
- **Timing settings:** [`encoder_bench_config.h`](Core/Inc/encoder_bench_config.h)
  sets only `ENCODER_NET_WINDOW_MS` (200) and `ENCODER_IDLE_SAMPLE_MS` (3000).
  The 3 s setting is an observation span, **not** a timeout that forces readiness.
- **Timing and limits:** the TFT refreshes every 100 ms, independently of rate
  sampling. Repeated reads cannot add the same window twice. Fractional signed
  edges carry between windows. Signed `COUNT`
  saturates at ±4,294,967,295 instead of flipping sign; a reversal moves it back
  from the limit. The live raw IRQ totals in `EncoderMonitor_Read()` remain unsigned,
  wrap at 2³², and are not cleared by BTN1. L2/R2 side-IR edges are excluded.

**Build only** from `ar-mainboard`, with the Arm GNU tools on `PATH` and
this board’s generated CubeMX support present:

```sh
cmake --preset EncoderBench
cmake --build --preset EncoderBench
```

With Arm GCC installed off `PATH`, use the same build-only step as the VS Code task:
`cmake -DAR_BUILD_PRESET=EncoderBench -P tools/build_release.cmake`. It can read a
custom compiler hint from `build/Release`; it does not rebuild or write there. You
can also pass `-DAR_GCC_BIN=...` before `-P`. The ELF/HEX/BIN are all named
`EncoderBench` under `build/EncoderBench/`. Do not reuse a drive build directory
or the historic `AR_ENCODER_BENCH` switch (that image could drive a wheel).
To return to F1/F2, select **Build + Flash AR MainBoard (Release)** with motor power
still disconnected. Do not select **Flash Existing AR MainBoard (Release)** when
you intend to load the passive bench.

## Digital side IR — the only inputs to `BR`

The calibration and drive screens show `L2:1/07 R2:0/02`. The first digit is
the **current electrical level** (1 = high, 0 = low); the two digits after `/`
are that pin’s **rising interrupts since reset, modulo 100**. PB4 = L2 / EXTI4,
PB6 = R2 / EXTI6; both are rising-edge inputs with **no internal pull**.
PB6 shares the EXTI9_5 vector with the J3 encoder on PC8, not its EXTI line.
HAL acknowledges each pending pin; the board callback counts side events and
passes encoder events to the unchanged reference driver. No LCD, ADC or motion
command runs in the side ISR.

**Both modules are active-high: 1 on black tape, 0 on clear floor.** With
motor-driver power disconnected, check each side at running height. A short
low-to-high pulse may increment the number after `/` even if the level is
already back to 0 at redraw. If the level floats or is inverted, check the
module and wiring; the analog calibration does not set side polarity.

`BR` is a separate, per-run count, **not the sum of the two IRQ totals**:

- At an accepted start, discard earlier pulses and read the live pins. If
  either is already high, ignore that starting crossing.
- Once armed, **L2 alone, R2 alone, or both together add one**. The board
  checks both levels on each run scan and retains the first serviced rise
  between scans, so a pulse need not last until the next ADC or TFT update.
- Holding either input high, or going `10 → 11 → 01`, is one crossing.
  Rearm only after **both have been clear for at least 75 ms**, measured
  from the first observation of `00`. A rise before 75 ms restarts the wait.

The three analog L/C/R sensors still supply `LINE`, steering, tape pickup
and finish. With L2/R2 clear, **no analog pattern adds to `BR`**.

## Operating the test

- Start with all motor power disconnected. Sensor supply can remain powered.
  After applying calibration, hold the three analog IR sensors at the operating height
  above the line. `I` shows the filtered left/centre/right ADC readings,
  `LINE` shows their `B` (black) or `W` (white) classification. Verify each
  one on tape and floor, particularly IR2. Check digital L2/R2 separately;
  the top row is their per-run crossing count, not a count of `B`s in `LINE`.
- US1 is PA1 trigger/PA0 echo. Verify its raw distance against a ruler and
  that the intended target is seen throughout the approach. Echo voltage must
  be MCU-safe; a 5 V module may require a divider or level shifter.
- Only after preflight, reconnect the motor power with the independent E-stop
  ready. **Tap and release B1 alone** to start the selected function once
  sensors are fresh and US1 is valid and at least
  60 mm. If no line is under the sensors, the screen shows `NO LINE B1 GO`;
  a start is still allowed. A button held at startup cannot start the run.
  **B2 alone** stops without switching functions.
  Software aborts are not a replacement for the independent E-stop.
- Unless the armed return finish qualifies, the **first valid US1 result below
  60 mm** in F1/F2 latches the ordinary turnaround. **Exactly 60 mm does not
  trigger it.** The controller continues polling IR, US1 and buttons:
  1. **Line-follow forward for 350 ms** (`TARGET APPROACH`), even if the reading
     then clears. Repeated near pings do not restart the timer. This replaces
     any remaining open-loop launch or F2 branch pivot with ordinary forward
     steering. Then command **zero on both wheels for 1 s** (`TARGET 60MM HOLD`),
     timed from the first zero command, not from the earlier ping;
  2. reverse straight back at **18%** for **500 ms** (`TARGET REV 18%`), with the
     IR ignored, including three black;
  3. pivot **anticlockwise at 11.25%** (left J3 backward, right J1 forward).
     For the first **1 s** (`TARGET SPIN CCW`), a tape hit cannot end the
     pivot. The IR sensors continue sampling; an all-white scan during this
     interval can establish that the robot has left the original tape;
  4. keep pivoting in the same direction (`TARGET FIND LINE`) until tape is
     reacquired. This requires **clear floor during this pivot**, then a new
     scan **started at or after 1 s** with black on any IR sensor. For a
     clear, all three raw and filtered values must be white; for a hit, a raw
     reading and its four-scan average must both be black on the same sensor.
     Pre-turn readings, a cached black result, or never leaving the original
     tape cannot count. If the line has not been reacquired by **10 s from the
     start of the pivot** (including the first 1 s), request zero on both
     wheels and latch `LINE NOT FOUND`; do not drive forward on a timeout;
  5. **after detection**, continue the same rotation for **450 ms** of
     alignment (`TARGET LINE GO`), then follow the line forwards (`TARGET FOLLOW`,
     `TARGET FOLLOW L/R`). The old fixed 3.9 s half-turn estimate is no longer
     used. Even the earliest pickup means **1.45 s** of pivoting before forward follow;
     the turn is not a guaranteed angle.

  **Return finish (F1 and F2): complete the anticlockwise turn and its 450 ms
  alignment → see a fresh filtered `111` (all black) on the forward return →
  stop when the current filtered mask is `000` (all white) AND the latest valid,
  fresh US1 distance is <55 mm** (`TARGET LINE END`). There is no 500 ms white
  dwell. At **55–59 mm**, even an armed white return starts the ordinary
  approach, not a finish. If white and a fresh **<55 mm** reading then agree
  before or on the hold decision, **finish immediately** instead of reversing.
  At 60 mm or farther, the finish cannot trigger; an already latched approach
  still runs to its deadline. A fresh return `111` may arm during the approach.
  Neither an outbound `111` nor a cached pivot/alignment scan can arm the stop.
  Black blocks the finish while present without erasing the marker. Each new
  return needs its own `111`. All-white following is straight except while F1 holds its `101` left correction.
  A qualifying finish still commands zero immediately, even during that correction.
  There is **no elapsed-run or return-leg timeout**. Even a late ordinary
  target gets the full 350 ms approach, 1 s hold and 500 ms reverse unless
  another stop applies. The 10 s `LINE NOT FOUND` limit applies only to the
  anticlockwise tape search. US1 distance cannot retrigger a hold while
  holding, reversing or pivoting; it can again during forward following. US1 errors, IR failures, stale data and BTN2 still
  request a stop while searching. Every new board run clears the standalone
  reverse-finish latch; `111` alone does not stop a fresh F1 or F2 run.
- BTN2 and invalid/stale sensor data can stop the wheels
  **during the 350 ms approach**; they do not wait for it. Outbound line loss
  alone is not a stop: do not rely on the clock to stop a robot that leaves
  the course. A terminal stop does not automatically restart the robot; use
  the displayed button hint. The key grabber is not controlled.

**60 mm is an approach trigger, not a stopping distance.** The wheels are still
commanded forward for 350 ms; 55 mm is the separate return-finish threshold.
Pings are at least 60 ms apart, and blocking measurements, display work,
processing, the set speed, motor response and inertia all add distance. Blind
spots, bad echoes, target shape and the sensor's minimum range may prevent a
valid measurement. The software cannot stop motion while the MCU is hung or
power/control hardware fails. Characterize the stopping distance safely with
an independent cutoff and adequate clearance; do not place a person or
valuable equipment in the test path.

## US1 diagnostics

The US1 screen row displays the current result, not a generic `INVALID`:

- `US1 ECHO HIGH`: echo was already high before trigger; check the echo pin,
  sensor power/ground and signal level.
- `US1 NO RISE`: no rising echo was seen before the timeout; check power,
  PA1 trigger/PA0 echo wiring, common ground, sensor direction and target.
- `US1 NO FALL`: echo rose but did not return low before the shared timeout;
  check the echo signal and target/range.
- `US1 SHORT PULSE`: echo was shorter than 100 microseconds; it is rejected.
- `US1 TIMER ERROR` or `US1 CHECK TIM2`: pulse timing or shared TIM2 startup
  failed. `US1 STALE` means the last valid ping is too old. `US1 WAIT PING`
  means a first measurement is pending.

If US1 fails during a run, the status row latches that error until another
start attempt, even if the current US1 row later recovers. Errors still
prevent starting and request motor stop; no last-good reading is substituted.
For diagnosis, leave motor power disconnected, check wiring and signal level,
and aim at a large flat target about 100–300 mm away. Record the displayed
error and sensor model. These messages indicate which stage failed, not a
proven cause. The software has not been validated on the hardware, and the
underlying cause of the reported `US1 INVALID` is not yet known.

## Logic

- IR inputs: left PA2, centre PC2, right PC3, sampled sequentially using
  ADC1. A rolling average of four complete scans at least 10 ms apart is used;
  motion is inhibited until the window is full and fresh. The averaging adds
  latency, and no hysteresis is currently applied.
- The `T` and `BLACK` rows carry over from setup. For each sensor separately, `H` means black **above** its T and `L` means black **below** its T. Equality is white in either case. The same rule supplies `LINE`, steering, tape pickup and finish, **not** `BR`. Weak or unrepresentative samples can still cause unexpected steering or line loss.
- **Motor polarity:** forward is defined per motor as `J3_FORWARD = -1` (J3
  at −22.5%) and `J1_FORWARD = +1` (J1 at **+22.5%**). J1 was inverted because
  it turned the wrong way; every J1 command, including the F2 pivot,
  derives its direction from that sign.
- **F1 forward steering** (outbound and return): read each four-scan **filtered**
  pattern in **left / centre / right** order, `1` = black. Wheel percentages
  below are physical left / right **commands**, not measured speeds.

  | Filtered L/C/R | Left / right PWM | F1 steering |
  |---|---:|---|
  | `101` | 9.9% / 23.85% | Start or keep the left correction. Does not add a crossing. |
  | Any other pattern except `010`, **while held** | 9.9% / 23.85% | Keep steering left, even on `001`, `000`, `011`, `110` or `111`. |
  | `010` | 22.5% / 22.5% | Release the correction and drive straight on this scan. |
  | `100` or `001`, **not held** | 9.9% / 23.85% or the reverse split | Steer left or right for this pattern only. |
  | `000`, `011`, `110` or `111`, **not held** | 22.5% / 22.5% | Drive straight. |

  `101 → 001 → 111 → 000 → 010 → 001` commands **left → left → left → left →
  straight → right**. Both trigger and release use the filtered `LINE` mask,
  not raw ADC or digital branch inputs. The same steering applies during the
  350 ms forward approach. A held correction has no timeout, but cannot delay
  a stop, reverse, pivot or finish. It is not carried through a turnaround or
  into a new run. The actual `111` on return still arms the `000` + US1 <55 mm
  finish, regardless of steering.
- **Launch:** an accepted B1 release commands both wheels forward at 13.5% (J3 −13.5%,
  J1 +13.5%) for **750 ms**, ignoring the line pattern. The screen shows
  `LAUNCH 13.5%`. After 750 ms normal line following resumes in the same run. US1
  invalid/stale readings, BTN2 and the physical E-stop still apply during
  launch. A valid US1 <60 mm preempts launch with the timed approach. The launch is a fixed time-based command, so the distance
  travelled depends on the surface, battery and mechanism, and it is not
  corrected by the sensors.
- **`111` (all three analog sensors black) is not by itself a crossing or finish on the board.**
  The portable F1 controller supports an armed 0.2 s reverse finish if a caller
  reuses its state after a US1 hold. The board explicitly reinitializes it on
  **every accepted B1 start**, even if F1 is run twice without switching.
- **Crossings and line loss:** F1 never pivots because of `BR`. Without a held
  split, `011`, `110` and `111` show `BRANCH STRAIGHT` during outbound following: this describes
  the wheel command, not a digital hit. All-white outbound shows `NO LINE STRAIGHT`
  unless the `101` correction is still active (`RUN LEFT`). Neither guarantees
  the robot is on course; abort if it leaves the tape.
- `F1 LINE BR n` records digital crossings throughout the active run, including
  launch and the return. The same physical junction can count again after a
  clear interval. The last count survives a stop or rejected start. A fresh run starts
  from zero; F2 has its own outbound count.
- IR conversion failure or out-of-range ADC value stops an active run and
  clears its averaging window. US1 echo/timer errors stop the run. Newest IR
  scan older than 50 ms, oldest scan older than 200 ms, or US1 ping older than
  150 ms also stops on the next foreground iteration. These are cooperative
  checks, **not** a watchdog or hardware interlock; blocking or a hung loop
  can delay or prevent a software stop.
- TIM2 is the shared 1 MHz counter started by `US2_Init`, which also configures
  US2 GPIO but does not ping US2. US2 uses PB11/PB10; US3 uses PB9/PB8. The
  reference ELCP files remain unchanged; the old rotation-run and estimate
  sources are excluded from the active CMake source list.

## F2 — right turn at the third crossing

Select F2 directly after calibration or after any run. Every F2 start begins with a fresh branch count. Its forward
following remains per-scan: 9.9% inside / 23.85% outside on a side-only
reading, otherwise straight. In ordinary F2 following, `101` commands 22.5% / 22.5%;
it does not store a left turn. The third-crossing pivot still takes priority.

1. The first two distinct L2/R2 crossings count without overriding normal
   outbound steering. The third requests a **clockwise right pivot**
   (`F2 TURN RIGHT`): left **J3 forward 15.75%**, right **J1 reverse 9.45%**.
   The commanded PWM magnitudes keep the **5:3** ratio (warmaster 4566284). With the configured
   polarity the signed motor API commands are **J3 −15.75%, J1 −9.45%**;
   these are not measured wheel speeds.
2. To **end the pivot and resume following**, F2 needs both:
   - **Junction clearance:** at most one of the three filtered analog L/C/R
     sensors is black for at least **75 ms**. Two or more black readings
     together reset an incomplete clearance interval.
   - **Centre pickup:** on a scan after the first pivot command, the filtered
     centre must be **white**, then **black** on a later scan. Centre black
     can end the pivot on the scan that completes clearance, or afterwards;
     it does not have to be the only black sensor. A centre that never left
     black, or tape found on a side sensor alone, cannot end the turn.

   `F2 FIND LINE` means clearance has qualified but centre pickup is still
   missing. White seen during launch doesn't count. This clockwise pivot has
   no elapsed cutoff; use BTN2 if the robot misses the exit tape. The separate
   anticlockwise US1 tape search is unchanged.
3. Later distinct crossings count but do not turn. The turn is available
   again on a fresh run of F2. If crossing 3 happens
   during the 750 ms launch, the turn waits until launch ends. F2 does not
   process crossings during the US1 turnaround or return. BTN2, sensor
   faults and stale readings still stop the motors during the pivot. The US1
   **anticlockwise** turnaround has the same priority as in F1.

## F3 — the top-right key (sim only, not field-tested)

Third menu entry (`F3 TOP RIGHT 0`): the key at the right wall, top right of the field. It runs on F1's code
(`F1_StartF3` in `function1.c`, numbers `F3_*` in `function1.h`); F1 and F2 are unchanged. Route, on the field photo:

1. **Out along the bottom line**, then from `F3_KEEP_FROM_MM` (300) **keep right**: the U-curve leaves the bottom line on
   the right ~500 mm out, so the AR takes it. Round the U it steers sharper than the normal follower (only the right IR on
   the tape: 3 % / 24 % before the 0.9 scale): the U's ~200 mm radius is tighter than the follower's hard turn (~280 mm).
2. **Along the U's top straight to the V** (where the diagonal from the long vertical line comes in from the left and the
   short leg leaves up-right). Still keeping right, so the diagonal is ignored. The **outer-left IR** touching the diagonal
   (~142 mm before the V) fixes where the V is; it drives on to `F3_CORNER_ADVANCE_MM` (110) past the V, then **pivots
   LEFT** until the centre IR finds the short up-right leg (10-105 deg, ~60 expected; the diagonal would be ~150).
3. **Up the short leg** (keeping left) to its **bend**: the outer-right IR crossing the east leg (or the centre + left IR
   off the tape for `F3_CORNER_CONFIRM_MM`, 65 mm) = the bend (the outer-right IR must stay high `F3_SIDE_CONFIRM_MM`, 6 mm:
   one noisy sample is not it); advance, **pivot RIGHT** onto the east leg (10-105 deg). Already turned `F3_ON_EAST_DEG`
   (35) right of the leg's averaged heading = the follower took the bend itself: no pivot, straight on as the east leg.
   A bend pivot with no tape by 105 deg turns back LEFT (`F3: BEND BACK`) onto the tape it left and tries the bend again,
   once; then `F3 BEND NO TAPE`.
4. **East to the key**: US1 only counts on this last leg (a box stands by the bend). Approach, 1 s hold, as F1.
5. **Home:** back off `F3_BACK_MM` (60), pivot CCW until the centre IR finds the **short leg** (150-290 deg, ~225-240:
   the bend is too close behind to find the east leg at 180), down it keeping left. At the V (outer-right IR on the
   diagonal) advance and **pivot RIGHT** onto the U's top straight (the first tape: ~60 deg; the diagonal ~85; stops
   at 78). Its heading is averaged there; + 180 = the bottom line's direction into the zone.
6. **Keep left round the U**, onto the bottom line heading home, then home like F1: outline, zone shift, wall.

Pivots happen over the corner (advance ~ the IRs' distance ahead of the axle), so the next leg is picked up nearly along
it: with F2's 80 mm the AR was ~17 deg short and the follower could lose the leg in the gap between two IRs.

**Stops (each leg has a limit, so a missed corner stops it instead of wandering off):**

| Status | Meaning |
|---|---|
| `F3 NO U` | kept right for `F3_U_MAX_MM` (1.4 m) without getting round the U |
| `F3 NO V` / `F3 NO BEND` / `HOME NO V` | a straight leg ran `F3_LEG_MAX_MM` (700 mm) without its corner |
| `F3 V NO TAPE` / `F3 BEND NO TAPE` / `HOME V NO TAPE` | a corner pivot passed its window with no tape |
| `F3 ADV TIMEOUT` / `F3 TURN TIMEOUT` | a corner advance / pivot overran 3 s / 6 s (dead encoder) |
| `HOME NO U` | home: 1.4 m past the V without getting round the U |
| `TURN NO LINE` | the home pivot passed 290 deg with no tape |

**Row 8 on F3:** `B214S  65  P 231` = the bend was seen 214 mm up the short leg, by `S` the outer-right IR / `W` the
centre + left going white / `E` already turned onto the east leg; its pivot turned 65 deg (`R` after it = turned back and
retried); `P` = the home pivot (as F1's `PIVOT`). `B  0-` = no bend yet.

**Tested only in the closed-loop sim** (`tests/test_f3_home.c`, the map measured off the field photo at ~0.78 mm/px):
147 runs pass - IRs 100/115/130 mm ahead, left motor 15 % weaker in reverse, IR knobs +-4 mm, start +-3 deg, the
whole map scaled 0.93 / 1.00 / 1.07, the measured skewed IR bar, US1 missing the wall, dead encoders, faults on the short leg (a
one-sample glitch of the outer-right IR, 40 mm with no tape, the outer-right IR blind, the bend taped round). Sensitive:
`F3_BACK_MM` 50 or 65 fails a few runs (the home pivot's start point), and at IRs 100 mm ahead with +-4 mm knobs the
home V is the weak spot. Things to adjust on the floor, by the message it stops with: `HOME V NO TAPE` or it follows the
diagonal home -> `F3_BACK_MM`; `F3 V NO TAPE` / `F3 BEND NO TAPE` -> `F3_CORNER_ADVANCE_MM` /
`F3_V_SIDE_ADVANCE_MM`; `F3 NO U` / it leaves the U -> `F3_SHARP_INSIDE` / `F3_SHARP_OUTSIDE`.

The legacy build (`F1_LEGACY 1`, the pinned host replays) keeps the old dormant generic right-branch policy
(`function3.c`, [design](../../docs/AR-F3-right-branch.md)) behind `DRIVE_F3`, still off the menu there.

## F4 — the bottom-left key, below the 8 (sim only, not field-tested)

Fourth menu entry (`F4 BOT LEFT 0`). Runs on F1's code (`F1_StartF4` in `function1.c`, numbers `F4_*` in
`function1.h`); F1, F2 and F3 are unchanged. The thin black lines on the map are walls, so it does **not** follow tape
past the F1 stone: it stops by US1 at the stone and drives measured encoder legs, with two fixes on the way.

1. **Out along the bottom line** as F1 (`F4: LINE`), with a **turn calibration** first (`F4: CALIBRATE`, leg 23): at
   350 mm out, and at least 250 mm past where the front IRs crossed the start zone's outline (all three black), it stops
   and spins a full turn right on the spot. The centre IR's four tape edges - leaves the line, crosses it behind (in,
   out), finds it in front - give exactly 360 deg whatever the axle's offset from the line (and the IR filter delay
   cancels); every later turn is scaled by that. A scale within 1.5 % of 1 is kept at 1 (the measurement scatters ~1 %).
   The line heading is averaged only while the centre IR alone is on the tape, from 100 mm after the calibration. **US1 <= 250 mm at the F1 stone** (was 150: more room for the turn) (and >= 1000 mm travelled) -> stop 300 ms and measure
   the coast (`F4: AT STONE`).
2. **Right 90, north 266.5 mm; left 90, west** (`F4: NORTH`, `F4: WEST`). The west leg ends on **US1 = 169 mm from the
   barrier face** (fix 1: W is measured, not dead-reckoned). Any reading of the face from 100 mm into the leg counts,
   wherever the encoders put it (a roughly placed stone moves the barrier earlier / later); never seen = 300 mm past the
   encoders' estimate.
3. **Right 64.1, 415 mm diagonally through the gap** (`F4: THRU GAP`) - ~38 mm clearance per side.
4. **Left 64.1, west to the 8** (`F4: TO THE 8`). Fix 2: the first tape crossing (>= 2 of L/C/R rising within 15 mm)
   from 385 mm on is the 8's west edge; it drives 125 mm past it.
5. **Left 90, south to the key** (`F4: TO KEY`). US1 approach and 1 s hold as F1 (US1 only counts on this leg). A stall
   after 300 mm on this leg counts as at the key.
6. **Home:** back off 100 mm, **reverse north** to the 8's row (no 180 pivot), then turn and drive **the out legs
   backwards, each for the length it actually drove out** (east, through the gap, east), so a wheel-size error cancels
   on distances. Right 90, south until the bottom line (all three IRs, or >= 2 with both outer ones, from 70 mm) + 125 mm,
   pivot left until the centre IR finds the line (45-135 deg), then home like F1 (zone, wall).

Every leg starts with a 250 ms stop (the encoders take direction from the command, so changing direction while still
rolling miscounts). Pivots keep both wheels' pulse counts matched. **During an F4 run the pulses at 0 % count in each
wheel's last commanded direction** (`SideIR_CountCoast`, `side_ir.c`): the board used to drop them, so every stop's
roll-on - most of all after a pivot - was never counted and the turns came out bigger than F4 thought (the first field run:
at an angle to the barrier, then into it). F1-F3 still drop them, as before. A missing US1 echo on F4's own legs (off the
tape) no longer stops the run (`US1 NO FALL` / `NO RISE`), and neither does a stale US1 (no valid reading for 150 ms: it stopped the first calibration run with `SENSOR STALE`), including during the calibration spin; the leg limits and `F4 STALLED` do. The line out to the stone still needs a fresh US1. If F4's turns now come out
**short**, `F1_TRACK_MM` (233, tuned on F1's 180 with the roll-on uncounted) is the number to raise for F4.

**Stops:**

| Status | Meaning |
|---|---|
| `F4 CAL BAD` | the turn calibration's scale is outside 0.6-1.4, or the line behind was wider than 40 deg |
| `F4 CAL NO LINE` | the calibration turned 700 encoder deg without all four tape edges |
| `F4 NO STONE` | > 2.1 m out without US1 seeing the stone |
| `F4 US1 BLOCKED` | US1 < 70 mm on one of F4's straights (not the key leg): something ahead, stopped short of it |
| `F4 OFF LINE` | out to the stone, the heading went > 25 deg off the bottom line's (it took a curve: the U and the C leave the line tangentially) |
| `F4 NO 8` | 640 mm west without crossing the 8 |
| `F4 NO KEY` | > 800 mm on the key leg |
| `F4 HOME NO LINE` | 300 mm south without the bottom line, or the last pivot passed 135 deg |
| `F4 STALLED` | both encoders still for 700 ms while a leg drives (pushing on a wall, or a dead encoder) |
| `F4 TURN TIMEOUT` / `F4 LEG TIMEOUT` | a pivot / straight overran its time (encoder; the calibration turn: 20 s) |

**Row 8 on F4:** `U241K103D  -4L 9`:
- `U` = US1 reading at the stone stop;
- `K` = the measured turn scale x100 (0 = not measured yet; 100 = the encoders' turns are right; ~300 would mean the
  wheels slip 3x when pivoting);
- `D` = its heading off the bottom line's at the stone, degrees (should be a few; large while it sits straight on the
  line = the encoder heading drifts while following the line);
- `L` = the leg it was on (0 = line ... 11 = key, 12-22 = home, 23 = calibrating).
The tape crossings are no longer on the screen. Write it down after every F4 run, with the row-5 message.

**Tested only in the closed-loop sim** (`tests/test_f4_home.c`, the map off the field image, thin lines as walls, the
AR as a rectangle): 135 runs pass - map scaled 0.97 / 1.00 / 1.03, IRs 100 / 115 / 130 mm ahead, a wheel 10 % weaker in
reverse, IR knobs +-4 mm, start +-3 deg, encoders +-1 %, the skewed IR bar, US1 blind to the zone wall going home, the stone moved +-100 / +-200 mm along the line, a box on the west leg (`F4 US1 BLOCKED`), frozen encoders (`F4 STALLED`). Rolling on 3x / 6x further after each stop: all pass; the same with the old counting: all hit a wall (the sim's own roll-on is small, ~5 mm, which is why it missed this before the field run).
**Tight:** the closest the AR gets to a wall in the sim is 4 mm. Harsher (reported only): a wheel 15 % weaker in reverse
or encoders +-2 % -> 6 of 66 runs touch a wall, mostly on the home gap. Stone stop at 250 mm. **Turns:** without the calibration, 1.5 % off (a 90 comes out ~88.7 / ~91.4) passed and 3 % off
(~87.4 / ~92.8) all failed - the gap is the tight spot; with it, turns 1.5 / 3 / 6 % off all pass. The report-only runs
(harsher, turn errors, roll-on) run only with `F4_FULL=1`; the normal suite takes ~26 s. So the wheel size / track width
(`ODO_*` calibration) must be right; the IRs' distance ahead of the axle shifts the key stop by up to ~15 mm. US3 is not
used.

## Verification

Run `sh ar-mainboard/tests/run_host_tests.sh` using GCC, GNU ld and
Python 3. The host suite runs the real F1/F2 selector through mocked board
I/O, including button captures and the apply chord on each simulated boot.
It varies L2/R2 independently of all eight L/C/R masks and checks held starts,
overlaps, between-scan pulses and the 74/75 ms rearm boundary. It also checks
mixed-polarity thresholds against the TFT and wheel commands, failed captures,
staggered button presses/releases, repeated runs and selection in both directions,
B2 during ADC and US1 polls, both branch-3 wheel commands,
centre loss/pickup versus held-centre and side-only tape,
the 59/60 mm threshold, live steering through 349 ms and zero at 350 ms,
late US1 targets and return finishes past 10 s, the independent lost-tape pivot
cutoff, finish/fault priority during approach and no production F3 calls.
Separate host-only replays pin individual routes for timing/fault regression. Their linker probes are not in
the firmware. The portable F1 controller and stationary IR screen also have
independent tests.

The passive bench replay drives the real EXTI vectors with synthetic pulses and verifies
hands-free startup, BTN1 debounce/re-arm, the learned BG and band on the TFT, and low PWM pins before display startup.
Rate fixtures exercise clean versus noisy stationary signals, within-block oscillation, drift,
partial readiness, a missing/returning carrier and locked profiles. They sweep transient doubles
across window boundaries and check 200 ms signed rates, independent confirmation and catch-up,
delayed polls, tick wrap, fractional reversals and signed count limits.

To test both build tasks (Python 3, host GCC, CMake 3.22+ and Ninja), run
`python3 ar-mainboard/tests/test_build_release.py` from the repository root.
It uses temporary projects and does not invoke the programmer.

Host mocks do not verify stopping distance, sensor accuracy, wheel direction,
or STM32 timing. Generated CMake/toolchain files, HAL/CMSIS drivers, startup
assembly and an ARM compiler must be supplied locally to build the board target.
