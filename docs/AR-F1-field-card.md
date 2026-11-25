# AR field card — select and run F1, F2, F3 or F4

**RDC Group 1 · AR mainboard · `warmaster726` · boot calibration · B1 run · B2 stop · B1+B2 select · F3 = top-right key, F4 = bottom-left key (sim only)**

Use the normal AR Release firmware; no calibration preset or second image.

**Digital line IRs (current firmware):** after reset the TFT says **`AR SETUP`** and shows the live `FRONT L0 C1 R0`, `SIDE L0 R0` and the `R WHEEL` / `L WHEEL` encoder counts; the bottom row `MOTORS HELD AT 0` is a reminder that this page sends 0 power (it does not sense motor power). With motor-driver power disconnected, set each module's knob at running height so it reads **1 on black, 0 on the floor**. Turn each wheel by hand: its number must go **up** (either direction; by hand it reads several hundred per turn, which is fine). If a count never moves, retry with motor power on and the wheels lifted (the page holds 0 PWM); still stuck = encoder wiring (FB: J1 PB15 = TIM12_CH2, J3 PC8 = TIM8_CH3, timer input capture with a hardware spike filter). **With motor power on and the wheels still, both counts must stay put**; if one climbs on its own, nudge that wheel a few mm: if it stops, the sensor was parked on a slot edge (harmless, ignored while stopped); if it keeps climbing, the FB line is picking up noise (fix the wiring before running F1). **Powered wheel check (robot lifted):** hold **B1 alone** for 1 s and the left wheel turns; **B2 alone** = right wheel; let go to stop. The bottom row shows the pulses of that hold (`L SPIN` / `R LAST`). Mark the wheel, hold for exactly 10 turns: pulses / 10 = pulses per turn; ours read 1150 / 1155 for 10 turns (115 per turn, what the firmware uses). Then **hold B1+B2 together and release both** → `F1 LINE BR 0`, stopped. No white/black capture. (Steps 1-3 below are the old analog calibration, used only with `IR_DIGITAL 0`.)

**New stop messages (encoder safety, sim only):** `R NO PULSES` / `L NO PULSES` / `NO PULSES L+R` = a driven wheel gave no encoder pulses for 1 s (encoder dead or unplugged, or the wheel stalled). `F2 ADV TIMEOUT`, `F2 TURN TIMEOUT`, `HOME T TIMEOUT`, `HOME BY TIME` = a move that ends on encoder distance or angle overran its time limit. If one appears, check that wheel on the `AR SETUP` page (turn it by hand: the number must go up).

**Analog line IRs (`IR_DIGITAL 0` only): after hardware reset the TFT must say `IR CAL 100 SAMP`.** With motor-driver power disconnected:

1. All three **analog L/C/R** sensors over white → press and release **B1 alone**. Wait for `W`.
2. All three over black → press and release **B2 alone**. Wait for `CAL READY` and check `T` and `BLACK` (L/C/R).
3. **Hold B1+B2 together, then release both** → `F1 LINE BR 0`, stopped. This applies the displayed T values and H/L polarities without recapturing. A rejected chord changes neither surface. Reset requires fresh calibration; BTN2 in drive mode keeps it.

During setup both wheels are commanded to **0% PWM on every pass**; keep motor-driver power disconnected. `L2` (PB4) and `R2` (PB6) show **raw 0/1 / rising-edge count** (last two digits). Check each side reads **1 on black, 0 on floor**. Only these two digital sensors count branches during a run. The numbers after `/` are independent IRQ totals since reset, not `BR`. The analog `T` and `LINE` rows still have just three channels.

## Buttons after calibration

The calibration apply gesture leaves **F1 selected, motors stopped**. It does not also switch to F2. Read the top row (`F1 LINE BR n` or `F2 LINE BR n`) before starting.

| Gesture | Result |
|---|---|
| **B1 alone: tap and release** | Run the displayed function from the beginning if the sensors are ready. Holding B1 does not start motion. |
| **B2 alone: press** | Stop the current run as soon as B2 is read. Keep the selection, last count and calibration. Releasing B2 does not start anything. |
| **B1+B2: hold together, then release both** | Cycle **F1 → F2 → F3 → F4 → F1**, once. If running, B2 stops the old function first; on release the new function is selected and stationary. Use a **separate B1 tap** to run it. |

The button hints are `B1 = RUN` when stopped, `B2 = STOP` when running, and `B1+B2 = TO F2` / `TO F3` / `TO F4` / `TO F1`. A run, finish, fault or rejected start never changes the selection. You can repeat either function; each accepted run clears its branch count and finish state.

**F3 = the top-right key (sim only, not field-tested yet).** Top row `F3 TOP RIGHT 0`. Start it in the start zone on the bottom line, as F1. Route: out along the bottom line, right onto the U curve, round it, along the U's top straight to the V (where the diagonal comes in), LEFT up the short leg, RIGHT at its bend, east to the key; home: back off, turn left onto the short leg, down to the V, RIGHT onto the U's top straight, round the U, home along the bottom line into the zone. Watch row 5 (`F3: U RIGHT`, `F3: TO THE V`, `F3: PAST CORNER`, `F3: V LEFT`, `F3: UP`, `F3: BEND RIGHT`, then the target, `HOME: DOWN`, `HOME: V RIGHT`, `HOME: U LEFT`, `HOME: INTO ZONE`). If it stops, note the message and where: `F3 NO U`, `F3 NO V`, `F3 NO BEND`, `HOME NO V`, `HOME NO U` (a corner was missed), `F3 V NO TAPE`, `F3 BEND NO TAPE`, `HOME V NO TAPE` (a corner pivot found no tape), `F3 ADV TIMEOUT` / `F3 TURN TIMEOUT` (encoder), `TURN NO LINE` (home pivot). Row 8 on F3 reads like `B214S  65  P 231`: write it down after every F3 run (the bend: mm up the short leg when seen, S / W / E = how, its pivot in degrees, R = turned back and retried; P = the home pivot). `F3: BEND BACK` on row 5 = the bend pivot found nothing and it is turning back to try again.

**F4 = the bottom-left key below the 8 (sim only, not field-tested yet).** Top row `F4 BOT LEFT 0`. Start in the start zone on the bottom line, as F1. Route: out along the bottom line, a full turn right on the spot ~350 mm out (`F4: CALIBRATE`: it measures its turns against the tape - normal), stop by US1 250 mm before the F1 stone, north, west to the barrier (US1), diagonally through the gap, west to the 8 (crosses its tape), south to the key; home: back off, reverse north, retrace east through the gap, south to the bottom line, left onto it, home into the zone. Every leg starts with a short stop - that is normal. Row 5 shows the leg (`F4: LINE`, `F4: CALIBRATE`, `F4: AT STONE`, `F4: NORTH`, `F4: WEST`, `F4: THRU GAP`, `F4: TO THE 8`, `F4: TO KEY`, `HOME: BACK NORTH`, `HOME: EAST`, `HOME: THRU GAP`, `HOME: TO LINE`, `HOME: FIND LINE`, ...). Stop messages: `F4 CAL BAD` / `F4 CAL NO LINE` (the calibration turn failed), `F4 NO STONE`, `F4 OFF LINE` (took a curve before the stone), `F4 US1 BLOCKED` (something < 70 mm ahead on a straight), `F4 NO 8`, `F4 NO KEY`, `F4 HOME NO LINE`, `F4 STALLED` (pushing a wall / dead encoder), `F4 TURN TIMEOUT`, `F4 LEG TIMEOUT`, `SENSOR STALE` on the line out (US1 lost its view along the line to the stone; it is ignored on F4's own legs). The gap is tight (~38 mm a side): watch it on the first runs, hand on the E-stop. The turns must be close to exact (within ~1.5 deg per 90 in the sim): note if a turn looks short or long. Row 8 on F4 reads like `U241K103D  -4L 9`: write it down after every run - **especially K and D** (U = US1 at the stone stop; K = the measured turn scale x100, 100 = right, 0 = not measured; D = its heading off the line at the stone in degrees, should be a few - note whether the robot actually sat straight on the line; L = the leg where it stopped, 23 = calibrating).

## Field procedure

All F1/F2 movement commands are **10% lower** than the previous profile. Wheel ratios, manoeuvre times and sensor thresholds are unchanged. Check that the slow wheel still turns on the actual floor; the timed reverse may travel less far, and the timed alignment may turn through a smaller angle.

1. **Power and sensors.** Disconnect motor-driver power before flashing, wiring or moving tape beneath the sensors. Leave the logic and sensors powered. At operating height, check that the left, centre and right readings each switch `W`/`B` on floor/tape (`H`: black above its T; `L`: black below; equality is white). Confirm US1 shows a plausible distance to a target. Clear the test area and put an operator on the independent motor-power cutoff before enabling drive power.
2. **Start.** Align the centre sensor over the tape. With fresh IR and a valid **US1 ≥60 mm**, check the selected function, then **tap and release B1 alone**. The count resets on an *accepted* start. Neither outbound nor return stops just because 10 s elapse. The robot launches straight at **13.5% on both wheels for 750 ms**, without line steering.
3. **F1 follow (outbound, target approach and return).** Read filtered analog **L/C/R** (`1` = black). **`101` holds left** (left wheel **9.9%**, right **23.85%**, both forward) **until exactly `010`**, which commands straight at **22.5% / 22.5%** on that scan. All other patterns keep the held left, even `001`, `000` or `111`. Example: `101 → 001 → 111 → 000 → 010 → 001` gives **left → left → left → left → straight → right**. Without a held split, `100` and `001` steer left and right for the current scan only; `000`, `011`, `110` and `111` go straight. Stops, the US1 hold/reverse/pivot and the return finish still take priority. Only L2/R2 count branches.
4. **At the US1 target.** On the first valid **<60 mm** reading, expect `TARGET APPROACH`: **continue line-following forward for 0.35 s**, then `TARGET 60MM HOLD`: both wheels stopped for **1 s**. A later clear ping does not cancel the approach. Allow clearance for this extra travel. The robot then reverses straight at 18% for 500 ms and pivots anticlockwise at 11.25%. It must see clear floor, then fresh black at or after 1 s of pivot, followed by 450 ms more to align. No pickup within 10 s of starting the pivot → `LINE NOT FOUND`.
5. **Return finish (F1 and F2).** After alignment, fresh filtered `111` arms the end. **Stop immediately (`TARGET LINE END`) on `000` plus fresh valid US1 <55 mm**. At 55–59 mm the ordinary approach starts; if the finish qualifies during those 350 ms, it wins. Black blocks the finish without clearing the marker. A new return needs its own `111`. BTN2 and sensor-fault stops do not wait for the approach. A finish leaves the same function selected.
6. **F2.** Select `F2 LINE BR 0` with B1+B2, then tap and release B1. F1 need not run first. Count distinct outbound crossings from zero. At crossing **3** it first drives **80 mm straight past the T** (`F2 PAST THE T`), then (`F2 TURN RIGHT`) the left wheel drives **forward 15.75%**, the right **reverse 9.45%** (5:3). To end the pivot, F2 needs **both** (a) at least 75 ms with at most one filtered analog sensor black and (b) a filtered **centre-white → centre-black** pickup during the commanded turn. A centre that stays on the old black tape cannot finish. Side-only tape or a digital L2/R2 hit cannot end the pivot either. On pickup, the robot **resumes line following**; it does not stop the run. This turn happens only once. Ordinary steering is per scan, **9.9% / 23.85%** inside/outside for side-only tape, **22.5% / 22.5%** otherwise. The US1 stop/turnaround and interlocks still take priority. This clockwise branch turn has no elapsed cutoff; use BTN2 if it misses the tape. A new B1 tap reruns F2 from branch count zero.

## What the counter measures

`F1 LINE BR n` counts **digital tape entries during this run**, not unique map junctions. Either **L2 or R2** on black adds one. A held hit, both at once, or one joining the other still adds only one. A mark underneath either side at an accepted start is ignored. After a hit or held start, **both must read 0 for at least 75 ms** before the next crossing. A serviced pulse can count even if it ends before the next scan or redraw.

F1 counts during launch and return as well as outbound; it never turns on the count. The last count stays visible after B2, any other stop or a rejected start. Cycling shows the newly selected function at zero; B1 starts a fresh tally. `BRANCH STRAIGHT` describes analog steering, not a counter event.

### Quick counter check — motor-driver power stays disconnected

Power the logic/sensors only. Keep US1 valid and at least 60 mm away throughout the counter check. With F1 selected and L2/R2 on floor, tap and release B1 on `WBW` (analog left/centre/right). After launch, move tape under the stationary sensors. Allow about 0.2 s per step for the display; there is no need to race the clock.

| Move the tape | L2 R2 | F1 `BR` |
|---|---|---:|
| Cover analog left and right only (`BWB`), then all three (`BBB`) | `0 0` | 0 throughout |
| Cover L2 alone; keep analog `BBB` | `1 0` | 1 |
| Cover R2 before uncovering L2 | `1 1` → `0 1` | Still 1 |
| Clear both for 0.2 s | `0 0` | Still 1 |
| Cover R2 alone | `0 1` | 2; no count-triggered F1 turn |

**F1 steering check — motor power still disconnected.** In forward follow with US1 clear, move the tape so the TFT `LINE` row shows `BWB → WWB → BBB → WBW → WWB` (allow for the four-scan average). Expect `RUN LEFT → RUN LEFT → RUN LEFT → RUN STRAIGHT → RUN RIGHT`. With L2/R2 clear, `BR` should not change. If the centre-only pattern never appears on the course, use B2 or the independent cutoff; there is no steering timeout.

**F2 bench check — motor power still disconnected.** No need to complete F1 or present a US1 target to select F2.

1. From F1, hold **B1+B2** together. If F1 is running, B2 stops it (`STOP BTN2`). Release both: confirm `F2 LINE BR 0` / `B1 = RUN`, with no launch.
2. Keep US1 valid and at least 60 mm, clear both side sensors, then tap and release **B1 alone**. Repeat the table above. R2 at the last step should give `F2 LINE BR 2` with no turn. Clear both sides for 0.2 s, then cover either side again: expect `F2 LINE BR 3` / `F2 TURN RIGHT`. Keep analog `BBB` until you see it.
3. Clear L2/R2. Keep the centre continuously on black as you uncover the two outer analog sensors (`WBW`). Hold for 0.2 s: expect `F2 FIND LINE`, still pivoting because the centre has not seen white.
4. Show right-only tape (`WWB`) for 0.2 s: still pivoting. Return to centre-only (`WBW`): expect `RUN STRAIGHT` at `BR 3`.
5. Press **B2 alone**: expect `STOP BTN2` and **F2 still selected**. To select F1 without moving, release B2 and use a fresh B1+B2 gesture. Finish checking before reconnecting motor power.

[F1 controller](../firmware/ar-mainboard/Core/Src/function1.c) · [F2 turn](../firmware/ar-mainboard/Core/Src/function2.c) · [Board I/O and thresholds](../firmware/ar-mainboard/Core/Src/calibration_app.c) · [Crossing detector](../firmware/ar-mainboard/Core/Inc/branch_latch.h) · [Full board guide](../firmware/ar-mainboard/README.md)
