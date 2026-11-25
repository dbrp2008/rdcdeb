# F3 — right-branch line following with a terminal US1 stop

**Dormant in the combined AR drive firmware.** After boot IR calibration, the production [selector](../firmware/ar-mainboard/Core/Src/drive_plan.c) offers **F1 and F2 only**. B1+B2 changes the selection; a separate B1 tap runs it. F3 is compiled and linked but is not initialized, given sensor readings, or allowed to command the motors. There is no F3 hardware preset, build flag, button, or automatic transition. On the F1 outbound leg, a valid US1 reading below 60 mm starts the **350 ms F1 approach, then hold/reverse**, not F3's terminal stop.

## Retained behavior (host simulation, not the active robot)

- **Start (host replay):** tap and release BTN1 with fresh IR/US1, a valid distance **at least 40 mm**, and tape confirmed on at least one sensor. Start straight on a line or at a junction; the first command already follows the sensors. There is **no 750 ms open-loop launch**. The same boot-calibrated T and polarity as F1 apply to each sensor: `H` is black above T, `L` below T, equality white.
- **Follow:** use the newest complete IR scan and its four-scan average. A sensor supports a choice only if both say black on that sensor. Patterns below are **left / centre / right** during ordinary tracking; `B` means supported black. Speeds are physical-direction PWM commands, not measured RPM.

| Evidence | Decision |
|---|---|
| `WBW` or `BBW` | Follow the centre/through line, 25% on both wheels. A left branch alone is not a right turn. |
| `BWW` or `WWB` | Follow the single visible line left or right: 11% inside, 26.5% outside, both forward. A right-only reading alone is not a fork. |
| `WBB`, `BWB` or `BBB` | Select the right branch. This also applies at the starting position, at later junctions, or if right becomes visible after the left side of the same junction. |
| Latest raw scan all white | Request zero immediately on that scan; latch `F3 LINE LOST`. No timed forward search or blind pivot. |
| Raw tape present, but no sensor agrees with the filter | Hold both wheels at zero (`F3 WAIT TAPE`). Resume only when a sensor agrees; **50 ms** of disagreement latches `F3 LINE UNSTABLE`. |

**Taking the exit:** `F3 TURN RIGHT` requests a clockwise pivot (left J3 **15% forward**, right J1 **12% reverse**). It tracks IR throughout. Do not declare success just because the old straight line is visible: the right sensor must first see **right-only tape in both raw and filtered scans**. The robot then steers along that tape. **Centre-only in both for at least 20 ms across fresh scans** completes the turn and rearms right selection. Repeated right evidence at the same junction before confirmation is one selection, not several. A turn still unresolved **2.5 s from selection** stops as `F3 TURN TIMEOUT`; that timer does not set a turn angle.

The top row, `F3 RIGHT n`, counts **right exits selected** in this run, not confirmed successes or all crossings. The next accepted BTN1 start resets it. Three reflectance sensors cannot distinguish every junction from a wide patch of tape. An all-black patch is treated as a right candidate; a gap or a missing exit may stop the run. Verify the actual tape and geometry before relying on the choice.

**Stop:** During **any phase**, the first valid forward US1 reading **below 40 mm** requests zero on both motors (`F3 US1 <40MM`). Exactly 40 mm does not trip it. No reverse, turnaround, finish arming, gripper action, or automatic resume when the target moves away. A new simulated run requires a fresh BTN1 tap and release, clear US1, and confirmed tape. There is no overall run deadline. BTN2, ADC/US1 errors, stale readings, line loss and the turn-specific limits still stop the run. The 40 mm trigger is a measured reading, **not a guaranteed stopping distance**. A software stop is not a physical power cutoff.

## Verify without enabling F3

Run `sh firmware/ar-mainboard/tests/run_host_tests.sh` from the repository root. The combined board objects are tested first with the real two-function selector (any F3 callback fails that test); isolated host replays then exercise the retained F3 policy with mock sensors and motors. Checks include repeated right exits, the 39/40 mm boundary, sensor faults, and no automatic restart. The test-only plan injection is never linked into a hardware image.

The ordinary AR `Release` firmware must display **`F1 LINE BR 0`**, not `F3 RIGHT 0`. Do not use a previously built F3 ELF for this F1/F2 drive program.
