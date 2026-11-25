# Boot IR calibration — B1 white, B2 black, B1+B2 apply

Calibration is the **first screen of the normal AR MainBoard program**. Follow the [board build instructions](../../README.md#build-the-drive-firmware); do not flash a separate bench image or select a calibration preset. Hardware reset discards the previous profile. Leave motor-driver power **disconnected** during setup; startup requests zero PWM, which is not a physical power cutoff.

## Capture and apply

| TFT / action | What happens |
|---|---|
| `IR CAL 100 SAMP` | Hold all three sensors at operating height under the intended lighting. Release any boot-held buttons. |
| On white, press and release **B1 alone** | White capture starts on release. Keep still until the `W` row fills. |
| On black, press and release **B2 alone** | Black capture starts on release. Keep still until the `B` row fills. Either surface can be sampled first or redone. |
| `CAL READY` | Check all three `T` values and the `BLACK` row, in **left, centre, right** order. Hold **B1+B2** together, then release **both**. |
| `F1 LINE BR 0` / `B1 RUN  1+2 NEXT` | The displayed profile is now active. Check that `LINE` changes correctly on floor/tape and verify US1. F1 is selected, stopped. A **separate B1 tap and release** starts it with fresh drive sensors. To select F2 instead, press B1+B2 together and release both. |

A single starts a capture only if that button was held alone and both buttons return to a stable release. There is **no deadline for adding the second button** while the first is held. Observing both keys in one gesture cancels both singles; a stable overlap (30 ms) and stable release (30 ms) are required to apply. A short/bouncing overlap, an incomplete or weak calibration, or a gesture begun while sampling cannot trigger another capture or start driving. Release both and try again after `CAL READY`.

## What the numbers mean

Each capture settles for **250 ms**, then collects **100 complete ADC scans**, at least 10 ms apart. `W` and `B` are rounded means. `T` is the integer midpoint of the **gap between the sampled ranges**, not the midpoint of the two displayed means:

| `BLACK` | Required separation | Displayed `T` | Drive classifies black if… |
|---|---|---|---|
| `H` | `black_min − white_max ≥ 32` | `(white_max + black_min) / 2` | reading **> T** |
| `L` | `white_min − black_max ≥ 32` | `(black_max + white_min) / 2` | reading **< T** |

Equality is white for either polarity. Each sensor may have a different polarity. `----` / `?` marks an invalid channel; **all three** must be valid. `NEED W AND B` means a capture is missing; `WEAK OR OVERLAP` means at least one gap is too small; `ADC ERROR RETRY` means a scan failed. A deliberate new capture invalidates the previous reading for that surface, even if the new attempt fails. Correct the placement, lighting or ADC fault and sample that surface again.

The drive screen repeats the accepted `T` and `BLACK` rows. Values remain in **RAM** when stopping, cycling F1/F2 or rerunning a selected function. To recalibrate, use hardware reset. There is no flash write, fixed fallback, manual copying, recompile or reflash between setup and drive.

From the repository root, `sh ar-mainboard/tests/run_host_tests.sh` exercises both the [setup UI](../../Core/Src/ir_setup.c) and its handoff to the real board program with mocked I/O. These tests do not measure the physical sensors.
