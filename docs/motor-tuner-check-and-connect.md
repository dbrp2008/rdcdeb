# Motor tuner: check, connect, and run safely

This is a bench setup and bring-up checklist for `firmware/mainboard-motor-tuner/`. The tuner is a separate local MainBoard firmware: it uses its TFT and buttons and sends trial data on USART2; it does **not** use the normal TR Controller link. Read the hardware labels and the motor/ESC documentation for the exact assembly on your bench. Never infer a connector pinout, ESC type, motor ID, gear ratio, or AR compatibility from a profile name.

> **Important safety boundary:** the tuner’s 5.0 A current ceiling and firmware aborts are software measures, not hardware protection or an emergency stop. Use an independently current-limited supply and a physical power disconnect within reach. Keep people and loose objects clear; secure the assembly and raise driven wheels clear of the bench.

## What the checked-in configuration assumes

- CAN1 at **1 Mbit/s** uses STM32F405 PA11 (RX)/PA12 (TX) for TR motor IDs 1 and 2. CAN2 at **1 Mbit/s** uses PB12 (RX)/PB13 (TX) for the trailing TR motor, keeping ESC ID 3 on that separate bus. Use the board’s CAN1/CAN2 transceiver connectors; do not connect CAN-H/CAN-L directly to MCU pins. CAN IDs may repeat across separate buses, but must be unique among devices sharing a bus.
- The motor backend sends/receives RM C620-style standard CAN frames. TR has three configured channels and default IDs 1–3; these are software defaults, **not confirmation** of fitted hardware. AR has been reported as possibly non-CAN; it is compile-time disabled (`TUNER_PROFILE_AR_ENABLED=0`) and has no AR-specific backend. The tuner waits for speed at or below 2 output RPM for 250 ms before each phase; failure to settle within 5 seconds latches a fault. A latched `FAULT` locks out another run until reset. `TX:ERR` means a CAN transmit queue operation failed: stop the test and use the physical disconnect if needed.
- Feedback is treated as motor-shaft RPM and divided by the configured 19:1 ratio to display/compare output-shaft RPM. Verify both the ESC feedback convention and the actual gearbox ratio before a powered test.
- A run starts with low-current steps (0.25 A, 0.50 A, 0.75 A), then adaptively repeats 2-second PID trials targeting 4,000 motor-shaft RPM (about 211 gearbox-output RPM at 19:1), with no fixed trial count. It stops after three consecutive trials meet rise time ≤500 ms, overshoot ≤8%, and steady-state error ≤3% held for 300 ms; BTN2 aborts, and safety faults still stop the run. The tuner searches gains sequentially P→I→D: first Kp for rise/overshoot with Ki=Kd=0 (Kp ceiling 0.225), then refines around an acceptable P by testing up to two lower values using adaptive 5–20% steps and keeping the last passing Kp, then Ki for steady-state accuracy only when normalized ARPM/TRPM ≥0.95 (Ki ceiling 0.045), and finally Kd only if overshoot needs damping (Kd ceiling 0.0045). It can flow backward to revisit P or I when a later adjustment harms rise time, overshoot, or steady accuracy; a P backtrack retains the partially successful I and any reduced D instead of erasing learned gains. Each failed trial selects changed gains for the next trial, and Kp/Ki/Kd leap sizes adapt to tracking deficit, steady error, and overshoot (Kp 10–50%, Ki 0.001–0.006, Kd 0.0001–0.0008); all gains remain bounded. After actual RPM drift remains within 3% of target for 300 ms, it records ARPM/TRPM for diagnostics only; it does not multiply the target by the ratio or its inverse. CSV samples include ARPM/TRPM ×1000 plus a `#RATIO` calibration row. It updates one live gain set rather than cycling through preset candidates. The tuner reports commanded and ESC-reported current, target and measured output RPM, rise time, overshoot, steady-state error, and gains in USART2 CSV. The overspeed trip is 4,370 motor-shaft RPM (230 output RPM safe ceiling); the per-motor software current ceiling is 5.0 A. Confirm every component’s ratings; lower the software cap if the weakest component requires it.
- The AR motor/controller interface is **unverified** and may not be CAN. The AR profile is disabled by default. Do not enable or run it until the exact controller/interface, protocol, IDs if applicable, pinout and limits are confirmed and a matching backend is implemented and tested.

For authoritative CAN and RM setup context, see the official Software Tutorial 2026 notes: [`advanced-tutorial-2-CAN/03-can.md`](https://github.com/UST-Robotics-Team/Software-Tutorial-2026-Notes/blob/main/advanced-tutorial-2-CAN/03-can.md) and [`advanced-tutorial-2-CAN/04-rm-motor.md`](https://github.com/UST-Robotics-Team/Software-Tutorial-2026-Notes/blob/main/advanced-tutorial-2-CAN/04-rm-motor.md). The CAN note configures CAN1 in CubeMX at 1 Mbit/s, explains the differential CAN-H/CAN-L pair, and stresses generating `can.h`/`can.c`. The RM note describes C620 wiring, unique ESC IDs, its ID-setting/LED check, and powering the motors off before flashing. Follow the exact hardware manual for your own connectors and ESCs.

## Step-by-step checks and connection

### 1. Identify the hardware before connecting power

1. Record the exact motor model, ESC/controller model, supply voltage/current rating, gearbox ratio, and connector/harness type for each motor. Confirm the assembled board revision and the intended CAN port from its board documentation.
2. Confirm that the chosen ESC implements the tuner’s RM C620-style protocol and 1 Mbit/s CAN. If you cannot confirm this, stop: do not use the tuner to discover compatibility by applying motor power.
3. The AR assembly has been reported as possibly using a non-CAN interface. The AR profile is disabled in firmware; keep it disabled. “AR” must not be treated as a supported hardware mode unless the actual controller/interface is identified and a compatible backend is implemented and verified.
4. Set and verify the ESC IDs before connecting the full bus. **For a confirmed DJI RoboMaster C620 only**, the tutorial points to the C620 SET-button procedure: with that ESC powered and operating normally, short-press SET once to enter individual ID-setting mode (the status LED turns off); then press SET the required number of times for the desired ID, from 1 to 8. Each valid press is acknowledged by an orange LED flash. Leave SET untouched for about 3 seconds so the ESC saves the value, then power-cycle that ESC. The tutorial notes that the normal status LED blink count indicates the set ID. Repeat for the other ESCs, choosing a different ID for each. Configure one ESC at a time; do not use the C620 quick-ID procedure unless you deliberately want to assign a whole compatible bus and understand its rotor-order process. Never use these button steps on an unverified/non-C620 controller. The C620 manual warns that duplicate IDs on one CAN bus cause a conflict and output cutoff.
5. For supported TR C620 hardware only, set the matching TR macros in `App/Core/Inc/motor_tuner_config.h` (`TUNER_TR_MOTOR_1_CAN_ID`, `_2_`, `_3_`). Keep the trailing motor at ESC ID 3 on CAN2 only; motors 1 and 2 use CAN1. Each macro must match its physical ESC ID and be in 1–8; IDs must be unique on each bus (the same ID may exist on the other isolated bus). These are ESC channel IDs—not CAN frame identifiers `0x200`, `0x1FF`, or feedback base `0x201`, which are protocol constants in `App/Core/Inc/rm_motor.h`. AR ID macros are placeholders while AR is disabled; changing them does not add non-CAN support. Rebuild and reflash with motor power disconnected, then recheck the TR channel before testing.
6. Verify the gear ratio and speed feedback convention. The current conversion assumes 19 motor-shaft rotations per output-shaft rotation and motor-side RPM feedback. Change the configured ratio/conversion only after measuring or confirming the real setup.

### 2. Make the bench safe and keep motor power off

1. Bolt down the board/motors or otherwise restrain them securely. Ensure the wheel/output can rotate freely and raise driven wheels clear of the surface. Keep hands, cables, clothing and tools away from moving parts.
2. Put a physical battery disconnect/switch within immediate reach. Use a supply with its own current limit, set conservatively for the lowest-rated component.
3. Disconnect motor/ESC power before wiring, changing IDs, configuring the firmware, or flashing. The tutorial explicitly says to power motors off before flashing.
4. With all power removed, inspect harnesses and connectors for damage, loose strands, shorts and reversed connections. Check supply polarity and confirm no motor supply voltage is present on signal lines.

### 3. Wire CAN with power removed

1. Use the board’s documented CAN transceiver connectors and matching harnesses: CAN1 for motors 1/2, CAN2 for trailing motor 3. Do not wire directly to MCU PA11/PA12 or PB12/PB13; those are logic-side peripheral pins, not the differential bus terminals.
2. Match **CAN-H to CAN-H** and **CAN-L to CAN-L** end-to-end, and connect the reference ground/shield only as specified by the board/ESC documentation. CAN-H/L reversal usually prevents communication even if it does not damage the devices.
3. Use a suitable twisted-pair bus topology. Terminate only the two physical ends of the bus as required by the transceivers and harness; avoid adding extra terminators or long star branches. Verify termination against the actual board and ESC documentation rather than guessing resistor values.
4. Check that each bus contains only its intended nodes, every node is set for 1 Mbit/s, and IDs are unique within that bus. The tuner IOC configures both CAN1 and CAN2 at 1 Mbit/s with the configured APB1 clock; keep the two physical networks separate, and connect the trailing motor/ID 3 to CAN2 only.
5. Before powering, use continuity/inspection to confirm H/L are not shorted together or to supply/ground, and that the connector mapping matches the board’s own pinout. The tutorial images are a reference, not a substitute for checking the assembled board revision.

### 4. Build and flash the tuner with motor power disconnected

1. Build the tuner Release target using the project’s documented VS Code task/build preset and GNU Arm toolchain **14.3.1+st.2**. Confirm the build completes and produces `build/Release/MainBoardMotorTuner.elf`.
2. Keep motor/ESC battery power disconnected while flashing through ST-LINK/SWD. Do not flash with powered motors attached, even if the UI appears idle.
3. Confirm the programmer reports a successful write/verification and reset. If the flash fails, stop and resolve the programmer/build issue before connecting motor power.
4. Connect a 3.3 V USB-UART adapter to the documented USART2 pins/ground for logging, if needed. Verify the adapter’s pinout and voltage level from board docs; capture at **115200 baud, 8-N-1**. Do not connect an RS-232-level interface.

### 5. First power-on and communication check

1. Keep the wheels elevated and the physical disconnect reachable. Follow the tutorial’s sequencing for this RM setup: power the motor ESCs first, then power the MainBoard. On shutdown, turn motor power off first, then the MainBoard. If your board/ESC documentation specifies a different safe sequence, stop and resolve that discrepancy before proceeding.
2. Observe the tuner TFT and USART2 startup guide. Do not press Start yet. Confirm the display is alive, the selected profile and motor index are as expected, and the board/ESCs show no fault indication.
3. Check CAN feedback for the selected motor. The TFT should show fresh feedback (`FB:OK`); the CSV stream should show sensible, stable feedback values. No fresh feedback, invalid values, or repeated CAN errors means stop: power off the motors, then the MainBoard; recheck CAN-H/L, ground/reference, termination, bitrate, IDs, connector mapping and ESC status. Do not compensate by increasing current.
4. Confirm direction only using the manufacturer-approved low-risk procedure with the mechanism secured and current/supply limits conservative. Do not hold a motor or wheel by hand to test torque. If feedback direction, physical direction, or reported speed is unexpected, abort and disconnect motor power.

### 6. Run one supervised auto-tuning session at a time

1. Before every run, re-check the TFT profile and motor index against the physical motor, and verify the ESC ID and wiring map. Initially test one motor/channel at a time; keep all other motor setpoints at zero.
2. AR selection is disabled. Hold BTN1 for about 0.9 s to advance among TR channels. Verify the displayed profile and motor index; do not assume that changing an ID macro enables support for another interface. If the TFT shows `TX:ERR`, abort any run and stop; CAN zero-current delivery is not guaranteed, so use the physical disconnect as required.
3. Press BTN2 only when ready to begin. Press BTN2 again to abort. The tuner also stops a run for stale/missing feedback, excessive speed or motor temperature threshold, but these checks can fail and are **not** an emergency stop.
4. Watch the wheel, commanded and ESC-reported current, target and measured speed, temperature and feedback continuously. The test uses 250 ms positive-current pulses (0.25/0.50/0.75 A), then adaptive 2-second PID trials to 4,000 motor-shaft RPM (about 211 gearbox-output RPM at 19:1); it continues without a fixed trial count until three consecutive trials pass the strict rise/overshoot/steady-error limits, or B2/safety fault stops it. It waits for speed at or below 2 output RPM continuously for 250 ms, at least 1 second rest, and ESC temperature below 50 C between phases; failure to settle/cool within 5 seconds faults. The overspeed trip is 4,370 motor-shaft RPM (230 output RPM safe ceiling). USART2 CSV records both current values, both RPM values, Kp/Ki/Kd, rise time, overshoot, steady error, and pass streak. `TRIAL` rows record every attempt; the final `RESULT` row appears after three consecutive passes. A zero-current command is not active braking. Immediately use the physical disconnect for unexpected motion/direction, excess speed/current, rising heat, noise, smoke, wiring fault, stale feedback, or a fault that cannot be diagnosed safely. Do not reach into a moving mechanism.
5. After a run, verify that output current is zero and the mechanism has stopped before approaching it. Save the USART2 CSV and review the readings before changing gains or testing another motor.
6. For any wiring/ID change or reflash: disconnect motor power first. Never hot-swap CAN, motor, encoder or supply connectors.

## Stop checklist

Stop and disconnect motor power if **any** of the following is true:

- ESC or motor identity/protocol, supply rating, ID, CAN pinout, bus termination, or gearbox ratio is unknown.
- Selected TR channel does not match the physical motor, IDs are duplicated, or any AR hardware/interface is involved while AR support remains disabled.
- CAN feedback is stale/absent, the bitrate or direction is wrong, or the ESC reports a fault.
- The fixture is loose, wheels are not clear, the supply is not independently current-limited, or a physical disconnect is not immediately reachable.
- There is unexpected motion, heating, noise, smell/smoke, excessive speed/current, or any uncertainty about safety.

## Sources and project settings

- Official CAN tutorial: [03-can.md](https://github.com/UST-Robotics-Team/Software-Tutorial-2026-Notes/blob/main/advanced-tutorial-2-CAN/03-can.md).
- Official RM motor tutorial: [04-rm-motor.md](https://github.com/UST-Robotics-Team/Software-Tutorial-2026-Notes/blob/main/advanced-tutorial-2-CAN/04-rm-motor.md).
- Tuner CAN/profile settings: [`../firmware/mainboard-motor-tuner/motortuner.ioc`](../firmware/mainboard-motor-tuner/motortuner.ioc), [`../firmware/mainboard-motor-tuner/App/Core/Inc/motor_tuner_config.h`](../firmware/mainboard-motor-tuner/App/Core/Inc/motor_tuner_config.h), and [`../firmware/mainboard-motor-tuner/App/Core/Src/rm_motor.c`](../firmware/mainboard-motor-tuner/App/Core/Src/rm_motor.c).
- Bench procedure, UI, CSV format, build commands and current tuning behavior: [`../firmware/mainboard-motor-tuner/README.md`](../firmware/mainboard-motor-tuner/README.md).
