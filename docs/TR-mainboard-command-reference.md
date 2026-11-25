# TR → MainBoard command reference

This document specifies the UART command and ACK scheme used by the TR Controller and drive MainBoard. The protocol transports normalized joystick ratios; it does **not** prescribe wheel kinematics, motor RPM, PWM, or current. The TR receiver maps ratios to its drive system and enforces its own command limits and link failsafe.

On `warmaster726`, the user Controller is `firmware/controller/` and the TR drive MainBoard is `firmware/tr-mainboard/`. Both use the shared codec in `firmware/common/rdc_protocol.c`; the unrelated status-demo MainBoard remains at `firmware/mainboard/`.

## Physical link

- UART: USART1, **115200 baud, 8 data bits, no parity, 1 stop bit (8-N-1)**.
- Controller pins: PB6 TX, PB7 RX. MainBoard pins: PA9 TX, PA10 RX.
- Wire TX→RX and RX→TX, and share signal ground. Use 3.3 V logic; do not connect these pins to RS-232 voltage levels.
- Commands are ASCII lines ending in **LF (`\n`) only**. CRLF is rejected by the demo parser.

## Controller-to-MainBoard command lines

Each line is independent. The Controller sends movement and rotation lines every 50 ms (20 Hz). Mechanism commands are repeated every 50 ms while exactly one debounced button is pressed, and STOP is sent when neither or both buttons are pressed.

| Line | Meaning | Example |
|---|---|---|
| `M±X.XX/±Y.YY\n` | Left-stick movement ratios X and Y | `M+0.50/-0.25\n` |
| `R±Z.ZZ\n` | Right-stick horizontal-axis rotation ratio | `R-0.40\n` |
| `GO\n` | Mechanism open while only Button 8 is pressed | `GO\n` |
| `GC\n` | Mechanism close while only Button 7 is pressed | `GC\n` |
| `GS\n` | Mechanism stop (sent on release, simultaneous-button conflict, and startup) | `GS\n` |
| `H0\n` / `H1\n` | Heading-relative mode off / on | `H1\n` |
| `HZ\n` | Request heading zero (Button 4 held) | `HZ\n` |
| `Y0\n` / `YL\n` / `YR\n` / `YU\n` / `YX\n` | Turn heartbeat/idle, 90° left, 90° right, 180°, cancel | `YL\n` |

- Each ratio is signed, has exactly two decimal places, and is in `[-1.00, +1.00]`. These are normalized full-scale input ratios, not physical speeds.
- `M` carries both movement axes in one line; `R` carries rotation separately. Left-stick Y is inverted during Controller normalization. The demo convention labels positive rotation clockwise, but the TR drive MainBoard deliberately reverses its previous rotation mapping (`TR_INVERT_W = 1`); verify physical direction on the robot.
- Mechanism buttons are debounced for 30 ms; there is no long-press threshold. Button 7 requests CLOSE while pressed alone; Button 8 requests OPEN while pressed alone. The command is repeated every 50 ms while held. When neither button is pressed, the Controller sends STOP. If both are pressed simultaneously, it also sends STOP; if one remains pressed after the other is released, its command resumes after debounce.
- Button 1 toggles the requested heading mode; Button 4 sends zero while held. The Controller display shows the **requested** mode, not confirmation of mainboard heading validity. The mainboard display shows `H:OFF`, `H:WAIT` or `H:ON` and estimated degrees.
- Before enabling heading mode, centre both sticks, stop the wheels, ensure all three ESC feedback IDs are fresh, and hold Button 4 to set the heading reference. The heading estimate integrates wheel speed, not an IMU, and can drift due to slip. Deb's wheel diameter and effective radius are starting values that must be calibrated; heading sign and transform direction must be verified on the physical robot.
- In requested heading mode the mainboard commands zero wheel-speed targets if heading feedback or H0/H1 becomes stale, the estimate is invalid, or the link is lost. After lost feedback, stale heading commands or a long control-loop delay, centre sticks, stop wheels and zero again. Mode changes require the sticks to pass through neutral before motion resumes. An ACK means the line parsed, not that zeroing succeeded. The mechanism deadman remains independent. Zero targets are not zero torque or a physical emergency stop.
- Button 2 requests a 90° left turn, Button 3 a 90° right turn, and both together request 180°. The controller debounces and uses a 120 ms chord window; release both before another request. `Y0` is a turn heartbeat and button release, not a cancellation. `YX` cancels; the controller sends it when joystick DMA is waiting or faulted.
- A turn requires a zeroed and valid heading, fresh CAN feedback and M/R/H/Y messages, neutral sticks and stopped wheels. It ends on target, or aborts for joystick input, heading/feedback or command loss, ADC cancel, heading zero/mode change, timeout, missing progress or wrong-way progress. A turn does not require heading-relative mode to be on. The mainboard shows `TURN:RUN`, `DONE`, `ABORT`, `NOFB`, `REJECT` or `WRONG`. No automatic turns occur merely by enabling heading mode.
- Turns use wheel-speed-derived heading and may be inaccurate due to wheel slip; radius, wheel diameter and signs must be verified. No acceleration ramp is applied, including to automatic turns. An abort only commands zero wheel-speed targets; it is not zero torque or an emergency power cutoff. Test the entire system at low power, wheels raised and a separate power disconnect within reach before any floor test.
- A MainBoard should keep movement, rotation, and mechanism state independently; receiving one line does not imply another line arrived at the same instant. ACK confirms parsing only, not physical mechanism movement.

## MainBoard-to-Controller ACK

The MainBoard replies once for every valid command line:

```text
@A,<sequence>,<status-hex>*<xor-hex>\n
```

Example: `@A,42,00*47\n`.

- `sequence` is an independently increasing 16-bit unsigned counter; it wraps from 65535 to 0.
- `status` is two hexadecimal digits; `00` means the line was parsed and accepted. Other status meanings are not defined by the current demo.
- `xor` is two hexadecimal digits: XOR all ASCII bytes after `@` and before `*` (for example, the bytes in `A,42,00`).
- The ACK confirms parsing/acceptance only. It is not a motor or mechanism position/limit acknowledgement, and the Controller does not match ACKs to individual commands.

## Receiver and failsafe behavior

- Accept only exact command forms above, with LF termination and ratios within range. Reject malformed input without changing command state.
- The TR drive MainBoard queues UART bytes in its interrupt handler, parses complete commands in the main loop, and returns one checksummed ACK for each valid line. The ACK confirms parsing/acceptance only; it is not confirmation of motor speed, arm position, travel limits, or safety.
- The TR receiver keeps movement, rotation, and mechanism state independently. If the corresponding command becomes stale (350 ms), that command is treated as released; if the overall link is stale, motion is commanded to zero and the mechanism stops.
- The TR drive MainBoard passes received M axis signs into its movement average without negation; it negates received R before rotation mixing. Both `TR_INVERT_VX` and `TR_INVERT_VY` default to 0. Verify actual directions on the robot.
- The TR drive MainBoard at `firmware/tr-mainboard/` independently averages the latest three M and R motion samples, then uses wheel PID with direct speed targets (no acceleration-slew limiter); mechanism commands remain discrete. The UART protocol is not a safety interlock. The separate status-demo MainBoard at `firmware/mainboard/` also averages motion, but does not actuate motors or the mechanism.

## Guidance for a TR drive implementation

1. Parse and validate each line before changing stored command state.
2. Keep movement and rotation setpoints separate; apply a documented coordinate convention and mix them according to the actual wheel layout and measured motor directions.
3. Convert normalized ratios to bounded target speeds using robot-specific parameters. Do not interpret `±1.00` as RPM, current, or duty cycle.
4. Apply filtering/slew limits intentionally and command motors from a periodic control loop, not the UART interrupt callback.
5. For real mechanism actuation, implement safe handling for GO/GC/GS, including physical end stops or position limits, timeout behavior, and a hardware emergency disconnect. Confirm action/direction at low power on a secured robot before normal operation.

The shared codec implementation is `firmware/common/rdc_protocol.c` in this repository.
