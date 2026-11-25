# Controller ↔ MainBoard communication reference

The user TR Controller (`firmware/controller/`) and TR drive receiver (`firmware/tr-mainboard/`) are separate firmware projects. Both compile the shared line codec from `firmware/common/`; UART RX/TX handling remains in each board project. The separate `firmware/mainboard/` project is a communication/status demo using the same wire protocol.

## UART

- USART1, 115200 baud, 8 data bits, no parity, 1 stop bit (8-N-1).
- Controller USART1: PB6 TX, PB7 RX.
- MainBoard USART1: PA9 TX, PA10 RX.
- Connect TX to the other board's RX, RX to TX, and grounds together. Use 3.3 V UART logic; this is not an RS-232 voltage interface.

## Controller commands

Every command is one ASCII line terminated by LF (`\n`) only; CRLF is rejected.

```text
M+X.XX/-Y.YY\n
R+Z.ZZ\n
GO\n
GC\n
GS\n
H0\n
H1\n
HZ\n
Y0\n
YL\n
YR\n
YU\n
YX\n
```

- `M` carries left joystick X/Y movement-speed ratios in `[-1.00,+1.00]`, exactly two decimals.
- `R` carries right joystick horizontal rotation-speed ratio in `[-1.00,+1.00]`. Positive is called clockwise in the demo convention; the TR drive MainBoard has `TR_INVERT_W = 1`, and physical direction must be verified.
- `GO` means mechanism open; `GC` means mechanism close; `GS` means stop. Controller Button 7 is CLOSE and Button 8 is OPEN. After 30 ms debounce, pressing and holding only one button sends its command every 50 ms; releasing both sends `GS`. Pressing both together also sends `GS`; if only one remains pressed, its command resumes after debounce. No long-press delay is used.
- `H0`/`H1` select heading-relative mode off/on and `HZ` requests zero. Button 1 toggles mode and Button 4 requests zero. Heading mode requires fresh wheel feedback and a stationary zero; see the TR mainboard command reference for the fail-closed behavior and calibration requirements.
- `Y0` is the turn heartbeat, `YL`/`YR` request 90° left/right, `YU` requests 180°, and `YX` cancels. Button 2/3 generate the requests; ADC faults send `YX`. The controller sends these every 50 ms. See the TR mainboard command reference for prerequisites, timeouts, limitations and testing precautions.
- Movement and rotation lines are sent every 50 ms. Joystick Y axes are inverted and have a center deadband.

## ACK

The MainBoard returns one ACK for each valid command:

```text
@A,<sequence>,<status-hex>*<xor-hex>\n
```

Sequence is an independent 16-bit counter. Status `00` indicates accepted parsing. XOR covers ASCII bytes after `@` and before `*`, printed as two hexadecimal digits.

## Status and testing

The Controller display shows ratios, mechanism state, ACK freshness/status, and transport counters. The status-demo receiver in `firmware/mainboard/` averages motion ratios and reports mechanism STOP after its 350 ms link timeout; it does not actuate motors or the mechanism. The TR drive receiver in `firmware/tr-mainboard/` uses the same command and ACK format, averages the latest three M and R motion samples independently, and controls the wheels and arm. Its wheel PID follows the averaged targets directly, without an acceleration-slew limiter. ACK means the command line was parsed and accepted, not that a motor or mechanism reached a physical position. The independent motor tuner lives at `firmware/mainboard-motor-tuner/`.

From the repository root, run the host codec test:

```sh
gcc -std=c11 -Wall -Wextra -Werror -pedantic \
  -Ifirmware/common \
  firmware/common/rdc_protocol.c \
  firmware/common/tests/test_rdc_protocol.c -o /tmp/test_rdc_protocol
/tmp/test_rdc_protocol
```
