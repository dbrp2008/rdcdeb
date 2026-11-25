# AR mainboard — analog IR, US2 and ACTIVE navigation

**This firmware can drive the motors.** After IR calibration and a valid US2
reading, BTN1 on the NAV screen starts Deb's Key A/B navigation with a 2 s
countdown. Deb's routes and timings have not been verified on the robot or
track. Disconnect motor power while flashing, setting up and calibrating. Use
an independent, accessible motor-power cutoff; BTN1 and zero PWM are software
requests, not an emergency stop or guaranteed power isolation. A previous
wiring setup accelerated at zero PWM, although the user later verified zero
stopped both motors in the current setup.

Never run unattended. Check physical wheel direction, sensor polarity/order,
line and US2 placement, stop distance, target geometry, route assumptions and
power cutoff before any powered test. Keep wheels off the ground and motor
power disconnected for initial checks. No target build, CubeMX regeneration or
hardware testing of this active implementation has been performed.

## Pins and configuration

| Signal | Pin | Function |
| --- | --- | --- |
| IR left | PA2 | ADC1 IN2 |
| IR centre | PC2 | ADC1 IN12 |
| IR right | PC3 | ADC1 IN13 |
| US2 echo | PA0 | TIM2_CH1 in IOC; GPIO input at runtime |
| US2 trigger | PA1 | GPIO output |
| US1 echo | PB10 | TIM2_CH3 in IOC; not used by navigation |
| US1 trigger | PB11 | input in IOC; not driven by this firmware |
| Right motor J1 | PB14 PWM, PC6 DIR | positive direction as motor driver |
| Left motor J3 | PC7 PWM, PC9 DIR | driver applies existing J3 inversion |

Verify sensor voltage levels: an ultrasonic echo from a 5 V module may need a
level shifter; never apply a 5 V analog output to an ADC pin. PB4/PB5/PB6
remain digital GPIO inputs. The user's US1/TIM2 assignments are left intact;
US1 has no active driver and PB11 is not configured as a trigger output.

ADC1 uses one 12-bit conversion at a time, PCLK2 / 4, 84-cycle sampling, no
ADC DMA. `IR_ReadAll()` selects/polls channels 2, 12 and 13 sequentially;
this is blocking, not simultaneous. Errors discard the entire IR frame. The
TFT retains SPI1 DMA2 Stream3. ADC1 belongs exclusively to this reader.

US2 uses GPIO polling and TIM2's free-running 1 MHz counter; it reconfigures
PA0 as a GPIO input and does not use the IOC's TIM2 input capture. It triggers
PA1 and samples approximately every 60 ms. No echo or edge timeout takes up
to approximately 15 ms (20 ms watchdog) instead of the prior 250 ms. This
limits, but does not eliminate, foreground blocking and response delay.

## Screens and controls

Hold both buttons for one second to cycle screens while not running:

1. **IR ANALOG TEST:** put all three sensors over white, tap BTN1 and wait for
   the capture; then put them over black and tap BTN2 and wait. Captures settle
   for 250 ms and take 100 scans. They require a 32-count gap and use
   direction-aware hysteresis. Calibration is RAM-only; repeat after reset.
   Check the raw values and physical left/centre/right order.
2. **US2 TEST:** check distance/status with a ruler. BTN1/BTN2 adjust a RAM-only
   -/+5 mm offset (limits ±100 mm). Calibrate `AR_STOP_MM` in `ar_nav.h` to the
   actual mechanism and sensor location; the default is only a guess.
3. **NAV RUN:** BTN2 selects Key A/B while stopped. To start, place the
   centre sensor over the line and tap BTN1. It requires valid IR
   classifications, a centre sensor on the line throughout the countdown, and a
   recent valid US2 reading. Motors are initialized only
   at the first nonzero command after the countdown. Press BTN1 while running
   to request a stop; this button is not a hardware cutoff. A failed/stale US2
   reading, invalid IR frame/calibration, or motor initialization failure
   requests zero PWM and aborts the run; a new run requires a fresh start.
   A lost line on the key leg is a fault, not proof of a key. Two distinct
   fresh close readings are required to enter the grab phase. US2 is not a
   general obstacle-avoidance or collision-safety system.
4. **ENCODER COUNTS:** raw J1/J3 feedback edges; BTN1 resets the baselines.
   Counts were observed increasing at rest and are not reliable odometry.

The TFT shows `STARTING RUN` and intentionally stops updating during navigation:
its SPI wait can block indefinitely. It refreshes after the run stops. Therefore
it does not show live progress. A stalled CPU, failed timer/interrupt, blocked
ADC or electrical fault can still prevent software from stopping the motors.
The robot's stop must not depend on the screen or this software.

The servo remains disabled: the grab phase is only a pause, with no powered
actuator command; it assumes a compatible passive spring hand. No pickup is
guaranteed.

Deb's Key A/B route still uses estimated speeds, line-detection turns and a
fixed home-zone advance. The home-zone boundary is inferred from loss of line,
and neither route, key pickup, distances nor physical stopping location is
proven. Key B's junction behavior is especially uncertain. The ultrasonic
interlock requires a valid echo throughout a run; open space beyond roughly
2.5 m, poor alignment or invalid echoes may abort the run.

## CubeMX / build

The current IOC uses one regular ADC rank, scan disabled, no ADC DMA. The
software selects the three channels at runtime and sets all three GPIOs to
analog. Preserve SPI1 DMA. TIM2/US1 capture assignments reflect the user's IOC,
but only US2's software polling is implemented. `Keep User Code` is on and
`Backup Previous` is off in the latest IOC. Review generated diffs before
building; generated CMake/toolchain files, HAL drivers, startup assembly and an
ARM compiler must be supplied locally. Confirm initialization calls remain
before `AR_MergedApp_Init()` and the ADC, TIM2 and motor timer code match the
IOC. Host tests are not a substitute for target build, CubeMX or hardware
verification.

From the project root, run behavioral host mocks with `tests/run_host_tests.sh`
(requires GCC). These do not exercise the MCU or real motors.
