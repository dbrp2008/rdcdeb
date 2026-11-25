# Project guide

## Firmware projects

- [`../firmware/controller/`](../firmware/controller/) — TR hand-controller firmware; sends the RDC protocol to the TR MainBoard over USART1.
- [`../firmware/mainboard/`](../firmware/mainboard/) — normal MainBoard communication and status-display firmware.
- [`../firmware/tr-mainboard/`](../firmware/tr-mainboard/) — TR drive MainBoard; pairs with the hand Controller and returns checksummed ACKs.
- [`../firmware/mainboard-motor-tuner/`](../firmware/mainboard-motor-tuner/) — independent local TFT/button bench firmware for TR (three channels), with CSV logging over USART2. AR support is disabled until its motor/controller interface is identified and supported; it does not use the TR Controller.
- [`../firmware/controller-calibration/`](../firmware/controller-calibration/) — separate on-device joystick calibration utility.
- [`../firmware/common/`](../firmware/common/) — shared protocol implementation and host tests.

Each board application has its own CMake project and CubeMX `.ioc`; the tuner reuses generated MainBoard board-support sources. See [`communication.md`](communication.md) for the UART frames and host codec test, and [`motor-tuner-check-and-connect.md`](motor-tuner-check-and-connect.md) for the step-by-step bench, CAN wiring, first-power, feedback-check, run, and shutdown procedure.

The tuner currently assumes an RM-style C620 CAN protocol. The AR motor/ESC interface is unconfirmed; do not run the AR profile until hardware compatibility and limits are verified.
