# Standalone Controller Joystick Calibration

This is a dedicated joystick calibration firmware for the Controller board. It uses the existing STM32F103, ADC1/DMA, SPI TFT, and button wiring; it is independent of the normal Controller application and does not run the RDC protocol or initialize USART1.

## Run

1. Build and flash this project using its `CMakePresets.json` (`Debug` or `Release`).
2. On startup, leave all four joystick axes centered for the center capture.
3. Sweep each axis to both physical ends and pause briefly at each end during the 15-second capture. Button1 can end the sweep after 4 seconds.
4. Read the min/center/max values on the TFT. Button1 exits; Button2 repeats the calibration.

The displayed values are temporary and are not written to flash. The 55.5-cycle ADC sampling time is configured in both `ControllerCalibration.ioc` and `Core/Src/adc.c`.

## Project layout

- `ControllerCalibration.ioc` — standalone project configuration for the Controller MCU.
- `Core/Src/joystick_calibration.c` — calibration acquisition and TFT results.
- `Core/Inc/joystick_calibration.h` — calibration entry point.
- `Core/Src/main.c` — standalone firmware entry point; T3 control code is not included.
