# Firmware projects

- `controller/` is the TR controller firmware and sends the shared RDC protocol over USART1.
- `mainboard/` is the normal MainBoard communication/status firmware. Its IOC remains user-owned.
- `mainboard-motor-tuner/` is an independent STM32F405 bench-tuning firmware for separate MainBoards. The TR profile (three channels) is enabled; AR is disabled until its motor/controller interface is identified and supported. It uses local TFT/button controls and USART2 CSV logging; it does not take commands from the TR Controller.
- `controller-calibration/` is the separate joystick calibration firmware.
- `common/` holds the shared RDC protocol codec and tests.

Each project has a local `.vscode/tasks.json` following the existing MainBoard workflow: build the Release preset, flash the produced image through ST-LINK, or run the combined build-and-flash task. The programmer is invoked by its explicit path, not added to `PATH`. The tuner owns its generated `Core/` and `Drivers/` tree and keeps its application/CAN backend under `App/` and its custom build configuration under `buildsystem/`; it does not depend on `mainboard/`. See each project README and the tuner startup operator guide before build or hardware tests.
