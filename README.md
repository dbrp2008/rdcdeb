# RDC 2026 - AR firmware

Firmware for the Artifact Retriever (AR) of HKUST Robotics RDC 2026: boot IR calibration and the autonomous routes
F1-F4, on the STM32F405 MainBoard.

```text
ar-mainboard/    the AR firmware (CubeMX project, CMake build, host tests)
docs/            field cards and route notes
```

Build, flash and test instructions are in [`ar-mainboard/README.md`](ar-mainboard/README.md). The
[drive build](ar-mainboard/README.md#build-the-drive-firmware) needs no CMake preset. The optional
[EncoderBench preset](ar-mainboard/README.md#passive-encoder-bench) reads hand-turned wheels without driving them.
Read the instructions before flashing or turning on motor power.

The TR firmware is on the `deb` branch.
