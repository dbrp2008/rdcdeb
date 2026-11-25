# Artifact Retriever (AR) firmware

The active AR project is [`ar-mainboard/`](ar-mainboard/). It was synchronized
from `warmaster726` commit `7fc3249`. Read its README for the current wiring,
calibration, operation, build prerequisites, and safety limitations.

**Important: this replaces Deb's earlier digital-IR/ultrasonic pinout.** The
current project expects analog IR on PA2, PC2 and PC3, US2 trigger on PA1 and
echo on PA0. It can drive motors after button start. Do not flash it onto a
board wired for the previous pinout without checking and changing the wiring.
Disconnect motor power during setup, use an independent motor-power cutoff,
and perform supervised bench tests before track testing. Routes, motor direction,
and stopping behavior have not been verified on this robot or track. The servo
remains disabled.
