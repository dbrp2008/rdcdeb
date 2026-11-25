# Inactive Deb TR distance-test reference

These are exact snapshots of the TR drive source and header from Deb commit
`35ecb88`, paths `Reference/MainBoard/Core/Src/tr_drive.c` and
`Reference/MainBoard/Core/Inc/tr_drive.h`. The `.reference` extension is
intentional: these files are **not included in the warmaster726 build**.

The reference includes an experimental wheel-encoder distance test under
`TR_ENCODER_TEST`; its original header defaults that setting to `0`. It also
contains many other older drive, heading, turn, CAN and servo behaviors. It is
not compatible as a drop-in replacement for the current TR firmware. The
current controller does not send the old test commands, and no button starts a
0.5 m test in the current build.

Before attempting integration: agree on a deliberate control and abort
mechanism, verify motor feedback and encoder signs, measure wheel diameter and
gearing, calibrate distance and stopping response, review compatibility and
failure modes, and use a separate emergency power cutoff. Host/source review is
not evidence that the reference builds or runs safely on the robot.
