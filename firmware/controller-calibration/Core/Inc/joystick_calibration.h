#ifndef JOYSTICK_CALIBRATION_H
#define JOYSTICK_CALIBRATION_H

/* Optional on-device joystick calibration utility.
 * Enter by holding Button8 during Controller power-up.
 * Results are displayed on the TFT only; no flash is written. */
void joystick_calibration_run(void);

#endif /* JOYSTICK_CALIBRATION_H */
