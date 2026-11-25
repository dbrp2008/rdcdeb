#ifndef WHEEL_SPIN_H
#define WHEEL_SPIN_H
/* Powered encoder check on the AR SETUP page (robot LIFTED): hold B1 alone >= WHEEL_SPIN_HOLD_MS and the LEFT wheel
 * (J3) turns forward at WHEEL_SPIN_PCT; B2 alone = the RIGHT wheel (J1). Release (or press both) stops it. The screen
 * shows the FB pulses of this hold, and keeps the last one: count the real wheel turns, divide = pulses per turn
 * under power. Pure logic here; calibration_app drives the motor. */
#include <stdbool.h>
#include <stdint.h>
#define WHEEL_SPIN_HOLD_MS 1000u
#define WHEEL_SPIN_PCT 12.0f
enum { SPIN_NONE = 0u, SPIN_LEFT = 1u, SPIN_RIGHT = 2u };
typedef struct {
  unsigned held, spinning, last_wheel; /* SPIN_* */
  uint32_t held_since, base, pulses;   /* pulses: of the current hold, or the last one after release */
} WheelSpin;
/* Call every loop; returns the wheel to drive now (SPIN_NONE = hold both at 0). raw_* = SideIR_EncoderRaw. */
unsigned WheelSpin_Update(WheelSpin *s, uint32_t now, bool b1, bool b2, uint32_t raw_right, uint32_t raw_left);
#endif
