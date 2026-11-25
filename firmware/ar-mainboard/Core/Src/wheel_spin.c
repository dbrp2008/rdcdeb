#include "wheel_spin.h"
unsigned WheelSpin_Update(WheelSpin *s, uint32_t now, bool b1, bool b2, uint32_t raw_right, uint32_t raw_left) {
  const unsigned key = (b1 && !b2) ? SPIN_LEFT : (b2 && !b1) ? SPIN_RIGHT : SPIN_NONE;
  if (key != s->held) { s->spinning = SPIN_NONE; s->held = key; s->held_since = now; } /* pulses keep the last hold */
  if (key != SPIN_NONE && s->spinning == SPIN_NONE && (uint32_t)(now - s->held_since) >= WHEEL_SPIN_HOLD_MS) {
    s->spinning = s->last_wheel = key;
    s->base = key == SPIN_LEFT ? raw_left : raw_right;
    s->pulses = 0u;
  }
  if (s->spinning != SPIN_NONE) s->pulses = (s->spinning == SPIN_LEFT ? raw_left : raw_right) - s->base;
  return s->spinning;
}
