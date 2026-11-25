/* The setup page's powered wheel check: hold one button 1 s -> that wheel turns; the pulses of the hold are kept. */
#include "wheel_spin.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
  WheelSpin s = {0};
  uint32_t r = 100u, l = 5000u;
  assert(WheelSpin_Update(&s, 0u, true, false, r, l) == SPIN_NONE);      /* B1 pressed: not yet */
  assert(WheelSpin_Update(&s, 999u, true, false, r, l) == SPIN_NONE);    /* still under 1 s */
  assert(WheelSpin_Update(&s, 1000u, true, false, r, l) == SPIN_LEFT);   /* B1 alone 1 s: LEFT turns */
  l += 1234u; r += 7u;                                                   /* right pulses don't count for a left hold */
  assert(WheelSpin_Update(&s, 3000u, true, false, r, l) == SPIN_LEFT && s.pulses == 1234u);
  assert(WheelSpin_Update(&s, 3010u, false, false, r, l + 50u) == SPIN_NONE); /* released: stop, keep the hold's count */
  assert(s.pulses == 1234u && s.last_wheel == SPIN_LEFT);
  assert(WheelSpin_Update(&s, 4000u, false, true, r, l) == SPIN_NONE);   /* B2: new hold, last result still shown */
  assert(s.pulses == 1234u);
  assert(WheelSpin_Update(&s, 5000u, false, true, r, l) == SPIN_RIGHT && s.pulses == 0u && s.last_wheel == SPIN_RIGHT);
  r += 99u;
  assert(WheelSpin_Update(&s, 6000u, false, true, r, l) == SPIN_RIGHT && s.pulses == 99u);
  assert(WheelSpin_Update(&s, 6001u, true, true, r, l) == SPIN_NONE);    /* both pressed (the drive chord): stop */
  assert(WheelSpin_Update(&s, 9000u, true, true, r, l) == SPIN_NONE);    /* and never spins while both are held */
  assert(WheelSpin_Update(&s, 9001u, false, false, r, l) == SPIN_NONE);
  s = (WheelSpin){0};                                                   /* raw counter wraps: the difference is still right */
  assert(WheelSpin_Update(&s, 0u, false, true, 4294967290u, 0u) == SPIN_NONE);
  assert(WheelSpin_Update(&s, 1000u, false, true, 4294967290u, 0u) == SPIN_RIGHT);
  assert(WheelSpin_Update(&s, 2000u, false, true, 10u, 0u) == SPIN_RIGHT && s.pulses == 16u);
  puts("Wheel spin check: 1 s hold of B1 / B2 alone turns L / R, both = never, pulses of the hold kept after release PASS");
  return 0;
}
