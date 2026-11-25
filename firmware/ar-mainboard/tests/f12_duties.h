#ifndef TEST_F12_DUTIES_H
#define TEST_F12_DUTIES_H
#include <math.h>
#include <stdbool.h>

/* Field-approved PWM magnitudes in percent, independent of the firmware scale.
 * The controller replay checks physical wheels; the board replay checks signed
 * J3/J1 motor writes. The passive bench and dormant F3 have separate tests. */
#define DUTY_LAUNCH       13.5f
#define DUTY_STRAIGHT     22.5f
#define DUTY_INSIDE        9.9f
#define DUTY_OUTSIDE      23.85f
#define DUTY_REVERSE      18.0f
#define DUTY_CCW          11.25f
#define DUTY_FINISH_INNER 11.25f
#define DUTY_F2_CW_LEFT   15.75f
#define DUTY_F2_CW_RIGHT  9.45f

static inline bool duty_is(float actual, float expected) {
  return fabsf(actual - expected) < 0.0001f; /* much finer than one PWM timer tick */
}
#endif
