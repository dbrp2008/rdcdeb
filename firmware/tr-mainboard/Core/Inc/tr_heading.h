#ifndef TR_HEADING_H
#define TR_HEADING_H
#include <stdbool.h>
#include <math.h>

/* Deb's effective geometry; both values and heading sign require calibration. */
#define TR_HEADING_WHEEL_DIAMETER_M 0.1524f
#define TR_HEADING_ROBOT_RADIUS_M   0.273f
#define TR_HEADING_SIGN             1.0f
#define TR_HEADING_PI 3.14159265358979323846f

typedef struct { float degrees; bool valid; } TRHeading;

/* wheel_rpm is wheel-shaft speed, corrected by each wheel's physical sign.
 * Heading comes from the common rotation term of the three-wheel mixer. */
static inline void tr_heading_step(TRHeading *heading, bool feedback_fresh,
                                   float fl, float fr, float back, float dt)
{
  if (!feedback_fresh || dt <= 0.0f || dt > 0.1f) {
    heading->valid = false;
    return;
  }
  if (!heading->valid) return;
  const float mean_rpm = (fl + fr + back) / 3.0f;
  heading->degrees += TR_HEADING_SIGN * mean_rpm * TR_HEADING_WHEEL_DIAMETER_M *
                      3.0f / TR_HEADING_ROBOT_RADIUS_M * dt;
  heading->degrees = fmodf(heading->degrees, 360.0f);
}

static inline void tr_heading_transform(float degrees, float *x, float *y)
{
  const float angle = degrees * (TR_HEADING_PI / 180.0f);
  const float c = cosf(angle), s = sinf(angle);
  const float old_x = *x, old_y = *y;
  *x = old_x * c - old_y * s;
  *y = old_x * s + old_y * c;
}
#endif
