#include "tr_heading.h"
#include <assert.h>

static int near(float a, float b) { return fabsf(a-b) < 0.001f; }
int main(void)
{
  TRHeading h = {0.0f, false};
  tr_heading_step(&h, true, 100, 100, 100, 0.1f);
  assert(!h.valid && h.degrees == 0.0f); /* explicit zero is required */
  h.valid = true;
  tr_heading_step(&h, true, 100, 100, 100, 0.1f);
  assert(near(h.degrees, 100.0f * TR_HEADING_WHEEL_DIAMETER_M * 3.0f /
                         TR_HEADING_ROBOT_RADIUS_M * 0.1f));
  float x=0.0f,y=1.0f;
  tr_heading_transform(90.0f, &x, &y);
  assert(near(x,-1.0f) && near(y,0.0f));
  tr_heading_step(&h, false, 0, 0, 0, 0.01f);
  assert(!h.valid);
  tr_heading_step(&h, true, 1, 1, 1, 0.01f);
  assert(!h.valid); /* feedback return must not silently re-arm */
  h.valid=true;
  tr_heading_step(&h, true, 0, 0, 0, 0.101f);
  assert(!h.valid); /* missing integration time */
  h.valid=true; h.degrees=359.0f;
  tr_heading_step(&h, true, 100, 100, 100, 0.1f);
  assert(h.degrees >= 0.0f && h.degrees < 360.0f);
  return 0;
}
