#ifndef CALIBRATION_REPLAY_H
#define CALIBRATION_REPLAY_H
#include <stddef.h>
#include <stdint.h>

/* Mock only the ADC and buttons. Never inject a profile or bypass the boot UI. */
static const uint16_t *capture_frame;
/* Preserve the timing of the existing all-high route fixtures (T = 362/712/4004). */
static const uint16_t route_white[3] = {324,624,3913}, route_black[3] = {400,800,4095};
static void replay_calibration(void (*keys)(unsigned, unsigned),
                               const uint16_t white[3], const uint16_t black[3]) {
  keys(0, 40); /* release any boot-held buttons */
  capture_frame = white; keys(1, 60); keys(0, 1400);
  capture_frame = black; keys(2, 60); keys(0, 1400);
  capture_frame = NULL; /* subsequent drive ADC samples are live, not capture frames */
  keys(2, 500); keys(3, 100); keys(1, 500); /* staggered chord, B1 held last */
  keys(0, 31); /* final tick: both up for 30 ms. No drive polling yet. */
}
#endif
