#ifndef ROTATION_ESTIMATE_H
#define ROTATION_ESTIMATE_H
#include <stdint.h>

/* J1 retains the reference default. J3 is a provisional empirical estimate:
 * (7 completed runs * 1140 events) / approximately 3 physical revolutions.
 * The J3 estimate includes possible noise and stop latency, not a proven PPR. */
#define ROTATION_J1_EVENTS_PER_REV 114u
#define ROTATION_J3_EVENTS_PER_REV 2660u
extern volatile uint32_t rotation_j1_events, rotation_j3_events;
typedef struct { uint32_t whole, hundredths; } RotationEstimate;
/* Called on pending EXTI before reference dispatch. Events are ignored at
 * zero commanded speed; no electrical PWM or physical-motion check. */
void RotationEstimate_RecordIRQ(uint16_t pin);
/* Convert unsigned events to a truncated pulse-derived estimate. */
RotationEstimate RotationEstimate_J1(uint32_t events);
RotationEstimate RotationEstimate_J3(uint32_t events);
#endif
