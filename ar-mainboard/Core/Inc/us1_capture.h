#ifndef US1_CAPTURE_H
#define US1_CAPTURE_H
/* US1 (front, trig PA1 / echo PA0) WITHOUT waiting: the echo is timed by TIM2 channel 1 input capture + interrupt, so the
 * main loop never stops for up to ~15 ms per ping (as US1_Measure does) and the line following keeps reacting.
 * Ported from our AR's us_capture (same pin, same 1 MHz 32-bit TIM2). Set up entirely in code: PA0 is switched to TIM2_CH1
 * (AF1), CH1 to input capture, and this file owns TIM2_IRQHandler - no CubeMX change. Call after US2_Init (which checks the
 * 1 MHz clock and starts TIM2). PA0 can still be read as a GPIO, so the blocking US1_Measure keeps working elsewhere. */
#include <stdbool.h>
#include <stdint.h>
#include "us2_sensor.h"

#ifndef US1C_PERIOD_MS
#define US1C_PERIOD_MS 50u   /* one ping every this long */
#endif
#ifndef US1C_TIMEOUT_MS
#define US1C_TIMEOUT_MS 18u  /* no echo (or no end of echo) within this: NO RISE / NO FALL (2.5 m = 15 ms) */
#endif

bool US1C_Init(void);                  /* false: TIM2 not as expected - use the blocking US1_Measure instead */
void US1C_Update(uint32_t now_ms);     /* every main-loop pass: fires the next ping, publishes finished ones */
bool US1C_Take(US2_Reading *out, uint32_t *fired_ms); /* true once per new reading; fired_ms = when its ping was sent */
#endif
