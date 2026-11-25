#ifndef TR_SENSORS_H
#define TR_SENSORS_H

/**
 * Titan Robot sensors: two digital IR sensors (D0), left and right, and one ultrasonic sensor.
 *
 *   Left = PB10   Right = PB11   (left and right as seen from behind the robot, looking forward)
 *   Ultrasonic: TRIG = PA0, ECHO = PA1, echo timed with TIM2 (set up here; no CubeMX timer needed)
 *
 * The pins are also configured in code, so they work even if CubeMX leaves them in another mode.
 * BTN1 on the MainBoard switches the screen between the drive page and the sensors page.
 */

#include <stdbool.h>
#include <stdint.h>

/* D0 level when the sensor DETECTS something (line / object). Most modules read LOW when they detect a reflective
 * surface and HIGH when they do not. If the sensors page shows "0" for detected, flip this. */
#ifndef TR_IR_DETECT_LEVEL
#define TR_IR_DETECT_LEVEL 0
#endif

/* Ping the ultrasonic only while the sensors page is open (0), or all the time (1). A ping waits up to
 * TR_US_TIMEOUT_US for the echo, which briefly delays the drive loop when nothing is in range, so the default
 * leaves it off during driving. */
#ifndef TR_US_ALWAYS
#define TR_US_ALWAYS 0
#endif
#ifndef TR_US_TIMEOUT_US
#define TR_US_TIMEOUT_US 7000U /* about 1.2 m */
#endif
#ifndef TR_US_PERIOD_MS
#define TR_US_PERIOD_MS 100U
#endif
/* Added to every distance (mm) to correct a constant error found when measuring against a ruler. */
#ifndef TR_US_OFFSET_MM
#define TR_US_OFFSET_MM 0
#endif

typedef struct {
  bool ir[2];       /* [0] = left, [1] = right: true = this sensor detects (after TR_IR_DETECT_LEVEL) */
  bool ir_raw[2];   /* raw pin level, true = HIGH */
  bool us_valid;    /* a distance was measured recently */
  int32_t us_mm;    /* distance in mm, only meaningful if us_valid */
  uint32_t us_echo_us; /* echo pulse width, for diagnosis */
  bool us_timer_ok; /* TIM2 clock could be set to 1 MHz */
  uint32_t us_seq;  /* counts every successful ping, so a caller can tell a NEW reading */
  uint8_t us_reason; /* why the last ping failed: 0 ok, 1 echo stuck high, 2 no echo start, 3 echo never ended, 4 pulse too short, 5 trigger timing fault */
} TRSensors_Status;

void TRSensors_Init(void);
/* Call every main-loop pass: reads the IR pins, pings the ultrasonic when due, handles BTN1 (page toggle). */
void TRSensors_Update(void);
void TRSensors_Get(TRSensors_Status *out);
/* Keep pinging the ultrasonic even when the sensors page is closed (the autonomous routine needs it). fast = every 50 ms. */
void TRSensors_UsKeepAlive(bool on, bool fast);
/* true while the sensors page should be shown instead of the drive page */
bool TRSensors_PageActive(void);
/* Draws the sensors page (rows 0..9); the caller then calls tft_update(). */
void TRSensors_DrawPage(void);

#endif
