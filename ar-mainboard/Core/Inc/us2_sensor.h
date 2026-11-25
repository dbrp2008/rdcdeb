#ifndef US2_SENSOR_H
#define US2_SENSOR_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  US2_OK, US2_ECHO_BUSY, US2_NO_RISE, US2_NO_FALL, US2_SHORT_PULSE, US2_TIMER_ERROR
} US2_Status;
typedef struct {
  US2_Status status;
  uint32_t pulse_us;
  uint32_t raw_mm;
} US2_Reading;
/* PB11 trigger, PB10 echo (swapped from the original pin mapping).
 * Starts TIM2 at 1 MHz shared by US1/US2/US3. Echoes use GPIO polling, not input capture. */
bool US2_Init(void);
US2_Reading US2_Measure(void); /* blocking, up to ~20 ms when the tick works */
const char *US2_StatusText(US2_Status status);
#endif
