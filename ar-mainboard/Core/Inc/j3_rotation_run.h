#ifndef J3_ROTATION_RUN_H
#define J3_ROTATION_RUN_H
#include <stdbool.h>
#include <stdint.h>
#include "rotation_estimate.h"

/* Motor command, not a safety function. Requires independent motor-power E-stop. */
#define J3_RUN_TARGET_EVENTS (10u * ROTATION_J3_EVENTS_PER_REV)
#define J3_RUN_SPEED_PERCENT 40.0f

typedef enum { J3_RUN_IDLE, J3_RUN_ACTIVE, J3_RUN_COMPLETE,
               J3_RUN_ABORTED, J3_RUN_NO_TIMER } J3RunStatus;
void J3Run_Init(void);
J3RunStatus J3Run_Status(void);
/* Timer-present only; the unchanged reference driver does not report HAL start status. */
bool J3Run_Start(bool j3_timer_present, uint32_t current_events);
/* Call often; no timeout. Event count is raw and may not equal real rotation. */
void J3Run_Update(uint32_t current_events);
void J3Run_Abort(void);
#endif
