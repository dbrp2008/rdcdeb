#include "j3_rotation_run.h"
#include "elcp_motor/elcp_motor.h"

_Static_assert(ROTATION_J3_EVENTS_PER_REV <= UINT32_MAX / 10u, "J3 target overflows");

static J3RunStatus status;
static uint32_t start_events;

void J3Run_Init(void) { status = J3_RUN_IDLE; start_events = 0u; }
J3RunStatus J3Run_Status(void) { return status; }
bool J3Run_Start(bool j3_timer_present, uint32_t current_events) {
  if (status == J3_RUN_ACTIVE) return false;
  if (!j3_timer_present) { status = J3_RUN_NO_TIMER; return false; }
  start_events = current_events;
  status = J3_RUN_ACTIVE;
  ELCP_Motor_SetSpeed(MOTOR_J3, J3_RUN_SPEED_PERCENT);
  return true;
}
void J3Run_Update(uint32_t current_events) {
  if (status == J3_RUN_ACTIVE && (uint32_t)(current_events - start_events) >= J3_RUN_TARGET_EVENTS) {
    ELCP_Motor_Stop(MOTOR_J3);
    status = J3_RUN_COMPLETE;
  }
}
void J3Run_Abort(void) {
  if (status == J3_RUN_ACTIVE) {
    ELCP_Motor_Stop(MOTOR_J3);
    status = J3_RUN_ABORTED;
  }
}
