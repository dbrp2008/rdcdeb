#include "rotation_estimate.h"
#include "main.h"
#include "elcp_motor/elcp_motor.h"

_Static_assert(ROTATION_J1_EVENTS_PER_REV > 0u && ROTATION_J1_EVENTS_PER_REV <= UINT32_MAX / 100u, "J1 scale invalid");
_Static_assert(ROTATION_J3_EVENTS_PER_REV > 0u && ROTATION_J3_EVENTS_PER_REV <= UINT32_MAX / 100u, "J3 scale invalid");

/* Command-gated interrupt observations, not verified physical rotations. */
volatile uint32_t rotation_j1_events, rotation_j3_events;

void RotationEstimate_RecordIRQ(uint16_t pin) {
  ELCP_Motor_ID id;
  volatile uint32_t *events;
  if (pin == ELCP_J1_FB_Pin) { id = MOTOR_J1; events = &rotation_j1_events; }
  else if (pin == ELCP_J3_FB_Pin) { id = MOTOR_J3; events = &rotation_j3_events; }
  else return;

  /* Match the reference zero-command gate. This reads the driver's requested
   * speed, not a measurement of PWM voltage, HAL success, or shaft motion. */
  const ELCP_Motor_State_t *motor = ELCP_Motor_GetState(id);
  if (motor != 0 && motor->current_speed_pct != 0.0f) ++*events;
}

static RotationEstimate from_events(uint32_t events, uint32_t per_rev) {
  /* All callers use the two bounded, nonzero compile-time scales. */
  return (RotationEstimate){events / per_rev, ((events % per_rev) * 100u) / per_rev};
}
RotationEstimate RotationEstimate_J1(uint32_t events) {
  return from_events(events, ROTATION_J1_EVENTS_PER_REV);
}
RotationEstimate RotationEstimate_J3(uint32_t events) {
  return from_events(events, ROTATION_J3_EVENTS_PER_REV);
}
