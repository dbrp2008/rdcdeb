#include "tr_arm.h"

#if TR_ARM_ENABLED

#include "tim.h"

#define SERVO_ABS_MIN_US 500U
#define SERVO_ABS_MAX_US 2500U

_Static_assert(TR_ARM_GRIP_US >= SERVO_ABS_MIN_US && TR_ARM_GRIP_US <= SERVO_ABS_MAX_US,
               "TR_ARM_GRIP_US must be within 500..2500 us");
_Static_assert(TR_ARM_OUT_US >= SERVO_ABS_MIN_US && TR_ARM_OUT_US <= SERVO_ABS_MAX_US,
               "TR_ARM_OUT_US must be within 500..2500 us");

static float pos_us = (float)TR_ARM_START_US;
static uint32_t last_tick = 0;

static void write_pulse(void) {
  __HAL_TIM_SET_COMPARE(&TR_ARM_TIM, TR_ARM_CHANNEL, (uint32_t)(pos_us + 0.5f));
}

void TRArm_Init(void) {
  pos_us = (float)TR_ARM_START_US;
  write_pulse(); /* set the pulse first so the servo doesn't see a stray value when PWM starts */
  HAL_TIM_PWM_Start(&TR_ARM_TIM, TR_ARM_CHANNEL);
  last_tick = HAL_GetTick();
}

void TRArm_Update(int8_t cmd) {
  uint32_t now = HAL_GetTick();
  float dt = (float)(now - last_tick) / 1000.0f;
  last_tick = now;
  if (dt > 0.05f)
    dt = 0.05f;

  if (cmd == 0)
    return; /* hold */

  float target = (cmd > 0) ? (float)TR_ARM_GRIP_US : (float)TR_ARM_OUT_US;
  float step = TR_ARM_SPEED_US_PER_S * dt;
  float d = target - pos_us;
  if (d > step)
    pos_us += step;
  else if (d < -step)
    pos_us -= step;
  else
    pos_us = target;
  write_pulse();
}

uint16_t TRArm_GetPulseUs(void) { return (uint16_t)(pos_us + 0.5f); }

#else /* arm disabled: no servo timer needed; everything below does nothing */

void TRArm_Init(void) {}
void TRArm_Update(int8_t cmd) { (void)cmd; }
uint16_t TRArm_GetPulseUs(void) { return 0; }

#endif /* TR_ARM_ENABLED */
