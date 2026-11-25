#include "motor_sequence.h"
#include "elcp_motor/elcp_motor.h"

#define BUTTON_DEBOUNCE_MS 30u

static MotorSequencePhase phase;
static uint32_t phase_started;
static bool button_candidate;
static bool button_stable;
static bool retry_armed;
static uint32_t button_changed;

static uint32_t duration_ms(MotorSequencePhase value) {
  switch (value) {
    case MOTOR_SEQ_ZERO:
    case MOTOR_SEQ_J1:
    case MOTOR_SEQ_J3: return 5000u;
    case MOTOR_SEQ_BOTH_25:
    case MOTOR_SEQ_BOTH_75: return 2500u;
    default: return 0u;
  }
}

/* Update the library's software command as well as the actual timer compare.
   Use ARR+1 and round to the nearest count (ARR=1049: 25%=263, 50%=525,
   75%=788). The timer is preloaded, so a write takes effect on an update. */
static void request_duty(ELCP_Motor_ID id, unsigned percent) {
  TIM_HandleTypeDef *timer = id == MOTOR_J1 ? &htim12 : &htim8;
  uint32_t channel = id == MOTOR_J1 ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
  ELCP_Motor_SetSpeed(id, (float)percent);
  /* TIM8 and TIM12 are 16-bit; period * percent fits uint32_t. */
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(timer) + 1u;
  __HAL_TIM_SET_COMPARE(timer, channel, (period * percent + 50u) / 100u);
}

static void enter_phase(MotorSequencePhase next, uint32_t now_ms, bool pressed) {
  phase = next;
  phase_started = now_ms;
  switch (next) {
    case MOTOR_SEQ_ZERO:
      request_duty(MOTOR_J1, 0u); request_duty(MOTOR_J3, 0u); break;
    case MOTOR_SEQ_J1:
      request_duty(MOTOR_J1, 50u); request_duty(MOTOR_J3, 0u); break;
    case MOTOR_SEQ_J3:
      request_duty(MOTOR_J1, 0u); request_duty(MOTOR_J3, 50u); break;
    case MOTOR_SEQ_BOTH_25:
      request_duty(MOTOR_J1, 25u); request_duty(MOTOR_J3, 25u); break;
    case MOTOR_SEQ_BOTH_75:
      request_duty(MOTOR_J1, 75u); request_duty(MOTOR_J3, 75u); break;
    case MOTOR_SEQ_WAIT:
      request_duty(MOTOR_J1, 0u); request_duty(MOTOR_J3, 0u);
      button_candidate = pressed;
      button_stable = pressed;
      button_changed = now_ms;
      retry_armed = !pressed;
      break;
    case MOTOR_SEQ_ERROR: break;
  }
}

void MotorSequence_Init(uint32_t now_ms, bool btn1_pressed) {
  if (!ELCP_Motor_IsInitialized(MOTOR_J1) || !ELCP_Motor_IsInitialized(MOTOR_J3)) {
    phase = MOTOR_SEQ_ERROR;
    return;
  }
  enter_phase(MOTOR_SEQ_ZERO, now_ms, btn1_pressed);
}

void MotorSequence_Update(uint32_t now_ms, bool btn1_pressed) {
  if (phase == MOTOR_SEQ_WAIT) {
    if (btn1_pressed != button_candidate) {
      button_candidate = btn1_pressed;
      button_changed = now_ms;
    }
    if (button_candidate != button_stable &&
        (uint32_t)(now_ms - button_changed) >= BUTTON_DEBOUNCE_MS)
      button_stable = button_candidate;
    if (!button_stable) retry_armed = true;
    if (button_stable && retry_armed) {
      retry_armed = false;
      enter_phase(MOTOR_SEQ_ZERO, now_ms, btn1_pressed);
    }
    return;
  }
  uint32_t duration = duration_ms(phase);
  if (duration == 0u || (uint32_t)(now_ms - phase_started) < duration)
    return;
  switch (phase) {
    case MOTOR_SEQ_ZERO: enter_phase(MOTOR_SEQ_J1, now_ms, btn1_pressed); break;
    case MOTOR_SEQ_J1: enter_phase(MOTOR_SEQ_J3, now_ms, btn1_pressed); break;
    case MOTOR_SEQ_J3: enter_phase(MOTOR_SEQ_BOTH_25, now_ms, btn1_pressed); break;
    case MOTOR_SEQ_BOTH_25: enter_phase(MOTOR_SEQ_BOTH_75, now_ms, btn1_pressed); break;
    case MOTOR_SEQ_BOTH_75: enter_phase(MOTOR_SEQ_WAIT, now_ms, btn1_pressed); break;
    default: break;
  }
}

MotorSequencePhase MotorSequence_GetPhase(void) { return phase; }

uint32_t MotorSequence_RemainingMs(uint32_t now_ms) {
  uint32_t duration = duration_ms(phase);
  uint32_t elapsed = now_ms - phase_started;
  return elapsed < duration ? duration - elapsed : 0u;
}
