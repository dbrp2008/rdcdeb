#include "elcp_zero_pwm.h"
#include "elcp_motor/elcp_motor.h"
#include "main.h"
#include "tim.h"
#include "enc_capture.h"

void ELCP_ZeroPWM_Hold(void) {
  /* Safe to repeat after MX_GPIO_Init, even if only one timer is valid. */
  if (htim12.Instance == TIM12) __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0u);
  if (htim8.Instance == TIM8) __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0u);
  HAL_GPIO_WritePin(ELCP_J1_DIR_GPIO_Port, ELCP_J1_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ELCP_J3_DIR_GPIO_Port, ELCP_J3_DIR_Pin, GPIO_PIN_RESET);
}

ELCP_ZeroPWM_Result ELCP_ZeroPWM_Init(void) {
  ELCP_ZeroPWM_Result result = {.j1_ok = htim12.Instance == TIM12, .j3_ok = htim8.Instance == TIM8};
  ELCP_ZeroPWM_Hold(); /* prepare both channels before starting either */
  if (result.j1_ok) ELCP_Motor_Init(MOTOR_J1);
  if (result.j3_ok) ELCP_Motor_Init(MOTOR_J3);
  EncCapture_Start(); /* FB pulses: timer capture with the noise filter, not EXTI (register-level: either wheel alone works) */
  /* The reference init discards HAL_PWM_Start status, so these indicate only
   * whether the request was made, not that PWM start succeeded. */
  return result;
}
