#include "us2_sensor.h"
#include "us1_sensor.h"
#include "us3_sensor.h"
#include "main.h"
#include "tim.h"

#define US2_TIMER_HZ 1000000u
#define US2_ECHO_TIMEOUT_US 15000u
#define US2_ECHO_WATCHDOG_MS 20u
#define US2_DELAY_WATCHDOG_MS 2u
#define US2_MIN_ECHO_US 100u

bool US2_Init(void) {
  GPIO_InitTypeDef gpio = {0};
  /* Preload trigger low before enabling the output. */
  HAL_GPIO_WritePin(US2_TRIG_GPIO_Port, US2_TRIG_Pin, GPIO_PIN_RESET);
  gpio.Pin = US2_TRIG_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US2_TRIG_GPIO_Port, &gpio);
  gpio.Pin = US2_ECHO_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(US2_ECHO_GPIO_Port, &gpio);

  uint32_t timer_hz = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1_2) != 0u) timer_hz *= 2u;
  /* Reject a changed CubeMX clock/period rather than silently mismeasure. */
  if (htim2.Instance != TIM2 || timer_hz != US2_TIMER_HZ * (htim2.Init.Prescaler + 1u) ||
      htim2.Init.Period != UINT32_MAX) return false;
  return HAL_TIM_Base_Start(&htim2) == HAL_OK;
}
static bool delay_us(uint32_t us) {
  const uint32_t start = TIM2->CNT, ms = HAL_GetTick();
  while ((uint32_t)(TIM2->CNT - start) < us)
    if ((uint32_t)(HAL_GetTick() - ms) >= US2_DELAY_WATCHDOG_MS) return false;
  return true;
}
static bool wait_echo(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState target, uint32_t start, uint32_t ms, uint32_t *edge) {
  for (;;) {
    GPIO_PinState level = HAL_GPIO_ReadPin(port, pin);
    uint32_t now = TIM2->CNT;
    if ((uint32_t)(now - start) >= US2_ECHO_TIMEOUT_US ||
        (uint32_t)(HAL_GetTick() - ms) >= US2_ECHO_WATCHDOG_MS) return false;
    if (level == target) { *edge = now; return true; }
  }
}
static US2_Reading measure(GPIO_TypeDef *trig_port, uint16_t trig_pin,
                           GPIO_TypeDef *echo_port, uint16_t echo_pin) {
  HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
  if (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_SET)
    return (US2_Reading){.status = US2_ECHO_BUSY};
  if (!delay_us(2u)) return (US2_Reading){.status = US2_TIMER_ERROR};
  HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
  const bool ok = delay_us(20u);
  HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
  if (!ok) return (US2_Reading){.status = US2_TIMER_ERROR};
  const uint32_t start = TIM2->CNT, ms = HAL_GetTick();
  uint32_t rise, fall;
  if (!wait_echo(echo_port, echo_pin, GPIO_PIN_SET, start, ms, &rise)) return (US2_Reading){.status = US2_NO_RISE};
  if (!wait_echo(echo_port, echo_pin, GPIO_PIN_RESET, start, ms, &fall)) return (US2_Reading){.status = US2_NO_FALL};
  uint32_t width = fall - rise;
  if (width < US2_MIN_ECHO_US) return (US2_Reading){.status = US2_SHORT_PULSE, .pulse_us = width};
  return (US2_Reading){.status = US2_OK, .pulse_us = width,
                       .raw_mm = (width * 343u + 1000u) / 2000u};
}
US2_Reading US1_Measure(void) {
  return measure(US1_TRIG_GPIO_Port, US1_TRIG_Pin, US1_ECHO_GPIO_Port, US1_ECHO_Pin);
}
US2_Reading US2_Measure(void) {
  return measure(US2_TRIG_GPIO_Port, US2_TRIG_Pin, US2_ECHO_GPIO_Port, US2_ECHO_Pin);
}
US2_Reading US3_Measure(void) {
  return measure(US3_TRIG_GPIO_Port, US3_TRIG_Pin, US3_ECHO_GPIO_Port, US3_ECHO_Pin);
}
const char *US2_StatusText(US2_Status status) {
  switch (status) {
    case US2_OK: return "ECHO OK";
    case US2_ECHO_BUSY: return "ECHO HIGH";
    case US2_NO_RISE: return "NO ECHO RISE";
    case US2_NO_FALL: return "NO ECHO FALL";
    case US2_SHORT_PULSE: return "SHORT PULSE";
    default: return "CHECK TIM2";
  }
}
