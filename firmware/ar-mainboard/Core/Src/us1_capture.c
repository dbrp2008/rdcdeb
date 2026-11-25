#include "us1_capture.h"
#include "main.h"
#include "tim.h"

#define TRIG_US 20u
#define MIN_ECHO_US 100u

typedef enum { C_IDLE = 0, C_WAIT_RISE, C_WAIT_FALL, C_DONE } CapState;
static volatile CapState cap;
static volatile uint32_t rise_cnt, width_us;
static uint32_t fired_at, last_fire;
static bool ready, have_new;
static US2_Reading result;
static uint32_t result_fired;

/* TIM2 CH1 capture: rising edge -> remember the count, switch to falling; falling edge -> pulse width done. */
void TIM2_IRQHandler(void) {
  const uint32_t sr = TIM2->SR;
  if (sr & TIM_SR_CC1IF) {
    const uint32_t ccr = TIM2->CCR1; /* reading CCR1 clears CC1IF */
    if (cap == C_WAIT_RISE) {
      rise_cnt = ccr;
      cap = C_WAIT_FALL;
      TIM2->CCER |= TIM_CCER_CC1P; /* next: falling edge */
    } else if (cap == C_WAIT_FALL) {
      width_us = ccr - rise_cnt; /* 32-bit counter: wrap-safe */
      cap = C_DONE;
      TIM2->CCER &= ~TIM_CCER_CC1P;
    }
  }
  TIM2->SR = ~(TIM_SR_CC1IF | TIM_SR_CC1OF); /* clear CC1 flags only (rc_w0) */
}

bool US1C_Init(void) {
  uint32_t timer_hz = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1_2) != 0u) timer_hz *= 2u;
  if (htim2.Instance != TIM2 || timer_hz != 1000000u * (htim2.Init.Prescaler + 1u) || htim2.Init.Period != UINT32_MAX ||
      (TIM2->CR1 & TIM_CR1_CEN) == 0u)
    return ready = false; /* US2_Init did not start TIM2 at 1 MHz / 32 bit */
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = US1_ECHO_Pin; /* PA0 -> TIM2_CH1; pull-down keeps an unplugged echo line low */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLDOWN;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(US1_ECHO_GPIO_Port, &gpio);
  HAL_GPIO_WritePin(US1_TRIG_GPIO_Port, US1_TRIG_Pin, GPIO_PIN_RESET);
  /* CH1 = input capture from TI1, no prescaler / filter, rising edge, interrupt on capture */
  TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP);
  TIM2->CCMR1 = (TIM2->CCMR1 & ~(TIM_CCMR1_CC1S | TIM_CCMR1_IC1PSC | TIM_CCMR1_IC1F)) | TIM_CCMR1_CC1S_0;
  TIM2->CCER |= TIM_CCER_CC1E;
  TIM2->SR = ~(TIM_SR_CC1IF | TIM_SR_CC1OF);
  TIM2->DIER |= TIM_DIER_CC1IE;
  if ((TIM2->CCMR1 & TIM_CCMR1_CC1S) != TIM_CCMR1_CC1S_0) return ready = false;
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  cap = C_IDLE;
  have_new = false;
  last_fire = HAL_GetTick() - US1C_PERIOD_MS;
  return ready = true;
}

static void publish(US2_Reading r) {
  result = r;
  result_fired = fired_at;
  have_new = true;
}

void US1C_Update(uint32_t now) {
  if (!ready) return;
  if (cap == C_DONE) {
    const uint32_t w = width_us;
    cap = C_IDLE;
    if (w < MIN_ECHO_US) publish((US2_Reading){.status = US2_SHORT_PULSE, .pulse_us = w});
    else publish((US2_Reading){.status = US2_OK, .pulse_us = w, .raw_mm = (w * 343u + 1000u) / 2000u});
  } else if (cap != C_IDLE) {
    if ((uint32_t)(now - fired_at) < US1C_TIMEOUT_MS) return;
    const CapState was = cap;
    cap = C_IDLE; /* a late edge after this is ignored */
    TIM2->CCER &= ~TIM_CCER_CC1P;
    publish((US2_Reading){.status = was == C_WAIT_RISE ? US2_NO_RISE : US2_NO_FALL});
  }
  if (cap != C_IDLE || (uint32_t)(now - last_fire) < US1C_PERIOD_MS) return;
  last_fire = fired_at = now;
  if (HAL_GPIO_ReadPin(US1_ECHO_GPIO_Port, US1_ECHO_Pin) == GPIO_PIN_SET) { /* echo stuck high: report, do not fire */
    publish((US2_Reading){.status = US2_ECHO_BUSY});
    return;
  }
  TIM2->CCER &= ~TIM_CCER_CC1P; /* wait for the rising edge */
  TIM2->SR = ~(TIM_SR_CC1IF | TIM_SR_CC1OF);
  cap = C_WAIT_RISE;
  HAL_GPIO_WritePin(US1_TRIG_GPIO_Port, US1_TRIG_Pin, GPIO_PIN_SET); /* 20 us trigger: the only wait left */
  const uint32_t t0 = TIM2->CNT;
  for (uint32_t spin = 0; spin < 20000u && (uint32_t)(TIM2->CNT - t0) < TRIG_US; spin++) {}
  HAL_GPIO_WritePin(US1_TRIG_GPIO_Port, US1_TRIG_Pin, GPIO_PIN_RESET);
}

bool US1C_Take(US2_Reading *out, uint32_t *fired_ms) {
  if (!have_new) return false;
  have_new = false;
  *out = result;
  if (fired_ms) *fired_ms = result_fired;
  return true;
}
