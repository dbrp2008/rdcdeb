#include "main.h"
#include "tim.h"
#include "us_capture.h"

#define US_TIMEOUT_MS 18u /* echo of a 2.5 m target is 15 ms */
#define US_GUARD_MS 8u    /* quiet gap after one sensor, before firing the next */
#define US_PERIOD_MS 30u  /* minimum time between two shots of the same sensor */
#define US_TRIG_US 12u

/* 1 (default): TIM4 comes from CubeMX (MX_TIM4_Init, htim4, TIM4_IRQHandler in stm32f4xx_it.c: PB8 = TIM4_CH3 input capture,
 * prescaler 83, period 65535, TIM4 global interrupt on). 0: this file sets TIM4 up itself and owns TIM4_IRQHandler. */
#ifndef US_TIM4_FROM_CUBEMX
#define US_TIM4_FROM_CUBEMX 1
#endif
#if US_TIM4_FROM_CUBEMX
#define htim4_us htim4
#else
static TIM_HandleTypeDef htim4_us;
#endif
/* Front-right sensor is on the UART1 header: PA9 (TX) = trigger, PA10 (RX) = echo = TIM1_CH3. TIM1 is set up in this file
 * (it is not in CubeMX). If that sensor stays silent, its two wires may be the other way round: build with
 * -DUS_FR_ECHO_ON_PA9=1 (trigger PA10, echo PA9 = TIM1_CH2). */
#ifndef US_FR_ECHO_ON_PA9
#define US_FR_ECHO_ON_PA9 0
#endif
#if US_FR_ECHO_ON_PA9
#define FR_TRIG_PIN GPIO_PIN_10
#define FR_ECHO_PIN GPIO_PIN_9
#define FR_CHANNEL TIM_CHANNEL_2
#else
#define FR_TRIG_PIN GPIO_PIN_9
#define FR_ECHO_PIN GPIO_PIN_10
#define FR_CHANNEL TIM_CHANNEL_3
#endif
static TIM_HandleTypeDef htim1_us;
typedef struct {
  GPIO_TypeDef *trig_port; uint16_t trig_pin;
  GPIO_TypeDef *echo_port; uint16_t echo_pin;
  TIM_HandleTypeDef *tim; uint32_t channel; uint32_t mask;
} Sensor;
static const Sensor sensor[US_COUNT] = {
    {GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_0, &htim2, TIM_CHANNEL_1, 0xFFFFFFFFu},
    {GPIOA, FR_TRIG_PIN, GPIOA, FR_ECHO_PIN, &htim1_us, FR_CHANNEL, 0xFFFFu},
    {GPIOB, GPIO_PIN_9, GPIOB, GPIO_PIN_8, &htim4_us, TIM_CHANNEL_3, 0xFFFFu},
};
static uint8_t enabled = 1u, init_code;
static volatile uint32_t irq_count;
static uint32_t shot_count;
static uint8_t echo_seen; /* bit i: the echo pin of sensor i was seen HIGH while armed (polled, independent of the capture) */
static int current = -1, rr = 0;
static uint32_t last_start[US_COUNT], last_end;

static HAL_TIM_ActiveChannel active_of(uint32_t channel) {
  return channel == TIM_CHANNEL_1 ? HAL_TIM_ACTIVE_CHANNEL_1
         : (channel == TIM_CHANNEL_2 ? HAL_TIM_ACTIVE_CHANNEL_2 : HAL_TIM_ACTIVE_CHANNEL_3);
}

/* Capture interrupt (TIM2 handler is in stm32f4xx_it.c via HAL_TIM_IRQHandler; TIM4's is below). */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  irq_count++;
  for (int i = 0; i < US_COUNT; i++) {
    const Sensor *s = &sensor[i];
    if (htim != s->tim || htim->Channel != active_of(s->channel)) continue;
    const bool falling = USC_OnCapture((US_Id)i, HAL_TIM_ReadCapturedValue(htim, s->channel), s->mask);
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, s->channel,
                                  falling ? TIM_INPUTCHANNELPOLARITY_FALLING : TIM_INPUTCHANNELPOLARITY_RISING);
  }
}
void TIM1_CC_IRQHandler(void) { HAL_TIM_IRQHandler(&htim1_us); }
#if !US_TIM4_FROM_CUBEMX
void TIM4_IRQHandler(void) { HAL_TIM_IRQHandler(&htim4_us); }
#endif

bool US_Init(void) {
  uint32_t timer_hz = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1_2) != 0u) timer_hz *= 2u;
  /* Reject a changed CubeMX clock/period rather than silently mismeasure. TIM4 below uses the same 1 MHz. */
  if (htim2.Instance != TIM2 || timer_hz != 1000000u * (htim2.Init.Prescaler + 1u) ||
      htim2.Init.Period != UINT32_MAX) { init_code = 1; return false; }
  USC_Reset();
  GPIO_InitTypeDef gpio = {0};
  for (int i = 0; i < US_COUNT; i++) { /* trigger pins: outputs, low */
    HAL_GPIO_WritePin(sensor[i].trig_port, sensor[i].trig_pin, GPIO_PIN_RESET);
    gpio.Pin = sensor[i].trig_pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(sensor[i].trig_port, &gpio);
  }
  /* Echo pins: alternate function into the capture timers, set explicitly here (the pin routing from CubeMX is not trusted).
   * Pull-down keeps an unplugged / idle echo line low. */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLDOWN;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_0;  gpio.Alternate = GPIO_AF1_TIM2; HAL_GPIO_Init(GPIOA, &gpio); /* PA0  = TIM2_CH1 */
  gpio.Pin = FR_ECHO_PIN; gpio.Alternate = GPIO_AF1_TIM1; HAL_GPIO_Init(GPIOA, &gpio); /* PA10 = TIM1_CH3 (or PA9 = CH2) */
  gpio.Pin = GPIO_PIN_8;  gpio.Alternate = GPIO_AF2_TIM4; HAL_GPIO_Init(GPIOB, &gpio); /* PB8  = TIM4_CH3 */
#if US_TIM4_FROM_CUBEMX
  /* Reject a CubeMX TIM4 that is not 1 MHz / 16 bit. */
  if (htim4.Instance != TIM4 || htim4.Init.Prescaler != htim2.Init.Prescaler || htim4.Init.Period != 0xFFFFu) { init_code = 2; return false; }
#else
  /* TIM4 CH3 on PB8 (back-right echo), 1 MHz, 16 bit. */
  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_8;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &gpio);
  htim4_us.Instance = TIM4;
  htim4_us.Init.Prescaler = htim2.Init.Prescaler;
  htim4_us.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4_us.Init.Period = 0xFFFFu;
  htim4_us.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_IC_Init(&htim4_us) != HAL_OK) { init_code = 3; return false; }
  TIM_IC_InitTypeDef ic = {0};
  ic.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
  ic.ICPrescaler = TIM_ICPSC_DIV1;
  ic.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4_us, &ic, TIM_CHANNEL_3) != HAL_OK) { init_code = 3; return false; }
  HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
#endif
  /* TIM1 (front-right echo): 1 MHz, 16 bit, capture interrupt. TIM1 sits on APB2. */
  {
    uint32_t t1_hz = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2_2) != 0u) t1_hz *= 2u;
    if (t1_hz % 1000000u != 0u) { init_code = 8; return false; }
    __HAL_RCC_TIM1_CLK_ENABLE();
    htim1_us.Instance = TIM1;
    htim1_us.Init.Prescaler = t1_hz / 1000000u - 1u;
    htim1_us.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1_us.Init.Period = 0xFFFFu;
    htim1_us.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1_us.Init.RepetitionCounter = 0;
    if (HAL_TIM_IC_Init(&htim1_us) != HAL_OK) { init_code = 8; return false; }
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  }
  /* Put the capture channels in input-capture mode explicitly (rising edge, direct TI, no prescaler, no filter).
   * Found on the board: the CubeMX-generated TIM2 left CCMR1/CCMR2 at 0 (output mode), so nothing was ever captured. */
  {
    TIM_IC_InitTypeDef cfg = {0};
    cfg.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    cfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
    cfg.ICPrescaler = TIM_ICPSC_DIV1;
    cfg.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim2, &cfg, TIM_CHANNEL_1) != HAL_OK) { init_code = 4; return false; }
    if (HAL_TIM_IC_ConfigChannel(&htim1_us, &cfg, FR_CHANNEL) != HAL_OK) { init_code = 5; return false; }
    if (HAL_TIM_IC_ConfigChannel(&htim4_us, &cfg, TIM_CHANNEL_3) != HAL_OK) { init_code = 6; return false; }
    if ((TIM2->CCMR1 & 3u) != 1u || (((FR_CHANNEL == TIM_CHANNEL_3) ? TIM1->CCMR2 : TIM1->CCMR1) & 3u) != 1u || (TIM4->CCMR2 & 3u) != 1u) { init_code = 7; return false; }
  }
  /* Start counters + capture interrupts. */
  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK) { init_code = 4; return false; }
  if (HAL_TIM_IC_Start_IT(&htim1_us, FR_CHANNEL) != HAL_OK) { init_code = 5; return false; }
  if (HAL_TIM_IC_Start_IT(&htim4_us, TIM_CHANNEL_3) != HAL_OK) { init_code = 6; return false; }
  init_code = 0;
  return true;
}

uint8_t US_InitCode(void) { return init_code; }
void US_DebugSnapshot(US_Debug *d) {
  d->cnt2 = TIM2->CNT;
  d->sr2 = TIM2->SR;
  d->ccer2 = TIM2->CCER;
  d->dier2 = TIM2->DIER;
  d->cr1_2 = TIM2->CR1;
  d->cnt4 = TIM4->CNT;
  d->pa0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET;
  d->pb10 = HAL_GPIO_ReadPin(GPIOA, FR_ECHO_PIN) == GPIO_PIN_SET;
  d->pb8 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET;
  d->pa0_mode = (GPIOA->MODER >> 0) & 3u; /* 0 input, 1 output, 2 alternate, 3 analog */
  d->pb10_mode = (GPIOA->MODER >> (FR_ECHO_PIN == GPIO_PIN_10 ? 20 : 18)) & 3u;
  d->pb8_mode = (GPIOB->MODER >> 16) & 3u;
  d->afr_pa0 = (GPIOA->AFR[0] >> 0) & 0xFu;
  d->afr_pb10 = (GPIOA->AFR[1] >> (FR_ECHO_PIN == GPIO_PIN_10 ? 8 : 4)) & 0xFu;
  d->afr_pb8 = (GPIOB->AFR[1] >> 0) & 0xFu;
  d->ccmr1 = TIM2->CCMR1;
  d->ccmr2 = TIM2->CCMR2;
  d->sr4 = TIM4->SR;
  d->ccer4 = TIM4->CCER;
  d->dier4 = TIM4->DIER;
  d->nvic_tim2 = (NVIC->ISER[TIM2_IRQn >> 5] >> (TIM2_IRQn & 31u)) & 1u;
}
uint32_t US_IrqCount(void) { return irq_count; }
uint32_t US_ShotCount(void) { return shot_count; }
uint8_t US_EchoSeenMask(void) { return echo_seen; }

void US_SetEnabled(uint8_t mask) { enabled = mask; }

static void fire(int i, uint32_t now) {
  const Sensor *s = &sensor[i];
  if (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_SET) { /* echo stuck high: report it, do not fire */
    USC_Fail((US_Id)i, US2_ECHO_BUSY);
    return;
  }
  __HAL_TIM_SET_CAPTUREPOLARITY(s->tim, s->channel, TIM_INPUTCHANNELPOLARITY_RISING);
  __HAL_TIM_CLEAR_FLAG(s->tim, TIM_FLAG_CC1 << (s->channel / 4u));
  USC_Arm((US_Id)i, now);
  /* 12 us trigger pulse (the only busy-wait left; the echo is captured by the interrupt). */
  HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
  const uint32_t t0 = TIM2->CNT;
  for (uint32_t spin = 0; spin < 20000u && (uint32_t)(TIM2->CNT - t0) < US_TRIG_US; spin++) {} /* bounded: never hangs */
  shot_count++;
  HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);
  last_start[i] = now;
  current = i;
}

void US_Update(uint32_t now) {
  if (current >= 0) {
    if (HAL_GPIO_ReadPin(sensor[current].echo_port, sensor[current].echo_pin) == GPIO_PIN_SET) echo_seen |= (uint8_t)(1u << current);
    if (!USC_Poll((US_Id)current, now, US_TIMEOUT_MS)) return;
    current = -1;
    last_end = now;
  }
  if ((uint32_t)(now - last_end) < US_GUARD_MS) return;
  for (int k = 0; k < US_COUNT; k++) {
    const int i = (rr + 1 + k) % US_COUNT;
    if (!(enabled & (1u << i)) || (uint32_t)(now - last_start[i]) < US_PERIOD_MS) continue;
    rr = i;
    fire(i, now);
    return;
  }
}

bool US_Take(US_Id id, US2_Reading *out) { return USC_Take(id, out); }
