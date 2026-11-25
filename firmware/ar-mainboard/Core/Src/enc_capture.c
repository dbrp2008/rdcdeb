#include "enc_capture.h"
#include "main.h"
#include "tim.h"

void HAL_GPIO_EXTI_Callback(uint16_t pin); /* the image's EXTI dispatcher (side_ir.c / encoder_monitor.c) */

/* Re-assert the capture set-up in the registers, whatever the generated code did: CC input on its own TI, rising, no
 * prescaler, filter 15, clock division /4, pull-up on the pin. Then enable its interrupt. */
void EncCapture_Start(void) {
  __HAL_RCC_TIM12_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();
  /* pull-ups (keeps the alternate-function mode): PB15 = bits 31:30, PC8 = bits 17:16, 01 = pull-up */
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << 30)) | (1u << 30);
  GPIOC->PUPDR = (GPIOC->PUPDR & ~(3u << 16)) | (1u << 16);

  /* TIM12 CH2 (J1 right): CCMR1 CC2S = 01 (TI2), IC2PSC = 0, IC2F = 15; CCER CC2P = CC2NP = 0 (rising), CC2E */
  TIM12->CCER &= ~TIM_CCER_CC2E;
  TIM12->CCMR1 = (TIM12->CCMR1 & ~(TIM_CCMR1_CC2S | TIM_CCMR1_IC2PSC | TIM_CCMR1_IC2F)) | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC2F;
  TIM12->CR1 = (TIM12->CR1 & ~TIM_CR1_CKD) | TIM_CR1_CKD_1;
  TIM12->CCER = (TIM12->CCER & ~(TIM_CCER_CC2P | TIM_CCER_CC2NP)) | TIM_CCER_CC2E;
  /* TIM8 CH3 (J3 left): CCMR2 CC3S = 01 (TI3), IC3PSC = 0, IC3F = 15; CCER CC3P = CC3NP = 0, CC3E */
  TIM8->CCER &= ~TIM_CCER_CC3E;
  TIM8->CCMR2 = (TIM8->CCMR2 & ~(TIM_CCMR2_CC3S | TIM_CCMR2_IC3PSC | TIM_CCMR2_IC3F)) | TIM_CCMR2_CC3S_0 | TIM_CCMR2_IC3F;
  TIM8->CR1 = (TIM8->CR1 & ~TIM_CR1_CKD) | TIM_CR1_CKD_1;
  TIM8->CCER = (TIM8->CCER & ~(TIM_CCER_CC3P | TIM_CCER_CC3NP)) | TIM_CCER_CC3E;

  /* the counters normally already run (PWM); harmless if they do */
  TIM12->CR1 |= TIM_CR1_CEN;
  TIM8->CR1 |= TIM_CR1_CEN;

  TIM12->SR = ~(TIM_SR_CC2IF | TIM_SR_CC2OF);
  TIM8->SR = ~(TIM_SR_CC3IF | TIM_SR_CC3OF);
  TIM12->DIER |= TIM_DIER_CC2IE;
  TIM8->DIER |= TIM_DIER_CC3IE;
  /* same priority as the side-IR EXTI lines the FB pins used to share */
  HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
  HAL_NVIC_SetPriority(TIM8_CC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
}

/* An overcapture (CCxOF: a second edge before this one was serviced) still counts as one: at ~60-600 pulses/s an
 * interrupt is never that late, so OF only follows noise the filter let through. */
void TIM8_BRK_TIM12_IRQHandler(void) {
  const uint32_t sr = TIM12->SR;
  if (sr & TIM_SR_CC2IF) {
    (void)TIM12->CCR2; /* reading CCR2 clears CC2IF */
    TIM12->SR = ~TIM_SR_CC2OF;
    HAL_GPIO_EXTI_Callback(ELCP_J1_FB_Pin);
  }
}

void TIM8_CC_IRQHandler(void) {
  const uint32_t sr = TIM8->SR;
  if (sr & TIM_SR_CC3IF) {
    (void)TIM8->CCR3; /* reading CCR3 clears CC3IF */
    TIM8->SR = ~TIM_SR_CC3OF;
    HAL_GPIO_EXTI_Callback(ELCP_J3_FB_Pin);
  }
}
