#ifndef ENC_CAPTURE_H
#define ENC_CAPTURE_H
/* Encoder FB pulses through timer input capture (MainBoard.ioc: PB15 = TIM12_CH2, PC8 = TIM8_CH3, rising, filter 15,
 * clock division /4), not EXTI. The capture input's digital filter only passes a level held for 8 samples at fDTS/32:
 * TIM12 84 MHz / 4 / 32 -> ~12 us, TIM8 168 MHz / 4 / 32 -> ~6 us. Shorter spikes (motor-electronics noise) never count;
 * real FB pulses are milliseconds apart.
 * Each captured edge goes to HAL_GPIO_EXTI_Callback(FB pin), exactly as the EXTI line did: in the drive image that is
 * side_ir.c (bounce filter, raw count, then the reference ELCP driver); in the bench image, encoder_monitor.c.
 * This module owns TIM8_BRK_TIM12_IRQHandler and TIM8_CC_IRQHandler: leave those two UNTICKED in CubeMX's NVIC (or
 * CubeMX generates duplicates). Call EncCapture_Start once after the timers are initialised and the PWM started. */
void EncCapture_Start(void);
#endif
