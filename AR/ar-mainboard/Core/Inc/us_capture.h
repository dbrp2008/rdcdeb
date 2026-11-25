#ifndef US_CAPTURE_H
#define US_CAPTURE_H
/* Include "main.h" first. Ultrasonics read by TIMER INPUT CAPTURE + INTERRUPT, no waiting in the main loop.
 *   US_FC  front-centre : trig PA1, echo PA0  = TIM2 CH1
 *   US_FR  front-right  : UART1 header: trig PA9, echo PA10 = TIM1 CH3 (set up in code here)
 *   US_BR  back-right   : trig PB9 (I2C1 header), echo PB8 = TIM4 CH3 (set up in code here; PB8 has no EXTI, line 8 is PC8)
 * Sensors are fired one at a time (no crosstalk). */
#include "us_capture_core.h"

bool US_Init(void);                       /* needs TIM2 at 1 MHz (prescaler 83, 32-bit); false if the clock/config is wrong */
/* Diagnostics: init code (0 ok; 1 TIM2 clock/period; 2 TIM4 setup; 3 TIM4 IC init; 4/5/6 configuring or starting TIM2 CH1 / TIM2 CH3 / TIM4 CH3, 7 capture mode not taken),
 * trigger pulses sent, capture interrupts seen. */
uint8_t US_InitCode(void);
uint32_t US_ShotCount(void);
uint32_t US_IrqCount(void);
uint8_t US_EchoSeenMask(void); /* bit0 FC, bit1 FR, bit2 BR: echo pin seen high by polling */
typedef struct {
  uint32_t cnt2, sr2, ccer2, dier2, cr1_2, cnt4;
  uint8_t pa0, pb10, pb8, pa0_mode, pb10_mode, pb8_mode, nvic_tim2, afr_pa0, afr_pb10, afr_pb8;
  uint32_t ccmr1, ccmr2, sr4, ccer4, dier4;
} US_Debug;
void US_DebugSnapshot(US_Debug *d); /* raw timer / pin state for the all-US test screen */
void US_SetEnabled(uint8_t mask);         /* bit i = US_Id i is measured (default: centre only) */
void US_Update(uint32_t now_ms);          /* call every main-loop pass */
bool US_Take(US_Id id, US2_Reading *out); /* true once for each new reading */
#endif
