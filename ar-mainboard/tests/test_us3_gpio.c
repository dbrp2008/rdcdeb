#include "gpio.h"
#include <assert.h>
#include <stdio.h>
GPIO_TypeDef ga,gb,gc;
static unsigned l2, r2, j1_fb, j3_fb, n4, n9_5, n15_10;
static int trig_preloaded, trig_configs, echo_configs, us2_low, us2_trig, us2_echo, us1_low, us1_trig, us1_echo, j1_low, j3_low, dir_configs;
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pins, uint32_t value) {
  if(port==GPIOB && (pins & GPIO_PIN_11)) { assert(value==GPIO_PIN_RESET); us2_low=1; }
  if(port==GPIOA && (pins & GPIO_PIN_1)) { assert(value==GPIO_PIN_RESET); us1_low=1; }
  if(port==GPIOB && (pins & GPIO_PIN_9)) { assert(value==GPIO_PIN_RESET); trig_preloaded=1; }
  if(port==GPIOC && (pins & GPIO_PIN_6)) { assert(value==GPIO_PIN_RESET); j1_low=1; }
  if(port==GPIOC && (pins & GPIO_PIN_9)) { assert(value==GPIO_PIN_RESET); j3_low=1; }
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *cfg) {
  if(port==GPIOC && (cfg->Pin & (GPIO_PIN_6|GPIO_PIN_9))) {
    assert(j1_low && j3_low && cfg->Mode==GPIO_MODE_OUTPUT_OD && cfg->Pull==GPIO_NOPULL);
    assert((cfg->Pin & (GPIO_PIN_6|GPIO_PIN_9))==(GPIO_PIN_6|GPIO_PIN_9));
    dir_configs++;
  }
  if(port==GPIOA && (cfg->Pin & GPIO_PIN_0)) {assert(cfg->Mode==GPIO_MODE_INPUT && cfg->Pull==GPIO_NOPULL);us1_echo++;}
  if(port==GPIOA && (cfg->Pin & GPIO_PIN_1)) {assert(us1_low && cfg->Mode==GPIO_MODE_OUTPUT_PP && cfg->Pull==GPIO_NOPULL);us1_trig++;}
  if(port==GPIOC && (cfg->Pin & GPIO_PIN_8)) ++j3_fb; /* must NOT happen: PC8 is TIM8_CH3 (tim.c), not EXTI */
  if(port!=GPIOB) return;
  if(cfg->Pin & GPIO_PIN_15) ++j1_fb; /* must NOT happen: PB15 is TIM12_CH2 (tim.c), not EXTI */
  if(cfg->Pin & (GPIO_PIN_4|GPIO_PIN_6)) {
    assert(cfg->Mode==GPIO_MODE_IT_RISING && cfg->Pull==GPIO_NOPULL);
    if(cfg->Pin & GPIO_PIN_4) ++l2;
    if(cfg->Pin & GPIO_PIN_6) ++r2;
  }
  if(cfg->Pin & GPIO_PIN_11) {assert(us2_low && cfg->Mode==GPIO_MODE_OUTPUT_PP && cfg->Pull==GPIO_NOPULL);us2_trig++;}
  if(cfg->Pin & GPIO_PIN_10) {assert(cfg->Mode==GPIO_MODE_INPUT && cfg->Pull==GPIO_NOPULL);us2_echo++;}
  if(cfg->Pin & GPIO_PIN_9) {
    assert(trig_preloaded && cfg->Mode==GPIO_MODE_OUTPUT_PP && cfg->Pull==GPIO_NOPULL);
    trig_configs++;
  }
  if(cfg->Pin & GPIO_PIN_8) {
    assert(cfg->Mode==GPIO_MODE_INPUT && cfg->Pull==GPIO_PULLDOWN);
    echo_configs++;
  }
}
void HAL_NVIC_SetPriority(int a, int b, int c) {(void)a;(void)b;(void)c;}
void HAL_NVIC_EnableIRQ(int irq) {
  if(irq==EXTI4_IRQn) ++n4;
  if(irq==EXTI9_5_IRQn) ++n9_5;
  if(irq==EXTI15_10_IRQn) ++n15_10;
}
int main(void) {
  MX_GPIO_Init();
  assert(trig_configs==1 && echo_configs==1);
  assert(dir_configs==1 && us1_trig==1 && us1_echo==1 && us2_trig==1 && us2_echo==1);
  puts("Ultrasonic GPIO and motor DIR preload/initialization PASS");
  assert(l2==1 && r2==1 && n4==1 && n9_5==1);
  assert(j3_fb==0 && j1_fb==0 && n15_10==0); /* encoder FB = timer capture (enc_capture.c), no EXTI */
  puts("PB4/PB6 rising-edge inputs, no internal pulls, side NVIC groups; encoder FB pins left to the timers PASS");
}
