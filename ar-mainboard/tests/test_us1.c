#include "main.h"
#include "tim.h"
#include "us1_sensor.h"
#include <assert.h>
#include <stdio.h>

GPIO_TypeDef gpioa,gpiob; RCC_Type rcc; TIM_Type tim2;
TIM_HandleTypeDef htim2={.Instance=&tim2,.Init={83,UINT32_MAX}};
static uint32_t tick_us, fired;
static int scenario, trig, trigger_high, frozen;
static void advance(void) { tick_us += 10; if (!frozen) tim2.CNT += 10; }
uint32_t HAL_GetTick(void) { advance(); return tick_us/1000; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return 42000000; }
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *h) { (void)h; return HAL_OK; }
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g) { (void)p; (void)g; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState v) {
  assert(p==GPIOA && pin==GPIO_PIN_1);
  if(v) { trig=1; trigger_high=1; }
  else { if(trig) fired=tim2.CNT; trig=0; }
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin) {
  assert(p==GPIOA && pin==GPIO_PIN_0); advance();
  if(scenario==1) return GPIO_PIN_SET;
  if(!fired || scenario==2) return GPIO_PIN_RESET;
  uint32_t d=tim2.CNT-fired;
  if(scenario==3) return d>=100 ? GPIO_PIN_SET : GPIO_PIN_RESET;
  if(scenario==4) return d>=100 && d<150 ? GPIO_PIN_SET : GPIO_PIN_RESET;
  return d>=100 && d<1100 ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
static void reset(int s) { tick_us=tim2.CNT=fired=0; scenario=s; trig=trigger_high=frozen=0; }
int main(void) {
  reset(0); US2_Reading r=US1_Measure();
  assert(r.status==US2_OK && r.raw_mm>160 && r.raw_mm<190 && trigger_high && !trig);
  reset(1); assert(US1_Measure().status==US2_ECHO_BUSY && !trigger_high);
  reset(2); assert(US1_Measure().status==US2_NO_RISE && !trig);
  reset(3); assert(US1_Measure().status==US2_NO_FALL && !trig);
  reset(4); assert(US1_Measure().status==US2_SHORT_PULSE && !trig);
  reset(0); frozen=1; assert(US1_Measure().status==US2_TIMER_ERROR && !trig);
  puts("US1 mocked PA1/PA0 timing/status checks PASS");
}
