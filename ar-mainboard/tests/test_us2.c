#include "main.h"
#include "us2_sensor.h"
#include "tim.h"
#include <assert.h>
#include <stdio.h>
GPIO_TypeDef gpioa,gpiob; RCC_Type rcc; TIM_Type tim2;
TIM_HandleTypeDef htim2={.Instance=&tim2,.Init={83,UINT32_MAX}};
static int start_fail;
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *h){assert(h==&htim2);return start_fail?HAL_ERROR:HAL_OK;}
static uint32_t tick_us, frequency=42000000, fired; static int scenario, trig, saw_low, saw_high, trigger_pin_ok=1, echo_pin_ok=1, freeze;
uint32_t HAL_RCC_GetPCLK1Freq(void){return frequency;}
static void advance(void){tick_us+=10;if(!freeze) tim2.CNT+=10;}
uint32_t HAL_GetTick(void){advance();return tick_us/1000;}
void HAL_GPIO_Init(GPIO_TypeDef*p,GPIO_InitTypeDef*g){assert(p==GPIOB); if(g->Pin==GPIO_PIN_11) assert(g->Mode==GPIO_MODE_OUTPUT_PP); else {assert(g->Pin==GPIO_PIN_10);assert(g->Mode==GPIO_MODE_INPUT);}}
void HAL_GPIO_WritePin(GPIO_TypeDef*p,uint16_t pin,GPIO_PinState v){if(p!=GPIOB||pin!=GPIO_PIN_11)trigger_pin_ok=0; if(v){saw_high=1;trig=1;}else{if(trig)fired=tim2.CNT;trig=0;saw_low=1;}}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*p,uint16_t pin){if(p!=GPIOB||pin!=GPIO_PIN_10)echo_pin_ok=0;advance();if(scenario==1)return GPIO_PIN_SET;if(!fired||scenario==2)return GPIO_PIN_RESET;uint32_t d=tim2.CNT-fired;if(scenario==3)return d>=100?GPIO_PIN_SET:GPIO_PIN_RESET;if(scenario==4)return d>=100&&d<150?GPIO_PIN_SET:GPIO_PIN_RESET;return d>=100&&d<1100?GPIO_PIN_SET:GPIO_PIN_RESET;}
static void reset(int sc){tim2.CNT=0;tick_us=0;fired=0;scenario=sc;trig=0;saw_low=saw_high=0;freeze=0;}
int main(void){rcc.CFGR=RCC_CFGR_PPRE1_2;assert(US2_Init());assert(htim2.Init.Prescaler==83);assert(htim2.Init.Period==UINT32_MAX);reset(0);US2_Reading r=US2_Measure();assert(r.status==US2_OK);assert(r.raw_mm>160&&r.raw_mm<190);assert(saw_low&&saw_high&&trigger_pin_ok&&echo_pin_ok);reset(1);assert(US2_Measure().status==US2_ECHO_BUSY);assert(!saw_high);reset(2);assert(US2_Measure().status==US2_NO_RISE);assert(!trig);reset(3);assert(US2_Measure().status==US2_NO_FALL);reset(4);assert(US2_Measure().status==US2_SHORT_PULSE);reset(0);freeze=1;assert(US2_Measure().status==US2_TIMER_ERROR);assert(!trig);frequency=123;assert(!US2_Init());frequency=42000000;htim2.Init.Period=65535;assert(!US2_Init());htim2.Init.Period=UINT32_MAX;start_fail=1;assert(!US2_Init());puts("US2 mocked PB11/PB10 pin/timing/status checks PASS");}
