#include "elcp_zero_pwm.h"
#include "side_ir.h"
#include "stm32f4xx_it.h"
#include "j3_rotation_run.h"
#include "rotation_estimate.h"
#include "elcp_motor/elcp_motor.h"
#include "tim.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
GPIO_TypeDef gpiob,gpioc;
uint32_t mock_primask;
TIM_TypeDef tim2,tim8,tim12;
/* Both physical PWM timers count 0..1049 (10.5 MHz counter, 10 kHz PWM). */
TIM_HandleTypeDef htim8={&tim8,1049}, htim12={&tim12,1049};
static int cmp1,cmp3,dir1,dir3,start1,start3,fail1,fail3,initializing=1;
static uint32_t val1,val3,level1,level3,tick;
static GPIO_PinState feedback_level=GPIO_PIN_SET;
static uint16_t pending, side_levels;
DMA_HandleTypeDef hdma_spi1_tx;
SPI_HandleTypeDef hspi1;
void HAL_IncTick(void) { ++tick; }
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *h) { (void)h; assert(0); }
void HAL_SPI_IRQHandler(SPI_HandleTypeDef *h) { (void)h; assert(0); }
/* enc_capture.c is register-level (built for the board only): its ISRs do exactly this per filtered edge. */
static unsigned capture_started;
void EncCapture_Start(void) { ++capture_started; }
static void capture(uint16_t fb_pin) { HAL_GPIO_EXTI_Callback(fb_pin); }
/* Only the HAL's pending-bit check/ack is mocked; vector and both callbacks are production. */
void HAL_GPIO_EXTI_IRQHandler(uint16_t pin) {
  if (pending & pin) { pending &= (uint16_t)~pin; HAL_GPIO_EXTI_Callback(pin); }
}
void tft_prints(uint8_t x,uint8_t y,const char *f,...) { (void)x;(void)y;(void)f;assert(0 && "no TFT in ISR"); }
void mock_set_compare(TIM_HandleTypeDef *h,uint32_t ch,uint32_t v) {
  if(initializing) assert(v==0);
  if(h==&htim12) {assert(ch==TIM_CHANNEL_1);cmp1++;val1=v;}
  else {assert(h==&htim8 && ch==TIM_CHANNEL_2);cmp3++;val3=v;}
}
void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,GPIO_PinState val) {
  assert(p==&gpioc);if(initializing)assert(val==GPIO_PIN_RESET);
  if(pin==GPIO_PIN_6){dir1++;level1=val;}else {assert(pin==GPIO_PIN_9);dir3++;level3=val;}
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t pin) {
  if (p==&gpiob && (pin==IR_L2_Pin || pin==IR_R2_Pin || pin==(IR_L2_Pin|IR_R2_Pin))) {
    if (pin==(IR_L2_Pin|IR_R2_Pin)) assert(mock_primask);
    return side_levels & pin ? GPIO_PIN_SET : GPIO_PIN_RESET;
  }
  assert((p==&gpiob && pin==ELCP_J1_FB_Pin) || (p==&gpioc && pin==ELCP_J3_FB_Pin));
  return feedback_level;
}
uint32_t HAL_GetTick(void) {return tick;}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h,uint32_t ch) {
  assert(dir1 && dir3);
  if(htim12.Instance) assert(cmp1 && val1==0);
  if(htim8.Instance) assert(cmp3 && val3==0);
  if(h==&htim12) {assert(ch==TIM_CHANNEL_1);start1++;return fail1?HAL_ERROR:HAL_OK;}
  assert(h==&htim8 && ch==TIM_CHANNEL_2);start3++;return fail3?HAL_ERROR:HAL_OK;
}
static void sides(bool l,bool r,uint32_t lc,uint32_t rc) {
  SideIR_Reading s=SideIR_Read();
  assert(s.left_high==l && s.right_high==r && s.left_rises==lc && s.right_rises==rc);
}
int main(int argc,char **argv) {
  assert(argc==2);int scenario=atoi(argv[1]);
  if(scenario==2) fail1=1;
  if(scenario==3) fail3=1;
  if(scenario==4) htim8.Instance=NULL;
  if(scenario==5) htim12.Instance=NULL;
  /* A side pin may already be high, or pulse before motor/display init. */
  side_levels=IR_L2_Pin; sides(true,false,0,0);
  pending=IR_L2_Pin|IR_R2_Pin; EXTI4_IRQHandler();
  assert(pending==IR_R2_Pin); sides(true,false,1,0);
  ++tick; EXTI9_5_IRQHandler(); EXTI9_5_IRQHandler(); assert(!pending); sides(true,false,1,1);
  mock_primask=1; BranchSample s=SideIR_TakeBranchSample();
  assert(s.at==1u && s.high && s.rose && s.first_rise==0u && mock_primask==1u);
  mock_primask=0; s=SideIR_TakeBranchSample(); assert(s.high && !s.rose && !mock_primask);
  ELCP_ZeroPWM_Result r=ELCP_ZeroPWM_Init();
  assert(r.j1_ok==(scenario!=5) && r.j3_ok==(scenario!=4));
  /* Even with one invalid handle, keep the other at zero, both DIR low,
   * and never restart PWM. Nonzero sentinels exist only in the mock. */
  for (unsigned i=0;i<3;++i) {
    const int c1=cmp1,c3=cmp3; val1=val3=777; level1=level3=GPIO_PIN_SET;
    ELCP_ZeroPWM_Hold();
    assert(cmp1==c1+r.j1_ok && cmp3==c3+r.j3_ok);
    assert(val1==(r.j1_ok?0u:777u) && val3==(r.j3_ok?0u:777u));
    assert(level1==GPIO_PIN_RESET && level3==GPIO_PIN_RESET);
    assert(start1==(int)r.j1_ok && start3==(int)r.j3_ok);
  }
  assert(capture_started==1); /* ELCP_ZeroPWM_Init starts the FB capture once both motors are up */
  initializing=0;
  if(scenario==1) {
    RotationEstimate_RecordIRQ(GPIO_PIN_15); HAL_GPIO_EXTI_Callback(GPIO_PIN_15);
    RotationEstimate_RecordIRQ(GPIO_PIN_8); HAL_GPIO_EXTI_Callback(GPIO_PIN_8);
    assert(rotation_j1_events==0 && rotation_j3_events==0);
    assert(ELCP_Motor_GetState(MOTOR_J1)->pulse_count==0&&ELCP_Motor_GetState(MOTOR_J3)->pulse_count==0);
    /* ...but the raw count sees them: a wheel turned by hand at 0 % shows on the setup page */
    assert(SideIR_EncoderRaw(true)==1 && SideIR_EncoderRaw(false)==1);
    tim2.CNT+=50;
    /* A 25% straight command uses the same compare value despite opposite DIR. */
    ELCP_Motor_SetSpeed(MOTOR_J1,25); ELCP_Motor_SetSpeed(MOTOR_J3,-25);
    assert(val1==262 && val3==262 && level1==GPIO_PIN_RESET && level3==GPIO_PIN_SET);
    ELCP_Motor_SetSpeed(MOTOR_J1,50);assert(val1==524&&level1==GPIO_PIN_RESET);
    ELCP_Motor_SetSpeed(MOTOR_J3,-120);assert(val3==1049&&level3==GPIO_PIN_SET);
    /* Side IRs stay on EXTI4 / EXTI9_5. Encoder FB is timer capture now: EXTI9_5 must NOT service PC8 any more, and
     * enc_capture.c's ISRs hand each filtered edge to the same callback (capture() below). */
    side_levels=IR_R2_Pin; pending=IR_L2_Pin|IR_R2_Pin|ELCP_J3_FB_Pin;
    EXTI9_5_IRQHandler();
    sides(false,true,1,2); assert(pending==(IR_L2_Pin|ELCP_J3_FB_Pin));
    EXTI4_IRQHandler(); sides(false,true,2,2); assert(pending==ELCP_J3_FB_Pin); pending=0;
    RotationEstimate_RecordIRQ(GPIO_PIN_8); capture(ELCP_J3_FB_Pin);
    RotationEstimate_RecordIRQ(GPIO_PIN_15); capture(ELCP_J1_FB_Pin);
    EXTI9_5_IRQHandler(); EXTI4_IRQHandler(); sides(false,true,2,2);
    assert(rotation_j1_events==1 && rotation_j3_events==1);
    assert(ELCP_Motor_GetState(MOTOR_J1)->pulse_count==1&&ELCP_Motor_GetState(MOTOR_J3)->pulse_count==-1);
    pending=IR_R2_Pin; EXTI9_5_IRQHandler(); sides(false,true,2,3);
    assert(ELCP_Motor_GetState(MOTOR_J3)->pulse_count==-1);
    s=SideIR_TakeBranchSample(); assert(s.high && s.rose); side_levels=0;
    feedback_level=GPIO_PIN_RESET; capture(ELCP_J3_FB_Pin);
    sides(false,false,2,3); assert(ELCP_Motor_GetState(MOTOR_J3)->pulse_count==-1);
    s=SideIR_TakeBranchSample(); assert(!s.rose && !s.high); /* encoder is not a crossing */
    puts("Real EXTI4 / EXTI9_5 side-IR dispatch, encoder FB by timer capture: independent side IR and encoder counts PASS");
    /* Reference rejects a low level and routes the two pins independently. */
    feedback_level=GPIO_PIN_RESET;
    HAL_GPIO_EXTI_Callback(GPIO_PIN_15);
    assert(ELCP_Motor_GetState(MOTOR_J1)->pulse_count==1 && ELCP_Motor_GetState(MOTOR_J3)->pulse_count==-1);
    feedback_level=GPIO_PIN_SET;
    HAL_GPIO_EXTI_Callback(GPIO_PIN_15);
    assert(ELCP_Motor_GetState(MOTOR_J3)->pulse_count==-1);
    assert(ELCP_Motor_GetState(MOTOR_J1)->pulse_count==2);
    tick+=1000;ELCP_Motor_UpdatePPS(MOTOR_J1);assert(ELCP_Motor_GetState(MOTOR_J1)->pps==2.0f);
    ELCP_Motor_Stop(MOTOR_J1);ELCP_Motor_Stop(MOTOR_J3);assert(val1==0&&val3==0);
    /* Host-only integration with the exact reference driver. */
    RotationEstimate_RecordIRQ(GPIO_PIN_15); RotationEstimate_RecordIRQ(GPIO_PIN_8);
    assert(rotation_j1_events==1 && rotation_j3_events==1);
    J3Run_Init();uint32_t base=rotation_j3_events;
    assert(J3Run_Start(true,base));assert(val3==419 && val1==0);
    for(uint32_t i=1;i<=J3_RUN_TARGET_EVENTS;i++) {
      RotationEstimate_RecordIRQ(GPIO_PIN_8); HAL_GPIO_EXTI_Callback(GPIO_PIN_8);
      J3Run_Update(rotation_j3_events);
    }
    assert(J3Run_Status()==J3_RUN_COMPLETE && val3==0 && val1==0);
    RotationEstimate_RecordIRQ(GPIO_PIN_8);assert(rotation_j3_events==base+J3_RUN_TARGET_EVENTS);
  }
  printf("ELCP reference driver mocked scenario %d PASS\n",scenario);
}
