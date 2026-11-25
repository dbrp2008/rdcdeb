#include "main.h"
#include "lcd/lcd.h"
#include "elcp_zero_pwm.h"
#include "elcp_motor/elcp_motor.h"
#include "ir_sensor.h"
#include "ir_mode.h"
#include "us1_sensor.h"
#include "calibration_app.h"
#include "calibration_replay.h"
#include "drive_probe.h"
#include "f12_duties.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef gpioa,gpioc; RCC_Type rcc; TIM_Type tim2;
static uint32_t now, last_ping, last_ir, ir_delay_once, ping_delay_once;
static int motor_init, ir_init, us_init, n_us, n_ir, b1, b2, adc_failure, timer_fail, motor_fail;
/* At these black levels, all three four-scan means cross on the same scan
 * when tape is placed on or removed from a floor reading of 100. */
enum { BLACK_L = 400, BLACK_C = 800, BLACK_R = 4095 };
static uint16_t ir[3]={100,BLACK_C,100};
static US2_Reading next_us={US2_OK,1000,171};
static float j1,j3;
static bool testing_f2; /* expected branch policy; does not select board firmware */
static int stop1,stop3, drives;
static unsigned zero_holds, lcd_inits, press_in_ir, press_in_ping;
static bool zero_at_ping;
static void inject_press(unsigned *mask) {
  if (*mask & 1u) b1=1;
  if (*mask & 2u) b2=1;
  *mask=0u;
}
static bool check_cal_zero;
static char row[10][100];
uint32_t HAL_GetTick(void) {return now;}
bool mock_btn(int b) {if(check_cal_zero) assert(j1==0 && j3==0); return b == BTN1 ? b1 : b2;}
ELCP_ZeroPWM_Result ELCP_ZeroPWM_Init(void) {motor_init++; j1=j3=0; return (ELCP_ZeroPWM_Result){!motor_fail,!motor_fail};}
void ELCP_ZeroPWM_Hold(void){++zero_holds; j1=j3=0;}
void tft_init(TFT_ORIENTATION o,uint16_t bg,uint16_t fg,uint16_t a,uint16_t h){
  ++lcd_inits;
  assert(lcd_inits==(unsigned)motor_init && j1==0 && j3==0); /* zero before even the first LCD delay */
  assert(o==PIN_ON_TOP && bg==BLACK && fg==WHITE && a==CYAN && h==DARK_GREY);
}
void ELCP_Motor_Stop(ELCP_Motor_ID id){if(id==MOTOR_J1){j1=0;stop1++;}else{assert(id==MOTOR_J3);j3=0;stop3++;}}
void ELCP_Motor_SetSpeed(ELCP_Motor_ID id,float val){
  const float mag=fabsf(val);
  assert(duty_is(mag,DUTY_INSIDE) || duty_is(mag,DUTY_OUTSIDE) || duty_is(mag,DUTY_CCW) ||
         duty_is(mag,DUTY_STRAIGHT) || duty_is(mag,DUTY_LAUNCH) || duty_is(mag,DUTY_REVERSE) ||
         (testing_f2 && ((id==MOTOR_J3 && duty_is(val,-DUTY_F2_CW_LEFT)) ||
                         (id==MOTOR_J1 && duty_is(val,-DUTY_F2_CW_RIGHT)))));
  if(id==MOTOR_J1)j1=val;else {assert(id==MOTOR_J3);j3=val;}drives++;
}
void IR_Sensor_Init(void) {ir_init++;}
HAL_StatusTypeDef IR_ReadAll(IR_RawScan *out) {
  if(check_cal_zero) assert(j1==0 && j3==0);
  if(n_ir) { assert((uint32_t)(now-last_ir)>=10); }
  last_ir=now;n_ir++; inject_press(&press_in_ir);
  if(adc_failure)return HAL_ERROR;
  memcpy(out->raw,capture_frame ? capture_frame : ir,sizeof(ir));now+=ir_delay_once;ir_delay_once=0;return HAL_OK;
}
bool US2_Init(void) {us_init++;return !timer_fail;}
US2_Reading US1_Measure(void) {
  if(n_us) assert((uint32_t)(now-last_ping)>=60);
  if(zero_at_ping) assert(j1==0 && j3==0);
  last_ping=now; n_us++; now+=ping_delay_once; ping_delay_once=0;
  inject_press(&press_in_ping); return next_us;
}
void tft_printc(uint8_t x,uint8_t y,const char *s){assert(x==0&&y<10&&strlen(s)<=16);snprintf(row[y],sizeof(row[y]),"%s",s);}
void tft_prints(uint8_t x,uint8_t y,const char *fmt,...){assert(x==0&&y<10);va_list ap;va_start(ap,fmt);int n=vsnprintf(row[y],sizeof(row[y]),fmt,ap);va_end(ap);assert(n>=0&&n<=16); /* full length, not the truncated copy */}
void tft_update(uint32_t x){(void)x; if(check_cal_zero) assert(j1==0 && j3==0);}
static void run(unsigned ms){for(unsigned i=0;i<ms;i++){now++;CalibrationApp_Update();}}
static void keys(unsigned mask, unsigned ms) { b1=mask&1u; b2=mask&2u; run(ms); }
/* Side GPIOs and their rising interrupts are independent of analog tape().
 * An immediate high/low pair models a serviced pulse entirely between scans. */
static void edges(bool left, bool right) {
  uint16_t levels=(left ? IR_L2_Pin : 0u) | (right ? IR_R2_Pin : 0u);
  uint16_t rising=levels & ~gpiob.dummy;
  gpiob.dummy=levels;
  if (rising & IR_L2_Pin) HAL_GPIO_EXTI_Callback(IR_L2_Pin);
  if (rising & IR_R2_Pin) HAL_GPIO_EXTI_Callback(IR_R2_Pin);
}
static void boot_profile(const uint16_t white[3], const uint16_t black[3]) {
  int speed_writes=drives, pings=n_us, scans=n_ir, timer_inits=us_init;
  const unsigned holds=zero_holds;
  CalibrationApp_Init(); run(160);
  assert(zero_holds==holds+160u);
  assert(strstr(row[0],"IR CAL 100 SAMP") && !strcmp(row[5],"T ---- ---- ----"));
  keys(0,40); keys(3,150); keys(1,500); keys(0,150); /* no usable old profile after reset */
  assert(strstr(row[7],"NEED W AND B") && !strcmp(row[5],"T ---- ---- ----"));
  assert(n_ir==scans && n_us==pings && us_init==timer_inits);
  replay_calibration(keys,white,black);
  assert(n_ir==scans+200 && n_us==pings && drives==speed_writes && j1==0 && j3==0);
}
static void boot_drive(void) { boot_profile(route_white,route_black); }
/* Inject stale DUTY in the mock before *every* setup pass, including the apply
 * tick. A one-off boot stop, or a stop only on screen refresh, cannot pass. */
static void cal_keys(unsigned mask,unsigned ms) {
  b1=mask&1; b2=mask&2;
  while(ms--) {
    unsigned before=zero_holds; j1=8; j3=-8; run(1);
    assert(zero_holds==before+1 && j1==0 && j3==0);
  }
}
/* Signed motor API order here: J1 (right), then J3 (left, inverted). */
static void expect_wheels(float right, float left) { assert(duty_is(j1,right) && duty_is(j3,left)); }
static void expect_stopped(void)  { assert(j1==0 && j3==0); }
static void expect_launch(void)   { expect_wheels(DUTY_LAUNCH,-DUTY_LAUNCH); }
static void expect_straight(void) { expect_wheels(DUTY_STRAIGHT,-DUTY_STRAIGHT); }
static void expect_left(void)     { expect_wheels(DUTY_OUTSIDE,-DUTY_INSIDE); }
static void expect_right(void)    { expect_wheels(DUTY_INSIDE,-DUTY_OUTSIDE); }
static void expect_reverse(void)  { expect_wheels(-DUTY_REVERSE,DUTY_REVERSE); }
static void expect_ccw(void)      { expect_wheels(DUTY_CCW,DUTY_CCW); }
static void expect_f2_right_pivot(void) {
  expect_wheels(-DUTY_F2_CW_RIGHT,-DUTY_F2_CW_LEFT); /* physical J3 forward : J1 reverse = 5:3 */
}
static void side_setup_test(void) {
  check_cal_zero=true; j1=8; j3=-8; gpiob.dummy=IR_L2_Pin; CalibrationApp_Init();
  assert(!strcmp(row[2],"L2:1/00 R2:0/00 ")); /* boot level need not wait for an edge */
  gpiob.dummy=0; HAL_GPIO_EXTI_Callback(IR_R2_Pin); HAL_GPIO_EXTI_Callback(IR_R2_Pin);
  cal_keys(0,160); assert(!strcmp(row[2],"L2:0/00 R2:0/02 ")); /* pulses ended before redraw */
  adc_failure=1; cal_keys(1,60); cal_keys(0,1400); assert(strstr(row[7],"ADC ERROR"));
  adc_failure=0; capture_frame=route_white;
  cal_keys(1,60); cal_keys(0,1400); cal_keys(2,60); cal_keys(0,1400);
  cal_keys(3,100); cal_keys(0,150); assert(strstr(row[7],"WEAK OR OVERLAP") && !us_init && !drives);
  /* Exercise modulo-100 readout independently from the raw level. */
  for(unsigned i=0;i<101;++i) HAL_GPIO_EXTI_Callback(IR_L2_Pin);
  gpiob.dummy=IR_R2_Pin; replay_calibration(cal_keys,route_white,route_black);
  assert(j1==0 && j3==0 && drives==0 && us_init==1 && !n_us);
  check_cal_zero=false; const unsigned holds=zero_holds;
  keys(0,200); assert(!drives && strstr(row[6],"B1 RUN") && !strcmp(row[1],"L2:0/01 R2:1/02 "));
  keys(1,60); keys(0,810); expect_straight(); /* fresh start, centre-only analog tape */
  gpiob.dummy=IR_L2_Pin|IR_R2_Pin; HAL_GPIO_EXTI_Callback(IR_L2_Pin);
  keys(0,160); assert(!strcmp(row[1],"L2:1/02 R2:1/02 ") && strstr(row[3],"LINE W B W"));
  expect_straight(); assert(zero_holds==holds && strstr(row[0],"F1 LINE BR 0"));
  edges(0,0); keys(0,160); edges(0,1); edges(0,0); keys(0,160);
  assert(!strcmp(row[1],"L2:0/02 R2:0/03 ") && strstr(row[0],"F1 LINE BR 1"));
  expect_straight();
  puts("Setup holds zero; independent side IRQ totals persist, held start is ignored and a later pulse counts without steering PASS");
}
static void start(void){b1=1;run(40);b1=0;run(40);}
/* These mode checks use only the board's public UI and motor API; no F2 stubs. */
static void tape(bool left, bool centre, bool right) {
  ir[0]=left ? BLACK_L : 100; ir[1]=centre ? BLACK_C : 100; ir[2]=right ? BLACK_R : 100;
}
static void expect_second_crossing(void) {
  expect_straight();
  assert(strstr(row[0],"BR 2 ") && strstr(row[5],"BRANCH STRAIGHT"));
}
static void expect_third_crossing(void) {
  assert(strstr(row[0],"BR 3 "));
  if (testing_f2) {
    expect_f2_right_pivot();
    assert(strstr(row[5],"F2 TURN RIGHT"));
  } else {
    expect_straight();
    assert(strstr(row[5],"BRANCH STRAIGHT"));
  }
}
/* Expected board timings, measured from first pivot command and tape pickup. */
enum { CCW_MIN_MS = 1000, CCW_ALIGN_MS = 450 };
/* Starting part-way into the pivot: clear the old tape, then find centre black. */
static void pick_up_centre(void) {
  ir[0]=ir[1]=ir[2]=100; run(CCW_MIN_MS+50); expect_ccw();
  ir[1]=BLACK_C; run(40); expect_ccw();
  run(CCW_ALIGN_MS+20); expect_straight();
}
static void mark_return_finish(void) {
  tape(1,1,1); run(160); expect_straight(); /* filtered BBB after forward return starts */
  assert(strstr(row[3],"LINE B B B") && strstr(row[5],"TARGET FOLLOW"));
}
/* Both inputs are qualified by the real board loop. 54 mm alone does not
 * stop; the first filtered WWW does, without the former 500 ms dwell. */
static void finish_return_on_white(void) {
  mark_return_finish();
  next_us.raw_mm=54; run(60); expect_straight();
  tape(0,0,0); run(10u-(now-last_ir)); expect_stopped();
  next_us.raw_mm=200; run(120); assert(strstr(row[5],"TARGET LINE END"));
}
/* Launch ignores IR steering, not the US1 interlock. */
static void launch_tests(void){
  motor_fail=0;timer_fail=0;n_us=0;n_ir=0;boot_drive();run(200);
  ir[0]=ir[1]=ir[2]=0;                 /* no line at all */
  next_us.status=US2_OK;next_us.raw_mm=59; /* under the stop distance */
  run(220);expect_stopped();
  tape(0,1,0);next_us.raw_mm=200;run(220);expect_stopped();
  start();expect_launch();run(600);expect_launch();  /* within the 750 ms launch: 13.5% both */
  ir[0]=0;ir[1]=0;ir[2]=0;run(1);expect_launch();     /* line loss ignored while launching */
  tape(0,1,0);run(200);            /* ~0.8 s: launch over, line present */
  expect_straight();
  run(220);
  b2=1;run(1);expect_stopped();b2=0;run(120);
  /* Start inhibited below 60 mm even though the launch ignores the line. */
  tape(0,1,0);next_us.raw_mm=59;run(220);
  start();run(120);expect_stopped();assert(strstr(row[5],"TARGET <60MM"));
  next_us.raw_mm=200;tape(0,1,0);run(1100);
  tape(1,1,1);run(300);run(120);expect_stopped();
  tape(0,1,0);run(220);
  /* A valid US1 target during launch takes over with the ordinary approach. */
  start();expect_launch();next_us.raw_mm=59;run(220);expect_straight();
  assert(strstr(row[5],"TARGET APPROACH")); run(350);expect_stopped();
  assert(strstr(row[5],"TARGET 60MM HOLD")); /* holds instead of ending the run */
  next_us.raw_mm=200;tape(0,1,0);run(1100); /* hold, then reverse */
  expect_reverse();assert(strstr(row[5],"TARGET REV 18%"));
  tape(1,1,1);run(100); /* IR (even three black) ignored */
  expect_reverse();
  ir[0]=ir[1]=ir[2]=100;run(350);
  expect_ccw();assert(strstr(row[5],"TARGET SPIN CCW"));
  pick_up_centre();
  finish_return_on_white();
  tape(0,1,0);run(220);
  /* A US1 sensor error during the launch also stops the run. */
  start();expect_launch();next_us=(US2_Reading){.status=US2_NO_RISE,.raw_mm=59};
  run(220);expect_stopped();assert(strstr(row[5],"US1 NO RISE")); /* fault, not a target hold */
  next_us=(US2_Reading){.status=US2_OK,.raw_mm=200};run(220);
  /* Outbound line loss after launch is straight following, not a terminal stop. */
  start();expect_launch();run(1100);
  assert(!strstr(row[5],"LINE LOST")); expect_straight();
  ir[0]=ir[1]=ir[2]=0;run(220);expect_straight();assert(strstr(row[5],"NO LINE STRAIGHT"));
  /* Analog BBB without a prior US1 stop: no finish or count-triggered turn. */
  ir[0]=ir[1]=ir[2]=100;next_us.raw_mm=200;run(220);start();run(1100);
  tape(1,1,1);run(140);
  expect_straight(); /* not armed yet: straight on, no reverse */
  /* Commit the US1 hold and return before testing a fresh board rerun below. */
  tape(0,1,0);run(220); /* back on the line */
  next_us.raw_mm=20;run(350+220);assert(j1==0&&j3==0); /* holding */
  next_us.raw_mm=200;run(1100); /* now in the 500 ms reverse at 18% */
  expect_reverse();
  ir[0]=ir[1]=ir[2]=100;run(400);expect_ccw(); /* rotate anticlockwise */
  pick_up_centre();
  finish_return_on_white();
  assert(strstr(row[6],"B1 RUN")); /* pinned replay only */
  /* A new board start is always fresh, even after F1 committed its US1 hold. */
  ir[0]=ir[1]=ir[2]=100;run(260);start();run(9800);
  tape(1,1,1);run(310);expect_straight();
  assert(strstr(row[5],"BRANCH STRAIGHT") && strstr(row[0],"BR 0 "));
  /* Two analog black sensors do not finish the run. */
  run(400);ir[0]=ir[1]=ir[2]=100;run(220);n_ir=0;n_us=0;boot_drive();run(200);start();run(1100);
  tape(1,1,0);run(220);expect_straight();
  assert(!strstr(row[5],"FINISH REV"));
  b2=1;run(1);expect_stopped();b2=0;run(120);
  /* Arm again: a run that holds and reverses on US1, then three black while running. */
  tape(0,1,0);run(220);start();run(1100);
  next_us.raw_mm=20;run(350+220);expect_stopped();assert(strstr(row[5],"TARGET 60MM HOLD"));
  next_us.raw_mm=200;run(1100); /* hold 1 s, then reverse 18% */
  expect_reverse();
  next_us.raw_mm=10;run(30);
  assert(!strstr(row[5],"TARGET 60MM HOLD")); /* no new hold during the reverse */
  next_us.raw_mm=200;
  ir[0]=ir[1]=ir[2]=100;run(400);expect_ccw(); /* rotate anticlockwise */
  pick_up_centre();
  ir[0]=ir[2]=100;run(220);expect_straight(); /* centre: still following */
  finish_return_on_white();
  /* Start is allowed with the line lost; the launch still runs at 13.5%. */
  run(400);b2=1;run(1);expect_stopped();b2=0;run(120);ir[0]=ir[1]=ir[2]=100;run(260);
  start();expect_launch();run(120);assert(strstr(row[5],"LAUNCH 13.5%")); /* redraw a row first */
  run(1100);expect_straight();assert(strstr(row[5],"NO LINE STRAIGHT"));
  b2=1;run(1);expect_stopped();b2=0;run(120);
}
/* Trigger the real US1 state machine; return on the very first tick on which
 * the motor API sees an anticlockwise pivot. No internal state is mocked. */
static void begin_pivot(bool start_on_tape) {
  edges(0,0);
  b1=b2=adc_failure=timer_fail=motor_fail=0; n_ir=n_us=0; ir_delay_once=0u;
  ir[0]=ir[2]=100; ir[1]=start_on_tape ? BLACK_C : 100;
  next_us=(US2_Reading){.status=US2_OK,.raw_mm=60};
  boot_drive(); run(160); start(); expect_launch();
  run(60); expect_launch(); /* 60 mm also remains clear on a fresh running ping */
  const uint32_t since_ping=now-last_ping; assert(since_ping<60u);
  next_us.raw_mm=59; run(59u-since_ping); expect_launch();
  run(1); assert(now==last_ping); expect_straight(); /* first 59 mm; supersedes launch */
  run(349); expect_straight(); assert(strstr(row[5],"TARGET APPROACH"));
  run(1); expect_stopped(); /* repeated near pings did not restart the timer */
  run(999); expect_stopped(); assert(strstr(row[5],"TARGET 60MM HOLD"));
  run(1); expect_reverse();
  run(499); expect_reverse(); assert(strstr(row[5],"TARGET REV 18%"));
  run(1); expect_ccw(); next_us.raw_mm=200;
  assert(now==last_ir); /* deterministic 10 ms cadence for the boundary checks */
}
/* Exercise both orders of arrival. In particular, a pending <60 mm approach must
 * not steal a fully qualified line end and start another hold/reverse. */
static void return_range_tests(void) {
  begin_pivot(false); next_us.raw_mm=60; run(CCW_MIN_MS+50);
  tape(1,1,1); run(60); tape(0,0,0); run(CCW_ALIGN_MS+700); /* BBB only during pickup/alignment */
  expect_straight(); assert(strstr(row[5],"TARGET FOLLOW"));
  tape(0,0,1); run(100); expect_right(); tape(0,0,0); run(600);
  expect_straight(); /* partial black did not arm; white clears both routes */

  mark_return_finish(); next_us.raw_mm=54; run(60u-(now-last_ping)); /* close on black: approach */
  /* Both are due on the same tick: the first filtered WWW must use the new
   * 55, NOT the previous 54. The normal <60 approach is still pending. */
  run(59u-(now-last_ping)); next_us.raw_mm=55; tape(0,0,0); run(1);
  assert(now==last_ping && now==last_ir); expect_straight();
  tape(0,0,1); run(100); next_us.raw_mm=54; run(60); expect_right(); /* past white does not count */
  tape(0,0,0); run(9u-(now-last_ir)); expect_right(); run(1); expect_stopped();
  assert(now==last_ir); run(120); assert(strstr(row[5],"TARGET LINE END"));

  for (unsigned event=0;event<5;++event) {
    const char *reason;
    begin_pivot(false); pick_up_centre(); mark_return_finish();
    if (event < 2) {
      next_us.raw_mm=60; run(60); tape(0,0,0); run(650); expect_straight();
      next_us.raw_mm=54; /* latest result is either a legitimate finish or a sensor fault */
      if (event==1) next_us.status=US2_NO_RISE;
      run(59u-(now-last_ping)); expect_straight(); run(1); assert(now==last_ping);
      reason=event==0 ? "TARGET LINE END" : "US1 NO RISE";
    } else if (event==4) {
      next_us.raw_mm=55; run(60u-(now-last_ping)); assert(now==last_ir);
      next_us.raw_mm=54; run(349); expect_straight(); /* still black */
      tape(0,0,0); run(1); assert(now==last_ir); /* first filtered white on the exact hold deadline */
      reason="TARGET LINE END";
    } else {
      next_us.raw_mm=54; run(60u-(now-last_ping)); assert(now==last_ping && now==last_ir);
      run(49); tape(0,0,0);
      if (event==2) {
        /* The white scan completes with fresh IR but the last close US1 now
         * 151 ms old. Fail closed before the next ping can hide that lapse. */
        ir_delay_once=101; run(1); reason="SENSOR STALE";
      } else { b2=1; run(1); reason="STOP BTN2"; }
    }
    expect_stopped(); int n=drives; run(1650); expect_stopped();
    assert(drives==n && strstr(row[5],reason)); /* no reverse or automatic resume */
  }
  puts("Return: post-CCW 111, current filtered 000 AND fresh <55 mm; immediate on scan or ping, <60 overlap and faults PASS");
}
static void pivot_tests(void) {
  /* An early hit cannot shorten the minimum. A fresh centre-black scan at
   * the minimum is eligible; forward drive must wait out the full alignment. */
  uint32_t t0=now; begin_pivot(false); uint32_t to_spin=now-t0;
  now=UINT32_MAX-to_spin-500u; begin_pivot(false); uint32_t first_spin=now;
  run(100); ir[1]=BLACK_C; run(CCW_MIN_MS-101); expect_ccw();
  run(1); assert(now<first_spin); /* HAL tick wrapped during the pivot */
  run(CCW_ALIGN_MS-1); expect_ccw(); run(1); expect_straight();

  /* Black sampled before the guard must not count, even if the scan finishes
   * after it. Its residual in the next rolling average must not count either. */
  begin_pivot(false); first_spin=now;
  run(CCW_MIN_MS-20); ir[0]=4095; ir_delay_once=13; run(10);
  assert(now-first_spin==CCW_MIN_MS+3u && last_ir-first_spin==CCW_MIN_MS-10u);
  ir[0]=100; run(CCW_ALIGN_MS+100); expect_ccw();
  assert(strstr(row[5],"TARGET FIND LINE"));
  ir[0]=1000; run(CCW_ALIGN_MS+60); expect_left(); /* fresh left tape -> steer left */

  /* A held black reading is not a re-acquisition. White alone is not a
   * reason to drive forward, even after the old fixed turn would have ended. */
  begin_pivot(true); run(4300); expect_ccw();
  assert(strstr(row[5],"TARGET FIND LINE"));
  ir[1]=100; run(200); expect_ccw();
  /* From 100s, full-scale samples give means of 1099, 2098, then 3096.
   * Only the fourth scan exceeds the right threshold of 4004. */
  ir[2]=4095; run(40); run(CCW_ALIGN_MS-1); expect_ccw();
  run(1); expect_right(); /* filtered right tape -> steer right */
  finish_return_on_white();

  /* No tape, or first black exactly at the deadline: fail stopped; no auto-restart. */
  for (unsigned late=0;late<=1;late++) {
    begin_pivot(false); run(9999); expect_ccw();
    if (late) ir[0]=4095;
    run(1); expect_stopped(); ir[1]=BLACK_C; run(250); expect_stopped();
    assert(strstr(row[5],"LINE NOT FOUND"));
  }
  for (unsigned fault=0;fault<4;fault++) {
    const char *why;
    begin_pivot(false); run(CCW_MIN_MS+150); expect_ccw();
    if (fault==0) { b2=1; run(1); why="STOP BTN2"; }
    else if (fault==1) { adc_failure=1; run(10); why="IR ERROR"; }
    else if (fault==2) { next_us.status=US2_NO_RISE; run(60); why="US1 NO RISE"; }
    else { now+=151; CalibrationApp_Update(); why="SENSOR STALE"; }
    expect_stopped(); run(110); assert(strstr(row[5],why));
  }
  puts("US1: 60 mm clear; 59 mm follows for 350 ms, holds for 1000 ms, reverse 18% for 500 ms; 1000 ms minimum pivot, tape pickup, 450 ms alignment, faults and tick wrap PASS");
}
/* Observe actual signed motor writes. Check time from completed ping, live IR,
 * and failure priority in the moving approach, not only during a stopped hold. */
static void approach_tests(void) {
  for (unsigned event=0;event<6;++event) {
    b1=b2=adc_failure=motor_fail=timer_fail=0; n_ir=n_us=0; edges(0,0); tape(0,1,0);
    next_us=(US2_Reading){.status=US2_OK,.raw_mm=60}; boot_drive(); run(160);
    keys(1,40); expect_stopped(); keys(0,31); expect_launch(); const uint32_t start_at=now;
    run((event==0 || event==5) ? 9780 : 800); expect_straight();
    const int old1=stop1, old3=stop3;
    next_us.raw_mm=59; ping_delay_once=17; run(60u-(now-last_ping));
    assert(now-last_ping==17 && stop1==old1 && stop3==old3); expect_straight();
    next_us.raw_mm=80; tape(0,0,1); run(105); expect_right();
    assert(strstr(row[5],"TARGET APPROACH")); /* fresh IR, clear ping, still moving */
    const char *reason;
    switch (event) {
      case 0:
        tape(0,0,0); run(244); expect_straight();
        assert(now-start_at>10000u && stop1==old1 && stop3==old3); run(1); expect_stopped();
        assert(stop1>old1 && stop3>old3); run(999); expect_stopped();
        assert(strstr(row[5],"TARGET 60MM HOLD")); run(1); expect_reverse(); continue;
      case 1: b2=1; run(1); reason="STOP BTN2"; break;
      case 2: adc_failure=1; run(10); reason="IR ERROR"; break;
      case 3: next_us.status=US2_NO_RISE; run(60); reason="US1 NO RISE"; break;
      case 4: now+=151u; CalibrationApp_Update(); reason="SENSOR STALE"; break;
      default: /* A bad echo during a late hold must still cancel the reverse. */
        run(245); expect_stopped(); assert(now-start_at>10000u);
        next_us.status=US2_NO_RISE; run(60u-(now-last_ping)); reason="US1 NO RISE";
        next_us.status=US2_OK; break;
    }
    expect_stopped(); const int n=drives; run(1800); expect_stopped();
    assert(n==drives && strstr(row[5],reason)); /* no deferred hold/reverse after an abort */
  }
  puts("Board US1 approach: result-time latch, live steering and 349/350 ms wheel writes; late approach continues; BTN2, ADC, echo and stale data still preempt PASS");
}
/* Start with R2 held on black and analog centre only. Count three later digital entries within the
 * 750 ms launch, but never replace the launch command with a pivot. */
static uint32_t three_during_launch(void) {
  b1=b2=adc_failure=timer_fail=motor_fail=0; n_us=n_ir=0; ir_delay_once=0;
  next_us=(US2_Reading){.status=US2_OK,.raw_mm=200}; edges(0,1); tape(0,1,0);
  boot_drive(); run(160); const uint32_t started=now+71u; start(); expect_launch();
  assert(strstr(row[0],testing_f2 ? "F2 LINE BR 0 " : "F1 LINE BR 0 "));
  edges(0,0); run(90); edges(1,0); run(100); expect_launch();
  assert(strstr(row[0],"BR 1 "));
  edges(0,0); run(90); edges(1,0); run(100); expect_launch();
  assert(strstr(row[0],"BR 2 "));
  edges(0,0); run(90); edges(0,1); tape(0,1,1); run(120); expect_launch();
  assert(strstr(row[0],"BR 3 ")); return started;
}
static void branch_priority_tests(void) {
  const uint32_t started=three_during_launch();
  /* Let the last launch scan see white and COMPLETE across 750 ms, before
   * either pivot wheel has been written. That scan is not centre loss. */
  edges(0,0); tape(0,0,0); assert(now-started<740u); run(740u-(now-started)); expect_launch();
  ir_delay_once=12; run(10u-(now-last_ir));
  assert(last_ir-started<=750u && now-started>=750u);
  if (testing_f2) {
    expect_f2_right_pivot(); ir[1]=4095; /* first post-command filtered centre is black */
    run(180); expect_f2_right_pivot(); assert(strstr(row[5],"F2 FIND LINE"));
    ir[1]=100; run(10u-(now-last_ir)); ir[1]=4095; run(100);
    expect_f2_right_pivot(); /* one raw white sample did not clear the filtered centre */
    tape(1,0,0); run(140); expect_f2_right_pivot(); /* loss, but left-only cannot pick up */
    tape(0,0,1); run(140); expect_f2_right_pivot(); /* neither can right-only */
    tape(0,1,0); run(10u-(now-last_ir)); expect_f2_right_pivot(); /* raw black, mean still white */
    run(20); expect_f2_right_pivot(); run(10); expect_straight(); /* first filtered centre black */
    puts("F2 board: 15.75%/9.45% pivot; launch-white, held centre, side-only and raw-only hits cannot end it; filtered centre pickup resumes following PASS");
  } else { tape(0,1,0); run(160); expect_straight(); }
  edges(1,0); tape(1,1,0); run(150); expect_straight(); assert(strstr(row[0],"BR 4 "));

  /* A fourth digital entry while analog tape is being reacquired counts for
   * F1, but F2 does not queue a second turn or add BR while pivoting. */
  three_during_launch(); run(320); expect_third_crossing();
  edges(0,0); tape(0,0,0); run(180); edges(1,0); tape(1,1,0); run(150);
  expect_straight();
  assert(strstr(row[0],testing_f2 ? "BR 3 " : "BR 4 "));

  /* Abort/faults stop both wheels. Even after 10 s a US1 target can take
   * over instead of resuming the F2 branch turn. */
  for (unsigned event=0;event<5;event++) {
    const char *why;
    three_during_launch(); run(320); expect_third_crossing();
    if (event==0) { b2=1; run(1); why="STOP BTN2"; }
    else if (event==1) { adc_failure=1; run(10); why="IR ERROR"; }
    else if (event==2) { next_us.status=US2_NO_RISE; run(60); why="US1 NO RISE"; }
    else if (event==3) { now+=151; CalibrationApp_Update(); why="SENSOR STALE"; }
    else {
      tape(0,1,0); run(12000); /* centre stays on the old tape; never use elapsed time as pickup */
      if (testing_f2) { expect_f2_right_pivot(); assert(strstr(row[5],"F2 FIND LINE")); }
      else expect_straight();
      next_us.raw_mm=59; run(60u-(now-last_ping)); expect_straight(); /* F2 pivot preempted */
      tape(1,0,0); next_us.raw_mm=90; run(349); expect_left(); /* clear ping cannot cancel */
      run(1); why="TARGET 60MM HOLD";
    }
    expect_stopped(); run(110); assert(strstr(row[5],why));
  }
  /* Stop out of the hold; the next accepted B1 resets the finish and tally. */
  next_us.raw_mm=200; b2=1; run(1); b2=0; edges(0,0); tape(0,1,0); run(160);
  start(); run(800); edges(1,0); tape(1,1,0); run(150); edges(0,0); tape(0,1,0); run(150);
  edges(0,1); tape(0,1,1); run(150); expect_second_crossing();
  edges(0,0); run(100); edges(1,0); run(150); expect_third_crossing();
  tape(1,1,1); run(300); expect_third_crossing(); /* no inherited finish */
}
/* F1's displayed tally is per active run, not a count of unique waypoints.
 * Verify it through the public LCD and motor API, without running F2 or F3. */
static void f1_counter_return_test(void) {
  begin_pivot(false); run(CCW_MIN_MS+50); edges(1,0); tape(1,1,0); run(120);
  assert(strstr(row[0],"F1 LINE BR 1 ") && strstr(row[5],"TARGET LINE GO"));
  edges(0,0); tape(0,1,0); run(CCW_ALIGN_MS+100); edges(0,1); tape(1,1,0); run(180);
  expect_straight();
  assert(strstr(row[0],"F1 LINE BR 2 ") && strstr(row[5],"TARGET FOLLOW"));

  b2=1; run(120); b2=0; expect_stopped();
  next_us.raw_mm=59; run(160); start(); run(120); expect_stopped();
  assert(strstr(row[0],"F1 LINE BR 2 ") && strstr(row[5],"TARGET <60MM"));
  next_us.raw_mm=200; run(160); start(); run(120); expect_launch();
  assert(strstr(row[0],"F1 LINE BR 0 ")); /* accepted start, R2 still on black */
  puts("F1 counter on return: stop/denial retain it; the next accepted start resets it PASS");
}
/* Compare expected masks, TFT and actual signed wheel commands on either
 * side of each captured T, including equality. Deltas point towards black. */
static void threshold_edges(const uint16_t t[3], const int sign[3]) {
  for (int l=-1; l<=1; ++l) for (int c=-1; c<=1; ++c) for (int r=-1; r<=1; ++r) {
    /* Give every threshold combination the same centre-only, cleared history. */
    ir[0]=t[0]-sign[0]; ir[1]=t[1]+sign[1]; ir[2]=t[2]-sign[2]; run(50); expect_straight();
    ir[0]=t[0]+l*sign[0]; ir[1]=t[1]+c*sign[1]; ir[2]=t[2]+r*sign[2]; run(150);
    char pattern[16];
    snprintf(pattern,sizeof(pattern),"LINE %c %c %c",l>0?'B':'W',c>0?'B':'W',r>0?'B':'W');
    assert(strncmp(row[3],pattern,strlen(pattern))==0 && strstr(row[0],"BR 0 "));
    const unsigned mask=(l>0 ? 1u : 0u) | (c>0 ? 2u : 0u) | (r>0 ? 4u : 0u);
    if (mask==1u || mask==5u) { expect_left(); assert(strstr(row[5],"RUN LEFT")); }
    else if (mask==4u) { expect_right(); assert(strstr(row[5],"RUN RIGHT")); }
    else {
      expect_straight();
      assert(strstr(row[5],mask==0u ? "NO LINE STRAIGHT" : mask==2u ? "RUN STRAIGHT" : "BRANCH STRAIGHT"));
    }
  }
}
static void f1_threshold_edges_test(void) {
  edges(0,0);
  b1=b2=adc_failure=timer_fail=motor_fail=0; n_ir=n_us=0; ir_delay_once=0;
  next_us=(US2_Reading){.status=US2_OK,.raw_mm=200}; tape(0,0,0);
  boot_drive(); run(160); start(); run(800);
  assert(!strcmp(row[7],"T  362  712 4004") && strstr(row[8],"BLACK H H H"));
  threshold_edges((uint16_t[]){362,712,4004},(int[]){1,1,1});
  puts("F1 captured H/H/H thresholds: 27 below/equal/above combinations match TFT and wheel commands PASS");
}
static void mixed_profile_drive(void) {
  edges(0,0);
  const uint16_t white[]={3900,1100,3700}, black[]={151,3133,431};
  const uint16_t t[]={2025,2116,2065}; const int sign[]={-1,1,-1};
  testing_f2=false;
  b1=b2=adc_failure=timer_fail=motor_fail=0; n_ir=n_us=0; ir_delay_once=0;
  next_us=(US2_Reading){.status=US2_OK,.raw_mm=200}; memcpy(ir,white,sizeof(ir));
  int n=drives; boot_profile(white,black); /* the apply chord is not a start */
  DriveProbe_Expect(DRIVE_F1); adc_failure=1; keys(1,160); keys(0,120); expect_stopped();
  assert(strstr(row[3],"IR NOT READY"));
  adc_failure=0; keys(0,160); assert(drives==n); /* good scans cannot resurrect a denied tap */
  start(); expect_launch(); run(800);
  assert(!strcmp(row[7],"T 2025 2116 2065") && strstr(row[8],"BLACK L H L"));
  threshold_edges(t,sign);
  /* The F1 pivot needs both a clear RAW+filtered scan and a later agreeing
   * hit. A fixed/all-high raw classifier cannot pick up this black-low left. */
  next_us.raw_mm=59; run(350+160); expect_stopped(); next_us.raw_mm=200;
  memcpy(ir,white,sizeof(ir)); run(2500); expect_ccw();
  run(200); ir[0]=black[0]; run(550); expect_left(); assert(strstr(row[5],"TARGET FOLLOW"));
  keys(3,1); expect_stopped(); keys(1,200); keys(0,120);
  assert(strstr(row[6],"B1 RUN") && !strcmp(row[7],"T 2025 2116 2065"));
  puts("Production handoff: fresh B1 and live sensors required; 27 mixed-polarity edges, raw pivot pickup and BTN2 retains T PASS");
}
/* Real selector, branch detector, debounce, route controllers and signed
 * motor dispatch. DriveProbe_Expect checks the route; it cannot choose one. */
static void expect_stage(DriveFunction f) {
  DriveProbe_Expect(f); testing_f2 = f == DRIVE_F2;
}
static void mission_boot(bool hold_b1) {
  edges(0,0);
  testing_f2=false;
  b1=hold_b1; b2=adc_failure=motor_fail=timer_fail=0; n_ir=n_us=0; ir_delay_once=0;
  tape(0,1,0); next_us=(US2_Reading){.status=US2_OK,.raw_mm=200};
  boot_drive(); run(160); expect_stage(DRIVE_F1); expect_stopped();
  assert(strstr(row[0],"F1 LINE BR 0") && strstr(row[6],"B1 RUN"));
}
/* A selection gesture is never a run request; watch the actual wheel writes. */
static void cycle_stopped(DriveFunction next) {
  const int before=drives; const unsigned old=DriveProbe_Cycles();
  keys(3,80); expect_stopped(); assert(DriveProbe_Cycles()==old);
  keys(0,31); expect_stage(next); expect_stopped();
  assert(DriveProbe_Cycles()==old+1u && before==drives && strstr(row[6],"B1 RUN"));
  assert(strstr(row[0],next==DRIVE_F1 ? "F1 LINE BR 0" : "F2 LINE BR 0"));
}
static void target_then_return(void) {
  next_us.raw_mm=59; run(350+160); expect_stopped(); assert(strstr(row[5],"TARGET 60MM HOLD"));
  next_us.raw_mm=200; run(1100); expect_reverse();
  tape(0,0,0); run(500); expect_ccw(); pick_up_centre();
}
static void mission_test(void) {
  mission_boot(true); run(200); expect_stopped(); assert(!drives);
  b1=0; run(40); b1=1; run(15); b1=0; run(40); assert(!drives); /* bounce */
  next_us.raw_mm=59; run(200); start(); run(120); expect_stopped();
  assert(!drives && strstr(row[5],"TARGET <60MM") && strstr(row[6],"B1 RUN"));
  next_us.raw_mm=60; run(160); start(); expect_launch(); run(800);
  tape(1,0,0); run(160); expect_left();
  edges(1,0); tape(1,1,0); run(160); expect_straight(); assert(strstr(row[0],"F1 LINE BR 1"));
  edges(0,0); tape(0,0,0); run(160); expect_straight();
  tape(0,0,1); run(160); expect_right(); edges(0,1); tape(0,1,1); run(160);
  expect_second_crossing(); assert(strstr(row[0],"F1 LINE BR 2")); /* no F2 callback */
  start(); expect_second_crossing(); /* fresh press during F1 neither switches nor queues */
  /* Only filtered 101 starts a hold. A single raw BWB from WWB is too short. */
  edges(0,0); tape(0,0,1); run(160); expect_right();
  tape(1,0,1); run(10u-(now-last_ir)); tape(0,0,1); run(160); expect_right();
  tape(1,0,1); run(160); expect_left();
  assert(strstr(row[0],"F1 LINE BR 2") && strstr(row[3],"LINE B W B") && strstr(row[5],"RUN LEFT"));
  tape(0,0,0); run(160); expect_left(); assert(strstr(row[3],"LINE W W W") && strstr(row[5],"RUN LEFT"));
  edges(1,0); edges(0,0); run(160); expect_left(); assert(strstr(row[0],"F1 LINE BR 3"));
  tape(1,1,1); run(160); expect_left(); /* centre plus sides is not a release */
  assert(strstr(row[3],"LINE B B B") && strstr(row[5],"RUN LEFT") && strstr(row[0],"F1 LINE BR 3"));
  tape(0,0,1); run(160); expect_left();
  tape(0,1,0); run(10u-(now-last_ir)); tape(0,0,1); run(160); expect_left(); /* raw-only WBW */
  tape(0,1,0); run(10u-(now-last_ir)); run(20); expect_left(); /* not yet filtered centre only */
  run(10); expect_straight(); run(160); assert(strstr(row[3],"LINE W B W") && strstr(row[5],"RUN STRAIGHT"));
  tape(0,0,1); run(160); expect_right(); /* no wider deadband after release */
  tape(1,0,1); run(160); expect_left(); /* fresh split before the US1 turnaround */
  target_then_return();
  tape(0,0,0); run(650); expect_straight(); /* outbound BBB and return centre did not arm */
  assert(strstr(row[5],"TARGET FOLLOW"));
  tape(1,0,1); run(160); expect_left();
  tape(1,1,1); run(160); expect_left(); /* genuine BBB arms the finish despite held steering */
  assert(strstr(row[3],"LINE B B B") && strstr(row[5],"TARGET FOLLOW L"));
  tape(0,0,0); run(650); expect_left(); /* held left, armed WWW, but US1 still far */
  b1=1; run(40); /* press began while moving */
  next_us.raw_mm=54; run(60u-(now-last_ping)); expect_stopped(); /* terminal, not a second turnaround */
  run(120); assert(strstr(row[5],"TARGET LINE END") && strstr(row[6],"B1 RUN"));
  int stopped_drives=drives; next_us.raw_mm=200; run(400); b1=0; run(40);
  expect_stopped(); assert(drives==stopped_drives); /* B1 held across a finish cannot restart F1 */
  next_us.status=US2_NO_RISE; run(160); start(); run(110); expect_stopped();
  assert(drives==stopped_drives && strstr(row[6],"B1 RUN") && strstr(row[0],"F1 LINE BR"));
  next_us.status=US2_OK; next_us.raw_mm=59; run(160); start(); run(110);
  assert(drives==stopped_drives && strstr(row[5],"TARGET <60MM") && strstr(row[6],"B1 RUN"));

  /* Rerun the SAME F1 on BBB after its committed hold/finish. If any legacy
   * next-run arm leaks in, this reverses instead of launching and following. */
  next_us.raw_mm=60; tape(1,1,1); run(160); start(); expect_launch(); run(800);
  expect_straight(); run(100); /* TFT redraw can lag the first post-launch motor command */
  assert(strstr(row[0],"F1 LINE BR 0") && strstr(row[5],"BRANCH STRAIGHT"));
  keys(2,1); expect_stopped(); keys(0,120);
  edges(0,1); edges(0,0); /* pre-start IRQs are diagnostics, not future BR */
  cycle_stopped(DRIVE_F2);
  start(); expect_launch(); run(800); expect_straight(); assert(strstr(row[0],"F2 LINE BR 0"));
  edges(1,0); tape(0,1,0); run(160); expect_straight();
  assert(strstr(row[0],"F2 LINE BR 1") && strstr(row[3],"LINE W B W"));
  edges(1,1); tape(1,1,1); run(160); expect_straight();
  assert(strstr(row[0],"F2 LINE BR 1")); /* delayed R2 while L2 is held */
  edges(0,1); tape(1,0,1); run(160); expect_straight(); /* ordinary F2 101 is still straight */
  assert(strstr(row[3],"LINE B W B") && strstr(row[5],"BRANCH STRAIGHT") && strstr(row[0],"BR 1 "));
  edges(0,0); tape(0,1,1); run(160); start(); expect_straight(); assert(strstr(row[0],"BR 1 "));
  tape(0,1,0); run(160); tape(1,0,1); run(160); expect_straight(); assert(strstr(row[0],"BR 1 "));
  edges(0,1); run(160); expect_second_crossing(); /* same 101, still following */
  edges(0,0); run(100); edges(0,1); run(160); expect_third_crossing(); /* R2 alone triggers the pivot */
  tape(1,1,1); run(160); expect_third_crossing(); /* BBB during F2's clockwise pivot */
  edges(0,0); tape(0,0,0); run(650); expect_f2_right_pivot(); assert(strstr(row[5],"F2 FIND LINE"));
  tape(0,0,1); run(160); expect_f2_right_pivot(); /* real F1 -> F2 plan: a side hit is not the exit */
  tape(0,1,0); run(160); edges(1,0); tape(1,1,0); run(160); expect_straight();
  assert(strstr(row[0],"F2 LINE BR 4"));
  edges(0,0); target_then_return(); edges(0,1); run(160);
  assert(strstr(row[0],"F2 LINE BR 4")); /* a return-leg side hit is not outbound BR */
  tape(0,0,0); run(650); expect_straight(); /* no return BBB in F2 yet */
  assert(strstr(row[5],"TARGET FOLLOW"));
  finish_return_on_white(); assert(strstr(row[6],"B1 RUN") && strstr(row[0],"F2 LINE BR 4"));
  /* F2 can run again without cycling, with no inherited branch count or finish. */
  tape(1,1,1); run(160); start(); expect_launch(); run(800); expect_straight();
  assert(strstr(row[0],"F2 LINE BR 0")); keys(2,1); expect_stopped(); keys(0,120);
  assert(strstr(row[5],"STOP BTN2") && strstr(row[0],"F2 LINE BR 0"));
  cycle_stopped(DRIVE_F1); start(); expect_launch(); run(800);
  tape(1,0,1); run(160); tape(0,0,1); run(160); expect_left();
  keys(2,1); expect_stopped(); keys(0,120);
  assert(strstr(row[5],"STOP BTN2") && strstr(row[0],"F1 LINE BR 0"));
  start(); expect_launch(); run(800); expect_right(); /* fresh run on 001, no inherited split */
  keys(2,1); expect_stopped(); keys(0,120);
  puts("Production selector: F1 return finish, explicit F2 selection and branch-3 turn, repeated F2 and wrap to F1 PASS");
}
/* Replay the late-target case on both routes using the production selector.
 * Inspect signed motor requests on both sides of each boundary, with the
 * board loop polling the sensor mocks every 10 ms (IR) and 60 ms (US1). */
static void late_target(DriveFunction stage, uint32_t near_ms) {
  expect_stage(stage);
  tape(0,1,0); next_us.raw_mm=200; run(160);
  /* Align the qualified B1 release, not its initial press, to the ping cadence. */
  keys(1,40); expect_stopped(); run((60u-(now-last_ping+31u+near_ms)%60u)%60u);
  keys(0,31); expect_launch(); const uint32_t started=now;
  run(near_ms-1u); expect_straight();
  next_us.raw_mm=35; run(1); assert(last_ping==now && now-started==near_ms);
  run(349); expect_straight(); run(1); expect_stopped();
  run(999); expect_stopped(); assert(strstr(row[5],"TARGET 60MM HOLD"));
  run(1); expect_reverse(); assert(now-started==near_ms+1350u);
  run(499); expect_reverse(); assert(strstr(row[5],"TARGET REV 18%"));
  run(1); expect_ccw(); next_us.raw_mm=200; pick_up_centre();
  tape(0,0,0); run(20000); expect_straight(); assert(strstr(row[5],"TARGET FOLLOW"));
  finish_return_on_white(); assert(strstr(row[6],"B1 RUN")); expect_stage(stage);
}
static void late_targets(void) {
  const uint32_t near_ms[]={8900u,12500u};
  for(unsigned i=0;i<sizeof near_ms/sizeof *near_ms;++i) {
    mission_boot(false); late_target(DRIVE_F1,near_ms[i]); cycle_stopped(DRIVE_F2); late_target(DRIVE_F2,near_ms[i]);
  }
  puts("F1 and F2 production replay: near at 8.90/12.50 s, reverse at 10.25/13.85 s for 500 ms, normal finish after long returns PASS");
}
static void selector_button_tests(void) {
  const uint32_t boot_at=now; mission_boot(false); const uint32_t boot_ms=now-boot_at;
  const int n=drives, mi=motor_init, ti=us_init; const unsigned cycles=DriveProbe_Cycles();
  char thresholds[sizeof row[7]], polarity[sizeof row[8]];
  strcpy(thresholds,row[7]); strcpy(polarity,row[8]);
  /* A near-chord can neither launch nor select. Non-overlapping singles with
   * too short an all-up gap are not a chord, whichever button was first. */
  keys(1,450); keys(3,29); keys(1,140); keys(0,100);
  keys(2,200); keys(0,10); keys(1,200); keys(0,100);
  assert(drives==n && DriveProbe_Cycles()==cycles && strstr(row[0],"F1 LINE BR 0"));
  for (unsigned i=0; i<4; ++i) {
    keys(i&1u ? 2u : 1u,240); keys(3,600); keys(i&2u ? 2u : 1u,180);
    keys(0,18); keys(1,3); keys(0,30); /* release bounce */
    assert(drives==n && DriveProbe_Cycles()==cycles+i); expect_stopped();
    keys(0,1); expect_stage(i&1u ? DRIVE_F1 : DRIVE_F2); keys(0,150);
    assert(DriveProbe_Cycles()==cycles+i+1u && drives==n);
    assert(strstr(row[0],i&1u ? "F1 LINE BR 0" : "F2 LINE BR 0"));
    assert(!strcmp(row[6],"B1 RUN  1+2 NEXT"));
  }
  /* Select F2 before ever running F1. Selection does not bypass a fault,
   * consume a run, recapture T or reinitialize the motor/timer hardware. */
  next_us.status=US2_NO_RISE; run(120); cycle_stopped(DRIVE_F2); start(); expect_stopped();
  assert(drives==n && strstr(row[5],"WAIT SENSORS"));
  next_us.status=US2_OK; run(160); expect_stopped(); keys(1,650); keys(0,30); assert(drives==n);
  keys(0,1); expect_launch(); run(800); expect_straight();
  edges(1,0); run(150); assert(strstr(row[0],"F2 LINE BR 1") && !strcmp(row[6],"B2 STOP 1+2 NEXT"));
  const int before_stop=drives; keys(2,1); expect_stopped(); keys(2,160); keys(0,120);
  assert(drives==before_stop && strstr(row[0],"F2 LINE BR 1") && strstr(row[5],"STOP BTN2"));
  edges(0,0);
  for (unsigned i=0; i<2; ++i) {
    start(); expect_launch(); run(800); expect_straight(); assert(strstr(row[0],"BR 0 "));
    const unsigned old=DriveProbe_Cycles();
    if (!i) keys(1,200); /* B1 alone while running does not stop or queue a start */
    const int moving=drives;
    if (i) { keys(2,1); expect_stopped(); keys(2,180); } /* B2 leads the overlap */
    keys(3,1); expect_stopped(); keys(3,160); keys(i ? 2u : 1u,180);
    keys(0,30); assert(DriveProbe_Cycles()==old && drives==moving && strstr(row[5],"STOP BTN2"));
    assert(strstr(row[0],i ? "F1 LINE BR" : "F2 LINE BR")); /* not switched while held */
    keys(0,1); expect_stage(i ? DRIVE_F2 : DRIVE_F1); expect_stopped();
    assert(DriveProbe_Cycles()==old+1u && strstr(row[0],i ? "F2 LINE BR 0" : "F1 LINE BR 0"));
    keys(0,160); assert(drives==moving); /* select, never auto-run on release */
  }
  assert(mi==motor_init && ti==us_init && !strcmp(thresholds,row[7]) && !strcmp(polarity,row[8]));
  /* Hardware reset from selected F2 requires calibration and defaults to F1.
   * Place the following chord and release across the 32-bit HAL tick wrap. */
  now=UINT32_MAX-boot_ms-12u; mission_boot(true); expect_stopped();
  const uint32_t chord_at=now; cycle_stopped(DRIVE_F2); assert(now<chord_at);
  puts("B1 release runs fresh; B2 stops and retains F2; deliberate chords stop then cycle once in either order, no F3 or recalibration PASS");
}

static void poll_button_tests(void) {
  mission_boot(false); start(); run(800); run(59u-(now-last_ping)); int n=drives;
  press_in_ir=2; zero_at_ping=true; run(1); zero_at_ping=false; /* B2 during ADC: zero BEFORE the ping */
  assert(!press_in_ir && last_ir==now && last_ping==now && drives==n); expect_stopped();
  keys(0,120); assert(strstr(row[5],"STOP BTN2"));
  start(); run(800); run(59u-(now-last_ping)); n=drives;
  press_in_ping=2; run(1); assert(!press_in_ping && drives==n); expect_stopped(); keys(0,120);
  /* A qualified B1-up event earlier in this update is cancelled by B2 in the ping. */
  keys(1,90); run((60u-(now-last_ping+31u)%60u)%60u); keys(0,30);
  press_in_ping=2; run(1); assert(!press_in_ping && last_ping==now && drives==n); expect_stopped();
  keys(0,160); assert(drives==n && strstr(row[0],"F1 LINE BR"));
  /* B1 first appears INSIDE the same measurement that faults a run. A later
   * good reading and release must not turn that mid-run press into a restart. */
  start(); run(800); run(59u-(now-last_ping)); n=drives;
  next_us.status=US2_NO_RISE; press_in_ping=1; run(1); expect_stopped();
  next_us.status=US2_OK; run(180); assert(b1 && strstr(row[5],"US1 NO RISE"));
  keys(0,150); assert(drives==n); expect_stopped(); start(); expect_launch();
  puts("ADC/ping button races: raw B2 preempts wheel dispatch, no queued B1 across a mid-poll fault or stop PASS");
}

#ifdef DIGITAL_SCREENS_TEST
/* Digital line IRs: the setup page shows live levels (no capture) and B1+B2 goes to drive; the friendlier pages. */
static void digital_screens(void) {
  ir[0]=0; ir[1]=IR_DIGITAL_BLACK; ir[2]=0;
  check_cal_zero=true; CalibrationApp_Init(); run(160);
  assert(!strcmp(row[0],"AR SETUP        ") && !strcmp(row[1],"1=TAPE  0=FLOOR "));
  assert(!strcmp(row[2],"FRONT L0 C1 R0  ") && !strcmp(row[3],"SIDE  L0     R0 "));
  assert(strstr(row[5],"R WHEEL") && strstr(row[5],"--") && strstr(row[6],"L WHEEL")); /* F1_LEGACY build: no encoders */
  assert(strstr(row[7],"READY") && !strcmp(row[8],"B1+B2 = DRIVE   ") && !strcmp(row[9],"MOTORS HELD AT 0"));
  ir[0]=IR_DIGITAL_BLACK; gpiob.dummy=IR_R2_Pin; run(150);
  assert(!strcmp(row[2],"FRONT L1 C1 R0  ") && !strcmp(row[3],"SIDE  L0     R1 "));
  ir[0]=0; gpiob.dummy=0;
  keys(1,60); keys(0,200); keys(2,60); keys(0,200); /* B1 or B2 alone: still setup, nothing captured or started */
  assert(strstr(row[0],"AR SETUP") && !us_init && !drives && j1==0 && j3==0);
  keys(3,100); keys(0,150); check_cal_zero=false;
  assert(us_init==1 && !drives && j1==0 && j3==0);
  run(300);
  assert(strstr(row[0],"F1 LINE BR 0") && !strcmp(row[1],"FRONT L0 C1 R0  ") && !strcmp(row[2],"SIDE  L0     R0 "));
  assert(!strcmp(row[3],"US1  171 mm     ") && !strcmp(row[6],"B1 = RUN        ") && !strcmp(row[7],"B1+B2 = TO F2   "));
  assert(strstr(row[9],"ESTOP REQUIRED"));
  ir[1]=0; run(100); assert(!strcmp(row[1],"FRONT L0 C0 R0  ")); /* tape gone on 2 scans */
  ir[1]=IR_DIGITAL_BLACK; run(100);
  keys(1,60); keys(0,100); expect_launch(); run(250); assert(j1!=0 && j3!=0); /* still driving */ assert(!strcmp(row[6],"B2 = STOP       "));
  keys(2,40); keys(0,100); expect_stopped(); assert(!strcmp(row[6],"B1 = RUN        "));
  puts("Digital IR pages: live FRONT/SIDE levels, B1+B2 to drive without capture, US1, B1 RUN / B2 STOP hints PASS");
}
#endif
int main(int argc,char **argv){
#ifdef DIGITAL_SCREENS_TEST
  if (argc==2 && !strcmp(argv[1],"digital-screens")) { digital_screens(); return 0; }
  assert(0 && "the analog cases need IR_DIGITAL=0");
#endif
  if (argc==2 && !strcmp(argv[1],"side-ir-setup")) { side_setup_test(); return 0; }
  if (argc==1) {
    mission_test(); late_targets(); selector_button_tests(); poll_button_tests(); mixed_profile_drive(); DriveProbe_Check(motor_init); return 0;
  }
  if (argc==2 && !strcmp(argv[1],"invalid-plan")) {
    DriveProbe_Simulate((DriveFunction)99); /* a corrupt/unknown plan must fail closed */
    boot_drive(); run(150); start(); run(500); expect_stopped();
    assert(!drives && !n_us && n_ir==200 && strstr(row[0],"DRIVE PLAN ERROR"));
    DriveProbe_Check(motor_init); puts("Unknown boot plan inhibits all drive callbacks and motor commands PASS"); return 0;
  }
  testing_f2=argc==2 && !strcmp(argv[1],"simulate-f2");
  assert(argc==2 && (testing_f2 || !strcmp(argv[1],"pin-f1")));
  DriveProbe_Simulate(testing_f2 ? DRIVE_F2 : DRIVE_F1); /* isolated regression only */
  b1=1;boot_drive();run(200);assert(motor_init==1&&ir_init==1&&us_init==1);expect_stopped();assert(!drives);
  b1=0;run(40);start();expect_launch();assert(strstr(row[8],"BLACK H H H"));
  run(1100);expect_straight(); /* launch finished */
  /* First digital crossing: L2 only. Analog 110 commands straight. */
  edges(1,0);tape(1,1,0);run(220);
  expect_straight();assert(strstr(row[5],"BRANCH STRAIGHT"));assert(strstr(row[0],"BR 1 "));
  /* Clear the branch for longer than the rearm time, then a normal line. */
  edges(0,0);ir[0]=100;run(220);expect_straight();assert(strstr(row[5],"RUN STRAIGHT"));
  /* Second digital crossing: R2 only, neither policy turns yet. */
  edges(0,1);tape(0,1,1);run(220);expect_second_crossing();
  run(200);expect_second_crossing(); /* holding the branch does not recount */
  edges(0,0);run(120);edges(1,0);run(150);expect_third_crossing();
  edges(0,0);tape(0,0,0);run(220);
  if (testing_f2) {
    expect_f2_right_pivot();assert(strstr(row[5],"F2 FIND LINE"));
  } else {
    expect_straight();assert(strstr(row[3],"LINE W W W") && strstr(row[5],"NO LINE STRAIGHT"));
  }
  /* Line reacquired: normal following in both policies. Subsequent crossings
   * increment BR but do not issue another turn in this run. */
  tape(0,1,0);run(220);expect_straight();assert(strstr(row[5],"RUN STRAIGHT"));
  edges(1,1);tape(0,1,1);run(220);expect_straight();assert(strstr(row[5],"BRANCH STRAIGHT"));
  assert(strstr(row[0],"BR 4 "));
  /* Without a split, a filtered 110 or 011 clears the ordinary F1 side correction.
   * F2 already took branch 3: its ordinary per-scan policy also goes straight. */
  tape(0,0,1);run(220);expect_right();assert(strstr(row[5],"RUN RIGHT"));
  tape(1,0,0);run(220);expect_left();assert(strstr(row[5],"RUN LEFT"));
  tape(1,1,0);run(220);assert(strstr(row[3],"LINE B B W"));
  expect_straight();assert(strstr(row[5],"BRANCH STRAIGHT"));
  tape(0,0,0);run(220);assert(strstr(row[3],"LINE W W W"));
  expect_straight();assert(strstr(row[5],"NO LINE STRAIGHT"));
  tape(0,0,1);run(220);expect_right();
  tape(0,1,1);run(220);assert(strstr(row[3],"LINE W B B"));
  expect_straight();assert(strstr(row[5],"BRANCH STRAIGHT"));
  tape(0,0,0);run(220);assert(strstr(row[3],"LINE W W W"));
  expect_straight();assert(strstr(row[5],"NO LINE STRAIGHT"));
  tape(0,1,0);run(220);expect_straight();assert(strstr(row[5],"RUN STRAIGHT"));
  next_us.raw_mm=59;run(350+120);assert(j1==0&&j3==0); /* stopped after the approach */
  next_us.raw_mm=200;run(1100); /* hold 1 s, then reverse 18% for 500 ms */
  expect_reverse();
  tape(1,1,1);run(100); /* IR (even three black) ignored */
  expect_reverse();
  ir[0]=ir[1]=ir[2]=100;run(350);expect_ccw(); /* pivots, no line */
  pick_up_centre();
  finish_return_on_white();
  tape(0,1,0);run(220);
  start();run(1100);expect_straight(); /* branch alone does not stop */
  next_us.raw_mm=59;run(350+220);expect_stopped();assert(stop1&&stop3); /* US1 hold starts */
  tape(0,1,0);next_us.raw_mm=200;run(1100); /* hold 1 s, then reverse */
  expect_reverse();
  ir[0]=ir[1]=ir[2]=100;run(400);expect_ccw(); /* rotate anticlockwise */
  pick_up_centre();
  finish_return_on_white();
  assert(strstr(row[6],"B1 RUN"));
  start();run(1100);expect_straight();
  next_us.status=US2_NO_RISE;run(60);expect_stopped(); /* within one ping period */
  run(100);assert(strstr(row[5],"US1 NO RISE") && strstr(row[4],"US1 NO RISE")); /* then one TFT refresh */
  next_us.status=US2_ECHO_BUSY;run(220);assert(strstr(row[4],"US1 ECHO HIGH") && strstr(row[5],"US1 NO RISE"));
  int old_drives=drives;start();expect_stopped();assert(drives==old_drives);
  next_us.status=US2_NO_FALL;run(220);assert(strstr(row[4],"US1 NO FALL"));
  next_us.status=US2_SHORT_PULSE;run(220);assert(strstr(row[4],"US1 SHORT PULSE"));
  next_us.status=US2_TIMER_ERROR;run(220);assert(strstr(row[4],"US1 TIMER ERROR"));
  next_us.status=US2_OK;tape(0,1,0);run(220);expect_stopped();
  assert(strstr(row[4],"US1  200 mm"));start();run(1100);expect_straight();
  b2=1;run(1);expect_stopped();b2=0;run(120);assert(strstr(row[5],"STOP BTN2"));
  start();run(1100);expect_straight();adc_failure=1;run(20);expect_stopped();run(100);assert(strstr(row[5],"IR ERROR"));
  adc_failure=0;run(100);start();run(1100);expect_straight();
  ir[0]=ir[1]=ir[2]=0;run(220);expect_straight();assert(strstr(row[5],"NO LINE STRAIGHT"));
  tape(0,1,0);run(220);start();run(1100);expect_straight();
  now+=151;CalibrationApp_Update();expect_stopped();run(100);assert(strstr(row[5],"SENSOR STALE"));
  run(150);start();run(1100);expect_straight();
  run(10100);expect_straight();assert(strstr(row[5],"RUN STRAIGHT"));
  b2=1;run(1);expect_stopped();b2=0;run(120);assert(strstr(row[5],"STOP BTN2"));
  /* Exercise the actual GPIO -> IRQ -> scan -> TFT path in both policies. */
  edges(0,0); edges(1,0); edges(0,0); /* old, completed pulse while stopped */
  tape(0,1,0);run(220);assert(strstr(row[0],"BR 0 "));
  start();run(1100);expect_straight();assert(strstr(row[0],"BR 0 "));
  for (unsigned mask=0;mask<8;mask++) {
    tape(0,1,0);run(120);tape(mask&1u,mask&2u,mask&4u);run(140);
    assert(strstr(row[0],"BR 0 ")); /* all eight analog L/C/R patterns, no edge inputs */
  }
  tape(1,1,0); edges(1,0); run(150); assert(strstr(row[0],"BR 1 "));
  edges(1,1); run(150); edges(0,1); run(150); assert(strstr(row[0],"BR 1 "));
  /* First low sample, then two IRQs between the 70 and 80 ms scans. Using
   * the latest IRQ or foreground tick instead of the first would double-count. */
  edges(0,0); run(10u-(now-last_ir)); assert(now==last_ir);
  run(74); edges(0,1); edges(0,0); run(2); edges(1,0); run(140);
  assert(strstr(row[0],"BR 1 "));
  edges(0,0); run(10u-(now-last_ir)); run(75); edges(0,1); edges(0,0); /* not high on any scan */
  run(160); expect_second_crossing();
  tape(0,0,0);run(120); /* still ordinary following after crossing two */
  tape(0,1,0);run(220);edges(1,0);tape(0,1,1);run(220);expect_third_crossing();
  b2=1;run(1);expect_stopped();b2=0;run(120);
  edges(0,0);tape(1,1,1);run(220);assert(strstr(row[0],"BR 3 ")); /* B2 is stop, not reset */
  start();expect_launch();edges(1,0);edges(0,0);run(140);
  assert(strstr(row[0],"BR 1 ")); /* analog BBB at start must NOT seed the digital latch */
  puts("Digital BR: analog-only 8/8 ignored, either side counts, overlapping hits merge, between-scan 74/75 ms boundary PASS");
  timer_fail=1;n_us=0;n_ir=0;boot_drive();run(120);start();expect_stopped();assert(n_us==0&&strstr(row[5],"CHECK TIM2") && strstr(row[4],"US1 CHECK TIM2"));
  timer_fail=0;motor_fail=1;n_us=0;n_ir=0;boot_drive();run(120);start();expect_stopped();assert(strstr(row[5],"MOTOR INIT ERR"));
  launch_tests();
  pivot_tests();
  approach_tests();
  branch_priority_tests();
  if (!testing_f2) { f1_counter_return_test(); f1_threshold_edges_test(); }
  return_range_tests();
  DriveProbe_Check(motor_init);
  puts(testing_f2 ? "Host-only F2 replay on combined drive objects: one right turn, interlocks and no F3 callbacks PASS"
                   : "Isolated F1 replay: centre-release steering, branch count, launch/finish/US1/faults, zero F2/F3 callbacks PASS");
}
