/* Host-only replay of the dormant F3 path in the same combined board objects. */
#include "main.h"
#include "calibration_app.h"
#include "calibration_replay.h"
#include "drive_probe.h"
#include "elcp_zero_pwm.h"
#include "elcp_motor/elcp_motor.h"
#include "ir_sensor.h"
#include "us1_sensor.h"
#include "lcd/lcd.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

enum { L=1u, C=2u, R=4u };
static uint32_t now, last_ir, last_us;
static unsigned raw, writes, boots;
static bool b1,b2,adc_fault,bad_motor,bad_timer,zero_at_ping;
static US2_Reading ping;
static float left,right;
static char row[10][17];
uint32_t HAL_GetTick(void) { return now; }
bool mock_btn(int btn) { return btn==BTN1 ? b1 : b2; }
ELCP_ZeroPWM_Result ELCP_ZeroPWM_Init(void) { ++boots; left=right=0; return (ELCP_ZeroPWM_Result){!bad_motor,!bad_motor}; }
void ELCP_ZeroPWM_Hold(void) { left=right=0; }
void tft_init(TFT_ORIENTATION o,uint16_t bg,uint16_t fg,uint16_t a,uint16_t h) {
  (void)o; (void)bg; (void)fg; (void)a; (void)h; assert(left==0 && right==0);
}
void ELCP_Motor_Stop(ELCP_Motor_ID id) {
  if (id==MOTOR_J3) left=0; else { assert(id==MOTOR_J1); right=0; }
}
void ELCP_Motor_SetSpeed(ELCP_Motor_ID id, float v) {
  if (id==MOTOR_J3) { assert(v==-25 || v==-11 || v==-26.5f || v==-15); left=v; }
  else { assert(id==MOTOR_J1 && (v==25 || v==11 || v==26.5f || v==-12)); right=v; }
  ++writes;
}
void IR_Sensor_Init(void) {}
HAL_StatusTypeDef IR_ReadAll(IR_RawScan *s) {
  last_ir=now;
  if (adc_fault) return HAL_ERROR;
  if (capture_frame) { memcpy(s->raw,capture_frame,sizeof(s->raw)); return HAL_OK; }
  /* A single 3000 among three 100s gives 825, above the centre cutoff (712). */
  s->raw[0]=raw&L ? 800u : 100u; s->raw[1]=raw&C ? 3000u : 100u; s->raw[2]=raw&R ? 4095u : 100u;
  return HAL_OK;
}
bool US2_Init(void) { return !bad_timer; }
US2_Reading US1_Measure(void) {
  if (zero_at_ping) assert(left==0 && right==0); /* IR loss stops before this blocking poll */
  last_us=now; return ping;
}
void tft_printc(uint8_t x,uint8_t y,const char *s) {
  assert(x==0 && y<10 && strlen(s)<=16); strcpy(row[y],s);
}
void tft_prints(uint8_t x,uint8_t y,const char *fmt,...) {
  assert(x==0 && y<10); va_list ap; va_start(ap,fmt); int n=vsnprintf(row[y],17,fmt,ap); va_end(ap);
  assert(n>=0 && n<=16);
}
void tft_update(uint32_t v) { (void)v; }
static void tick(unsigned ms) { while(ms--) { ++now; CalibrationApp_Update(); } }
static void press(void) { b1=true; tick(40); b1=false; tick(40); }
static void wheels(float j3,float j1) { assert(left==j3 && right==j1); }
static void label(unsigned y,const char *s) { assert(strstr(row[y],s)); }
static void keys(unsigned mask,unsigned ms) { b1=mask&1u; b2=mask&2u; tick(ms); }
static void boot_drive(void) {
  unsigned prev=writes;
  CalibrationApp_Init(); tick(140); label(0,"IR CAL 100 SAMP");
  replay_calibration(keys,route_white,route_black);
  wheels(0,0); assert(writes==prev);
}
static void setup(unsigned tape, unsigned mm) {
  now=last_ir=last_us=writes=0; raw=tape; b1=b2=adc_fault=bad_motor=bad_timer=zero_at_ping=false;
  ping=(US2_Reading){.status=US2_OK,.raw_mm=mm}; memset(row,0,sizeof(row));
  boot_drive(); tick(160); label(0,"F3 RIGHT 0 "); wheels(0,0); assert(writes==0);
}
static void latched(const char *msg) {
  wheels(0,0); unsigned n=writes;
  raw=C; ping=(US2_Reading){.status=US2_OK,.raw_mm=200}; adc_fault=false; b2=false;
  tick(2700); wheels(0,0); assert(writes==n); label(5,msg); assert(!strstr(row[6],"ARMED"));
}
static void route(void) {
  setup(L,40); press(); wheels(-11,26.5f); /* line steers from the first command, no 750 ms launch */
  raw=L|C; tick(120); wheels(-25,25); label(0,"RIGHT 0 ");
  const unsigned choices[]={L|C|R,C|R,L|R};
  for (unsigned i=0;i<3;i++) {
    raw=choices[i]; tick(120); wheels(-15,-12); label(5,"F3 TURN RIGHT");
    char t[16]; snprintf(t,sizeof(t),"RIGHT %u ",i+1); label(0,t);
    raw=C; tick(120); wheels(-15,-12); label(0,t); /* old centre is not the selected exit */
    raw=R; tick(120); wheels(-26.5f,11); label(0,t);
    raw=C; tick(120); wheels(-25,25); label(0,t);
  }
  raw=R; tick(10u-(now-last_ir)); wheels(0,0); /* new raw right, filter still centre */
  tick(30); wheels(-26.5f,11); /* resumes only on agreeing tape */
  raw=0; tick(10u-(now-last_ir)); latched("F3 LINE LOST");
  puts("F3 board: immediate steering, left-only fork straight, three successive right exits, filter wait and no blind search PASS");
}
static void ultrasonic(void) {
  for(unsigned phase=0;phase<3;phase++) {
    setup(phase ? C|R : C,40); press();
    if (phase==2) {raw=R;tick(100);wheels(-26.5f,11);}
    else if (phase==1) wheels(-15,-12); else wheels(-25,25);
    ping.raw_mm=39;
    unsigned remaining=60u-(now-last_us); tick(remaining-1u); unsigned n=writes;
    tick(1); assert(writes==n); latched("F3 US1 <40MM"); /* no hold/reverse/turnaround or auto restart */
    ping.raw_mm=39;tick(120);press();wheels(0,0);assert(writes==n);
    ping.raw_mm=40;raw=L|C|R;tick(160);press();wheels(-15,-12);tick(110);label(0,"F3 RIGHT 1 ");
  }
  /* Force a new right fork and a near-target ping on the same board update. */
  setup(R,200); press(); tick(59u-(now-last_us)); unsigned before=writes;
  raw=C|R; ping.raw_mm=39; tick(1); assert(last_ir==now && last_us==now && writes==before);
  latched("F3 US1 <40MM"); label(0,"F3 RIGHT 1 "); /* fork seen, no pivot command issued */
  /* If raw tape is already gone, stop before spending time on the due US poll. */
  setup(C|R,200); press(); tick(59u-(now-last_us)); before=writes;
  raw=0; zero_at_ping=true; tick(1); zero_at_ping=false;
  assert(last_ir==now && last_us==now && writes==before); latched("F3 LINE LOST");
  puts("F3 board: valid US1 at 40 mm allows driving, 39 mm stops following/pivot/alignment and requires a fresh start PASS");
}
static void faults(void) {
  setup(0,200);press();tick(120);label(5,"F3 NEED LINE");wheels(0,0);assert(!writes);
  raw=C;tick(160);wheels(0,0);press();wheels(-25,25);
  for(unsigned f=0;f<6;f++) {
    setup(C|R,200);press();wheels(-15,-12);
    if (f==0) { b2=true;tick(1);latched("STOP BTN2"); }
    if (f==1) { adc_fault=true;tick(10);latched("IR ERROR"); }
    if (f==2) { ping.status=US2_NO_RISE;tick(60);latched("US1 NO RISE"); }
    if (f==3) { now+=151;CalibrationApp_Update();latched("SENSOR STALE"); }
    if (f==4) { raw=0;tick(10);latched("F3 LINE LOST"); }
    if (f==5) { tick(2510);latched("F3 TURN TIMEOUT"); }
  }
  setup(C,200); b1=true; boot_drive(); tick(500);wheels(0,0);assert(!writes);
  b1=false;tick(80);press();wheels(-25,25);
  for(unsigned f=0;f<2;f++) {
    setup(C,200);bad_motor=f==0;bad_timer=f==1;boot_drive();tick(160);press();wheels(0,0);assert(!writes);
  }
  puts("F3 board: starts gated, BTN2/ADC/US/staleness and turn deadline latch both motors stopped PASS");
}
static void long_follow(void) {
  setup(C,200); press(); tick(30000); wheels(-25,25); label(5,"F3 FOLLOW");
  raw=C|R; tick(120); wheels(-15,-12); label(0,"RIGHT 1 "); /* still accepts a new fork */
  raw=R; tick(90); raw=C; tick(120); wheels(-25,25);
  ping.raw_mm=39; tick(60); latched("F3 US1 <40MM");
  puts("F3 board: long run still follows, selects a right exit and stops on near US1 PASS");
}
int main(void) {
  DriveProbe_Simulate(DRIVE_F3); route(); ultrasonic(); faults(); long_follow(); DriveProbe_Check(boots);
}
