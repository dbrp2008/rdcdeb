/* Stationary UI only: linking this replay supplies no motor or drive API. */
#include "ir_setup.h"
#include "ir_sensor.h"
#include "lcd/lcd.h"
#include "main.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static uint32_t now, last_read;
static unsigned buttons, reads, accepts;
static bool fail;
static const uint16_t white[3]={1000,3201,1500}, black[3]={3000,1000,3500};
static const uint16_t *frame;
static char row[10][17];
static IR_Profile applied;
uint32_t HAL_GetTick(void) { return now; }
bool mock_btn(int btn) { return buttons & (btn==BTN1 ? 1u : 2u); }
HAL_StatusTypeDef IR_ReadAll(IR_RawScan *s) {
  if (reads) assert((uint32_t)(now-last_read)>=10);
  last_read=now;
  /* Rare outliers: the midpoint between ranges is NOT the midpoint of means. */
  unsigned bump = (reads++ % 100u == 99u) ? 90u : 0u;
  if (fail) return HAL_ERROR;
  for (unsigned i=0;i<3;i++) s->raw[i]=frame[i]+bump;
  return HAL_OK;
}
void tft_printc(uint8_t x,uint8_t y,const char *s) {
  assert(x==0 && y<10 && strlen(s)<=16); strcpy(row[y],s);
}
void tft_prints(uint8_t x,uint8_t y,const char *fmt,...) {
  assert(x==0 && y<10); va_list ap; va_start(ap,fmt);
  int n=vsnprintf(row[y],sizeof(row[y]),fmt,ap); va_end(ap); assert(n>=0 && n<=16);
}
void tft_update(uint32_t x) { (void)x; }
static void keys(unsigned mask,unsigned ms) {
  buttons=mask;
  while (ms--) {
    ++now;
    if (IRSetup_Update(now,&applied)) ++accepts;
    else if (!accepts) {
      /* Rejected/partial applications must not leak even a single channel. */
      for (unsigned i=0;i<3;i++) { assert(applied.t[i]==7+i); assert(applied.black_high[i]==(i==1)); }
    }
  }
}
static void reset(void) {
  reads=accepts=0; fail=false; frame=white;
  applied=(IR_Profile){.t={7,8,9},.black_high={false,true,false}};
  IRSetup_Init(now); assert(strstr(row[0],"IR CAL 100 SAMP"));
  assert(!strcmp(row[5],"T ---- ---- ----"));
}
static void capture(unsigned button,const uint16_t sample[3]) {
  frame=sample; keys(button,55); keys(0,1400);
}
static void chord(unsigned first) {
  keys(first,1500); keys(3,150); keys(3u^first,1500); keys(0,150);
}
static void ready(void) {
  assert(!strcmp(row[3],"W 1001 3202 1501")); assert(!strcmp(row[4],"B 3001 1001 3501"));
  assert(!strcmp(row[5],"T 2045 2145 2545")); assert(strstr(row[6],"BLACK H L H"));
  assert(strstr(row[7],"CAL READY") && strstr(row[8],"B1+B2")); assert(!accepts);
}
static void sample_and_apply(unsigned first,unsigned last) {
  now=UINT32_MAX-1600u; reset();
  keys(3,350); keys(first,100); keys(0,100); /* held at boot then staggered release */
  chord(first); assert(!reads && !accepts && strstr(row[7],"NEED W AND B"));
  keys(1,1500); assert(!reads); keys(0,31); keys(0,249); assert(!reads);
  keys(0,1); assert(reads==1); keys(0,989); assert(reads==99); keys(0,1); assert(reads==100);
  keys(0,140); assert(!strcmp(row[3],"W 1001 3202 1501"));
  chord(3u^first); assert(reads==100 && !accepts && !strcmp(row[5],"T ---- ---- ----"));
  assert(!strcmp(row[3],"W 1001 3202 1501") && !strcmp(row[4],"B ---- ---- ----"));

  frame=black; keys(2,55); keys(0,400); /* capture is busy, hold apply across its completion */
  keys(3,1500); keys(first,300); keys(0,200); assert(reads==200); ready();
  keys(first,600); keys(3,8); keys(3u^first,500); keys(0,200); /* short overlap cancels BOTH singles */
  assert(reads==200); ready();
  keys(first,1500); ready(); /* even a long first press doesn't invalidate a capture */
  keys(3,150); assert(strstr(row[7],"RELEASE BOTH")); keys(last,1500);
  keys(0,12); keys(last,100); keys(0,30); assert(!accepts && reads==200);
  keys(0,1); assert(accepts==1 && reads==200);
  assert(applied.t[0]==2045 && applied.t[1]==2145 && applied.t[2]==2545);
  assert(applied.black_high[0] && !applied.black_high[1] && applied.black_high[2]);
}
static void reject_and_retry(void) {
  reset(); keys(0,100); keys(1,10); keys(0,1500); assert(!reads); /* bounce alone */
  capture(2,black); capture(1,white); ready();
  /* Intentional solo recapture invalidates the old W immediately. An ADC
   * failure must not permit falling back to the last successful calibration. */
  fail=true; capture(1,white); fail=false;
  assert(strstr(row[7],"ADC ERROR") && !strcmp(row[3],"W ---- ---- ----"));
  unsigned n=reads; chord(1); assert(reads==n && !accepts);
  const uint16_t bad_adc[]={0,4096,0}; capture(1,bad_adc);
  assert(strstr(row[7],"ADC ERROR") && !strcmp(row[5],"T ---- ---- ----"));
  const uint16_t overlap[]={1000,1040,1500}; capture(1,overlap);
  assert(strstr(row[7],"WEAK OR OVERLAP") && !strcmp(row[5],"T 2045 ---- 2545"));
  /* Centre gap of 31 is invalid although the other two thresholds exist. */
  const uint16_t weak[]={1000,1121,1500}; capture(1,weak);
  assert(strstr(row[7],"WEAK OR OVERLAP") && !strcmp(row[5],"T 2045 ---- 2545"));
  n=reads; chord(2); assert(!accepts && reads==n && !strcmp(row[5],"T 2045 ---- 2545"));
  /* Exact 32-count gap is usable. */
  const uint16_t edge[]={1000,1122,1500}; capture(1,edge);
  assert(!strcmp(row[5],"T 2045 1106 2545") && strstr(row[7],"CAL READY"));
  keys(3,100); keys(0,31); assert(accepts==1 && applied.t[1]==1106 && !applied.black_high[1]);
}
int main(void) {
  for (unsigned first=1; first<=2; ++first)
    for (unsigned last=1; last<=2; ++last) sample_and_apply(first,last);
  puts("IR setup: either stagger order, held/bouncing/busy keys, no chord recaptures, exact displayed T/H-L on release PASS");
  reject_and_retry();
  puts("IR setup: missing samples, ADC faults, partial/weak gaps reject all channels; intentional recapture and retry PASS");
}
