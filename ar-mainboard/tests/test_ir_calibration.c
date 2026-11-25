#include "ir_calibration.h"
#include <assert.h>
#include <stdio.h>
static void capture(IR_Calibration *c, IR_Surface surface, uint32_t *t, uint16_t v[3]) {
  assert(IR_CalibrationBegin(c,surface,*t));
  *t+=IR_CAL_SETTLE_MS;
  for (unsigned i=0;i<IR_CAL_CAPTURE_SAMPLES;i++) IR_CalibrationFeed(c,(*t)++,v,true);
}
int main(void) {
  IR_Calibration c; uint32_t t=0; uint16_t white[3]={1000,3000,2000},black[3]={3000,1000,3500};
  IR_CalibrationInit(&c); capture(&c,IR_SURFACE_WHITE,&t,white);capture(&c,IR_SURFACE_BLACK,&t,black);
  for (int i=0;i<3;i++) {assert(c.snapshot.threshold[i].valid);assert(c.snapshot.surface[i]==IR_SURFACE_BLACK);}
  IR_CalibrationFeed(&c,t++,white,true);for (int i=0;i<3;i++) assert(c.snapshot.surface[i]==IR_SURFACE_WHITE);
  uint16_t middle[3]={2000,2000,2750};IR_CalibrationFeed(&c,t++,middle,true);for (int i=0;i<3;i++) assert(c.snapshot.surface[i]==IR_SURFACE_WHITE);
  assert(IR_CalibrationBegin(&c,IR_SURFACE_WHITE,t));for(int i=0;i<3;i++)assert(!c.snapshot.threshold[i].valid);
  IR_CalibrationFeed(&c,t,NULL,false);assert(c.snapshot.capture_failed&&!c.snapshot.frame_valid);
  IR_CalibrationInit(&c);uint16_t bad[3]={1001,2999,1999};capture(&c,IR_SURFACE_WHITE,&t,white);capture(&c,IR_SURFACE_BLACK,&t,bad);for(int i=0;i<3;i++)assert(!c.snapshot.threshold[i].valid);
  puts("Three-channel calibration polarity, hysteresis, capture failure and overlap PASS");
}
