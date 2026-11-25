/* Host test of the interrupt-driven ultrasonic core: edge sequencing, width, 16-bit wrap, timeouts, late edges. */
#include <assert.h>
#include <stdio.h>
#include "us_capture_core.h"
int main(void){
  US2_Reading r;
  USC_Reset(); assert(!USC_Take(US_FC,&r)); assert(!USC_Waiting(US_FC));
  /* normal: 1000 us pulse = 171 mm */
  USC_Arm(US_FC,100); assert(USC_Waiting(US_FC));
  assert(USC_OnCapture(US_FC,5000,0xFFFFFFFFu));        /* rising -> next edge falling */
  assert(!USC_Poll(US_FC,105,18));                       /* not finished yet */
  assert(!USC_OnCapture(US_FC,6000,0xFFFFFFFFu));       /* falling -> next edge rising */
  assert(USC_Poll(US_FC,106,18)); assert(USC_Take(US_FC,&r));
  assert(r.status==US2_OK&&r.pulse_us==1000&&r.raw_mm>=170&&r.raw_mm<=172); assert(!USC_Take(US_FC,&r));
  /* 16-bit timer wrap: rise at 65000, fall at 464 (=1000 us) */
  USC_Arm(US_BR,0); assert(USC_OnCapture(US_BR,65000,0xFFFFu)); assert(!USC_OnCapture(US_BR,464,0xFFFFu));
  assert(USC_Poll(US_BR,1,18)); assert(USC_Take(US_BR,&r)); assert(r.status==US2_OK&&r.pulse_us==1000);
  /* 32-bit wrap */
  USC_Arm(US_FR,0); assert(USC_OnCapture(US_FR,0xFFFFFF00u,0xFFFFFFFFu)); assert(!USC_OnCapture(US_FR,0x2E8u,0xFFFFFFFFu));
  assert(USC_Poll(US_FR,1,18)); assert(USC_Take(US_FR,&r)); assert(r.pulse_us==1000);
  /* no echo at all */
  USC_Arm(US_FC,200); assert(!USC_Poll(US_FC,210,18)); assert(USC_Poll(US_FC,218,18));
  assert(USC_Take(US_FC,&r)&&r.status==US2_NO_RISE);
  /* rise but no fall */
  USC_Arm(US_FC,300); USC_OnCapture(US_FC,10,0xFFFFFFFFu); assert(USC_Poll(US_FC,320,18));
  assert(USC_Take(US_FC,&r)&&r.status==US2_NO_FALL);
  /* a late edge after the timeout is ignored and does not corrupt the next shot */
  assert(!USC_OnCapture(US_FC,50,0xFFFFFFFFu)); assert(!USC_Waiting(US_FC));
  /* too-short pulse */
  USC_Arm(US_FC,400); USC_OnCapture(US_FC,0,0xFFFFFFFFu); USC_OnCapture(US_FC,50,0xFFFFFFFFu);
  assert(USC_Poll(US_FC,401,18)); assert(USC_Take(US_FC,&r)&&r.status==US2_SHORT_PULSE);
  /* stuck-high echo reported without arming */
  USC_Fail(US_FR,US2_ECHO_BUSY); assert(USC_Take(US_FR,&r)&&r.status==US2_ECHO_BUSY); assert(!USC_Waiting(US_FR));
  /* sensors are independent */
  USC_Arm(US_FC,0); USC_Arm(US_FR,0); USC_OnCapture(US_FR,0,0xFFFFFFFFu); assert(USC_Waiting(US_FC)&&USC_Waiting(US_FR));
  puts("US capture core PASS"); return 0;
}
