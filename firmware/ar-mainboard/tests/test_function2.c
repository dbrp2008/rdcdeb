#include "function2.h"
#include "line_drive.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* The digital branch evidence and filtered analog L/C/R are independent. */
static uint32_t now;
static void scan(unsigned ms, bool high, unsigned line) {
  now += ms;
  F2_Update((BranchSample){.at=now, .high=high}, line);
}
static void pulse(unsigned ms, unsigned edge_after, unsigned line) {
  assert(edge_after <= ms);
  uint32_t rise = now + edge_after;
  now += ms;
  F2_Update((BranchSample){.at=now, .first_rise=rise, .rose=true}, line);
}

static void start_and_reach_pivot(unsigned line) {
  F2_Start(false); scan(10,true,line); scan(10,false,line); pulse(80,75,line);
  assert(F2_BranchCount()==2 && !F2_Turning());
  pulse(80,75,line); assert(F2_BranchCount()==3 && F2_Turning());
  F2_Advanced(F2_T_OUT_ADVANCE_MM); /* past the T: the pivot is requested */
}

/* No pre-pivot white (including a scan that crossed the launch boundary)
 * may arm the exit. During the pivot only the centre bit can reacquire. */
static void centre_pickup(void) {
  for (unsigned mask=0; mask<8; ++mask) {
    start_and_reach_pivot(0);
    scan(10,false,0); scan(200,false,0); /* request still pending: wheels not pivoting */
    assert(!strcmp(F2_Status(),"F2 TURN RIGHT"));
    F2_PivotCommanded(); scan(10,false,LINE_CENTRE); scan(75,false,LINE_CENTRE);
    scan(250,false,LINE_CENTRE); assert(F2_Turning() && !strcmp(F2_Status(),"F2 FIND LINE"));
    scan(10,false,LINE_RIGHT); assert(F2_Turning()); /* centre lost, not an all-white floor */
    F2_PivotCommanded(); /* repeated motor writes must not erase the loss */
    scan(10,true,mask);
    assert(F2_Turning() == ((mask & LINE_CENTRE) == 0)); /* including multi-black pickups */
    assert(F2_BranchCount()==3); /* ignore a digital hit even on the pickup scan */
  }
  puts("F2 branch 3: pending scans ignored, held centre cannot finish, filtered white then centre black, side-only never finishes PASS");
}

int main(void) {
  F2_Init();
  assert(F2_BranchCount() == 0 && !F2_Turning() && !F2_Done() && strstr(F2_Status(), "READY"));
  F2_Start(true); scan(100, true, 0); /* held starting mark, even on analog white */
  assert(F2_BranchCount() == 0);
  for (unsigned line=0; line<8; ++line) { scan(10, false, 2); scan(100, false, line); }
  assert(F2_BranchCount() == 0 && !F2_Turning());
  scan(10, true, 0); scan(200, true, 7);
  assert(F2_BranchCount() == 1 && !F2_Turning() && strstr(F2_Status(), "BRANCH 1"));
  scan(1, false, 2); pulse(80, 74, 0); /* late scan must not turn a short gap into 75 ms */
  assert(F2_BranchCount() == 1);
  pulse(80, 75, 2); assert(F2_BranchCount() == 2 && !F2_Turning());
  pulse(80, 75, 2); /* the T: drive F2_T_OUT_ADVANCE_MM past it first, then the right turn (even if only centre is black) */
  assert(F2_BranchCount() == 3 && F2_Turning() && F2_Advancing() && strstr(F2_Status(), "PAST THE T"));
  scan(100, false, 7); F2_PivotAngle(200.0f); assert(F2_Advancing() && !F2_Failed()); /* scans, angle ignored while advancing */
  F2_Advanced(F2_T_OUT_ADVANCE_MM - 1.0f); assert(F2_Advancing());
  F2_Advanced(F2_T_OUT_ADVANCE_MM); assert(!F2_Advancing() && F2_Turning() && strstr(F2_Status(), "TURN RIGHT"));

  F2_PivotCommanded();
  /* Leaving the junction and picking up the line are still based on analog
   * tape; another digital hit on white is not a line pickup or an extra BR. */
  scan(100, false, 7); scan(100, false, 3); assert(strstr(F2_Status(), "TURN RIGHT"));
  scan(1, false, 2); scan(74, false, 0); assert(strstr(F2_Status(), "TURN RIGHT"));
  scan(1, false, 6); /* multi-black again: cancel the incomplete analog gap */
  scan(1, false, 0); scan(74, false, 0); assert(strstr(F2_Status(), "TURN RIGHT"));
  scan(1, false, 0); pulse(10, 5, 0); assert(strstr(F2_Status(), "FIND LINE"));
  scan(1, true, 7); assert(!F2_Turning() && F2_BranchCount() == 3);
  scan(1, false, 7); scan(75, false, 7); scan(1, true, 7);
  assert(F2_BranchCount() == 4 && !F2_Turning());
  scan(1, false, 0); pulse(80, 75, 0);
  assert(F2_BranchCount() == 5 && !F2_Turning());

  /* Wrap once during digital rearm and once during analog clearance. The
   * centre can return early, but cannot end the pivot before the full 75 ms. */
  for (unsigned i=0; i<2; ++i) {
    now = UINT32_MAX - (i ? 280u : 35u);
    F2_Start(false); scan(1, true, 7); scan(1, false, 7); pulse(80, 74, 7);
    assert(F2_BranchCount() == 1 && !F2_Turning());
    pulse(80, 75, 7); assert(F2_BranchCount() == 2 && !F2_Turning());
    pulse(80, 75, 7); assert(F2_BranchCount() == 3 && F2_Turning());
    F2_Advanced(F2_T_OUT_ADVANCE_MM);
    F2_PivotCommanded();
    scan(1, false, LINE_RIGHT); const uint32_t gap=now;
    scan(74, false, LINE_CENTRE); assert(F2_Turning() && (!i || now < gap));
    scan(1, false, LINE_CENTRE); assert(!F2_Turning());
  }

  start_and_reach_pivot(0); F2_Stop("STOP BTN2");
  scan(100, false, 0); scan(100, true, 2);
  assert(F2_Done() && !F2_Turning() && F2_BranchCount() == 3 && strstr(F2_Status(), "STOP BTN2"));
  centre_pickup();
  puts("F2: only digital crossings trigger branch three; separate analog gap/pickup, once per run, stop and tick wrap PASS");
}
