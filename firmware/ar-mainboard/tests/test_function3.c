#include "function3.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/* L=1, C=2, R=4. No HAL, no F1/F2, and no timing advances from cached scans. */
static void scan(uint32_t t, unsigned raw, unsigned filtered, F3_Command expected) {
  F3_Scan(t,raw,filtered); assert(F3_CommandAt(t)==expected);
}
int main(void) {
  const F3_Command on_start[8]={F3_LOST_LINE,F3_STEER_LEFT,F3_STRAIGHT,F3_STRAIGHT,
                                F3_STEER_RIGHT,F3_PIVOT_RIGHT,F3_PIVOT_RIGHT,F3_PIVOT_RIGHT};
  for(unsigned raw=0;raw<8;raw++) for(unsigned filtered=0;filtered<8;filtered++) {
    unsigned agreed=raw & filtered;
    F3_Command expect=!raw ? F3_LOST_LINE : !agreed ? F3_WAIT_TAPE : on_start[agreed];
    F3_Reset(); assert(F3_CommandAt(0)==F3_WAIT_TAPE); scan(0,raw,filtered,expect);
    assert(F3_RightTurns()==(unsigned)(expect==F3_PIVOT_RIGHT));
  }
  F3_Reset(); scan(0,3,3,F3_STRAIGHT); /* left+centre is not a right turn */
  scan(10,4,7,F3_STEER_RIGHT); /* filter residue is not a junction */
  scan(20,7,3,F3_STRAIGHT);   /* raw right not yet confirmed */
  scan(30,7,7,F3_PIVOT_RIGHT); assert(F3_RightTurns()==1);
  scan(90,6,6,F3_PIVOT_RIGHT); scan(200,2,2,F3_PIVOT_RIGHT); /* old centre, no exit yet */
  scan(210,4,6,F3_PIVOT_RIGHT); /* right-only must also be separated in the filter */
  scan(220,4,4,F3_STEER_RIGHT); scan(230,2,2,F3_STRAIGHT);
  scan(249,6,6,F3_PIVOT_RIGHT); assert(F3_RightTurns()==1); /* chatter before 20 ms */
  scan(260,4,4,F3_STEER_RIGHT); scan(270,2,2,F3_STRAIGHT); scan(290,2,2,F3_STRAIGHT);
  scan(300,6,6,F3_PIVOT_RIGHT); assert(F3_RightTurns()==2);
  scan(320,4,4,F3_STEER_RIGHT); scan(330,2,2,F3_STRAIGHT); scan(350,2,2,F3_STRAIGHT);
  scan(360,5,5,F3_PIVOT_RIGHT); assert(F3_RightTurns()==3);
  scan(370,0,5,F3_LOST_LINE); scan(380,6,6,F3_LOST_LINE); assert(F3_RightTurns()==3);
  puts("F3 mask truth table, late right branch, repeated distinct choices, edge/centre confirmation and latched loss PASS");

  /* Deadlines at the boundary (including tick wrap). A blocking poll is not
   * a new IR observation and cannot confirm centre or extend a time limit. */
  const uint32_t t=UINT32_MAX-19u;
  F3_Reset(); scan(t,6,6,F3_PIVOT_RIGHT);
  scan(t+2480u,4,4,F3_STEER_RIGHT); scan(t+2490u,2,2,F3_STRAIGHT);
  assert(F3_CommandAt(t+2499u)==F3_STRAIGHT);
  scan(t+2500u,2,2,F3_TURN_TIMEOUT); scan(t+2600u,7,7,F3_TURN_TIMEOUT);
  F3_Reset(); scan(t,6,6,F3_PIVOT_RIGHT); scan(t+10u,4,4,F3_STEER_RIGHT);
  scan(t+15u,2,2,F3_STRAIGHT); assert(F3_CommandAt(t+200u)==F3_STRAIGHT);
  scan(t+200u,6,6,F3_PIVOT_RIGHT); assert(F3_RightTurns()==1); /* no fresh confirming scan */
  F3_Reset(); scan(t,4,2,F3_WAIT_TAPE); scan(t+49u,4,4,F3_STEER_RIGHT);
  scan(t+60u,2,4,F3_WAIT_TAPE); assert(F3_CommandAt(t+109u)==F3_WAIT_TAPE);
  scan(t+110u,2,2,F3_UNSTABLE_LINE); scan(t+120u,2,2,F3_UNSTABLE_LINE);
  puts("F3 filter disagreement is motionless, bounded deadlines latch at 50/2500 ms across tick wrap PASS");
}
