#include "j3_rotation_run.h"
#include "elcp_motor/elcp_motor.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
static int starts, stops;
void ELCP_Motor_SetSpeed(ELCP_Motor_ID id, float pct) {assert(id==MOTOR_J3 && pct==40.0f);starts++;}
void ELCP_Motor_Stop(ELCP_Motor_ID id) {assert(id==MOTOR_J3);stops++;}
int main(void) {
  J3Run_Init();assert(J3Run_Status()==J3_RUN_IDLE && !starts && !stops);
  assert(!J3Run_Start(false,100));assert(J3Run_Status()==J3_RUN_NO_TIMER && !starts);
  assert(J3Run_Start(true,100));assert(J3Run_Status()==J3_RUN_ACTIVE && starts==1);
  assert(!J3Run_Start(true,200) && starts==1);
  assert(J3_RUN_TARGET_EVENTS == 26600u);
  J3Run_Update(100u+J3_RUN_TARGET_EVENTS-1u);assert(J3Run_Status()==J3_RUN_ACTIVE && stops==0);
  J3Run_Update(100u+J3_RUN_TARGET_EVENTS);assert(J3Run_Status()==J3_RUN_COMPLETE && stops==1);
  J3Run_Update(100u+J3_RUN_TARGET_EVENTS+1u);J3Run_Abort();assert(stops==1);
  assert(J3Run_Start(true,UINT32_MAX-100u));
  J3Run_Update((uint32_t)(UINT32_MAX-100u+J3_RUN_TARGET_EVENTS-1u));assert(J3Run_Status()==J3_RUN_ACTIVE);
  J3Run_Update((uint32_t)(UINT32_MAX-100u+J3_RUN_TARGET_EVENTS));assert(J3Run_Status()==J3_RUN_COMPLETE && stops==2);
  assert(J3Run_Start(true,0));J3Run_Abort();assert(J3Run_Status()==J3_RUN_ABORTED && stops==3);
  puts("J3 start, target, wrap and abort simulated PASS (no physical stop validation)");
}
