#include "rotation_estimate.h"
#include "elcp_motor/elcp_motor.h"
#include "main.h"
#include <assert.h>
#include <stdio.h>
static ELCP_Motor_State_t motors[2];
const ELCP_Motor_State_t *ELCP_Motor_GetState(ELCP_Motor_ID id) {assert(id==MOTOR_J1 || id==MOTOR_J3);return &motors[id];}
int main(void) {
  RotationEstimate r = RotationEstimate_J1(0u);
  assert(r.whole == 0 && r.hundredths == 0);
  r=RotationEstimate_J1(1u); assert(r.whole==0 && r.hundredths==0);
  r=RotationEstimate_J1(57u); assert(r.whole==0 && r.hundredths==50);
  r=RotationEstimate_J1(114u); assert(r.whole==1 && r.hundredths==0);
  r=RotationEstimate_J1(171u); assert(r.whole==1 && r.hundredths==50);
  r=RotationEstimate_J1(UINT32_MAX);
  assert(r.whole==UINT32_MAX/114u && r.hundredths==((UINT32_MAX%114u)*100u)/114u);
  r=RotationEstimate_J3(1140u); assert(r.whole==0 && r.hundredths==42);
  r=RotationEstimate_J3(7u*1140u); assert(r.whole==3 && r.hundredths==0);
  r=RotationEstimate_J3(1330u); assert(r.whole==0 && r.hundredths==50);
  r=RotationEstimate_J3(2660u); assert(r.whole==1 && r.hundredths==0);
  r=RotationEstimate_J3(26600u); assert(r.whole==10 && r.hundredths==0);
  r=RotationEstimate_J3(UINT32_MAX);
  assert(r.whole==UINT32_MAX/2660u && r.hundredths==((UINT32_MAX%2660u)*100u)/2660u);
  for (int i=0; i<114; ++i) {RotationEstimate_RecordIRQ(ELCP_J1_FB_Pin);RotationEstimate_RecordIRQ(ELCP_J3_FB_Pin);}
  assert(rotation_j1_events==0 && rotation_j3_events==0);
  motors[MOTOR_J1].current_speed_pct=40.0f;
  for (int i=0; i<114; ++i) RotationEstimate_RecordIRQ(ELCP_J1_FB_Pin);
  for (int i=0; i<57; ++i) RotationEstimate_RecordIRQ(ELCP_J3_FB_Pin);
  assert(rotation_j1_events==114 && rotation_j3_events==0);
  motors[MOTOR_J1].current_speed_pct=0.0f;motors[MOTOR_J3].current_speed_pct=-40.0f;
  for (int i=0; i<57; ++i) RotationEstimate_RecordIRQ(ELCP_J3_FB_Pin);
  RotationEstimate_RecordIRQ(ELCP_J1_FB_Pin);
  assert(rotation_j1_events==114 && rotation_j3_events==57);
  RotationEstimate_RecordIRQ(GPIO_PIN_0);
  assert(rotation_j1_events==114 && rotation_j3_events==57);
  rotation_j3_events=UINT32_MAX;RotationEstimate_RecordIRQ(ELCP_J3_FB_Pin);
  assert(rotation_j3_events==0 && rotation_j1_events==114);
  motors[MOTOR_J3].current_speed_pct=0.0f;RotationEstimate_RecordIRQ(ELCP_J3_FB_Pin);
  assert(rotation_j3_events==0);
  puts("Rotation estimate arithmetic and independent zero-command gating PASS");
}
