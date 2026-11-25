#ifndef MOTOR_UI_MOCK_H
#define MOTOR_UI_MOCK_H
#include <stdint.h>
typedef enum { MOTOR_J1, MOTOR_J3, ELCP_MOTOR_COUNT } ELCP_Motor_ID;
typedef struct { volatile float current_speed_pct; volatile int32_t pulse_count; volatile float pps; volatile float rpm; } ELCP_Motor_State_t;
const ELCP_Motor_State_t *ELCP_Motor_GetState(ELCP_Motor_ID id);
void ELCP_Motor_UpdatePPS(ELCP_Motor_ID id);
void ELCP_Motor_SetSpeed(ELCP_Motor_ID id, float speed);
void ELCP_Motor_Stop(ELCP_Motor_ID id);
#endif
