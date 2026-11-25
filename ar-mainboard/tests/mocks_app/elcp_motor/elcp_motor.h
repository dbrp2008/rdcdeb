#ifndef MOCK_ELCP_MOTOR_H
#define MOCK_ELCP_MOTOR_H
typedef enum { MOTOR_J1, MOTOR_J3, ELCP_MOTOR_COUNT } ELCP_Motor_ID;
void ELCP_Motor_SetSpeed(ELCP_Motor_ID id, float pct);
void ELCP_Motor_Stop(ELCP_Motor_ID id);
#endif
