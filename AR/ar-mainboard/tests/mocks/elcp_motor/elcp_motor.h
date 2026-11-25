#ifndef MOCK_MOTOR_H
#define MOCK_MOTOR_H
#include <stdint.h>
#include <stdbool.h>
extern volatile uint32_t elcp_j1_fb_edges, elcp_j3_fb_edges;
extern volatile uint32_t elcp_j1_fb_min_dt_us, elcp_j3_fb_min_dt_us, elcp_j1_fb_rejected, elcp_j3_fb_rejected;
void ELCP_FB_ResetStats(void);
typedef enum {MOTOR_J1, MOTOR_J3} ELCP_Motor_ID;
void ELCP_Motor_Init(ELCP_Motor_ID);
bool ELCP_Motor_IsInitialized(ELCP_Motor_ID);
void ELCP_Motor_SetSpeed(ELCP_Motor_ID, float);
void ELCP_Motor_Stop(ELCP_Motor_ID);
#endif
