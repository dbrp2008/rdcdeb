#ifndef RM_MOTOR_H
#define RM_MOTOR_H

#include "rdc_can_bus.h"
#include <stdint.h>

#define RM_CURRENT_MAX       20.0f
#define RM_CURRENT_RAW_MAX   16384
#define RM_CURRENT_SOFT_MAX   5.0f
#define RM_CAN_TX_GRP1_ID    0x200U
#define RM_CAN_TX_GRP2_ID    0x1FFU
#define RM_CAN_RX_ID_BASE    0x201U

typedef enum {
  RM_MOTOR_1 = 0,
  RM_MOTOR_2,
  RM_MOTOR_3,
  RM_MOTOR_4,
  RM_MOTOR_5,
  RM_MOTOR_6,
  RM_MOTOR_7,
  RM_MOTOR_8,
  NUM_OF_RM_MOTORS
} RM_Motor;

typedef struct {
  float current_a;
  uint16_t encoder;
  int16_t velocity_rpm;
  int16_t raw_current;
  uint8_t temperature_c;
  uint8_t received;
  uint32_t last_rx_tick;
} RM_Feedback;

/* Call after MX_CAN1_Init()/MX_CAN2_Init(); uses shared CAN1/CAN2 filter banks. */
HAL_StatusTypeDef RM_CAN_Init(CAN_HandleTypeDef *hcan);
/* Service one bus, draining feedback and sending only that bus's channel group. */
HAL_StatusTypeDef RM_CAN_Service(CAN_HandleTypeDef *hcan);
/* Service the selected motor's assigned bus, or both buses while idle. */
HAL_StatusTypeDef RM_CAN_ServiceMotor(RM_Motor motor);
HAL_StatusTypeDef RM_CAN_ServiceAll(void);
void RM_SetCurrentA(RM_Motor motor, float current_a);
float RM_GetCurrentSetpointA(RM_Motor motor);
const volatile RM_Feedback *RM_GetFeedback(RM_Motor motor);

#endif
