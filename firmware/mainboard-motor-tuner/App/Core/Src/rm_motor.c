#include "rm_motor.h"
#include "motor_tuner_config.h"
#include "stm32f4xx_hal_can.h"
#include <stddef.h>

static int16_t current_command_raw[NUM_OF_RM_MOTORS];
static volatile RM_Feedback feedback[NUM_OF_RM_MOTORS];

static CAN_HandleTypeDef *bus_for_motor(RM_Motor motor)
{
  if ((motor == (RM_Motor)(TUNER_TR_MOTOR_1_CAN_ID - 1U)) ||
      (motor == (RM_Motor)(TUNER_TR_MOTOR_2_CAN_ID - 1U))) return &hcan1;
  if (motor == (RM_Motor)(TUNER_TR_MOTOR_3_CAN_ID - 1U)) return &hcan2;
  return NULL;
}

static int16_t float_current_to_raw(float current_a)
{
  if (current_a > RM_CURRENT_SOFT_MAX) current_a = RM_CURRENT_SOFT_MAX;
  if (current_a < -RM_CURRENT_SOFT_MAX) current_a = -RM_CURRENT_SOFT_MAX;
  return (int16_t)(current_a * (float)RM_CURRENT_RAW_MAX / RM_CURRENT_MAX);
}

static void pack_group(uint8_t data[8], const int16_t *commands)
{
  for (unsigned i = 0U; i < 4U; ++i) {
    const uint16_t value = (uint16_t)commands[i];
    data[i * 2U] = (uint8_t)(value >> 8);
    data[i * 2U + 1U] = (uint8_t)value;
  }
}

static HAL_StatusTypeDef send_group(CAN_HandleTypeDef *hcan, uint32_t id,
                                    const int16_t *commands)
{
  uint8_t data[8];
  uint32_t mailbox;
  CAN_TxHeaderTypeDef header = {0};

  pack_group(data, commands);
  header.StdId = id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8U;
  header.TransmitGlobalTime = DISABLE;
  return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

HAL_StatusTypeDef RM_CAN_Init(CAN_HandleTypeDef *hcan)
{
  if ((hcan == NULL) || ((hcan->Instance != CAN1) && (hcan->Instance != CAN2))) {
    return HAL_ERROR;
  }

  CAN_FilterTypeDef filter = {0};
  filter.FilterIdHigh = 0U;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0U;
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterActivation = ENABLE;
  filter.FilterBank = (hcan->Instance == CAN2) ? 14U : 0U;
  filter.SlaveStartFilterBank = 14U;

  HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(hcan, &filter);
  if (status != HAL_OK) return status;
  return HAL_CAN_Start(hcan);
}

static void consume_feedback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) break;
    if ((header.IDE != CAN_ID_STD) ||
        (header.StdId < RM_CAN_RX_ID_BASE) ||
        (header.StdId >= RM_CAN_RX_ID_BASE + NUM_OF_RM_MOTORS) ||
        (header.DLC != 8U)) continue;

    const RM_Motor motor = (RM_Motor)(header.StdId - RM_CAN_RX_ID_BASE);
    /* Ignore a matching feedback ID arriving from the other physical bus. */
    if (bus_for_motor(motor) != hcan) continue;
    const uint16_t encoder = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    const int16_t velocity = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    const int16_t raw_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    feedback[motor].encoder = encoder;
    feedback[motor].velocity_rpm = velocity;
    feedback[motor].raw_current = raw_current;
    feedback[motor].current_a = (float)raw_current * RM_CURRENT_MAX /
                                (float)RM_CURRENT_RAW_MAX;
    feedback[motor].temperature_c = data[6];
    feedback[motor].last_rx_tick = HAL_GetTick();
    feedback[motor].received = 1U;
  }
}

HAL_StatusTypeDef RM_CAN_Service(CAN_HandleTypeDef *hcan)
{
  if ((hcan == NULL) || ((hcan->Instance != CAN1) && (hcan->Instance != CAN2))) {
    return HAL_ERROR;
  }

  consume_feedback(hcan);

  /* RM group commands are broadcast to IDs 1-4. Build a bus-specific payload
   * so each motor's nonzero command is transmitted only on its assigned bus. */
  int16_t bus_group1[4] = {0};
  int16_t bus_group2[4] = {0};
  uint8_t has_group2_channel = 0U;
  for (unsigned i = 0U; i < 4U; ++i) {
    if (bus_for_motor((RM_Motor)i) == hcan) {
      bus_group1[i] = current_command_raw[i];
    }
    if (bus_for_motor((RM_Motor)(i + 4U)) == hcan) {
      bus_group2[i] = current_command_raw[i + 4U];
      has_group2_channel = 1U;
    }
  }

  HAL_StatusTypeDef status = send_group(hcan, RM_CAN_TX_GRP1_ID, bus_group1);
  if ((status != HAL_OK) || (has_group2_channel == 0U)) return status;
  return send_group(hcan, RM_CAN_TX_GRP2_ID, bus_group2);
}

HAL_StatusTypeDef RM_CAN_ServiceMotor(RM_Motor motor)
{
  CAN_HandleTypeDef *hcan = bus_for_motor(motor);
  return (hcan == NULL) ? HAL_ERROR : RM_CAN_Service(hcan);
}

HAL_StatusTypeDef RM_CAN_ServiceAll(void)
{
  const HAL_StatusTypeDef can1_status = RM_CAN_Service(&hcan1);
  const HAL_StatusTypeDef can2_status = RM_CAN_Service(&hcan2);
  return ((can1_status == HAL_OK) && (can2_status == HAL_OK)) ? HAL_OK : HAL_ERROR;
}

void RM_SetCurrentA(RM_Motor motor, float current_a)
{
  if ((unsigned)motor >= (unsigned)NUM_OF_RM_MOTORS) return;
  current_command_raw[motor] = float_current_to_raw(current_a);
}

float RM_GetCurrentSetpointA(RM_Motor motor)
{
  if ((unsigned)motor >= (unsigned)NUM_OF_RM_MOTORS) return 0.0f;
  return (float)current_command_raw[motor] * RM_CURRENT_MAX /
         (float)RM_CURRENT_RAW_MAX;
}

const volatile RM_Feedback *RM_GetFeedback(RM_Motor motor)
{
  if ((unsigned)motor >= (unsigned)NUM_OF_RM_MOTORS) return NULL;
  return &feedback[motor];
}
