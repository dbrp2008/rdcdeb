#ifndef IR_SENSOR_H
#define IR_SENSOR_H
#include "main.h"
#define IR_SENSOR_COUNT 3U
typedef struct { uint16_t raw[IR_SENSOR_COUNT]; } IR_RawScan;
void IR_Sensor_Init(void);
HAL_StatusTypeDef IR_ReadAll(IR_RawScan *);
#endif
