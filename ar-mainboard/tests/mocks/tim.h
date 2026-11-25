#ifndef MOCK_TIM_H
#define MOCK_TIM_H
#include "main.h"
typedef struct {TIM_Type *Instance; struct {uint32_t Prescaler,Period;} Init;} TIM_HandleTypeDef;
extern TIM_HandleTypeDef htim2;
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *);
#endif
