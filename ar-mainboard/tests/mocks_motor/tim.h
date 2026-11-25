#ifndef MOTOR_TEST_TIM_H
#define MOTOR_TEST_TIM_H
#include "main.h"
typedef struct {int id;uint32_t CNT;} TIM_TypeDef;
typedef struct {TIM_TypeDef *Instance; uint32_t arr;} TIM_HandleTypeDef;
extern TIM_TypeDef tim2, tim8, tim12;
extern TIM_HandleTypeDef htim8, htim12;
#define TIM2 (&tim2)
#define TIM8 (&tim8)
#define TIM12 (&tim12)
#define TIM_CHANNEL_1 1u
#define TIM_CHANNEL_2 2u
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->arr)
#define __HAL_TIM_SET_COMPARE(h,c,v) mock_set_compare(h,c,v)
void mock_set_compare(TIM_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
#endif
