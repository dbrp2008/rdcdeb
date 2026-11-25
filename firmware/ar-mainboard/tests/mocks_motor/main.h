#ifndef MOTOR_TEST_MAIN_H
#define MOTOR_TEST_MAIN_H
#include <stdint.h>
#include "../mock_primask.h"
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { int id; } GPIO_TypeDef;
extern GPIO_TypeDef gpiob, gpioc;
#define GPIO_PIN_4 (1u<<4)
#define GPIO_PIN_6 (1u<<6)
#define IR_L2_Pin GPIO_PIN_4
#define IR_R2_Pin GPIO_PIN_6
#define IR_L2_GPIO_Port (&gpiob)
#define IR_R2_GPIO_Port (&gpiob)
#define GPIO_PIN_9 (1u<<9)
#define GPIO_PIN_8 (1u<<8)
#define GPIO_PIN_15 (1u<<15)
#define GPIO_PIN_SET 1u
#define ELCP_J1_FB_Pin GPIO_PIN_15
#define ELCP_J1_FB_GPIO_Port (&gpiob)
#define ELCP_J3_FB_Pin GPIO_PIN_8
#define ELCP_J3_FB_GPIO_Port (&gpioc)
#define GPIO_PIN_RESET 0u
#define BTN1_Pin (1u<<1)
#define BTN1_GPIO_Port (&gpiob)
#define btn_read(btn) (HAL_GPIO_ReadPin(btn##_GPIO_Port, btn##_Pin) == GPIO_PIN_RESET)
#define ELCP_J1_DIR_GPIO_Port (&gpioc)
#define ELCP_J1_DIR_Pin GPIO_PIN_6
#define ELCP_J3_DIR_GPIO_Port (&gpioc)
#define ELCP_J3_DIR_Pin GPIO_PIN_9
#define ELCP_J1_PWM_Pin (1u<<14)
#define ELCP_J1_PWM_GPIO_Port (&gpiob)
#define ELCP_J3_PWM_Pin (1u<<7)
#define ELCP_J3_PWM_GPIO_Port (&gpioc)
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
#define GPIO_MODE_OUTPUT_PP 1u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_FREQ_LOW 0u
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
typedef uint32_t GPIO_PinState;
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
uint32_t HAL_GetTick(void);
/* Interfaces called by the real CubeMX interrupt vectors. */
typedef struct {int dummy;} DMA_HandleTypeDef, SPI_HandleTypeDef;
void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_GPIO_EXTI_IRQHandler(uint16_t);
void HAL_IncTick(void);
void HAL_SPI_IRQHandler(SPI_HandleTypeDef *);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *);
#endif
