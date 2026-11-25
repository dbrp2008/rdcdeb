#ifndef GPIO_TEST_H
#define GPIO_TEST_H
#include <stdint.h>
typedef struct { unsigned port; } GPIO_TypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
extern GPIO_TypeDef ga,gb,gc;
#define GPIOA (&ga)
#define GPIOB (&gb)
#define GPIOC (&gc)
#define GPIO_PIN_0 (1u<<0)
#define GPIO_PIN_1 (1u<<1)
#define GPIO_PIN_2 (1u<<2)
#define GPIO_PIN_3 (1u<<3)
#define GPIO_PIN_4 (1u<<4)
#define GPIO_PIN_5 (1u<<5)
#define GPIO_PIN_6 (1u<<6)
#define GPIO_PIN_7 (1u<<7)
#define GPIO_PIN_8 (1u<<8)
#define GPIO_PIN_9 (1u<<9)
#define GPIO_PIN_10 (1u<<10)
#define GPIO_PIN_11 (1u<<11)
#define GPIO_PIN_14 (1u<<14)
#define GPIO_PIN_15 (1u<<15)
#define GPIO_MODE_INPUT 0u
#define GPIO_MODE_OUTPUT_PP 1u
#define GPIO_MODE_OUTPUT_OD 2u
#define GPIO_MODE_IT_RISING 3u
#define GPIO_NOPULL 0u
#define GPIO_PULLUP 1u
#define GPIO_PULLDOWN 2u
#define GPIO_SPEED_FREQ_LOW 0u
#define GPIO_PIN_RESET 0u
#define GPIO_PIN_SET 1u
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOH_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define EXTI4_IRQn 3
#define EXTI9_5_IRQn 1
#define EXTI15_10_IRQn 2
#define TFT_DC_Pin GPIO_PIN_4
#define TFT_CS_Pin GPIO_PIN_5
#define ELCP_J1_DIR_Pin GPIO_PIN_6
#define ELCP_J3_DIR_Pin GPIO_PIN_9
#define LED_D6_Pin GPIO_PIN_15
#define LED_D7_Pin GPIO_PIN_0
#define LED_D8_Pin GPIO_PIN_1
#define US1_TRIG_Pin GPIO_PIN_1
#define US1_ECHO_Pin GPIO_PIN_0
#define US1_ECHO_GPIO_Port GPIOA
#define US2_ECHO_Pin GPIO_PIN_10
#define US2_ECHO_GPIO_Port GPIOB
#define IR_L2_Pin GPIO_PIN_4
#define IR_R2_Pin GPIO_PIN_6
#define TFT_RES_Pin GPIO_PIN_6
#define TFT_BL_Pin GPIO_PIN_0
#define TFT_BL_GPIO_Port GPIOB
#define US3_TRIG_Pin GPIO_PIN_9
#define US3_ECHO_Pin GPIO_PIN_8
#define US3_ECHO_GPIO_Port GPIOB
#define ELCP_J1_PWM_Pin GPIO_PIN_14
#define ELCP_J3_PWM_Pin GPIO_PIN_7
#define BTN2_Pin GPIO_PIN_4
#define BTN2_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_1
#define BTN1_GPIO_Port GPIOB
#define US2_TRIG_Pin GPIO_PIN_11
#define ELCP_J1_FB_Pin GPIO_PIN_15
#define ELCP_J1_FB_GPIO_Port GPIOB
#define ELCP_J3_FB_Pin GPIO_PIN_8
#define ELCP_J3_FB_GPIO_Port GPIOC
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint32_t, uint32_t);
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
void HAL_NVIC_SetPriority(int, int, int);
void HAL_NVIC_EnableIRQ(int);
void MX_GPIO_Init(void);
#endif
