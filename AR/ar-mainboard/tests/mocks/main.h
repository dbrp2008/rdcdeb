#ifndef MOCK_MAIN_H
#define MOCK_MAIN_H
#include <stdint.h>
#include <stdbool.h>
typedef enum {HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3} HAL_StatusTypeDef;
typedef enum {GPIO_PIN_RESET=0,GPIO_PIN_SET=1} GPIO_PinState;
typedef struct { int dummy; } GPIO_TypeDef;
typedef struct {uint32_t Pin, Mode, Pull, Speed;} GPIO_InitTypeDef;
typedef struct {uint32_t CFGR;} RCC_Type;
typedef struct {uint32_t PSC, ARR, EGR, SR, CR1, CNT;} TIM_Type;
extern GPIO_TypeDef gpioa,gpiob;
extern RCC_Type rcc;
extern TIM_Type tim2;
#define GPIOA (&gpioa)
#define GPIOB (&gpiob)
#define RCC (&rcc)
#define TIM2 (&tim2)
#define RCC_CFGR_PPRE1_2 0x1000u
#define TIM_EGR_UG 1u
#define TIM_CR1_CEN 1u
#define GPIO_PIN_0 1u
#define GPIO_PIN_1 2u
#define GPIO_PIN_2 4u
#define GPIO_PIN_3 8u
#define GPIO_PIN_8 256u
#define GPIO_PIN_9 512u
#define GPIO_PIN_10 1024u
#define GPIO_PIN_11 2048u
#define GPIO_MODE_OUTPUT_PP 1u
#define GPIO_MODE_INPUT 0u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_FREQ_LOW 0u
#define BTN1 1
#define BTN2 2
#define btn_read(x) mock_btn(x)
bool mock_btn(int);
void HAL_GPIO_Init(GPIO_TypeDef *,GPIO_InitTypeDef *);
void HAL_GPIO_WritePin(GPIO_TypeDef *,uint16_t,GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *,uint16_t);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_GetTick(void);
#define __HAL_RCC_TIM2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM2_FORCE_RESET() ((void)0)
#define __HAL_RCC_TIM2_RELEASE_RESET() ((void)0)
#endif
