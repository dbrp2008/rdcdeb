#ifndef ADC_TEST_HOST_HAL_H
#define ADC_TEST_HOST_HAL_H
#define __MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { int unused; } ADC_HandleTypeDef;
typedef struct { uint32_t Channel, Rank, SamplingTime; } ADC_ChannelConfTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
extern int gpioa_device, gpioc_device;
#define GPIOA (&gpioa_device)
#define GPIOC (&gpioc_device)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_3 (1U << 3)
#define ADC_CHANNEL_2 2U
#define ADC_CHANNEL_12 12U
#define ADC_CHANNEL_13 13U
#define ADC_SAMPLETIME_84CYCLES 84U
#define GPIO_MODE_ANALOG 3U
#define GPIO_MODE_INPUT 0U
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
GPIO_PinState HAL_GPIO_ReadPin(void *, uint16_t);
#define GPIO_NOPULL 0U
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
void HAL_GPIO_Init(void *, GPIO_InitTypeDef *);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *, ADC_ChannelConfTypeDef *);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *, uint32_t);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *);
#endif
