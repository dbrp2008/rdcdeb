#ifndef TR_TEST_HOST_HAL_H
#define TR_TEST_HOST_HAL_H
#include <stdint.h>
#define __MAIN_H
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
typedef struct { int unused; } CAN_HandleTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
#endif
