#ifndef US1_SENSOR_H
#define US1_SENSOR_H
#include "us2_sensor.h"

/* PA1 trigger / PA0 echo, GPIOs set by MX_GPIO_Init. Shares free-running
 * TIM2 and statuses with US2; call only after US2_Init succeeds. Poll one
 * sensor at a time. Blocking up to ~20 ms when SysTick works. */
US2_Reading US1_Measure(void);
#endif
