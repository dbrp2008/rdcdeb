#ifndef US3_SENSOR_H
#define US3_SENSOR_H
#include "us2_sensor.h"

/* PB9 trigger / PB8 echo. GPIOs are set by MX_GPIO_Init. Shares the
 * free-running TIM2 and status types with US1/US2: call only after US2_Init()
 * succeeds. Do not trigger US2 and US3 simultaneously. Blocking, up to
 * approximately 20 ms when SysTick works. */
US2_Reading US3_Measure(void);
#endif
