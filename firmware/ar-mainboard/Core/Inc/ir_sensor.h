#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "main.h"
#include "ir_mode.h"
#include <stdint.h>

#define IR_SENSOR_COUNT 3U
#define IR_ADC_TIMEOUT_MS 10U /* per conversion */

typedef struct {
  /* Left PA2, centre PC2, right PC3. Analog: raw 12-bit counts (0..4095). Digital (IR_DIGITAL): 4095 black, 0 white. */
  uint16_t raw[IR_SENSOR_COUNT];
} IR_RawScan;

/* After GPIO init, explicitly set the three IR pins: digital inputs with no pull (IR_DIGITAL), or analog mode
 * (ADC1 then exclusively owned by IR_ReadAll during foreground operation). */
void IR_Sensor_Init(void);
/* Digital: reads the three pins (never fails with a valid out). Analog: sequential blocking, one-channel-at-a-time
 * conversions, no ADC DMA; call from foreground with SysTick enabled. On any failure out is not modified. */
HAL_StatusTypeDef IR_ReadAll(IR_RawScan *out);

#endif
