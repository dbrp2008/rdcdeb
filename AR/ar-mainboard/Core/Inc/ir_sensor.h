#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "main.h"
#include <stdint.h>

#define IR_SENSOR_COUNT 3U
#define IR_ADC_TIMEOUT_MS 10U /* per conversion */

typedef struct {
  /* Left PA2, centre PC2, right PC3, raw 12-bit counts (0..4095). */
  uint16_t raw[IR_SENSOR_COUNT];
} IR_RawScan;

/* After GPIO/ADC initialization, explicitly put the three IR pins in analog
 * mode. ADC1 is exclusively owned by IR_ReadAll during foreground operation. */
void IR_Sensor_Init(void);
/* Sequential blocking, one-channel-at-a-time conversions, no ADC DMA. Call
 * from foreground with SysTick enabled. On any failure out is not modified. */
HAL_StatusTypeDef IR_ReadAll(IR_RawScan *out);

#endif
