#include "ir_sensor.h"
#include "adc.h"
#include <stddef.h>

/* CubeMX may omit an unranked ADC pin from its generated ADC MSP GPIO setup.
 * Explicitly make all three selected pins analog after peripheral init. */
void IR_Sensor_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  gpio.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  HAL_GPIO_Init(GPIOC, &gpio);
}

HAL_StatusTypeDef IR_ReadAll(IR_RawScan *out)
{
  if (out == NULL) return HAL_ERROR;
  static const uint32_t channels[IR_SENSOR_COUNT] = {
    ADC_CHANNEL_2, ADC_CHANNEL_12, ADC_CHANNEL_13
  };
  IR_RawScan scan;
  ADC_ChannelConfTypeDef config = {0};
  config.Rank = 1u;
  config.SamplingTime = ADC_SAMPLETIME_84CYCLES;

  /* ADC1 is exclusively owned here. One conversion is triggered and read
   * before selecting the next channel: no automatic multi-rank scan. */
  for (unsigned i = 0u; i < IR_SENSOR_COUNT; ++i) {
    config.Channel = channels[i];
    HAL_StatusTypeDef status = HAL_ADC_ConfigChannel(&hadc1, &config);
    if (status != HAL_OK) return status;
    status = HAL_ADC_Start(&hadc1);
    if (status != HAL_OK) {
      (void)HAL_ADC_Stop(&hadc1);
      return status;
    }
    status = HAL_ADC_PollForConversion(&hadc1, IR_ADC_TIMEOUT_MS);
    if (status == HAL_OK) scan.raw[i] = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_StatusTypeDef stop_status = HAL_ADC_Stop(&hadc1);
    if (status != HAL_OK) return status;
    if (stop_status != HAL_OK) return stop_status;
  }
  *out = scan;
  return HAL_OK;
}
