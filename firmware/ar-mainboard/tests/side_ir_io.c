/* Mock only the GPIO pins in the setup/board UI replays. The separate vector
 * replay links the real motor callback too. No encoder edges in these UI runs. */
#include "main.h"
#include <assert.h>
GPIO_TypeDef gpiob;
uint32_t mock_primask;
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
  assert(port == GPIOB);
  if (pin == (IR_L2_Pin | IR_R2_Pin)) assert(mock_primask);
  else assert(pin == IR_L2_Pin || pin == IR_R2_Pin);
  return (gpiob.dummy & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void ELCP_EncoderEXTI_Callback(uint16_t pin) { (void)pin; assert(0 && "unexpected encoder IRQ in UI replay"); }
