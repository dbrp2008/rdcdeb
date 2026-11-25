/* Odometry counts (side_ir.c): every FB pulse is counted raw, and signed by the wheel's commanded direction at that
 * moment (none at 0 %). Host build: no DWT, so the 250 us bounce filter forwards every edge. */
#include "main.h"
#include "side_ir.h"
#include <assert.h>
#include <stdio.h>
GPIO_TypeDef gpiob;
uint32_t mock_primask;
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) { (void)port; (void)pin; return GPIO_PIN_RESET; }
void tft_prints(uint8_t x, uint8_t y, const char *f, ...) { (void)x; (void)y; (void)f; }
uint32_t HAL_GetTick(void) { return 0u; }
static unsigned forwarded;
void ELCP_EncoderEXTI_Callback(uint16_t pin) { (void)pin; ++forwarded; }
static float cmd_r, cmd_l;
float SideIR_WheelCommandPct(bool right) { return right ? cmd_r : cmd_l; } /* overrides the weak default */
static void pulses(uint16_t pin, int n) { while (n--) HAL_GPIO_EXTI_Callback(pin); }
int main(void) {
  cmd_r = 20.0f; cmd_l = -15.0f;
  pulses(ELCP_J1_FB_Pin, 657); pulses(ELCP_J3_FB_Pin, 768);   /* one wheel turn each, measured figures */
  assert(SideIR_EncoderSigned(true) == 657 && SideIR_EncoderSigned(false) == -768);
  cmd_r = -11.25f; cmd_l = 0.0f;                               /* right reverses; left stopped: no direction */
  pulses(ELCP_J1_FB_Pin, 100); pulses(ELCP_J3_FB_Pin, 50);
  assert(SideIR_EncoderSigned(true) == 557 && SideIR_EncoderSigned(false) == -768);
  assert(SideIR_EncoderRaw(true) == 757 && SideIR_EncoderRaw(false) == 818); /* raw counts every pulse, any speed */
  assert(forwarded == 1575u);                                  /* the reference driver still sees every pulse */
  puts("Signed odometry counts: every pulse, +/- by commanded direction, none at 0 %, raw unchanged PASS");
  return 0;
}
