/* Digital line IRs (IR_DIGITAL 1): the driver reads PA2/PC2/PC3 as plain inputs (1 = black -> 4095), and the setup
 * page has no capture - it shows the live levels and B1+B2 hands drive the fixed profile. Build twice:
 *   -DPART_SENSOR with mocks_adc (ir_sensor.c), -DPART_SETUP with the board mocks (ir_setup.c). */
#ifdef PART_SENSOR
#include "mocks_adc/host_hal.h"
#include "ir_sensor.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int gpioa_device, gpioc_device;
static unsigned init_a, init_c, level_a, level_c, reads;
void HAL_GPIO_Init(void *p, GPIO_InitTypeDef *g) {
  assert(g->Mode == GPIO_MODE_INPUT && g->Pull == GPIO_NOPULL); /* the side IRs' setup: no pull */
  if (p == GPIOA) init_a = g->Pin; else { assert(p == GPIOC); init_c = g->Pin; }
}
GPIO_PinState HAL_GPIO_ReadPin(void *p, uint16_t pin) {
  reads++;
  if (p == GPIOA) { assert(pin == GPIO_PIN_2); return (level_a & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET; }
  assert(p == GPIOC && (pin == GPIO_PIN_2 || pin == GPIO_PIN_3));
  return (level_c & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
/* the ADC is never touched */
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c) { (void)h; (void)c; assert(0); return HAL_ERROR; }
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *h) { (void)h; assert(0); return HAL_ERROR; }
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *h) { (void)h; assert(0); return HAL_ERROR; }
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *h, uint32_t t) { (void)h; (void)t; assert(0); return HAL_ERROR; }
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h) { (void)h; assert(0); return 0; }
int main(void) {
  IR_Sensor_Init();
  assert(init_a == GPIO_PIN_2 && init_c == (GPIO_PIN_2 | GPIO_PIN_3));
  assert(IR_ReadAll(NULL) == HAL_ERROR);
  for (unsigned m = 0; m < 8u; ++m) { /* bit 0 left PA2, bit 1 centre PC2, bit 2 right PC3 */
    level_a = (m & 1u) ? GPIO_PIN_2 : 0u;
    level_c = ((m & 2u) ? GPIO_PIN_2 : 0u) | ((m & 4u) ? GPIO_PIN_3 : 0u);
    IR_RawScan s = {{1, 2, 3}};
    assert(IR_ReadAll(&s) == HAL_OK);
    for (unsigned i = 0; i < 3u; ++i) assert(s.raw[i] == ((m >> i) & 1u ? IR_DIGITAL_BLACK : 0u));
  }
  assert(reads == 24u);
  /* the drive's 2-scan average ((a + b + 1) / 2) against IR_DIGITAL_T: black only when both scans read black */
  assert((IR_DIGITAL_BLACK + IR_DIGITAL_BLACK + 1u) / 2u > IR_DIGITAL_T);
  assert(!((IR_DIGITAL_BLACK + 0u + 1u) / 2u > IR_DIGITAL_T) && !((0u + 0u + 1u) / 2u > IR_DIGITAL_T));
  puts("Digital line IRs: PA2/PC2/PC3 inputs with no pull, 1 = black read as 4095, no ADC; filtered black = 2 scans PASS");
  return 0;
}
#endif

#ifdef PART_SETUP
#include "ir_setup.h"
#include "ir_mode.h"
#include "ir_sensor.h"
#include "lcd/lcd.h"
#include "main.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
static uint32_t now;
static unsigned buttons, reads;
static uint16_t level[3];
static char row[10][17];
uint32_t HAL_GetTick(void) { return now; }
bool mock_btn(int btn) { return buttons & (btn == BTN1 ? 1u : 2u); }
HAL_StatusTypeDef IR_ReadAll(IR_RawScan *s) { reads++; memcpy(s->raw, level, sizeof level); return HAL_OK; }
void tft_printc(uint8_t x, uint8_t y, const char *s) { assert(x == 0 && y < 10 && strlen(s) <= 16); strcpy(row[y], s); }
void tft_prints(uint8_t x, uint8_t y, const char *fmt, ...) {
  assert(x == 0 && y < 10); va_list ap; va_start(ap, fmt);
  int n = vsnprintf(row[y], sizeof(row[y]), fmt, ap); va_end(ap); assert(n >= 0 && n <= 16);
}
void tft_update(uint32_t x) { (void)x; }
static IR_Profile applied;
static unsigned keys(unsigned mask, unsigned ms) {
  unsigned accepted = 0; buttons = mask;
  while (ms--) { ++now; if (IRSetup_Update(now, &applied)) accepted++; }
  return accepted;
}
int main(void) {
  applied = (IR_Profile){.t = {7, 8, 9}, .black_high = {false, false, false}};
  level[1] = IR_DIGITAL_BLACK; /* centre on the tape */
  IRSetup_Init(now);
  assert(strstr(row[0], "AR SETUP") && !strcmp(row[2], "FRONT L0 C1 R0  ") && !strcmp(row[8], "B1+B2 = DRIVE   "));
  assert(strstr(row[5], "R WHEEL") && strstr(row[5], "--")); /* no counts reported yet */
  level[0] = IR_DIGITAL_BLACK; level[1] = 0; IRSetup_Wheels(1234u, 56u); keys(0, 150);
  assert(!strcmp(row[2], "FRONT L1 C0 R0  ") && reads >= 10u); /* live, refreshed */
  assert(strstr(row[5], "R WHEEL") && strstr(row[5], " 1234") && strstr(row[6], "L WHEEL") && strstr(row[6], " 56"));
  IRSetup_Wheels(4000000000u, 12345678u); keys(0, 150);
  assert(strstr(row[5], "9999999") && strstr(row[6], "9999999") && strlen(row[5]) <= 16); /* clamped, fits */
  assert(!strcmp(row[9], "MOTORS HELD AT 0")); /* no powered check reported (F1_LEGACY build) */
  IRSetup_Spin(0, false, 0u); keys(0, 150); assert(!strcmp(row[9], "HOLD B1=L B2=R  "));
  IRSetup_Spin('L', true, 431u); keys(0, 150); assert(!strcmp(row[9], "L SPIN      431 "));
  IRSetup_Spin('R', false, 12345678u); keys(0, 150); assert(!strcmp(row[9], "R LAST  9999999 "));
  /* B1 or B2 alone: no capture, nothing applied */
  assert(!keys(1, 60) && !keys(0, 200) && !keys(2, 60) && !keys(0, 200));
  assert(applied.t[0] == 7 && !applied.black_high[0]);
  /* B1+B2, released: the fixed digital profile (black = above IR_DIGITAL_T) */
  assert(!keys(3, 250)); assert(strstr(row[7], "RELEASE BOTH"));
  assert(keys(0, 60) == 1u);
  for (unsigned i = 0; i < 3u; ++i) assert(applied.t[i] == IR_DIGITAL_T && applied.black_high[i]);
  puts("Digital IR page: live FRONT levels, wheel counts, no capture, B1/B2 alone do nothing, B1+B2 applies the digital profile PASS");
  return 0;
}
#endif
