#include "side_ir.h"
#include "main.h"
#include "lcd/lcd.h"

/* BSS is ready before MX_GPIO_Init enables the IRQs. A single IRQ owns each
 * naturally aligned counter; foreground only reads them, never resets them. */
static volatile uint32_t left_rises, right_rises;
/* Both side IRQs have the same preemption priority. They never run mission
 * logic; retain the earliest rise until the foreground takes its sample. */
static volatile bool pending_rise;
static volatile uint32_t first_rise_at;

/* Original motor dispatcher, renamed only in its own CMake source rule. */
void ELCP_EncoderEXTI_Callback(uint16_t pin);

/* Called by HAL after acknowledging this specific pending line. The shared
 * EXTI9_5 vector dispatches PB6 and PC8 separately; don't count vector entries. */
#if defined(DWT) && defined(CoreDebug)
/* Encoder edge filter (the reference ELCP driver is kept byte-identical, so it lives here, in front of it): an FB edge
 * closer than FB_MIN_DT_US to that wheel's previous one is ringing / a double pulse, not a new pulse. On our AR this took
 * the noisy wheel from ~2600 to ~115 counts per turn (the same as the clean one; 115 confirmed under power), so both wheels can measure distance.
 * Measured with the core cycle counter; host builds (no DWT) forward every edge. 0 = off. */
#ifndef FB_MIN_DT_US
#define FB_MIN_DT_US 250u
#endif
static uint32_t fb_last_cyc[2];
static bool fb_seen[2], fb_cyc_on;
static bool fb_too_soon(unsigned idx) {
  if (!fb_cyc_on) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    fb_cyc_on = true;
  }
  const uint32_t now = DWT->CYCCNT;
  const uint32_t dt_us = (now - fb_last_cyc[idx]) / (SystemCoreClock / 1000000u);
  const bool soon = fb_seen[idx] && FB_MIN_DT_US != 0u && dt_us < FB_MIN_DT_US;
  if (!soon) { fb_last_cyc[idx] = now; fb_seen[idx] = true; } /* a dropped edge does not restart the spacing */
  return soon;
}
#else
static bool fb_too_soon(unsigned idx) { (void)idx; return false; }
#endif

static volatile uint32_t fb_raw[2]; /* [0] J1 right, [1] J3 left; one IRQ writes each, foreground only reads */
uint32_t SideIR_EncoderRaw(bool right) { return fb_raw[right ? 0u : 1u]; }
/* Signed odometry count: every filtered pulse, +/- by the wheel's commanded direction, none at 0 % (no direction).
 * The reference driver's pulse_count follows the same rule but also drops a pulse when the pin is already low by the time
 * its handler reads it, so odometry counts here instead (~115 per wheel turn under power, measured on both wheels).
 * The drive app supplies the commanded speed; the weak default (no motors) gives 0. */
static volatile int32_t fb_signed[2];
__attribute__((weak)) float SideIR_WheelCommandPct(bool right) { (void)right; return 0.0f; }
int32_t SideIR_EncoderSigned(bool right) { return fb_signed[right ? 0u : 1u]; }
/* F4 only (SideIR_CountCoast): at 0 % a pulse counts in the wheel's last commanded direction - the roll-on after a stop.
 * F4 stops before every leg and dead-reckons its turns; uncounted coast there = turns it never sees. Off for F1-F3. */
static volatile bool coast_on;
static volatile int8_t last_dir[2];
void SideIR_CountCoast(bool on) { coast_on = on; }
static void count_signed(unsigned idx, bool right) {
  const float pct = SideIR_WheelCommandPct(right);
  if (pct > 0.0f) { ++fb_signed[idx]; last_dir[idx] = 1; }
  else if (pct < 0.0f) { --fb_signed[idx]; last_dir[idx] = -1; }
  else if (coast_on) fb_signed[idx] += last_dir[idx];
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  if (pin == IR_L2_Pin) ++left_rises;
  else if (pin == IR_R2_Pin) ++right_rises;
  else {
    if ((pin == ELCP_J1_FB_Pin && fb_too_soon(0u)) || (pin == ELCP_J3_FB_Pin && fb_too_soon(1u))) return; /* bounce */
    /* Raw count of every filtered FB pulse, at ANY speed. The reference driver's pulse_count drops pulses while the
     * commanded speed is 0 (no direction to give them), so a wheel turned by hand never shows there: this does. */
    if (pin == ELCP_J1_FB_Pin) { ++fb_raw[0]; count_signed(0u, true); }
    else if (pin == ELCP_J3_FB_Pin) { ++fb_raw[1]; count_signed(1u, false); }
    ELCP_EncoderEXTI_Callback(pin);
    return;
  }
  if (!pending_rise) {
    first_rise_at = HAL_GetTick();
    pending_rise = true;
  }
}

BranchSample SideIR_TakeBranchSample(void) {
  /* One GPIOB IDR read for the pair. Do not lose an IRQ between copying and
   * clearing the mailbox; restore the incoming interrupt mask, not always ON. */
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const BranchSample s = {
    .at = HAL_GetTick(), .first_rise = first_rise_at, .rose = pending_rise,
    .high = HAL_GPIO_ReadPin(IR_L2_GPIO_Port, IR_L2_Pin | IR_R2_Pin) == GPIO_PIN_SET
  };
  pending_rise = false;
  __set_PRIMASK(mask);
  return s;
}

SideIR_Reading SideIR_Read(void) {
  return (SideIR_Reading){
    .left_high = HAL_GPIO_ReadPin(IR_L2_GPIO_Port, IR_L2_Pin) == GPIO_PIN_SET,
    .right_high = HAL_GPIO_ReadPin(IR_R2_GPIO_Port, IR_R2_Pin) == GPIO_PIN_SET,
    .left_rises = left_rises, .right_rises = right_rises
  };
}

void SideIR_DrawRow(uint8_t row) {
  const SideIR_Reading r = SideIR_Read();
  tft_prints(0, row, "L2:%u/%02u R2:%u/%02u ", (unsigned)r.left_high, (unsigned)(r.left_rises % 100u),
             (unsigned)r.right_high, (unsigned)(r.right_rises % 100u));
}
