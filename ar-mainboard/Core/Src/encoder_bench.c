#include "enc_capture.h"
#include "calibration_app.h" /* CubeMX main's link-time application entry points */
#include "encoder_monitor.h"
#include "lcd/lcd.h"
#include "main.h"

#define REDRAW_MS 100u
#define DEBOUNCE_MS 30u
static uint32_t last_draw, button_changed;
static bool raw_down, press_armed, recal_available;

static void hold_pwm_pin_low(GPIO_TypeDef *port, uint16_t pin) {
  /* Preload ODR low before leaving the timer alternate function. */
  GPIO_InitTypeDef output = {.Pin = pin, .Mode = GPIO_MODE_OUTPUT_PP,
                             .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_FREQ_LOW};
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  HAL_GPIO_Init(port, &output);
}

void CalibrationApp_Init(void) {
  hold_pwm_pin_low(ELCP_J3_PWM_GPIO_Port, ELCP_J3_PWM_Pin);
  hold_pwm_pin_low(ELCP_J1_PWM_GPIO_Port, ELCP_J1_PWM_Pin);
  /* MX_GPIO_Init already set both DIR pins low. No button drives a motor. */
  tft_init(PIN_ON_TOP, BLACK, WHITE, CYAN, DARK_GREY);
  EncoderMonitor_Init(); /* start with fresh observations after TFT startup */
  EncCapture_Start(); /* FB pulses arrive by timer capture (filtered), not EXTI; PWM pins stay GPIO-low */
  recal_available = false;
  raw_down = btn_read(BTN1);
  press_armed = !raw_down; /* ignore a button held through reset */
  button_changed = HAL_GetTick();
  last_draw = button_changed - REDRAW_MS;
}

static bool calibration_press(uint32_t now) {
  const bool down = btn_read(BTN1);
  if (down != raw_down) { raw_down = down; button_changed = now; }
  /* No queued re-arm from a press that began before any wheel was ready. */
  if (down && !recal_available) press_armed = false;
  if ((uint32_t)(now - button_changed) < DEBOUNCE_MS) return false;
  if (!down) { press_armed = true; return false; }
  if (!press_armed) return false;
  press_armed = false; /* one event until a stable release, even if capture is busy */
  return true;
}

static void draw_field(uint8_t row, const char *label, int64_t value, bool show_sign, bool valid) {
  tft_printc(0, row, label);
  if (!valid) { tft_printc(6, row, "--"); return; }
  /* A sign and ten digits use all eleven columns after the five-letter label. */
  uint8_t x = show_sign && (value <= -1000000000 || value >= 1000000000) ? 5u : 6u;
  if (show_sign && value > 0) tft_printc(x++, row, "+");
  tft_printl(x, row, value);
}
static uint8_t digits(uint32_t n) {
  uint8_t len = 1;
  while (n >= 10u) { n /= 10u; ++len; }
  return len;
}
static void draw_profile(uint8_t row, const char *id, EncoderWheel w, bool show_dz) {
  tft_printc(0, row, id);
  if (!w.idle_pps) { tft_printc(3, row, "WAIT STABLE"); return; }
  const uint8_t bg_end = 5u + digits(w.idle_pps);
  if (bg_end + 4u + digits(w.deadzone_pps) <= 16u) {
    tft_printc(3, row, "BG"); tft_printl(5, row, w.idle_pps);
    tft_printc(bg_end, row, " +/-"); tft_printl(bg_end + 4u, row, w.deadzone_pps);
  } else { /* Alternating full-width values for unusually large rates. */
    tft_printc(3, row, show_dz ? "DZ" : "BG");
    tft_printl(6, row, show_dz ? w.deadzone_pps : w.idle_pps);
  }
}
static void draw_wheel(uint8_t row, const char *id, EncoderWheel w, bool show_dz) {
  draw_profile(row, id, w, show_dz);
  draw_field(row + 1u, "RAW/s", w.pps,       false, true);
  draw_field(row + 2u, "NET/s", w.net_pps,   true,  w.idle_pps != 0u);
  if (w.rise_pending) tft_printc(8, row + 2u, "HOLD"); /* always follows a numeric zero */
  draw_field(row + 3u, "COUNT", w.net_edges, true,  true);
}

void CalibrationApp_Update(void) {
  const uint32_t now = HAL_GetTick();
  /* Poll BTN1 every pass, not just at 10 Hz when the TFT is redrawn. */
  const bool started = calibration_press(now) && EncoderMonitor_StartIdleSample();
  if (!started && (uint32_t)(now - last_draw) < REDRAW_MS) {
    EncoderMonitor_Poll(now); /* signal timing is independent of TFT refresh */
    return;
  }
  last_draw = now;
  const EncoderReading r = EncoderMonitor_Read();
  recal_available = r.mode != ENCODER_SETTLING;
  tft_printc(0, 0, "ENCODER NO DRIVE");
  const char *status = "AUTO: KEEP STILL";
  if (r.mode == ENCODER_MEASURING) status = "READY BTN1:RECAL";
  else if (r.mode == ENCODER_PARTIAL) status = r.left_j3.idle_pps ? "WAIT J1: STILL" : "WAIT J3: STILL";
  tft_printc(0, 1, status);
  const bool show_dz = ((now / 2000u) & 1u) != 0u;
  draw_wheel(2, "J3", r.left_j3, show_dz);
  draw_wheel(6, "J1", r.right_j1, show_dz);
  tft_update(0);
}
