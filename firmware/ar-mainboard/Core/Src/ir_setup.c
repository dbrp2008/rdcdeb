#include "ir_setup.h"
#include "button_gesture.h"
#include "ir_calibration.h" /* shared capture limits, not the hysteresis classifier */
#include "ir_sensor.h"
#include "side_ir.h"
#include "lcd/lcd.h"
#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ir_mode.h"

#define SAMPLE_MS 10u
#define SCREEN_MS 100u

#if IR_DIGITAL
/* Digital line IRs: nothing to capture (each module's knob is its threshold). The page shows the live L/C/R and side
 * levels so each knob can be set (1 on black, 0 on the floor); B1+B2 hands the fixed digital profile to drive. */
static ButtonGesture gesture;
static uint32_t last_sample, last_screen;
static IR_RawScan live;
static bool live_valid, read_error, wheels_known;
static uint32_t wheel_r, wheel_l;

void IRSetup_Wheels(uint32_t right, uint32_t left) { wheels_known = true; wheel_r = right; wheel_l = left; }
static bool spin_known, spin_live;
static char spin_wheel;
static uint32_t spin_pulses;
void IRSetup_Spin(char wheel, bool live, uint32_t pulses) { spin_known = true; spin_wheel = wheel; spin_live = live; spin_pulses = pulses; }
static unsigned clamp_count(uint32_t v) { return v > 9999999u ? 9999999u : (unsigned)v; }

static void sample(uint32_t now) {
  if (live_valid && (uint32_t)(now - last_sample) < SAMPLE_MS) return;
  last_sample = now;
  IR_RawScan scan;
  if (IR_ReadAll(&scan) != HAL_OK) { read_error = true; live_valid = false; return; }
  read_error = false;
  live = scan;
  live_valid = true;
}
static char level(unsigned i) { return !live_valid ? '-' : live.raw[i] > IR_DIGITAL_T ? '1' : '0'; }
/* 16 x 10. Set each IR's knob: 1 on tape, 0 on the floor. Turn each wheel by hand: its count must change. */
static void draw(void) {
  const SideIR_Reading side = SideIR_Read();
  tft_printc(0, 0, "AR SETUP        ");
  tft_printc(0, 1, "1=TAPE  0=FLOOR ");
  tft_prints(0, 2, "FRONT L%c C%c R%c  ", level(0), level(1), level(2));
  tft_prints(0, 3, "SIDE  L%u     R%u ", (unsigned)side.left_high, (unsigned)side.right_high);
  tft_printc(0, 4, "WHEELS (BY HAND)");
  if (wheels_known) {
    /* raw pulses, always counting UP whichever way the wheel turns. Plain %u: the TFT's printf has no 'l' / '+' */
    tft_prints(0, 5, "R WHEEL %7u ", clamp_count(wheel_r));
    tft_prints(0, 6, "L WHEEL %7u ", clamp_count(wheel_l));
  } else { tft_printc(0, 5, "R WHEEL      -- "); tft_printc(0, 6, "L WHEEL      -- "); }
  if (read_error) tft_printc(0, 7, "IR READ ERROR   ");
  else if (gesture.qualified == KEY_BOTH) tft_printc(0, 7, "RELEASE BOTH    ");
  else tft_printc(0, 7, "READY           ");
  tft_printc(0, 8, "B1+B2 = DRIVE   ");
  if (!spin_known) tft_printc(0, 9, "MOTORS HELD AT 0");
  else if (spin_wheel != 'L' && spin_wheel != 'R') tft_printc(0, 9, "HOLD B1=L B2=R  ");
  else tft_prints(0, 9, "%c %s  %7u ", spin_wheel, spin_live ? "SPIN" : "LAST", clamp_count(spin_pulses));
  tft_update(0u);
}
void IRSetup_Init(uint32_t now) {
  gesture = (ButtonGesture){.changed = now};
  live_valid = false; read_error = false; wheels_known = false;
  last_screen = now;
  sample(now);
  draw();
}
bool IRSetup_Update(uint32_t now, IR_Profile *applied) {
  const unsigned keys = (btn_read(BTN1) ? KEY_B1 : 0u) | (btn_read(BTN2) ? KEY_B2 : 0u);
  if (ButtonGesture_Release(&gesture, now, keys, GESTURES_ALL) == KEY_BOTH) { /* B1 or B2 alone: nothing to do */
    for (unsigned i = 0; i < 3u; ++i) { applied->t[i] = IR_DIGITAL_T; applied->black_high[i] = true; }
    return true;
  }
  sample(now);
  now = HAL_GetTick();
  if ((uint32_t)(now - last_screen) >= SCREEN_MS) {
    last_screen = now;
    draw();
  }
  return false;
}
#else /* analog: capture white and black, threshold between them */
void IRSetup_Wheels(uint32_t right, uint32_t left) { (void)right; (void)left; }
void IRSetup_Spin(char wheel, bool live, uint32_t pulses) { (void)wheel; (void)live; (void)pulses; }

typedef enum { NONE, CAP_WHITE, CAP_BLACK } Capture;
typedef struct { bool valid; uint16_t avg[3], min[3], max[3]; } Result;
static Result white, black;
static IR_Profile measured; /* one snapshot for both the T / BLACK rows and the drive handoff */
static bool threshold_valid[3];
static ButtonGesture gesture;
static Capture capturing;
static bool capture_error;
static uint32_t sum[3], last_sample, started, last_screen;
static uint16_t minimum[3], maximum[3];
static unsigned count;

static unsigned released_gesture(uint32_t now) {
  const unsigned keys = (btn_read(BTN1) ? KEY_B1 : 0u) | (btn_read(BTN2) ? KEY_B2 : 0u);
  return ButtonGesture_Release(&gesture, now, keys, capturing == NONE ? GESTURES_ALL : GESTURES_OFF);
}
static void begin_capture(Capture surface, uint32_t now) {
  if (surface == CAP_WHITE) white.valid = false; else black.valid = false;
  capturing = surface;
  capture_error = false;
  started = now;
  count = 0;
  memset(sum, 0, sizeof(sum));
}
static void sample(uint32_t now) {
  if (capturing == NONE || (uint32_t)(now - started) < IR_CAL_SETTLE_MS ||
      (count && (uint32_t)(now - last_sample) < SAMPLE_MS)) return;
  last_sample = now;
  IR_RawScan scan;
  if (IR_ReadAll(&scan) != HAL_OK) {
    capturing = NONE;
    capture_error = true;
    return;
  }
  for (unsigned i = 0; i < 3u; i++) {
    if (scan.raw[i] > 4095u) { capturing = NONE; capture_error = true; return; }
  }
  for (unsigned i = 0; i < 3u; i++) {
    uint16_t v = scan.raw[i];
    sum[i] += v;
    if (count == 0u || v < minimum[i]) minimum[i] = v;
    if (count == 0u || v > maximum[i]) maximum[i] = v;
  }
  if (++count == IR_CAL_CAPTURE_SAMPLES) {
    Result *result = capturing == CAP_WHITE ? &white : &black;
    for (unsigned i = 0; i < 3u; i++) {
      result->avg[i] = (uint16_t)((sum[i] + IR_CAL_CAPTURE_SAMPLES / 2u) / IR_CAL_CAPTURE_SAMPLES);
      result->min[i] = minimum[i]; result->max[i] = maximum[i];
    }
    result->valid = true;
    capturing = NONE;
  }
}
static bool threshold(unsigned i, uint16_t *out, bool *black_high) {
  if (!white.valid || !black.valid) return false;
  bool high = black.min[i] > white.max[i];
  unsigned lower = high ? white.max[i] : black.max[i];
  unsigned upper = high ? black.min[i] : white.min[i];
  if (upper < lower + IR_CAL_MIN_GAP) return false;
  /* Midpoint of the gap, not the means: stays outside both sampled ranges. */
  *out = (uint16_t)((lower + upper) / 2u);
  *black_high = high;
  return true;
}
static void update_thresholds(void) {
  for (unsigned i = 0; i < 3u; ++i) threshold_valid[i] = threshold(i, &measured.t[i], &measured.black_high[i]);
}
static bool ready(void) { return threshold_valid[0] && threshold_valid[1] && threshold_valid[2]; }
static void row_values(unsigned row, char label, const uint16_t values[3], const bool valid[3]) {
  char fields[3][5], line[17];
  for (unsigned i = 0; i < 3u; i++) {
    if (valid[i]) (void)snprintf(fields[i], sizeof(fields[i]), "%4u", (unsigned)values[i] % 10000u);
    else (void)snprintf(fields[i], sizeof(fields[i]), "----");
  }
  (void)snprintf(line, sizeof(line), "%c %s %s %s", label, fields[0], fields[1], fields[2]);
  tft_printc(0, row, line);
}
static void draw(void) {
  bool w[3], b[3]; char p[3];
  for (unsigned i = 0; i < 3u; i++) {
    p[i] = threshold_valid[i] ? (measured.black_high[i] ? 'H' : 'L') : '?';
    w[i] = white.valid; b[i] = black.valid;
  }
  tft_printc(0, 0, "IR CAL 100 SAMP ");
  tft_printc(0, 1, "B1:W B2:B  L/C/R");
  SideIR_DrawRow(2); /* raw digital levels, independent of the three ADC thresholds */
  row_values(3, 'W', white.avg, w);
  row_values(4, 'B', black.avg, b);
  row_values(5, 'T', measured.t, threshold_valid);
  tft_prints(0, 6, "BLACK %c %c %c     ", p[0], p[1], p[2]);
  if (capturing != NONE) tft_prints(0, 7, "CAP %c %3u/100   ", capturing == CAP_WHITE ? 'W' : 'B', count);
  else if (capture_error) tft_printc(0, 7, "ADC ERROR RETRY ");
  else if (!white.valid || !black.valid) tft_printc(0, 7, "NEED W AND B    ");
  else if (!ready()) tft_printc(0, 7, "WEAK OR OVERLAP ");
  else if (gesture.qualified == KEY_BOTH) tft_printc(0, 7, "RELEASE BOTH    ");
  else tft_printc(0, 7, "CAL READY       ");
  tft_printc(0, 8, ready() ? "B1+B2: APPLY T  " : capturing != NONE ? "KEEP STILL      " : "TAP THEN RELEASE");
  tft_printc(0, 9, "MOTOR POWER OFF ");
  tft_update(0u);
}
void IRSetup_Init(uint32_t now) {
  memset(&white, 0, sizeof(white)); memset(&black, 0, sizeof(black));
  gesture = (ButtonGesture){.changed = now};
  capturing = NONE; capture_error = false; count = 0;
  last_screen = now;
  update_thresholds();
  draw();
}
bool IRSetup_Update(uint32_t now, IR_Profile *applied) {
  const Capture before = capturing;
  switch (released_gesture(now)) {
    case KEY_B1: begin_capture(CAP_WHITE, now); break;
    case KEY_B2: begin_capture(CAP_BLACK, now); break;
    case KEY_BOTH:
      if (ready()) { *applied = measured; return true; } /* copy all six displayed values, or none */
      break;
    default: break;
  }
  sample(now);
  now = HAL_GetTick();
  /* Capture start, failure and completion change the snapshot. Show it before
   * polling another gesture; don't wait for the periodic progress refresh. */
  if (before != capturing) update_thresholds();
  if (before != capturing || (uint32_t)(now - last_screen) >= SCREEN_MS) {
    last_screen = now;
    draw();
  }
  return false;
}
#endif /* IR_DIGITAL */
