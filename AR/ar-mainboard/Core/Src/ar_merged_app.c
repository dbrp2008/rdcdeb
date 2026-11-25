#include "main.h"
#include "ar_merged_app.h"
#include "ar_nav.h"
#include "us_capture.h"
#include "ir_sensor.h"
#include "ir_calibration.h"
#include "encoder_calibration.h"
#include "elcp_motor/elcp_motor.h"
#include "lcd/lcd.h"
#include "main.h"
#include <stddef.h>

#ifndef AR_APP_DEFAULT_KEY
#define AR_APP_DEFAULT_KEY AR_KEY_ALL /* hardcoded encoder route; B2 on the run screen cycles A, B, MID, TOP, RGT, TOP2, ALL, TOP1, RGT1, TRN4, STR, F20 */
#endif
#define DEBOUNCE_MS 30u
#define HOLD_BOTH_MS 1000u
#define US2_MAX_AGE_MS 200u
#define SCREEN_MS 100u
#define IR_FRAME_MAX_AGE_MS 500u

typedef enum { SCREEN_IR, SCREEN_US2, SCREEN_RUN, SCREEN_ENCODER, SCREEN_USALL, SCREEN_WHEELS, SCREEN_COUNT } Screen;
typedef struct {
  unsigned sample, stable, gesture;
  uint32_t changed, both_since;
  bool hold_done;
} Buttons;
static Screen screen;
static Buttons buttons;
static AR_Nav nav;
static AR_Cmd motor_cmd;
static bool motors_initialized, have_us, ignore_buttons_until_release;
/* Wheel test screen: B1 held runs J3 forward, B2 held runs J1 forward (one at a time); the result shows after release. */
static int wt_running; /* 0 none, 1 = J3, 2 = J1 */
static uint32_t wt_start_edges, wt_start_ms, wt_edges[3], wt_ms[3];
static uint32_t us_valid_ms, us_any_ms, us_ok_mm, us_ok_ms; /* last good distance (mm) and when */ /* us_any_ms: last time ANY centre-US result (even an error) was published */
static const char *run_message = "READY";
static US2_Reading us2, us_fr, us_br;
static bool us2_ready;
static int32_t offset_mm;
static uint32_t last_screen;
static IR_Calibration ir_cal;
static uint32_t run_l0, run_r0; /* wheel edges when the last run started (the run screen shows the edges / mm since then) */

/* Individual button actions occur on release; hold-both changes screens. */
static unsigned poll_buttons(uint32_t now) {
  unsigned s = (btn_read(BTN1) ? 1u : 0u) | (btn_read(BTN2) ? 2u : 0u);
  if (s != buttons.sample) { buttons.sample = s; buttons.changed = now; }
  if (s != buttons.stable && (uint32_t)(now - buttons.changed) >= DEBOUNCE_MS) {
    buttons.stable = s;
    if (s) {
      buttons.gesture |= s;
      if (s == 3u) buttons.both_since = now;
    } else {
      unsigned gesture = buttons.gesture;
      bool held = buttons.hold_done;
      buttons.gesture = 0u;
      buttons.hold_done = false;
      if (!held && (gesture == 1u || gesture == 2u)) return gesture;
    }
  }
  if (buttons.stable == 3u && !buttons.hold_done &&
      (uint32_t)(now - buttons.both_since) >= HOLD_BOTH_MS) {
    buttons.hold_done = true;
    return 3u;
  }
  return 0u;
}
static bool get_line(uint32_t now, AR_Line *line) {
  const IR_CalibrationSnapshot *s = &ir_cal.snapshot;
  if (!s->frame_valid || s->capturing != IR_SURFACE_UNKNOWN ||
      (uint32_t)(now - s->frame_tick_ms) > IR_FRAME_MAX_AGE_MS) return false;
  for (unsigned i = 0u; i < IR_CAL_SENSOR_COUNT; ++i)
    if (!s->threshold[i].valid || s->surface[i] == IR_SURFACE_UNKNOWN) return false;
  *line = (AR_Line){s->surface[0] == IR_SURFACE_BLACK,
                    s->surface[1] == IR_SURFACE_BLACK,
                    s->surface[2] == IR_SURFACE_BLACK};
  return true;
}
static void draw(uint32_t now, AR_Line line, bool line_valid) {
  const IR_CalibrationSnapshot *ir = &ir_cal.snapshot;
  if (screen == SCREEN_IR) {
    tft_printc(0, 0, "IR ANALOG TEST  ");
    if (ir->frame_valid) {
      tft_prints(0, 1, "L PA2 : %04u   ", (unsigned)ir->raw[0]);
      tft_prints(0, 2, "C PC2 : %04u   ", (unsigned)ir->raw[1]);
      tft_prints(0, 3, "R PC3 : %04u   ", (unsigned)ir->raw[2]);
    } else {
      tft_printc(0, 1, "L PA2 : ----    ");
      tft_printc(0, 2, "C PC2 : ----    ");
      tft_printc(0, 3, "R PC3 : ----    ");
    }
    if (ir->capturing != IR_SURFACE_UNKNOWN)
      tft_prints(0, 4, "CAPTURE %3u/100 ", ir->capture_count);
    else if (ir->capture_failed) tft_printc(0, 4, "ADC CAPTURE ERR ");
    else if (!ir->frame_valid) tft_printc(0, 4, "ADC READ ERROR  ");
    else if (line_valid) tft_prints(0, 4, "BLACK %u %u %u     ", line.l, line.c, line.r);
    else tft_printc(0, 4, "CAL IR FIRST    ");
    tft_prints(0, 5, "WHITE:%u BLACK:%u ", ir->have_white, ir->have_black);
    tft_printc(0, 6, "B1 CAP WHITE    ");
    tft_printc(0, 7, "B2 CAP BLACK    ");
  } else if (screen == SCREEN_US2) {
    tft_printc(0, 0, "US2 TEST        ");
    tft_printc(0, 1, "TRIG PA1 ECHO PA0");
    if (us2.status == US2_OK) {
      int32_t mm = (int32_t)us2.raw_mm + offset_mm;
      if (mm < 0) mm = 0;
      tft_prints(0, 2, "US2 %5u mm    ", (unsigned)mm);
    } else tft_printc(0, 2, "US2 ---- mm     ");
    tft_prints(0, 3, "%-16s", US2_StatusText(us2.status));
    tft_prints(0, 4, "Offset %c%03u mm ", offset_mm < 0 ? '-' : '+',
               (unsigned)(offset_mm < 0 ? -offset_mm : offset_mm));
    tft_printc(0, 5, "B1 -5  B2 +5    ");
    tft_printc(0, 6, "                ");
    tft_printc(0, 7, "                ");
  } else if (screen == SCREEN_RUN) {
    tft_prints(0, 0, "NAV RUN KEY %-4s", AR_Nav_KeyText(&nav));
    tft_prints(0, 1, "%-16s", run_message);
    if (!AR_Nav_NeedsIR(&nav)) /* encoder route: most the AR rolled after a stop (edges) and the longest settle (ms) */
      tft_prints(0, 2, "ROLL%3u WAIT%3u ", (unsigned)nav.coast_max, (unsigned)nav.settle_max_ms);
    else if (line_valid) tft_prints(0, 2, "IR L C R: %u %u %u ", line.l, line.c, line.r);
    else tft_printc(0, 2, "IR L C R: ? ? ? ");
    if (us2.status == US2_OK) tft_prints(0, 3, "US2 %5u mm    ", (unsigned)((int32_t)us2.raw_mm + offset_mm > 0 ? (int32_t)us2.raw_mm + offset_mm : 0));
    else tft_prints(0, 3, "US2 %-12s", US2_StatusText(us2.status));
    tft_prints(0, 4, "CMD L%c%03u R%c%03u", motor_cmd.left < 0 ? '-' : '+', (unsigned)(motor_cmd.left < 0 ? -motor_cmd.left : motor_cmd.left), motor_cmd.right < 0 ? '-' : '+', (unsigned)(motor_cmd.right < 0 ? -motor_cmd.right : motor_cmd.right));
    { /* this run: left / right edges and the distance they mean (mean of both wheels) */
      const uint32_t el = elcp_j3_fb_edges - run_l0, er = elcp_j1_fb_edges - run_r0;
      const uint32_t mm = (uint32_t)((float)(el + er) * 0.5f * 10.0f / AR_EDGES_PER_CM + 0.5f);
      tft_prints(0, 5, "%4u/%4u %4umm", (unsigned)(el % 10000u), (unsigned)(er % 10000u), (unsigned)(mm % 10000u));
    }
    tft_printc(0, 6, "B1 START / STOP ");
    tft_printc(0, 7, "B2 SWITCH KEY   ");
  } else if (screen == SCREEN_WHEELS) {
    tft_printc(0, 0, "WHEELS OFF FLOOR");
    tft_printc(0, 1, "B1 HOLD: J3 FWD ");
    tft_printc(0, 2, "B2 HOLD: J1 FWD ");
    for (int w = 1; w <= 2; w++) {
      const unsigned rpm = wt_ms[w] ? (unsigned)((unsigned)((float)wt_edges[w] * 60000.0f / (AR_FB_EDGES_PER_REV * (float)wt_ms[w]))) : 0u;
      tft_prints(0, (uint8_t)(2 + w), "J%d E%5u R%4u   ", w == 1 ? 3 : 1, (unsigned)wt_edges[w], rpm);
    }
    tft_prints(0, 5, "MS %5u/%5u   ", (unsigned)wt_ms[1], (unsigned)wt_ms[2]);
    tft_prints(0, 6, "RJ %5u/%5u   ", (unsigned)elcp_j3_fb_rejected, (unsigned)elcp_j1_fb_rejected);
    tft_prints(0, 7, "MIN%5u/%5uus", (unsigned)(elcp_j3_fb_min_dt_us > 99999u ? 99999u : elcp_j3_fb_min_dt_us), (unsigned)(elcp_j1_fb_min_dt_us > 99999u ? 99999u : elcp_j1_fb_min_dt_us));
  } else if (screen == SCREEN_USALL) {
    US_Debug d;
    US_DebugSnapshot(&d);
    tft_printc(0, 0, "ALL US TEST     ");
    if (us2.status == US2_OK) tft_prints(0, 1, "FC %5u mm    ", (unsigned)us2.raw_mm);
    else tft_prints(0, 1, "FC %-13s", US2_StatusText(us2.status));
    if (us_fr.status == US2_OK) tft_prints(0, 2, "FR %5u mm    ", (unsigned)us_fr.raw_mm);
    else tft_prints(0, 2, "FR %-13s", US2_StatusText(us_fr.status));
    if (us_br.status == US2_OK) tft_prints(0, 3, "BR %5u mm    ", (unsigned)us_br.raw_mm);
    else tft_prints(0, 3, "BR %-13s", US2_StatusText(us_br.status));
    tft_prints(0, 4, "I%u S%u C%u H%u%u%u", (unsigned)US_InitCode(), (unsigned)US_ShotCount(), (unsigned)US_IrqCount(), (unsigned)(US_EchoSeenMask() & 1u), (unsigned)((US_EchoSeenMask() >> 1) & 1u), (unsigned)((US_EchoSeenMask() >> 2) & 1u));
    tft_prints(0, 5, "AF%X%X%X CM%02X%02X   ", (unsigned)d.afr_pa0, (unsigned)d.afr_pb10, (unsigned)d.afr_pb8, (unsigned)(d.ccmr1 & 0xFFu), (unsigned)(d.ccmr2 & 0xFFu));
    tft_prints(0, 6, "T4 S%02X E%03X D%02X  ", (unsigned)(d.sr4 & 0xFFu), (unsigned)(d.ccer4 & 0xFFFu), (unsigned)(d.dier4 & 0xFFu));
    tft_prints(0, 7, "DI%03X C%u N%u M%u%u%u", (unsigned)d.dier2, (unsigned)(d.cr1_2 & 1u), (unsigned)d.nvic_tim2, (unsigned)d.pa0_mode, (unsigned)d.pb10_mode, (unsigned)d.pb8_mode);
  } else {
    tft_printc(0, 0, "ENCODER COUNTS  ");
    tft_prints(0, 1, "J1:%010u   ", (unsigned)EncoderCalibration_J1(elcp_j1_fb_edges));
    tft_prints(0, 2, "J3:%010u   ", (unsigned)EncoderCalibration_J3(elcp_j3_fb_edges));
    tft_printc(0, 3, "RAW EDGES ONLY  ");
    tft_printc(0, 4, "B1: RESET       ");
    tft_printc(0, 5, "RAW EDGES ONLY  ");
    tft_printc(0, 6, "                ");
    tft_printc(0, 7, "                ");
  }
  (void)now;
  tft_printc(0, 8, "HOLD BOTH: NEXT ");
  tft_printc(0, 9, "POWER CUTOFF!   ");
  tft_update(0u);
}
static bool navigation_active(void) {
  return nav.phase != AR_PH_IDLE && nav.phase != AR_PH_DONE && nav.phase != AR_PH_FAULT;
}

/* Zero PWM is only a software stop request, not a power disconnect. */
static void request_stop(const char *reason) {
  if (motors_initialized) {
    ELCP_Motor_Stop(MOTOR_J3);
    ELCP_Motor_Stop(MOTOR_J1);
  }
  motor_cmd = (AR_Cmd){0, 0};
  AR_Nav_Abort(&nav);
  run_message = reason;
}

/* Key A / B (line following) need a valid reading all the time. The hardcoded route uses the US only to stop at a key:
 * "no echo" (nothing in range ahead, or the sensor faces away while reversing) is normal there, and "too close" means we
 * are at the stone. It only requires that the driver keeps publishing results. */
static bool us_strict(void) { return nav.key < AR_KEY_MID; }
static bool us_ready_for_run(uint32_t now) {
  if (!us_strict()) return us2_ready && (uint32_t)(now - us_any_ms) <= US2_MAX_AGE_MS;
  return us2_ready && have_us && us2.status == US2_OK &&
         (uint32_t)(now - us_valid_ms) <= US2_MAX_AGE_MS;
}

void AR_MergedApp_Init(void) {
  /* Drive both motor PWM pins at 0 % duty straight away. Until ELCP_Motor_Init runs, the PWM pins are not driven and the
   * motor drivers can read them as a speed command (wheels spun at power-up). Zero PWM is still not a power cutoff. */
  ELCP_Motor_Init(MOTOR_J1);
  ELCP_Motor_Init(MOTOR_J3);
  motors_initialized = ELCP_Motor_IsInitialized(MOTOR_J1) && ELCP_Motor_IsInitialized(MOTOR_J3);
  uint32_t now = HAL_GetTick();
  screen = SCREEN_IR;
  AR_Nav_Init(&nav);
  AR_Nav_SetKey(&nav, AR_APP_DEFAULT_KEY);
  IR_Sensor_Init();
  IR_CalibrationInit(&ir_cal);
  us2_ready = US_Init();
  us2.status = US2_TIMER_ERROR;
  last_screen = now - SCREEN_MS;
  EncoderCalibration_Init(now, false, elcp_j1_fb_edges, elcp_j3_fb_edges);
}
void AR_MergedApp_Update(void) {
  uint32_t now = HAL_GetTick();
  unsigned event = poll_buttons(now);
  /* A press, not release, requests stop; suppress its release so it cannot
   * accidentally start another run. This is not a hardware emergency stop. */
  if (navigation_active() && btn_read(BTN1)) {
    request_stop("USER STOP");
    ignore_buttons_until_release = true;
  }
  if (ignore_buttons_until_release) {
    event = 0u;
    if (!btn_read(BTN1) && !btn_read(BTN2) && buttons.stable == 0u) {
      buttons.gesture = 0u;
      ignore_buttons_until_release = false;
    }
  }
  if (event == 3u && !navigation_active()) {
    request_stop("READY");
    screen = (Screen)((screen + 1) % SCREEN_COUNT);
    last_screen = now - SCREEN_MS;
  } else if (screen == SCREEN_IR) {
    if (event == 1u) (void)IR_CalibrationBegin(&ir_cal, IR_SURFACE_WHITE, now);
    if (event == 2u) (void)IR_CalibrationBegin(&ir_cal, IR_SURFACE_BLACK, now);
  } else if (screen == SCREEN_US2) {
    if (event == 1u && offset_mm > -100) offset_mm -= 5;
    if (event == 2u && offset_mm < 100) offset_mm += 5;
  } else if (screen == SCREEN_RUN && !navigation_active()) {
    if (event == 1u) {
      AR_Line initial = {false, false, false};
      if (AR_Nav_NeedsIR(&nav) && !get_line(now, &initial)) run_message = "CHECK IR CAL/ADC";
      else if (AR_Nav_NeedsIR(&nav) && !initial.c) run_message = "CENTRE ON LINE";
      else if (!us_ready_for_run(now)) run_message = "CHECK US2";
      else {
        /* Draw this notice before starting the countdown or enabling motors.
         * Do not update the TFT during a run: its DMA wait can block forever. */
        run_message = "STARTING RUN";
        draw(now, initial, true);
        now = HAL_GetTick();
        if ((!AR_Nav_NeedsIR(&nav) || (get_line(now, &initial) && initial.c)) && us_ready_for_run(now))
        {
          run_l0 = elcp_j3_fb_edges;
          run_r0 = elcp_j1_fb_edges;
          AR_Nav_Start(&nav, now);
        }
        else run_message = "SENSORS STALE";
      }
    } else if (event == 2u) {
      AR_Nav_SetKey(&nav, (AR_Key)((nav.key + 1) % AR_KEY_COUNT));
      run_message = "READY";
    }
  }
  if (screen == SCREEN_ENCODER)
    EncoderCalibration_Update(now, btn_read(BTN1), elcp_j1_fb_edges, elcp_j3_fb_edges);
  bool fresh = false;
  if (us2_ready) {
    /* Ultrasonics are read by timer capture interrupts; only the centre one during a run. */
    US_SetEnabled(screen == SCREEN_USALL && !navigation_active() ? 7u : 1u);
    US_Update(now);
    US2_Reading r;
    if (US_Take(US_FC, &r)) {
      us2 = r;
      fresh = true;
      us_any_ms = now;
      if (us2.status == US2_OK) { us_ok_mm = us2.raw_mm + (uint32_t)(offset_mm > 0 ? offset_mm : 0); us_ok_ms = now; }
      if (us2.status == US2_OK) { have_us = true; us_valid_ms = now; }
      else have_us = false;
    }
    if (US_Take(US_FR, &r)) us_fr = r;
    if (US_Take(US_BR, &r)) us_br = r;
  }
  if (screen == SCREEN_WHEELS && !navigation_active()) {
    const bool b1 = btn_read(BTN1), b2 = btn_read(BTN2);
    const int want = (b1 && !b2) ? 1 : ((b2 && !b1) ? 2 : 0);
    if (want != wt_running) {
      if (wt_running) { /* stop and record the finished run */
        if (motors_initialized) { ELCP_Motor_Stop(MOTOR_J3); ELCP_Motor_Stop(MOTOR_J1); }
        wt_edges[wt_running] = (wt_running == 1 ? elcp_j3_fb_edges : elcp_j1_fb_edges) - wt_start_edges;
        wt_ms[wt_running] = now - wt_start_ms;
        motor_cmd = (AR_Cmd){0, 0};
      }
      if (want) {
        if (!motors_initialized) {
          ELCP_Motor_Init(MOTOR_J1);
          ELCP_Motor_Init(MOTOR_J3);
          motors_initialized = true;
        }
        if (ELCP_Motor_IsInitialized(MOTOR_J1) && ELCP_Motor_IsInitialized(MOTOR_J3)) {
          wt_start_edges = want == 1 ? elcp_j3_fb_edges : elcp_j1_fb_edges;
          wt_start_ms = now;
          ELCP_FB_ResetStats();
          ELCP_Motor_SetSpeed(want == 1 ? MOTOR_J3 : MOTOR_J1, (float)AR_BASE_PCT);
          motor_cmd = want == 1 ? (AR_Cmd){AR_BASE_PCT, 0} : (AR_Cmd){0, AR_BASE_PCT};
        } else {
          wt_running = 0;
          return;
        }
      }
      wt_running = want;
    }
  } else if (wt_running) { /* left the wheel screen while running */
    if (motors_initialized) { ELCP_Motor_Stop(MOTOR_J3); ELCP_Motor_Stop(MOTOR_J1); }
    motor_cmd = (AR_Cmd){0, 0};
    wt_running = 0;
  }
  IR_RawScan ir;
  bool frame_ok = IR_ReadAll(&ir) == HAL_OK;
  now = HAL_GetTick();
  IR_CalibrationFeed(&ir_cal, now, frame_ok ? ir.raw : NULL, frame_ok);
  AR_Line line = {false, false, false};
  bool line_valid = get_line(now, &line);
  if (screen == SCREEN_RUN && navigation_active()) {
    const bool needs_ir = AR_Nav_NeedsIR(&nav);
    if (needs_ir && !line_valid) request_stop("IR FAULT STOP");
    else if (needs_ir && nav.phase == AR_PH_COUNTDOWN && !line.c) request_stop("LINE LOST START");
    else if (us_strict() ? !us_ready_for_run(now) : (uint32_t)(now - us_any_ms) > 1500u) request_stop("US2 FAULT STOP");
    else {
      int32_t distance = (int32_t)us2.raw_mm + offset_mm;
      if (distance < 0) distance = 0;
      if (!us_strict()) { /* route: error = no valid distance; a too-short echo = closer than the sensor can measure = at the stone */
        if (us2.status == US2_SHORT_PULSE) distance = 0;
        else if (us2.status != US2_OK) {
          /* No echo right after a close reading = inside the sensor's blind zone / angled stone: treat as at the stone. */
          distance = ((uint32_t)(now - us_ok_ms) <= 600u && us_ok_mm <= 250u) ? 0 : -1;
        }
      }
      /* Wheel feedback edge counts (direction-blind): distance, turn angle and straight-line balance of the hardcoded route. */
      AR_Nav_SetWheelOdometer(&nav, elcp_j3_fb_edges, elcp_j1_fb_edges); /* left = J3, right = J1 */
      AR_Cmd cmd = AR_Nav_Step(&nav, now, line, distance, fresh);
      if (cmd.left != 0 || cmd.right != 0) {
        if (!motors_initialized) {
          ELCP_Motor_Init(MOTOR_J1);
          ELCP_Motor_Init(MOTOR_J3);
          motors_initialized = true;
        }
        if (!ELCP_Motor_IsInitialized(MOTOR_J1) || !ELCP_Motor_IsInitialized(MOTOR_J3)) {
          request_stop("MOTOR INIT FAIL");
        } else {
          /* Deb's mapping: J3 left, J1 right. J3 inversion lives in driver. */
          ELCP_Motor_SetSpeed(MOTOR_J3, (float)cmd.left);
          ELCP_Motor_SetSpeed(MOTOR_J1, (float)cmd.right);
          motor_cmd = cmd;
        }
      } else {
        if (motors_initialized) { ELCP_Motor_Stop(MOTOR_J3); ELCP_Motor_Stop(MOTOR_J1); }
        motor_cmd = (AR_Cmd){0, 0};
      }
      if (nav.phase == AR_PH_DONE || nav.phase == AR_PH_FAULT)
        run_message = AR_Nav_PhaseText(&nav);
    }
  }
  /* An LCD SPI transfer can wait forever. Never enter a display update while
   * navigation is running or the commanded PWM is nonzero. */
  if (!navigation_active() && motor_cmd.left == 0 && motor_cmd.right == 0 &&
      (uint32_t)(now - last_screen) >= SCREEN_MS) {
    last_screen = now;
    draw(now, line, line_valid);
  }
}
