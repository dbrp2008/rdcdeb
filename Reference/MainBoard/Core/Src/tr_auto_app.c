#include "tr_auto_app.h"
#include "tr_auto.h"
#include "tr_linetest.h"
#include "tr_drive.h"
#include "tr_sensors.h"
#include "tr_arm.h"
#include "main.h"
#include "lcd/lcd.h"
#include <stdlib.h>

/* The IR sensors read ON (detect) over the reflective grey floor and off over the black line (see tr_sensors.h).
 * 1 = the line is "ON": flip this only if your modules read the other way round. */
#ifndef TRA_IR_LINE_IS_ON
#define TRA_IR_LINE_IS_ON 0
#endif
#define STICK_ABORT_PCT 25 /* a joystick pushed past this stops the routine */

static TRAuto nav;
static TRLine lt;          /* line-following test (BTN1 on the AUTO page switches to it) */
static bool line_mode = false;
static bool page = false;
static uint32_t last_seq = 0;
static float page_base_mm = 0.0f; /* odometer reading when the page was opened (for measuring distances by hand) */
static TRAuto_Out last_out;
static uint32_t last_step_ms = 0, loop_max_ms = 0; /* loop timing during a run */

static bool running(void) { return TRAuto_IsRunning(&nav) || TRLine_IsRunning(&lt); }

/* BTN1 on the AUTO page (not running): routine 1 (blocks 1 + 2) -> routine 2 (the two far-right blocks) -> LINE TEST ->
 * back to the drive page. */
void TRAutoApp_Btn1(void) {
  if (running()) return;
  if (!line_mode && nav.mode != 2U) {
    TRAuto_Init(&nav);
    TRAuto_SetMode(&nav, 2U);
  } else if (!line_mode) {
    TRAuto_Init(&nav);
    TRAuto_SetMode(&nav, 1U);
    line_mode = true;
    TRLine_Init(&lt);
  } else {
    line_mode = false;
    page = false;
  }
}

void TRAutoApp_Btn2(void) {
  if (!page) {
    page = true;
    TRAuto_Init(&nav);
    page_base_mm = TRDrive_ForwardDistanceM() * 1000.0f;
    return;
  }
  if (running()) {
    TRAuto_Abort(&nav);
    TRLine_Abort(&lt);
    TRDrive_AutoSet(false, 0, 0, 0, 0);
  } else if (line_mode) {
    loop_max_ms = 0;
    if (lt.st == TRL_IDLE) TRLine_Start(&lt, HAL_GetTick());
    else TRLine_Init(&lt); /* ended: back to READY */
  } else if (nav.st == TRA_IDLE) {
    loop_max_ms = 0;
    TRAuto_Start(&nav, HAL_GetTick());
  } else { /* DONE / TIME UP / FAULT: back to READY */
    TRAuto_Init(&nav);
    page_base_mm = TRDrive_ForwardDistanceM() * 1000.0f;
  }
}

bool TRAutoApp_PageActive(void) { return page; }
bool TRAutoApp_Running(void) { return running(); }

void TRAutoApp_Update(void) {
  if (!page && !running()) {
    TRSensors_UsKeepAlive(false, false); /* nothing needs the ultrasonic: back to pinging only on the sensors page */
    return;
  }

  TRDrive_Status st;
  TRSensors_Status ss;
  TRDrive_GetStatus(&st);
  TRSensors_Get(&ss);

  const bool run = running();
  /* ping while the AUTO page is open; every 50 ms while running. A ping blocks the loop for up to ~10 ms when nothing echoes,
   * so while the routine is driving out and searching for the T (where it does not use the ultrasonic) it does not ping:
   * the IRs are then read as often as possible. */
  const bool us_needed = !run || line_mode || TRAuto_NeedsUs(&nav);
  TRSensors_UsKeepAlive(page && us_needed, run && us_needed);
  { /* longest gap between two routine steps during the run (shown on the page): the IRs are read once per loop */
    const uint32_t tnow = HAL_GetTick();
    if (run && last_step_ms) { const uint32_t gap = tnow - last_step_ms; if (gap > loop_max_ms) loop_max_ms = gap; }
    last_step_ms = run ? tnow : 0u;
  }

  if (run && (abs(st.move_x_pct) > STICK_ABORT_PCT || abs(st.move_y_pct) > STICK_ABORT_PCT ||
              abs(st.rot_pct) > STICK_ABORT_PCT)) {
    TRAuto_Abort(&nav); /* stick moved: hand control back */
    TRLine_Abort(&lt);
    TRDrive_AutoSet(false, 0, 0, 0, 0);
    return;
  }

  TRAuto_In in;
  in.left = (ss.ir[0] == (TRA_IR_LINE_IS_ON != 0));
  in.right = (ss.ir[1] == (TRA_IR_LINE_IS_ON != 0));
  in.us_mm = ss.us_valid ? ss.us_mm : -1;
  in.us_fresh = (ss.us_seq != last_seq);
  last_seq = ss.us_seq;
  in.dist_mm = TRDrive_ForwardDistanceM() * 1000.0f;
  in.strafe_mm = TRDrive_StrafeDistanceM() * 1000.0f;
  in.heading_deg = TRDrive_HeadingDeg();
  in.heading_sign = TR_HEADING_SIGN;
  in.turn_state = st.turn_state;
  in.arm_us = st.arm_us;
  in.arm_closed_us = TR_ARM_GRIP_US;
  in.arm_open_us = TR_ARM_OUT_US;
  in.now = HAL_GetTick();

  TRAuto_Out out;
  if (line_mode) TRLine_Step(&lt, &in, &out);
  else TRAuto_Step(&nav, &in, &out);
  last_out = out;
  if (out.zero_heading)
    TRDrive_ZeroHeading();
  if (out.turn_req)
    TRDrive_AutoTurnAt(out.turn_deg, out.turn_frac > 0.0f ? out.turn_frac : TR_TURN_MAX_FRAC);
  TRDrive_AutoSet(out.active, out.x, out.y, out.w, out.arm);
}

static void signed_num(uint8_t x, uint8_t y, int v) { /* this screen's printf has no %+d */
  tft_printc(x, y, v < 0 ? "-" : "+");
  tft_prints((uint8_t)(x + 1u), y, "%4d", v < 0 ? -v : v);
}

void TRAutoApp_DrawPage(void) {
  TRDrive_Status st;
  TRSensors_Status ss;
  TRDrive_GetStatus(&st);
  TRSensors_Get(&ss);
  const uint32_t now = HAL_GetTick();
  const bool run = running();

  if (line_mode) { /* line-following test screen */
    tft_prints(0, 0, "LINE %-11s", TRLine_StateText(lt.st));
    const uint32_t tl = (run && lt.st != TRL_COUNTDOWN) ? (uint32_t)(now - lt.t0) / 100u : 0u;
    tft_prints(0, 1, "T %2u.%u s / 30", (unsigned)(tl / 10u), (unsigned)(tl % 10u));
    tft_printc(0, 2, "HDG");
    signed_num(3, 2, (int)st.heading_deg);
    tft_printc(8, 2, "        ");
    const bool l1 = (ss.ir[0] == (TRA_IR_LINE_IS_ON != 0));
    const bool r1 = (ss.ir[1] == (TRA_IR_LINE_IS_ON != 0));
    tft_prints(0, 3, "LINE L%u R%u      ", l1 ? 1u : 0u, r1 ? 1u : 0u);
    tft_prints(0, 4, "DRIFT FIXED %4u", (unsigned)lt.drifts);
    tft_prints(0, 5, "JUNCTIONS   %4u", (unsigned)lt.junctions);
    if (lt.gap_mm > 0.0f) tft_prints(0, 6, "IR GAP    %3u mm", (unsigned)(lt.gap_mm + 0.5f)); /* free slide between the IRs */
    else tft_printc(0, 6, "IR GAP     -- mm");
    tft_prints(0, 7, "LOOP MAX %4u ms", (unsigned)(loop_max_ms % 10000u));
    tft_printc(0, 8, run ? "B2: STOP        " : (lt.st == TRL_IDLE ? "B2: START       " : "B2: RESET       "));
    tft_printc(0, 9, run ? "                " : "B1: BACK        ");
    return;
  }
  tft_prints(0, 0, "R%u %-13s", (unsigned)nav.mode, TRAuto_StateText(nav.st)); /* R1 = blocks 1 + 2, R2 = far-right blocks */
  /* while running: the time so far; after a run: its finish time (kept until RESET) */
  const uint32_t t = (run && nav.st != TRA_COUNTDOWN) ? (uint32_t)(now - nav.t0) / 100u
                     : (nav.t_end ? (uint32_t)(nav.t_end - nav.t0) / 100u : 0u);
  tft_prints(0, 1, "T %2u.%u s / 20%s", (unsigned)(t / 10u), (unsigned)(t % 10u), (!run && nav.t_end) ? " END" : "    ");
  /* D: distance travelled on this leg while running; while stopped, since the page opened (push the TR by hand or
   * drive it with the joystick to MEASURE the distances for tr_auto.h) */
  const float base = run ? nav.base_mm : page_base_mm;
  const int d = (int)(TRDrive_ForwardDistanceM() * 1000.0f - base);
  tft_printc(0, 2, "D");
  signed_num(2, 2, d);
  tft_printc(8, 2, "mm   ");
  tft_printc(0, 3, "HDG");
  signed_num(3, 3, (int)st.heading_deg);
  const bool l = (ss.ir[0] == (TRA_IR_LINE_IS_ON != 0));
  const bool r = (ss.ir[1] == (TRA_IR_LINE_IS_ON != 0));
  tft_prints(0, 4, "LINE L%u R%u", l ? 1u : 0u, r ? 1u : 0u);
  if (ss.us_valid)
    tft_prints(0, 5, "US %4d mm    ", (int)ss.us_mm);
  else
    tft_printc(0, 5, "US ---- mm   ");
  if (nav.st == TRA_FAULT && nav.t_by) /* row 3: where the T was taken / which rule / how far FIND LINE slid */
    tft_prints(0, 3, "T%4d %s S%+4d  ", (int)nav.t_found_mm, nav.t_by == 1 ? "OLD" : "NEW", (int)nav.align_mm);
  if (nav.st == TRA_FAULT) { /* why it stopped, where, and where each IR last went black on the way out */
    tft_prints(0, 4, "@%4d D%2u LP%3u ", (int)nav.fault_mm, (unsigned)(nav.search_drifts % 100u), (unsigned)(loop_max_ms % 1000u)); /* where, drifts, loop max ms */
    tft_prints(0, 5, "%-16.16s", nav.why[0] ? nav.why : "FAULT");
    for (int s = 0; s < 2; s++) { /* row 6 left IR, row 7 right IR: oldest -> newest of the last 3, '-' = none */
      const int16_t *v = s ? nav.r_on : nav.l_on;
      const unsigned n = s ? nav.r_n : nav.l_n;
      char row[17];
      int p = 0;
      row[p++] = s ? 'R' : 'L';
      for (unsigned k = 0; k < 3u; k++) {
        const int idx = (int)n - 3 + (int)k; /* the k-th of the last 3 */
        int val = idx >= 0 ? v[(unsigned)idx % 3u] : -1;
        if (val > 9999) val = 9999;
        row[p++] = ' ';
        if (val < 0) { row[p++] = ' '; row[p++] = ' '; row[p++] = ' '; row[p++] = '-'; }
        else { row[p++] = (char)('0' + val / 1000); row[p++] = (char)('0' + val / 100 % 10); row[p++] = (char)('0' + val / 10 % 10); row[p++] = (char)('0' + val % 10); }
      }
      row[p] = 0;
      tft_prints(0, (uint8_t)(6 + s), "%-16s", row);
    }
  } else {
    if (nav.gray_seen) /* routine 2: FIND LINE met the boundary's gray strip, took it for what it is and went back */
      tft_prints(0, 6, "T%4d %s GRAY! ", (int)nav.t_found_mm, nav.t_by == 1 ? "OLD" : "NEW");
    else if (nav.mode == 2U && nav.diag_touch) /* routine 2: the diagonal was long and stopped on the far-right line */
      tft_prints(0, 6, "T%4d %s LONG  ", (int)nav.t_found_mm, nav.t_by == 1 ? "OLD" : "NEW");
    else if (nav.t_by) /* where the T was taken and by which rule (compare with where the T really is) */
      tft_prints(0, 6, "T AT %4dmm %s", (int)nav.t_found_mm, nav.t_by == 1 ? "OLD" : "NEW");
    else
      tft_prints(0, 6, "ARM %4u us   ", (unsigned)st.arm_us);
    if (nav.mode == 1U && nav.j2_by) { /* R1: how J2 (the reference for the drop and the turn west) was taken */
      static const char *const j2_text[4] = {"", "IR    ", "SQUARE", "ASSUMD"};
      tft_prints(0, 7, "J2 %s %s", j2_text[nav.j2_by < 4u ? nav.j2_by : 0u], nav.j2_green ? "GREEN" : "     ");
    } else if (nav.mode == 2U && nav.sq_result) { /* R2 SQUARE UP: the angle the line showed after the diagonal (+ = it turned
                                            * clockwise to fix it) and what it did */
      static const char *const sq_text[6] = {"", "TURNED", "SQUARE", "LOST", "FEW PTS", "TOO BIG"};
      const int t10 = (int)(nav.sq_deg * 10.0f + (nav.sq_deg < 0.0f ? -0.5f : 0.5f));
      const int a10 = t10 < 0 ? -t10 : t10;
      tft_prints(0, 7, "SQ %c%2d.%d %-7s", t10 < 0 ? '-' : '+', a10 / 10, a10 % 10, sq_text[nav.sq_result < 6u ? nav.sq_result : 0u]);
    } else {
      tft_prints(0, 7, "LOOP MAX %4u ms", (unsigned)(loop_max_ms % 10000u));
    }
  }
  tft_printc(0, 8, run ? "B2: STOP        " : (nav.st == TRA_IDLE ? "B2: START       " : "B2: RESET       "));
  tft_printc(0, 9, run ? "                " : (nav.mode == 2U ? "B1: LINE TEST   " : "B1: ROUTINE 2   "));
}
