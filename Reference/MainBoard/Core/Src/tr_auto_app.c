#include "tr_auto_app.h"
#include "tr_auto.h"
#include "tr_linetest.h"
#include "tr_drive.h"
#include "tr_sensors.h"
#include "tr_arm.h"
#include "tr_record.h"
#include "tr_store.h"
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
/* Teach and replay (tr_record.h / tr_store.h). The AUTO page modes, B1 steps through them:
 *   R1 -> R2 -> REC (record a run) -> PLAY (replay the saved run) -> PLAY MIRROR (the other field side) -> LINE TEST -> out */
typedef enum { TM_OFF = 0, TM_REC, TM_PLAY, TM_PLAYM } TeachMode;
static TeachMode teach = TM_OFF;
static TRR_Recorder rec;   /* the recording being made (RAM) */
typedef enum { RS_IDLE = 0, RS_REC, RS_STOPPED, RS_SAVED, RS_FAILED } RecState;
static RecState rec_state = RS_IDLE;
static const char *rec_msg = "";
static TRR_Player play;
static bool play_countdown = false;
static uint32_t play_countdown_end = 0;
static const char *play_msg = "";
static const TRR_Sample *flash_rec = 0; /* the saved recording (straight from flash), checked once when needed */
static uint16_t flash_n = 0;
static bool flash_checked = false;
#define STILL_RPM 3.0f /* a wheel slower than this = standing still (needed to save: saving freezes the CPU for 1-2 s) */
static uint32_t last_seq = 0;
static float page_base_mm = 0.0f; /* odometer reading when the page was opened (for measuring distances by hand) */
static TRAuto_Out last_out;
static uint32_t last_step_ms = 0, loop_max_ms = 0; /* loop timing during a run */

static bool running(void) {
  return TRAuto_IsRunning(&nav) || TRLine_IsRunning(&lt) || TRR_PlayRunning(&play) || play_countdown;
}

static void flash_check(void) {
  if (!flash_checked) {
    flash_rec = TRStore_Load(&flash_n);
    flash_checked = true;
  }
}

static void stop_all(void) {
  TRAuto_Abort(&nav);
  TRLine_Abort(&lt);
  play.st = TRR_IDLE;
  play_countdown = false;
  TRDrive_AutoSet(false, 0, 0, 0, 0);
}

/* BTN1 on the AUTO page (not running, not recording): R1 (blocks 1 + 2) -> R2 (the two far-right blocks) -> REC -> PLAY ->
 * PLAY MIRROR -> LINE TEST -> back to the drive page. */
void TRAutoApp_Btn1(void) {
  if (running() || rec.on) return;
  if (line_mode) {
    line_mode = false;
    page = false;
  } else if (teach == TM_PLAYM) {
    teach = TM_OFF;
    line_mode = true;
    TRLine_Init(&lt);
  } else if (teach == TM_PLAY) {
    teach = TM_PLAYM;
    play.st = TRR_IDLE;
    play_msg = "";
  } else if (teach == TM_REC) {
    teach = TM_PLAY;
    play.st = TRR_IDLE;
    play_msg = "";
    flash_checked = false; /* look at the flash again (a recording may just have been saved) */
  } else if (nav.mode == 2U) {
    TRAuto_Init(&nav);
    TRAuto_SetMode(&nav, 1U);
    teach = TM_REC;
    if (rec_state != RS_STOPPED) { rec_state = RS_IDLE; rec_msg = ""; } /* an unsaved recording is kept until replaced */
    flash_checked = false;
  } else {
    TRAuto_Init(&nav);
    TRAuto_SetMode(&nav, 2U);
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
    stop_all();
  } else if (teach == TM_REC) {
    if (rec_state == RS_REC) { /* stop recording */
      TRR_RecStop(&rec);
      rec_state = RS_STOPPED;
      rec_msg = rec.n < 2U ? "IT NEVER MOVED" : "";
    } else if (rec_state == RS_STOPPED || rec_state == RS_FAILED) { /* save it (again after a failure): only standing still
                                                                       * (the save freezes the CPU for 1-2 s) */
      TRDrive_Status st;
      TRDrive_GetStatus(&st);
      bool still = true;
      for (int i = 0; i < TR_WHEEL_COUNT; i++)
        if (st.actual_rpm[i] > STILL_RPM || st.actual_rpm[i] < -STILL_RPM) still = false;
      if (!still) {
        rec_msg = "STOP THE TR 1ST";
      } else {
        TRDrive_AutoSet(false, 0, 0, 0, 0);
        const TRS_Result r = TRStore_Save(rec.buf, rec.n);
        rec_state = (r == TRS_OK) ? RS_SAVED : RS_FAILED;
        rec_msg = TRStore_ResultText(r);
        flash_checked = false;
      }
    } else { /* start a new recording here: this is the start pose every replay must use */
      TRR_RecStart(&rec, HAL_GetTick(), TRDrive_ForwardDistanceM() * 1000.0f, TRDrive_StrafeDistanceM() * 1000.0f,
                   TRDrive_HeadingDeg());
      rec_state = RS_REC;
      rec_msg = "";
    }
  } else if (teach == TM_PLAY || teach == TM_PLAYM) {
    if (play.st == TRR_DONE || play.st == TRR_FAULT) { /* back to READY */
      play.st = TRR_IDLE;
      play_msg = "";
    } else {
      flash_check();
      if (flash_rec == 0) {
        play_msg = "NO RECORDING";
      } else {
        play_countdown = true;
        play_countdown_end = HAL_GetTick() + TRA_COUNTDOWN_MS;
        play_msg = "";
        loop_max_ms = 0;
      }
    }
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
bool TRAutoApp_Running(void) { return running() || rec.on; }

void TRAutoApp_Update(void) {
  if (!page && !running() && !rec.on) {
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
  const bool us_needed = !run || line_mode || (teach == TM_OFF && TRAuto_NeedsUs(&nav)); /* the replay does not use it */
  TRSensors_UsKeepAlive(page && us_needed, run && us_needed);
  { /* longest gap between two routine steps during the run (shown on the page): the IRs are read once per loop */
    const uint32_t tnow = HAL_GetTick();
    if (run && last_step_ms) { const uint32_t gap = tnow - last_step_ms; if (gap > loop_max_ms) loop_max_ms = gap; }
    last_step_ms = run ? tnow : 0u;
  }

  if (run && (abs(st.move_x_pct) > STICK_ABORT_PCT || abs(st.move_y_pct) > STICK_ABORT_PCT ||
              abs(st.rot_pct) > STICK_ABORT_PCT)) {
    stop_all(); /* stick moved: hand control back */
    if (teach != TM_OFF) play_msg = "STICK STOP";
    return;
  }

  if (rec.on) { /* recording: the driver drives (joystick), the TR only logs where it goes */
    if (!TRR_RecStep(&rec, HAL_GetTick(), TRDrive_ForwardDistanceM() * 1000.0f, TRDrive_StrafeDistanceM() * 1000.0f,
                     TRDrive_HeadingDeg(), st.arm_us)) {
      TRR_RecStop(&rec); /* full (TRR_MAX_SAMPLES): trim like a normal stop */
      rec_state = RS_STOPPED;
      rec_msg = "FULL: 30 S MAX";
    }
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
  else if (teach == TM_PLAY || teach == TM_PLAYM) { /* replay the saved recording */
    if (play_countdown && (int32_t)(in.now - play_countdown_end) >= 0) {
      play_countdown = false;
      flash_check();
      if (!TRR_PlayStart(&play, flash_rec, flash_n, teach == TM_PLAYM, in.now, in.dist_mm, in.strafe_mm, in.heading_deg))
        play_msg = "NO RECORDING";
    }
    TRR_PlayStep(&play, &in, &out); /* inactive (joystick has control) unless replaying */
  } else TRAuto_Step(&nav, &in, &out);
  last_out = out;
  if (out.zero_heading)
    TRDrive_ZeroHeading();
  if (out.turn_req)
    TRDrive_AutoTurn(out.turn_deg);
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

  if (teach == TM_REC) { /* record screen */
    static const char *const rs_text[] = {"READY", "RECORDING", "STOPPED", "SAVED", "SAVE FAILED"};
    tft_prints(0, 0, "REC %-12s", rs_text[rec_state]);
    const uint32_t tr = (uint32_t)rec.n * TRR_PERIOD_MS / 100u;
    tft_prints(0, 1, "T %2u.%u s / 30", (unsigned)(tr / 10u), (unsigned)(tr % 10u));
    tft_printc(0, 2, "X");
    signed_num(1, 2, rec_state == RS_IDLE ? 0 : (int)rec.pose.x);
    tft_printc(6, 2, " Y");
    signed_num(8, 2, rec_state == RS_IDLE ? 0 : (int)rec.pose.y);
    tft_printc(13, 2, "mm ");
    tft_printc(0, 3, "HDG");
    signed_num(3, 3, rec_state == RS_IDLE ? 0 : (int)TRR_PoseHeading(&rec.pose, TRDrive_HeadingDeg()));
    tft_printc(8, 3, "        ");
    /* the replay can go at most TRR_VMAX_MM_S: a faster recording is replayed slower */
    tft_prints(0, 4, "VMAX %4u mm/s%s", (unsigned)rec.vmax_mm_s, rec.vmax_mm_s > TRR_VMAX_MM_S ? "!" : " ");
    tft_prints(0, 5, "%-16.16s", rec_msg);
    tft_printc(0, 6, "START = SAME SPOT");
    tft_printc(0, 7, "AS EVERY REPLAY ");
    tft_printc(0, 8, rec_state == RS_REC ? "B2: STOP        " :
                     (rec_state == RS_STOPPED ? "B2: SAVE        " : (rec_state == RS_FAILED ? "B2: SAVE AGAIN  " : "B2: START REC   ")));
    tft_printc(0, 9, rec.on ? "                " : "B1: REPLAY      ");
    return;
  }
  if (teach == TM_PLAY || teach == TM_PLAYM) { /* replay screen */
    flash_check();
    tft_prints(0, 0, "%s %-12.12s", teach == TM_PLAYM ? "PLM" : "PLY",
               play_countdown ? "COUNTDOWN" : TRR_PlayStateText(play.st));
    const uint32_t tp = (uint32_t)(play.t_ms / 100.0f);
    const uint32_t tt = (uint32_t)flash_n * TRR_PERIOD_MS / 100u;
    tft_prints(0, 1, "T %2u.%u / %2u.%u s", (unsigned)(tp / 10u), (unsigned)(tp % 10u), (unsigned)(tt / 10u), (unsigned)(tt % 10u));
    tft_prints(0, 2, "OFF %3u MAX %3u ", (unsigned)(play.err_mm > 999.0f ? 999.0f : play.err_mm),
               (unsigned)(play.max_err_mm > 999.0f ? 999.0f : play.max_err_mm)); /* mm from the path */
    tft_printc(0, 3, "X");
    signed_num(1, 3, (int)play.pose.x);
    tft_printc(6, 3, " Y");
    signed_num(8, 3, (int)play.pose.y);
    tft_printc(13, 3, "mm ");
    tft_prints(0, 4, "%s", flash_rec ? "FLASH: RECORDING" : "FLASH: EMPTY    ");
    tft_prints(0, 5, "%-16.16s", play_msg);
    tft_printc(0, 6, teach == TM_PLAYM ? "MIRRORED L<->R  " : "                ");
    tft_prints(0, 7, "LOOP MAX %4u ms", (unsigned)(loop_max_ms % 10000u));
    tft_printc(0, 8, run ? "B2: STOP        " : ((play.st == TRR_DONE || play.st == TRR_FAULT) ? "B2: RESET       " : "B2: START       "));
    tft_printc(0, 9, run ? "                " : (teach == TM_PLAYM ? "B1: LINE TEST   " : "B1: MIRROR      "));
    return;
  }
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
  const uint32_t t = (run && nav.st != TRA_COUNTDOWN) ? (uint32_t)(now - nav.t0) / 100u : 0u;
  tft_prints(0, 1, "T %2u.%u s / %u", (unsigned)(t / 10u), (unsigned)(t % 10u), nav.mode == 2U ? 30u : 20u);
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
    if (nav.t_by) /* where the T was taken and by which rule (compare with where the T really is) */
      tft_prints(0, 6, "T AT %4dmm %s", (int)nav.t_found_mm, nav.t_by == 1 ? "OLD" : "NEW");
    else
      tft_prints(0, 6, "ARM %4u us   ", (unsigned)st.arm_us);
    tft_prints(0, 7, "LOOP MAX %4u ms", (unsigned)(loop_max_ms % 10000u));
  }
  tft_printc(0, 8, run ? "B2: STOP        " : (nav.st == TRA_IDLE ? "B2: START       " : "B2: RESET       "));
  tft_printc(0, 9, run ? "                " : (nav.mode == 2U ? "B1: RECORD      " : "B1: ROUTINE 2   "));
}
