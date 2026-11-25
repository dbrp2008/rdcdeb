/* Stationary IR setup, then a stopped route selector with independent run/stop. */
#include "calibration_app.h"
#include "button_gesture.h"
#include "drive_plan.h"
#include "elcp_zero_pwm.h"
#include "function1.h"
#if !F1_LEGACY
#include "retrace.h"
#include "us1_capture.h"
#endif
#include "function2.h"
#include "function3.h"
#include "elcp_motor/elcp_motor.h"
#include "ir_sensor.h"
#include "ir_setup.h"
#include "ir_mode.h"
#include "side_ir.h"
#include "wheel_spin.h"
#include "enc_watch.h"
#include "us1_sensor.h"
#include "lcd/lcd.h"
#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* DRIVE_F3: the legacy build (the pinned host replays) runs the old generic right-turn policy (function3.c); the fixed
 * build runs F3 = the top-right key on F1's run (F1_StartF3), fed exactly as F1 */
#if F1_LEGACY
#define OLD_F3(route) ((route) == DRIVE_F3)
#else
#define OLD_F3(route) ((void)(route), false)
#endif
#if F1_LEGACY
#define IR_WINDOW 4u
#else
#define IR_WINDOW 2u /* average of 2 scans (was 4): the steering sees the line ~10-20 ms sooner */
#endif
#define IR_SAMPLE_MS 10u
#define IR_NEWEST_MAX_MS 50u
#define IR_OLDEST_MAX_MS 200u
#define US_PERIOD_MS 60u
#define US_MAX_AGE_MS 150u
#define SCREEN_MS 100u
#if !F1_LEGACY
#define SCREEN_RUN_MS 300u /* while driving: a TFT update blocks the loop while it sends, so fewer of them (as on the TR) */
#endif
/* Forward sign per motor, from the measured wheel direction. */
#define J3_FORWARD (-1.0f)
#define J1_FORWARD (1.0f)

/* No baked-in fallback: drive is unreachable until the setup page returns a valid profile. */
static IR_Profile profile;
static bool in_setup;
typedef enum { IDLE, RUNNING, FINISHED } State;
static ButtonGesture buttons;
static State state;
static DriveFunction selected; /* never changes while the wheels are running */
static const char *status;
static bool motor_ready, timer_ready, us_valid, us_have_reading;
static uint32_t ir_sum[3], ir_times[IR_WINDOW], last_ir, last_ping, us_tick, last_screen;
static uint16_t ir_samples[IR_WINDOW][3], ir_average[3];
static unsigned ir_count, ir_next, raw_mask, filtered_mask;
static US2_Reading us1;
#if !F1_LEGACY
static Retrace trail; /* the outbound wheel commands, replayed backwards to drive home (retrace.h) */
static bool us_async; /* US1 timed by TIM2 input capture (us1_capture.h); false: the original blocking ping */
/* AR geometry: 65 mm wheels, 233 mm between the wheels (tuned on this AR with ~115 pulses per turn; 246 centre to centre).
 * Pulses per wheel turn MEASURED UNDER POWER with the setup page's wheel check (10 turns): J1 right 1155, J3 left 1150.
 * Slow hand turns over-count (~650-770 per turn: the slot edges flicker as they pass slowly) - don't use them. */
#define AR_PULSES_PER_TURN_R 115.5f
#define AR_PULSES_PER_TURN_L 115.0f
#define AR_MM_PER_PULSE_R (3.14159265f * 65.0f / AR_PULSES_PER_TURN_R)
#define AR_MM_PER_PULSE_L (3.14159265f * 65.0f / AR_PULSES_PER_TURN_L)
#define AR_TRACK_MM 233.0f
static bool pivot_on;
static bool advance_on; /* F2: driving straight past the T before the pivot */
static float advance_odo;
static int32_t pivot_j1, pivot_j3;
/* odometry counts: every pulse, signed by the commanded direction (side_ir.c) - not the driver's lossy pulse_count */
static int32_t enc_j1(void) { return SideIR_EncoderSigned(true); }
static int32_t enc_j3(void) { return SideIR_EncoderSigned(false); }
float SideIR_WheelCommandPct(bool right) { return ELCP_Motor_GetState(right ? MOTOR_J1 : MOTOR_J3)->current_speed_pct; }
/* every FB pulse, at any speed: pulse_count ignores pulses at 0 % (no direction), so the setup page shows these */
static uint32_t enc_raw_j1(void) { return SideIR_EncoderRaw(true); }
static uint32_t enc_raw_j3(void) { return SideIR_EncoderRaw(false); }
static EncWatch enc_watch; /* stops a run whose driven wheel gives no pulses (dead encoder or stalled wheel) */
/* encoder totals for F1's turn-home (distance driven and pivot angle) */
static int32_t odo_j1, odo_j3;
static float odo_mm, arc_mm, hdg_deg;
static bool straight_on;
static int32_t straight_j1, straight_j3;
static void feed_odometry(void) {
  const int32_t j1 = enc_j1(), j3 = enc_j3();
  const int32_t a = j1 - odo_j1, b = j3 - odo_j3;
  odo_j1 = j1; odo_j3 = j3;
  const float dr = (float)(a < 0 ? -a : a) * AR_MM_PER_PULSE_R, dl = (float)(b < 0 ? -b : b) * AR_MM_PER_PULSE_L;
  odo_mm += 0.5f * (dl + dr);
  arc_mm += dl + dr;
  /* heading (CCW +): the right wheel's forward travel minus the left's / track. ELCP counts follow the commanded direction;
   * J1 (right) counts up going forward, J3 (left) is wired inverted */
  hdg_deg += ((float)(J1_FORWARD * (float)a) * AR_MM_PER_PULSE_R - (float)(J3_FORWARD * (float)b) * AR_MM_PER_PULSE_L)
             / AR_TRACK_MM * 57.29578f;
  F1_Odometry(odo_mm, arc_mm);
  F1_Heading(hdg_deg);
}
/* F1 drives STRAIGHT on the encoders (backing off, into the start zone): hold both wheels to equal distance */
static LineWheels match_wheels(LineWheels w) {
  if (!straight_on) { straight_on = true; straight_j1 = enc_j1(); straight_j3 = enc_j3(); }
  const int32_t a = enc_j1() - straight_j1, b = enc_j3() - straight_j3;
  const float dr = (float)(a < 0 ? -a : a) * AR_MM_PER_PULSE_R, dl = (float)(b < 0 ? -b : b) * AR_MM_PER_PULSE_L;
  /* the wheel ahead by d mm slows by 4.5 % per mm (the old 8 % per 1.78 mm pulse) */
  float kl = 1.0f - 0.045f * (dl - dr), kr = 1.0f - 0.045f * (dr - dl);
  kl = kl < 0.3f ? 0.3f : (kl > 1.0f ? 1.0f : kl);
  kr = kr < 0.3f ? 0.3f : (kr > 1.0f ? 1.0f : kr);
  return (LineWheels){w.left * kl, w.right * kr};
}
#endif

static void zero_wheels(void) {
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);
}
static void stop(const char *reason) {
  zero_wheels();
  if (selected == DRIVE_F1 || selected == DRIVE_F2 || (selected == DRIVE_F3 && !OLD_F3(selected)) || selected == DRIVE_F4) F1_Stop();
  if (selected == DRIVE_F2) F2_Stop(reason);
  state = FINISHED;
  status = reason;
}
/* Apply only zero/terminal commands here: a newly unsafe scan must not wait
 * for the next blocking ping. Positive drive is only issued after interlocks. */
static bool f3_inhibit(F3_Command cmd) {
  switch (cmd) {
    case F3_WAIT_TAPE:
      zero_wheels();
      status = "F3 WAIT TAPE"; return true;
    case F3_LOST_LINE: stop("F3 LINE LOST"); return true;
    case F3_UNSTABLE_LINE: stop("F3 LINE UNSTABLE"); return true;
    case F3_TURN_TIMEOUT: stop("F3 TURN TIMEOUT"); return true;
    case F3_STRAIGHT: case F3_STEER_LEFT: case F3_STEER_RIGHT: case F3_PIVOT_RIGHT: return false;
  }
  stop("F3 MODE ERROR"); return true;
}
/* Poll before ADC, after ADC and after the blocking ping. Keep a B1 release
 * pending only for this update, and cancel it if either key goes down before
 * motor dispatch. A B1 begun during a run cannot launch after a stop/fault. */
static void poll_buttons(uint32_t now, GestureMode mode, unsigned *action) {
  const unsigned keys = (btn_read(BTN1) ? KEY_B1 : 0u) | (btn_read(BTN2) ? KEY_B2 : 0u);
  if ((keys & KEY_B2) && state == RUNNING) stop("STOP BTN2");
  if (keys && *action == KEY_B1) *action = 0u;
  const unsigned released = ButtonGesture_Release(&buttons, now, keys, mode);
  if (released) *action = released;
}
static bool ir_fresh(uint32_t now) {
  if (ir_count != IR_WINDOW) return false;
  unsigned newest = (ir_next + IR_WINDOW - 1u) % IR_WINDOW;
  return (uint32_t)(now - ir_times[newest]) <= IR_NEWEST_MAX_MS &&
         (uint32_t)(now - ir_times[ir_next]) <= IR_OLDEST_MAX_MS;
}
static bool us_fresh(uint32_t now) {
#if !F1_LEGACY
  /* driving home on the encoders (reversing away from the target) does not use US1: a lost / stale echo must not stop it */
  if (state == RUNNING && !OLD_F3(selected) && F1_PhaseAt(now) == F1_RETRACE) return true;
  /* turning home (back off, pivot, return): a missing / stale echo while it points away must not stop it; the finish
   * itself still needs a valid reading under DRIVE_US1_FINISH_MM */
  if (state == RUNNING && !OLD_F3(selected) && F1_TurnsHome() &&
      (F1_PhaseAt(now) == F1_TARGET_REVERSE || F1_PhaseAt(now) == F1_PIVOT || F1_PhaseAt(now) == F1_RETURN)) return true;
  /* F4 off the tape (its own legs, the calibration spin): US1 often has nothing in range there; the leg limits and the
   * stall check stop it, not a missing echo (the line out to the stone still needs it) */
  if (state == RUNNING && selected == DRIVE_F4 && F1_F4OffLine()) return true;
#endif
  return timer_ready && us_valid && (uint32_t)(now - us_tick) <= US_MAX_AGE_MS;
}
static unsigned black_mask(const uint16_t readings[3]) {
  unsigned mask = 0u;
  for (unsigned i = 0; i < 3u; i++) {
    if (profile.black_high[i] ? readings[i] > profile.t[i] : readings[i] < profile.t[i])
      mask |= 1u << i; /* equal to T is white for either polarity */
  }
  return mask;
}
static void read_ir(uint32_t now) {
  last_ir = now;
  IR_RawScan scan;
  if (IR_ReadAll(&scan) != HAL_OK || scan.raw[0] > 4095u || scan.raw[1] > 4095u || scan.raw[2] > 4095u) {
    ir_count = ir_next = 0; memset(ir_sum, 0, sizeof(ir_sum));
    if (state == RUNNING) stop("IR ERROR");
    return;
  }
  uint32_t completed = HAL_GetTick();
  for (unsigned i = 0; i < 3u; i++) {
    if (ir_count == IR_WINDOW) ir_sum[i] -= ir_samples[ir_next][i];
    ir_samples[ir_next][i] = scan.raw[i];
    ir_sum[i] += scan.raw[i];
    ir_average[i] = (uint16_t)((ir_sum[i] + (ir_count < IR_WINDOW ? (ir_count + 1u) / 2u : IR_WINDOW / 2u)) /
                               (ir_count < IR_WINDOW ? ir_count + 1u : IR_WINDOW));
  }
  ir_times[ir_next] = completed;
  ir_next = (ir_next + 1u) % IR_WINDOW;
  if (ir_count < IR_WINDOW) ir_count++;
  if (ir_count != IR_WINDOW) return;
  raw_mask = black_mask(scan.raw);
  filtered_mask = black_mask(ir_average);
  if (state != RUNNING) return;
  if (OLD_F3(selected)) {
    F3_Scan(completed, raw_mask, filtered_mask);
    (void)f3_inhibit(F3_CommandAt(completed)); /* before a potentially blocking ping */
    return;
  }
  const BranchSample side = SideIR_TakeBranchSample();
  if (selected == DRIVE_F2) {
    /* F2 extends the ordinary outbound route, never the turnaround or finish.
     * Hold a pending launch-time turn until the wheels are free to pivot. */
    F1_Phase phase = F1_PhaseAt(completed);
    if (phase == F1_OUTBOUND || (phase == F1_LAUNCH && !F2_Turning()))
      F2_Update(side, filtered_mask);
  } else F1_SideSample(side);
#if !F1_LEGACY
  { const SideIR_Reading sr = SideIR_Read(); F1_SideLevels(sr.left_high, sr.right_high); }
  feed_odometry();
#endif
  F1_Scan(now, completed, raw_mask, filtered_mask);
#if !F1_LEGACY
  Retrace_Line(&trail, filtered_mask); /* start-zone outline checkpoint (out: note it; home: correct the distance) */
#endif
}
static const char *us_error_text(US2_Status error) {
  switch (error) {
    case US2_ECHO_BUSY: return "US1 ECHO HIGH";
    case US2_NO_RISE: return "US1 NO RISE";
    case US2_NO_FALL: return "US1 NO FALL";
    case US2_SHORT_PULSE: return "US1 SHORT PULSE";
    case US2_TIMER_ERROR: return "US1 TIMER ERROR";
    default: return "US1 ERROR";
  }
}
static void handle_us(US2_Reading r, uint32_t now);
static void read_us(uint32_t now) {
  last_ping = now;
  handle_us(US1_Measure(), now);
}
/* a finished US1 reading; now = when its ping was SENT (freshness is aged from the trigger) */
static void handle_us(US2_Reading r, uint32_t now) {
  us1 = r;
  us_tick = now; /* Age from ping start, not completion. */
  us_have_reading = true;
  us_valid = us1.status == US2_OK;
  if (state != RUNNING) return;
#if !F1_LEGACY
  if (!us_valid && !OLD_F3(selected) && F1_PhaseAt(now) == F1_RETRACE) return; /* not used while retracing home */
  if (!us_valid && !OLD_F3(selected) && F1_TurnsHome() &&
      (F1_PhaseAt(now) == F1_TARGET_REVERSE || F1_PhaseAt(now) == F1_PIVOT || F1_PhaseAt(now) == F1_RETURN)) return;
  /* F4 off the tape: US1 is only read at the stone, the barrier face and the key; a missing echo (nothing in range on
   * the diagonal, or US1 blocked) must not stop it - the leg limits and the stall check do that */
  if (!us_valid && selected == DRIVE_F4 && F1_F4OffLine()) return;
#endif
  if (!us_valid) { stop(us_error_text(us1.status)); return; }
  if (OLD_F3(selected)) {
    if (us1.raw_mm < F3_US1_STOP_MM) stop("F3 US1 <" DRIVE_MM_LABEL(F3_US1_STOP_MM));
  } else {
    /* Ping freshness is aged from trigger; the approach starts when the
     * valid result exists. Wheel output still goes through the interlocks. */
    (void)F1_TargetPing(HAL_GetTick(), us1.raw_mm);
  }
}
static const char *range_inhibit(DriveFunction route, uint32_t mm) {
  if (OLD_F3(route)) return mm < F3_US1_STOP_MM ? "TARGET <" DRIVE_MM_LABEL(F3_US1_STOP_MM) : NULL;
  return mm < DRIVE_US1_STOP_MM ? "TARGET <" DRIVE_MM_LABEL(DRIVE_US1_STOP_MM) : NULL;
}
static void drive_wheels(LineWheels wheels) {
  ELCP_Motor_SetSpeed(MOTOR_J3, J3_FORWARD * wheels.left);
  ELCP_Motor_SetSpeed(MOTOR_J1, J1_FORWARD * wheels.right);
}
static void drive_f3(uint32_t now) {
  const F3_Command cmd = F3_CommandAt(now);
  if (f3_inhibit(cmd)) return;
  if (cmd == F3_PIVOT_RIGHT) {
    drive_wheels((LineWheels){F3_PIVOT_LEFT_PERCENT, -F3_PIVOT_RIGHT_PERCENT});
    status = "F3 TURN RIGHT";
  } else {
    unsigned mask = cmd == F3_STEER_RIGHT ? 4u : cmd == F3_STEER_LEFT ? 1u : 2u;
    drive_wheels(Line_Forward(mask));
    status = cmd == F3_STEER_RIGHT ? "F3 FOLLOW R" : cmd == F3_STEER_LEFT ? "F3 FOLLOW L" : "F3 FOLLOW";
  }
}
#if !F1_LEGACY
/* the outbound wheel commands are recorded (breadcrumbs) so the AR can drive home backwards along the same path */
static void drive_recorded(uint32_t now, LineWheels w) {
  drive_wheels(w);
  Retrace_Record(&trail, now, w, enc_j1(), enc_j3());
}
#else
#define drive_recorded(now, w) drive_wheels(w)
#endif
static void drive_line(uint32_t now) {
#if !F1_LEGACY
  if (F1_PhaseAt(now) == F1_RETRACE) { /* home: replay the outbound run backwards on the encoders */
    LineWheels w;
    if (Retrace_Step(&trail, now, enc_j1(), enc_j3(), &w)) {
      drive_wheels(w);
      status = "RETRACE HOME";
    } else {
      stop(trail.state == RETRACE_DONE ? "HOME (RETRACED)" : trail.state == RETRACE_STALLED ? "HOME STALLED" : "HOME PARTIAL");
    }
    return;
  }
#endif
#if !F1_LEGACY
  if (selected == DRIVE_F2) F2_Tick(now); /* time backups for the advance and the pivot */
  if (selected == DRIVE_F2 && F2_Failed()) { stop(F2_Status()); return; } /* no tape in the window, or a timeout */
  if (selected == DRIVE_F2) F1_BranchTurning(F2_Turning()); /* F2 home needs the start -> T and T -> key distances */
  if (selected == DRIVE_F2 && F1_PhaseAt(now) == F1_OUTBOUND && F2_Advancing()) {
    /* past the T first (F2_T_OUT_ADVANCE_MM), straight on the encoders, then the pivot */
    feed_odometry();
    if (!advance_on) { advance_on = true; advance_odo = odo_mm; straight_on = false; }
    F2_Advanced(odo_mm - advance_odo);
    if (F2_Advancing()) {
      drive_recorded(now, match_wheels((LineWheels){F12_DUTY(LINE_NOMINAL_PERCENT), F12_DUTY(LINE_NOMINAL_PERCENT)}));
      status = F2_Status();
      return;
    }
    straight_on = false;
  } else advance_on = false;
  if (selected == DRIVE_F2 && F1_PhaseAt(now) == F1_OUTBOUND && F2_Turning()) {
    /* encoder angle of the pivot so far: both wheels' arcs / track width */
    if (!pivot_on) { pivot_on = true; pivot_j1 = enc_j1(); pivot_j3 = enc_j3(); }
    const int32_t a = enc_j1() - pivot_j1, b = enc_j3() - pivot_j3;
    const float arc_mm = (float)(a < 0 ? -a : a) * AR_MM_PER_PULSE_R + (float)(b < 0 ? -b : b) * AR_MM_PER_PULSE_L;
    F2_PivotAngle(arc_mm / AR_TRACK_MM * 57.29578f);
  } else pivot_on = false;
#endif
  if (selected == DRIVE_F2 && F1_PhaseAt(now) == F1_OUTBOUND && F2_Turning()) {
    drive_recorded(now, ((LineWheels){F2_LEFT_FORWARD_PCT, -F2_RIGHT_REVERSE_PCT}));
    F2_PivotCommanded(); /* only subsequent scans can clear or reacquire tape */
    status = F2_Status();
    return;
  }
#if !F1_LEGACY
  feed_odometry();
#endif
#if F1_LEGACY
  const F1_Command cmd = F1_CommandAt(now, us1.raw_mm);
#else
  const F1_Command cmd = F1_CommandAt(now, us_valid ? us1.raw_mm : 9999u); /* an invalid echo is never "close" */
#endif
#if !F1_LEGACY
  if (cmd.request == F1_MOVE && F1_EncoderStraight(now)) { drive_wheels(match_wheels(cmd.wheels)); status = cmd.status; return; }
  straight_on = false;
#endif
  if (cmd.request == F1_MOVE) { drive_recorded(now, cmd.wheels); status = cmd.status; }
  else if (cmd.request == F1_STOP) stop(cmd.status);
  else { /* also the first tick of the nonterminal hold */
    zero_wheels();
    status = cmd.status;
#if !F1_LEGACY
    Retrace_Record(&trail, now, (LineWheels){0.0f, 0.0f}, enc_j1(), enc_j3()); /* standing still is not recorded */
#endif
  }
}
static void draw(uint32_t now) {
  char buf[17];
  if (OLD_F3(selected)) {
    tft_prints(0, 0, "F3 RIGHT %-4u", F3_RightTurns() % 10000u);
  } else {
    const char *title = selected == DRIVE_F2 ? "F2 LINE BR %-4u" : selected == DRIVE_F3 ? "F3 TOP RIGHT %-3u" :
                        selected == DRIVE_F4 ? "F4 BOT LEFT %-4u" : "F1 LINE BR %-4u";
    unsigned count = selected == DRIVE_F2 ? F2_BranchCount() : F1_BranchCount();
    tft_prints(0, 0, title, count % 10000u);
  }
#if IR_DIGITAL
  /* 16 x 10, digital IRs: 1 = tape. FRONT is what F1/F2 use (tape on 2 scans in a row). */
  { const SideIR_Reading side = SideIR_Read();
    if (ir_fresh(now))
      tft_prints(0, 1, "FRONT L%u C%u R%u  ", filtered_mask & 1u ? 1u : 0u, filtered_mask & 2u ? 1u : 0u,
                 filtered_mask & 4u ? 1u : 0u);
    else tft_printc(0, 1, "FRONT NOT READY ");
    tft_prints(0, 2, "SIDE  L%u     R%u ", (unsigned)side.left_high, (unsigned)side.right_high); }
  if (!timer_ready) tft_printc(0, 3, "US1 CHECK TIM2  ");
  else if (!us_have_reading) tft_printc(0, 3, "US1 WAITING     ");
  else if (!us_valid) tft_prints(0, 3, "%-16s", us_error_text(us1.status));
  else if (!us_fresh(now)) tft_printc(0, 3, "US1 STALE       ");
  else tft_prints(0, 3, "US1 %4u mm     ", (unsigned)us1.raw_mm);
#if !F1_LEGACY
  { const int32_t r = enc_j1(), l = enc_j3();
    /* the TFT's own printf (lcd/strhelper.h) knows only %s %d %u %x %c %f with width/-/0: no '+', no 'l' */
    tft_prints(0, 4, "R%6d  L%6d", (int)(r > 99999 ? 99999 : r < -99999 ? -99999 : r),
               (int)(l > 99999 ? 99999 : l < -99999 ? -99999 : l)); }
#else
  tft_printc(0, 4, "                ");
#endif
  tft_prints(0, 5, "%-16s", status);
  tft_printc(0, 6, state == RUNNING ? "B2 = STOP       " : "B1 = RUN        ");
#if F1_LEGACY
  tft_printc(0, 7, selected == DRIVE_F2 ? "B1+B2 = TO F1   " : "B1+B2 = TO F2   ");
#else
  tft_printc(0, 7, selected == DRIVE_F1 ? "B1+B2 = TO F2   " : selected == DRIVE_F2 ? "B1+B2 = TO F3   " :
                   selected == DRIVE_F3 ? "B1+B2 = TO F4   " : "B1+B2 = TO F1   ");
#endif
#if !F1_LEGACY
  { /* the home pivot's angle as measured on the encoders: live while pivoting, kept after a stop (0 = no pivot yet) */
    const float d = F1_LastPivotDeg();
    int up = 0, bdeg = 0; char kind = '-'; bool retried = false;
    if (selected == DRIVE_F3) { /* F3: the bend - mm up the leg when seen, S / W = by the outer-right IR / centre + left
                                 * white, its pivot (R = turned back and retried) - then the home pivot */
      if (!F1_F3Bend(&up, &kind, &bdeg, &retried)) { up = 0; kind = '-'; bdeg = 0; retried = false; }
      tft_prints(0, 8, "B%3d%c%4d%c P%4d", up > 999 ? 999 : up, kind, bdeg > 9999 ? 9999 : bdeg, retried ? 'R' : ' ',
                 (int)(d > 9999.0f ? 9999.0f : d));
    } else if (selected == DRIVE_F4) { /* F4: US1 at the stone stop, turn scale x100 (0 = not yet), crossings out, the leg */
      /* U241K103D  -4L 9: US1 at the stone, turn scale x100 (0 = not measured yet), heading off the line at the stone, leg */
      int us = 0, dr = 0; unsigned xo = 0u, xh = 0u, leg = 0u, k = 0u;
      if (!F1_F4Info(&us, &xo, &xh, &leg, &k, &dr)) { us = dr = 0; xo = xh = leg = k = 0u; }
      (void)xo; (void)xh;
      tft_prints(0, 8, "U%3dK%3uD%4dL%2u", us > 999 ? 999 : us, k > 999u ? 999u : k, dr > 999 ? 999 : (dr < -999 ? -999 : dr),
                 leg % 100u);
    } else tft_prints(0, 8, "PIVOT %4d DEG   ", (int)(d > 9999.0f ? 9999.0f : d)); }
#else
  tft_printc(0, 8, "                ");
#endif
  (void)buf;
#else
  SideIR_DrawRow(1); /* raw edge counts since boot; BR is a separate per-run tally */
  if (ir_fresh(now)) {
    (void)snprintf(buf, sizeof(buf), "I %4u %4u %4u", (unsigned)ir_average[0] % 10000u,
                   (unsigned)ir_average[1] % 10000u, (unsigned)ir_average[2] % 10000u);
    tft_printc(0, 2, buf);
    tft_prints(0, 3, "LINE %c %c %c L/C/R", filtered_mask & 1u ? 'B' : 'W',
               filtered_mask & 2u ? 'B' : 'W', filtered_mask & 4u ? 'B' : 'W');
  } else { tft_printc(0, 2, "I ---- ---- ----"); tft_printc(0, 3, "IR NOT READY    "); }
  if (!timer_ready) tft_printc(0, 4, "US1 CHECK TIM2  ");
  else if (!us_have_reading) tft_printc(0, 4, "US1 WAIT PING   ");
  else if (!us_valid) tft_prints(0, 4, "%-16s", us_error_text(us1.status));
  else if (!us_fresh(now)) tft_printc(0, 4, "US1 STALE       ");
  else tft_prints(0, 4, "US1 %4u mm     ", (unsigned)us1.raw_mm);
  tft_prints(0, 5, "%-16s", status);
  const char *hint = state == RUNNING ? "B2 STOP 1+2 NEXT" : "B1 RUN  1+2 NEXT";
  tft_prints(0, 6, "%-16s", hint);
  tft_prints(0, 7, "T %4u %4u %4u", (unsigned)profile.t[0], (unsigned)profile.t[1], (unsigned)profile.t[2]);
  tft_prints(0, 8, "BLACK %c %c %c     ", profile.black_high[0] ? 'H' : 'L',
             profile.black_high[1] ? 'H' : 'L', profile.black_high[2] ? 'H' : 'L');
#endif
  tft_printc(0, 9, "ESTOP REQUIRED  ");
  tft_update(0u);
}
void CalibrationApp_Init(void) {
  ELCP_ZeroPWM_Result result = ELCP_ZeroPWM_Init();
  motor_ready = result.j1_ok && result.j3_ok;
  in_setup = true;
  tft_init(PIN_ON_TOP, BLACK, WHITE, CYAN, DARK_GREY); /* PWM already at zero during LCD delays */
  IR_Sensor_Init();
  IRSetup_Init(HAL_GetTick());
}
static void plan_error(void) {
  selected = DRIVE_DISABLED;
  stop("DRIVE PLAN ERROR");
  tft_printc(0, 0, status);
  tft_update(0u);
}
/* Selection is a stopped operation. Keep the calibrated profile, live sensor
 * window, IRQ totals and motor/timer setup. Discard the previous route tally. */
static void select_stopped(DriveFunction route, uint32_t now) {
  selected = route;
  if (OLD_F3(selected)) F3_Reset(); /* reachable only in the pinned host replay */
  else if (selected == DRIVE_F2) F2_Init();
  else F1_Init();
  state = IDLE;
  last_screen = now - SCREEN_MS; /* show the selection on the release tick */
}
static void enter_drive(uint32_t now) {
  const DriveFunction first = DrivePlan_InitialFunction();
  if (first != DRIVE_F1 && first != DRIVE_F2 && first != DRIVE_F3) { plan_error(); return; }
  timer_ready = US2_Init();
#if !F1_LEGACY
  us_async = timer_ready && US1C_Init(); /* falls back to the blocking ping if TIM2 is not as expected */
#endif
  buttons = (ButtonGesture){.armed = true, .changed = now}; /* setup already qualified both up */
  memset(ir_sum, 0, sizeof(ir_sum));
  ir_count = ir_next = 0; us_valid = false; us_have_reading = false;
  raw_mask = filtered_mask = 0u;
#if IR_DIGITAL
  last_ir = now; /* the digital setup page sampled live until this release: keep the 10 ms scan spacing */
#else
  last_ir = now - IR_SAMPLE_MS;
#endif
  last_ping = now - US_PERIOD_MS;
  select_stopped(first, now);
}
static const char *start_inhibit(uint32_t now) {
  if (!motor_ready) return "MOTOR INIT ERR";
  if (!timer_ready) return "CHECK TIM2";
  if (!ir_fresh(now) || !us_fresh(now)) return "WAIT SENSORS";
  const char *range = range_inhibit(selected, us1.raw_mm);
  if (range) return range;
  if (OLD_F3(selected) && !(raw_mask & filtered_mask)) return "F3 NEED LINE";
  return NULL;
}
static void start_selected(uint32_t now) {
  state = RUNNING;
  if (OLD_F3(selected)) {
    F3_Reset();
    F3_Scan(now, raw_mask, filtered_mask);
  } else {
    const bool side_high = SideIR_TakeBranchSample().high; /* ignore pre-start edges */
#if !F1_LEGACY
    SideIR_CountCoast(selected == DRIVE_F4); /* F4 counts the roll-on after each stop; F1-F3 as before */
    feed_odometry(); /* bring F1's distance totals up to date BEFORE the start is noted (they run since boot) */
#endif
    F1_Init(); /* every accepted B1 starts fresh, even consecutive F1 runs */
#if !F1_LEGACY
    Retrace_Start(&trail, now, enc_j1(), enc_j3()); /* a fresh trail from this start position */
#endif
    if (selected == DRIVE_F2) {
      F1_StartPerScan(now, filtered_mask);
      F2_Start(side_high);
    } else if (selected == DRIVE_F3) F1_StartF3(now, filtered_mask, side_high); /* the top-right key */
    else if (selected == DRIVE_F4) F1_StartF4(now, filtered_mask, side_high); /* the bottom-left key (sim only) */
    else F1_Start(now, filtered_mask, side_high);
  }
}

void CalibrationApp_Update(void) {
  uint32_t now = HAL_GetTick();
  if (in_setup) {
#if !F1_LEGACY && IR_DIGITAL
    { /* powered wheel check (robot lifted): B1 alone held 1 s = left wheel forward, B2 alone = right; else both at 0 */
      static WheelSpin spin;
      static unsigned was;
      const unsigned w = WheelSpin_Update(&spin, now, btn_read(BTN1), btn_read(BTN2), enc_raw_j1(), enc_raw_j3());
      if (w == SPIN_NONE) {
        if (was != SPIN_NONE) { ELCP_Motor_SetSpeed(MOTOR_J1, 0.0f); ELCP_Motor_SetSpeed(MOTOR_J3, 0.0f); }
        ELCP_ZeroPWM_Hold(); /* before every button, ADC or display operation, including apply */
      } else {
        ELCP_Motor_SetSpeed(w == SPIN_LEFT ? MOTOR_J1 : MOTOR_J3, 0.0f);
        if (w == SPIN_LEFT) ELCP_Motor_SetSpeed(MOTOR_J3, J3_FORWARD * WHEEL_SPIN_PCT);
        else ELCP_Motor_SetSpeed(MOTOR_J1, J1_FORWARD * WHEEL_SPIN_PCT);
      }
      was = w;
      IRSetup_Spin(spin.last_wheel == SPIN_LEFT ? 'L' : spin.last_wheel == SPIN_RIGHT ? 'R' : 0, w != SPIN_NONE, spin.pulses);
    }
#else
    ELCP_ZeroPWM_Hold(); /* before every button, ADC or display operation, including apply */
#endif
#if !F1_LEGACY
    IRSetup_Wheels(enc_raw_j1(), enc_raw_j3()); /* raw FB pulses: turn each wheel by hand, its count must go up */
#endif
    if (IRSetup_Update(now, &profile)) {
      in_setup = false;
      enter_drive(HAL_GetTick()); /* idle, empty IR window, no cached US1 reading */
    }
    return; /* never dispatch a motor or mission button on the apply gesture */
  }
  if (selected == DRIVE_DISABLED) return;
  /* A poll may fault/stop this run. A key first seen inside that poll still
   * belongs to the old run and cannot request a restart on release. */
  const GestureMode mode = state == RUNNING ? GESTURES_CHORD_ONLY : GESTURES_ALL;
  unsigned action = 0u;
  poll_buttons(now, mode, &action);
  if (state == RUNNING && (!ir_fresh(now) || !us_fresh(now))) stop("SENSOR STALE");
  if ((uint32_t)(now - last_ir) >= IR_SAMPLE_MS) read_ir(now);
  now = HAL_GetTick();
  poll_buttons(now, mode, &action);
  if (state == RUNNING && (!ir_fresh(now) || !us_fresh(now))) stop("SENSOR STALE");
#if !F1_LEGACY
  if (us_async) { /* no waiting: the echo is timed by the TIM2 capture interrupt */
    US1C_Update(now);
    US2_Reading r;
    uint32_t fired;
    if (US1C_Take(&r, &fired)) handle_us(r, fired);
  } else
#endif
  if (timer_ready && (uint32_t)(now - last_ping) >= US_PERIOD_MS) read_us(now);
  now = HAL_GetTick();
  poll_buttons(now, mode, &action);
  if (state == RUNNING && !OLD_F3(selected)) {
    const char *reason = F1_Advance(now);
    if (reason) stop(reason);
  }
  if (state == RUNNING && (!ir_fresh(now) || !us_fresh(now))) stop("SENSOR STALE");
  if (state != RUNNING) {
    if (action == KEY_BOTH) {
      const DriveFunction next = DrivePlan_NextFunction(selected);
      if (next != DRIVE_F1 && next != DRIVE_F2 && (F1_LEGACY || (next != DRIVE_F3 && next != DRIVE_F4))) { plan_error(); return; }
      select_stopped(next, now); /* never also a start */
    }
    const char *blocked = start_inhibit(now);
    if (action == KEY_B1) {
      if (blocked) status = blocked;
      else start_selected(now);
    } else if (state == IDLE) {
      status = blocked ? blocked : (filtered_mask == 0u ? "NO LINE B1 GO" : "READY TAP B1");
    }
  }
  if (state == RUNNING) {
    if (OLD_F3(selected)) drive_f3(now);
    else drive_line(now);
  }
#if !F1_LEGACY
  if (state == RUNNING) {
    const char *dead = EncWatch_Update(&enc_watch, now, ELCP_Motor_GetState(MOTOR_J1)->current_speed_pct, enc_raw_j1(),
                                       ELCP_Motor_GetState(MOTOR_J3)->current_speed_pct, enc_raw_j3());
    if (dead) stop(dead);
  } else EncWatch_Reset(&enc_watch); /* every run starts afresh (a 0 % pause inside a run disarms that wheel) */
#endif
#if F1_LEGACY
  if ((uint32_t)(now - last_screen) >= SCREEN_MS) { last_screen = now; draw(now); }
#else
  if ((uint32_t)(now - last_screen) >= (state == RUNNING ? SCREEN_RUN_MS : SCREEN_MS)) { last_screen = now; draw(now); }
#endif
}
